"""Version 1 newline-delimited JSON over local TCP; no ROS, Qt or MuJoCo imports."""
from dataclasses import asdict
from collections import deque
import json
import math
import select
import socket
import socketserver
import threading
import time
from types import SimpleNamespace


def packet(value):
    return (json.dumps(value,allow_nan=False,separators=(',',':'))+'\n').encode('utf-8')


class StateServer:
    """Adapt an engine with lock, simulation and feedback() to the public protocol."""
    def __init__(self, engine, host='127.0.0.1', port=8767, component='xuanniu'):
        if host not in ('127.0.0.1', 'localhost'):
            raise ValueError('The unauthenticated demo server only accepts a loopback host')
        self.engine=engine; self.component=component; self.owner=None
        self.done=threading.Event(); outer=self

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address=True
            daemon_threads=True
            block_on_close=False

        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                buffer=b''; next_send=time.monotonic()
                self.request.settimeout(.2)
                self.request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                try:
                    while not outer.done.is_set():
                        now=time.monotonic()
                        if now>=next_send:
                            state,received=outer.engine.feedback()
                            # Don't turn stalled physics into apparently fresh feedback.
                            if time.monotonic()-received<=.5:
                                self.request.sendall(packet({'version':1,'component':outer.component,'type':'status','state':asdict(state)}))
                            next_send+=.02
                            if next_send<now: next_send=now+.02
                        ready,_,_=select.select([self.request],[],[],min(.005,max(0,next_send-time.monotonic())))
                        if not ready: continue
                        chunk=self.request.recv(4096)
                        if not chunk: break
                        buffer+=chunk
                        if len(buffer)>16384: break
                        while b'\n' in buffer:
                            line,buffer=buffer.split(b'\n',1)
                            try: outer.dispatch(self,json.loads(line))
                            except (ValueError,TypeError,KeyError):
                                outer.execute(lambda:setattr(outer.engine.simulation,'error','INVALID_COMMAND'))
                except (OSError,ValueError):
                    pass
                finally:
                    def disconnect():
                        if outer.owner is self:
                            outer.engine.simulation.end_drag(); outer.owner=None
                    outer.execute(disconnect)

        self.server=Server(('127.0.0.1',port),Handler)
        self.address=self.server.server_address
        self.thread=threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.05},daemon=True)
        self.thread.start()

    def dispatch(self, client, message):
        if not isinstance(message,dict): raise ValueError('Expected object')
        if message.get('version')!=1 or message.get('component')!=self.component:
            return  # Another component must never receive this command.
        def apply():
            try: self.apply(client,message)
            except (ValueError,TypeError,KeyError): self.engine.simulation.error='INVALID_COMMAND'
        self.execute(apply)

    def execute(self, callback):
        if hasattr(self.engine,'enqueue'):
            self.engine.enqueue(callback)
        else:
            with self.engine.lock: callback()

    def apply(self, client, message):
        # Called by the physics owner, already under the model lock.
        sim=self.engine.simulation
        stamp=message.get('stamp')
        if not isinstance(stamp,(float,int)) or not math.isfinite(stamp) or not -.1<=time.time()-stamp<=.5:
            sim.error='STALE_COMMAND'; return
        action=message.get('type')
        if action=='command':
            angle=message['angle_rad']; dragging=message.get('dragging',False)
            if isinstance(angle,bool) or not isinstance(angle,(float,int)) or not isinstance(dragging,bool):
                raise ValueError('Invalid command fields')
            if sim.command(angle,dragging): self.owner=client if dragging else None
        elif action=='end_drag':
            if self.owner is client: sim.end_drag(); self.owner=None
        elif action=='pause':
            if not isinstance(message.get('paused'),bool): raise ValueError('Invalid pause')
            if hasattr(self.engine,'apply_pause'): self.engine.apply_pause(message['paused'])
            else: sim.set_paused(message['paused'])
            self.owner=None
        elif action=='reset':
            if hasattr(self.engine,'apply_reset'): self.engine.apply_reset()
            else: sim.reset()
            self.owner=None
        else: raise ValueError('Unknown command')

    def close(self):
        self.done.set(); self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=1)


class RemoteConnection:
    """Reconnect automatically; never queue or replay commands across reconnects."""
    def __init__(self, host='127.0.0.1', port=8767, component='xuanniu'):
        self.host=host; self.port=port; self.component=component
        self.lock=threading.RLock(); self.done=threading.Event(); self.socket=None
        self._outgoing=deque()
        self.sample=None; self.received=0.0
        self.thread=threading.Thread(target=self._run,name='selector-feedback',daemon=True); self.thread.start()

    def _run(self):
        while not self.done.is_set():
            sock=None
            try:
                sock=socket.create_connection((self.host,self.port),timeout=.25); sock.setblocking(False)
                sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                with self.lock: self.socket=sock
                buffer=b''; pending=b''
                while not self.done.is_set():
                    with self.lock:
                        if not pending and self._outgoing: pending=packet(self._outgoing.popleft())
                    reads,writes,_=select.select([sock],[sock] if pending else [],[],.005)
                    if writes:
                        try: pending=pending[sock.send(pending):]
                        except BlockingIOError: pass
                    if not reads: continue
                    try: chunk=sock.recv(4096)
                    except BlockingIOError: continue
                    if not chunk: break
                    buffer+=chunk
                    if len(buffer)>32768: break
                    while b'\n' in buffer:
                        line,buffer=buffer.split(b'\n',1); value=json.loads(line)
                        if value.get('version')==1 and value.get('component')==self.component and value.get('type')=='status':
                            state=SimpleNamespace(**value['state'])
                            with self.lock: self.sample=state; self.received=time.monotonic()
            except (OSError,ValueError,TypeError,KeyError):
                pass
            finally:
                with self.lock:
                    self.socket=None; self.received=0.0; self._outgoing.clear()
                if sock: sock.close()
            self.done.wait(.25)

    def feedback(self):
        with self.lock: return self.sample,self.received

    def command_ready(self):
        with self.lock:
            return self.socket is not None and self.sample is not None and time.monotonic()-self.received<=.5 and not self.sample.paused

    def request(self, action, **fields):
        with self.lock:
            if self.socket is None: return False
            try:
                message={'version':1,'component':self.component,'stamp':time.time(),'type':action,**fields}
                packet(message)  # Validate now; socket writes belong to the worker.
                if action=='command' and message.get('dragging') and self._outgoing and self._outgoing[-1].get('dragging'):
                    self._outgoing[-1]=message
                else:
                    # New discrete actions supersede unsent drag targets.
                    if action!='command' or not message.get('dragging'):
                        self._outgoing=deque(m for m in self._outgoing if not m.get('dragging'))
                    if len(self._outgoing)>=32: return False
                    self._outgoing.append(message)
                return True
            except (OSError,ValueError): return False

    def send(self, angle, dragging=False):
        return self.request('command',angle_rad=angle,dragging=dragging)

    def pause(self, paused): return self.request('pause',paused=paused)
    def reset(self): return self.request('reset')

    def close(self):
        # EOF tells the server to end the owning drag; no blocking final write.
        self.done.set()
        with self.lock:
            if self.socket:
                try: self.socket.shutdown(socket.SHUT_RDWR)
                except OSError: pass
        self.thread.join(timeout=1)
