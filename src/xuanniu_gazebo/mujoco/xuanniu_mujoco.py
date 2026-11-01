#!/usr/bin/env python3
"""MuJoCo native Simulate and selector service; launch the Qt GUI separately."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys
import time
import mujoco

from selector_backend import ROOT, SelectorSimulation
from runtime import SimulationConnection
from transport import StateServer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless','--no-viewer',action='store_true')
    parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--target',choices=['manual','stop','auto'],default='stop')
    parser.add_argument('--host',choices=('127.0.0.1','localhost'),default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8767)
    parser.add_argument('--component',default='xuanniu')
    parser.add_argument('--model',type=Path,default=ROOT/'mujoco/xuanniu.xml')
    parser.add_argument('--camera',choices=['free','front','side','top','oblique'],default='free')
    parser.add_argument('--ready-file',type=Path)
    args=parser.parse_args()
    simulation=SelectorSimulation(args.model)
    from xuanniu_gazebo.logic import MODES
    if args.target!='stop': simulation.command(MODES[args.target.upper()])
    if args.headless: engine=SimulationConnection(simulation)
    else:
        from native_viewer import NativeViewer
        engine=NativeViewer(simulation,args.model,args.camera)
    server=None
    # Native exit is thread-safe and does not take the model/render mutex.
    def stop(*_):
        engine.done.set()
        if hasattr(engine,'viewer'): engine.viewer.exit()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        server=StateServer(engine,args.host,args.port,args.component)
        address={'host':server.address[0],'port':server.address[1],'component':args.component,
                 'python':sys.executable,'mujoco':mujoco.__version__,
                 'viewer':'headless' if args.headless else 'native Simulate'}
        print('Selector simulation ready: '+json.dumps(address),flush=True)
        if args.ready_file: args.ready_file.write_text(json.dumps(address))
        if args.headless:
            started=time.monotonic()
            while not engine.done.wait(.05):
                if args.duration and time.monotonic()-started>=args.duration: break
        else: engine.run(args.duration)
        print(json.dumps(asdict(engine.feedback()[0]),indent=2))
    finally:
        if server: server.close()
        # Native run() already destroyed the C++ object.
        if args.headless or engine.thread is None: engine.close()


if __name__=='__main__': main()
