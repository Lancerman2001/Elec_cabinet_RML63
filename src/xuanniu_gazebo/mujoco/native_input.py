"""Send native shortcuts only to this process's Simulate window (Linux/WSLg).

MuJoCo 3.12 exposes Simulate.run as read-only. Remote Pause/Resume therefore
uses the same Space event as the native UI, rather than maintaining two run
states or changing the installed MuJoCo binary. No desktop focus is changed.
"""
import ctypes as C
import os
import re
import subprocess
import threading


class KeyEvent(C.Structure):
    _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),
              ('display',C.c_void_p),('window',C.c_ulong),('root',C.c_ulong),
              ('subwindow',C.c_ulong),('time',C.c_ulong),('x',C.c_int),('y',C.c_int),
              ('x_root',C.c_int),('y_root',C.c_int),('state',C.c_uint),
              ('keycode',C.c_uint),('same_screen',C.c_int)]


class Event(C.Union):
    _fields_=[('key',KeyEvent),('padding',C.c_long*24)]


class NativeInput:
    def __init__(self):
        self.lib=C.CDLL('libX11.so.6'); self.lock=threading.Lock()
        self.lib.XOpenDisplay.argtypes=[C.c_char_p]; self.lib.XOpenDisplay.restype=C.c_void_p
        self.lib.XDefaultRootWindow.argtypes=[C.c_void_p]; self.lib.XDefaultRootWindow.restype=C.c_ulong
        self.lib.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong]; self.lib.XKeysymToKeycode.restype=C.c_uint
        self.lib.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
        self.lib.XFlush.argtypes=[C.c_void_p]
        self.lib.XCloseDisplay.argtypes=[C.c_void_p]
        self.display=self.lib.XOpenDisplay(None)
        if not self.display: raise RuntimeError('Native viewer requires the Linux/WSLg X11 display')
        self.window=None

    def find_window(self):
        tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
        for window,title in re.findall(r'(0x[0-9a-f]+) "([^"]*)"',tree):
            if 'MuJoCo' not in title: continue
            pid=subprocess.check_output(['xprop','-id',window,'_NET_WM_PID'],text=True)
            if re.search(r'=\s*'+str(os.getpid())+r'\s*$',pid):
                self.window=int(window,16); return self.window
        raise RuntimeError('Cannot find this process\'s MuJoCo window')

    def key(self, keysym, state=0):
        with self.lock:
            window=self.window or self.find_window()
            event=Event(); key=event.key
            key.display=self.display; key.window=window
            key.root=self.lib.XDefaultRootWindow(self.display)
            key.same_screen=1; key.state=state
            key.keycode=self.lib.XKeysymToKeycode(self.display,keysym)
            for kind,mask in ((2,1),(3,2)):
                key.type=kind
                if not self.lib.XSendEvent(self.display,window,0,mask,C.byref(event)):
                    raise RuntimeError('Native key event was not accepted')
            self.lib.XFlush(self.display)

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display); self.display=None
