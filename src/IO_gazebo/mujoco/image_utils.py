"""Small PNG and X11 capture helpers; no Pillow dependency in the ros environment."""
import ctypes
import ctypes.util
from pathlib import Path
import struct
import time
import zlib
import numpy as np


def write_png(path,pixels):
    pixels=np.asarray(pixels,dtype=np.uint8)
    h,w,channels=pixels.shape
    if channels!=3: raise ValueError('Expected RGB pixels')
    def chunk(name,data): return struct.pack('>I',len(data))+name+data+struct.pack('>I',zlib.crc32(name+data)&0xffffffff)
    data=b''.join(b'\0'+row.tobytes() for row in pixels)
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(data))+chunk(b'IEND',b'')
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(png)


def capture_tk(root,path,pump):
    # Read the composited window pixmap: ordinary screen-grab returns black on WSLg.
    class XImage(ctypes.Structure):
        _fields_=[('width',ctypes.c_int),('height',ctypes.c_int),('xoffset',ctypes.c_int),('format',ctypes.c_int),('data',ctypes.c_void_p),('byte_order',ctypes.c_int),('bitmap_unit',ctypes.c_int),('bitmap_bit_order',ctypes.c_int),('bitmap_pad',ctypes.c_int),('depth',ctypes.c_int),('bytes_per_line',ctypes.c_int),('bits_per_pixel',ctypes.c_int)]
    x11=ctypes.CDLL(ctypes.util.find_library('X11')); comp=ctypes.CDLL(ctypes.util.find_library('Xcomposite'))
    x11.XOpenDisplay.argtypes=[ctypes.c_char_p]; x11.XOpenDisplay.restype=ctypes.c_void_p
    x11.XCloseDisplay.argtypes=[ctypes.c_void_p]; x11.XSync.argtypes=[ctypes.c_void_p,ctypes.c_int]
    x11.XFreePixmap.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
    x11.XGetImage.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint,ctypes.c_ulong,ctypes.c_int]
    x11.XGetImage.restype=ctypes.POINTER(XImage); x11.XDestroyImage.argtypes=[ctypes.POINTER(XImage)]
    for name in ['XCompositeRedirectWindow','XCompositeUnredirectWindow']:
        getattr(comp,name).argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int]
    comp.XCompositeNameWindowPixmap.argtypes=[ctypes.c_void_p,ctypes.c_ulong]; comp.XCompositeNameWindowPixmap.restype=ctypes.c_ulong
    display=x11.XOpenDisplay(None)
    if not display: raise RuntimeError('Cannot connect to X11 display for screenshot')
    pixmap=0; pointer=None; window=root.winfo_id()
    try:
        comp.XCompositeRedirectWindow(display,window,0); x11.XSync(display,0); pump(.25)
        pixmap=comp.XCompositeNameWindowPixmap(display,window)
        pointer=x11.XGetImage(display,pixmap,0,0,root.winfo_width(),root.winfo_height(),ctypes.c_ulong(-1),2)
        if not pointer: raise RuntimeError('Cannot read Tk pixmap')
        frame=pointer.contents
        if frame.bits_per_pixel!=32 or frame.byte_order!=0: raise RuntimeError('Unsupported X11 pixel layout')
        raw=ctypes.string_at(frame.data,frame.bytes_per_line*frame.height)
        pixels=np.frombuffer(raw,dtype=np.uint8).reshape(frame.height,frame.bytes_per_line)[:,:frame.width*4].reshape(frame.height,frame.width,4)
        write_png(path,pixels[:,:,2::-1])
    finally:
        if pointer: x11.XDestroyImage(pointer)
        if pixmap: x11.XFreePixmap(display,pixmap)
        comp.XCompositeUnredirectWindow(display,window,0); x11.XCloseDisplay(display)
