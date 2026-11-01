"""Use the system's Xft-enabled Tk 8.6 in this Conda Linux environment.

The ros environment ships Tk without Xft: its legacy X11 bitmap fonts cannot
render the CJK panel correctly. Load the ABI-compatible system Tcl/Tk before
_tkinter, for this process only. No Conda packages or shell settings are changed.
"""
import ctypes
from pathlib import Path
import sys


def prepare_tk():
    prefix=Path(sys.prefix)
    if not sys.platform.startswith('linux') or not (prefix/'conda-meta').is_dir():
        return
    if not (prefix/'lib/libtk8.6.so').exists():
        return
    for directory in [Path('/usr/lib/x86_64-linux-gnu'),Path('/usr/lib/aarch64-linux-gnu')]:
        tcl=directory/'libtcl8.6.so'; tk=directory/'libtk8.6.so'
        if tcl.exists() and tk.exists():
            if '_tkinter' in sys.modules:
                raise RuntimeError('请通过 breaker_gui.py 启动，确保先加载支持中文字体的 Tk。')
            ctypes.CDLL(str(tcl),mode=ctypes.RTLD_GLOBAL)
            ctypes.CDLL(str(tk),mode=ctypes.RTLD_GLOBAL)
            return
