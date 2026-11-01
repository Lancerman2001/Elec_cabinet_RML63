#!/usr/bin/env python3
"""MuJoCo viewer and a three-button Chinese panel, independent of ROS."""
from __future__ import annotations
import argparse
from contextlib import nullcontext
from dataclasses import dataclass
import math
from pathlib import Path
import queue
import threading
import time
from tk_runtime import prepare_tk
prepare_tk()
import tkinter as tk
from tkinter import ttk, font

from breaker_backend import BreakerBackend, MODEL_PATH, State


@dataclass(frozen=True)
class Update:
    state: State | None
    running: bool
    paused: bool
    error: str = ''


class Worker(threading.Thread):
    """Owns MuJoCo model/data; the Tk thread exchanges commands and snapshots."""
    def __init__(self,use_viewer=True,show_panel=True,model_path=MODEL_PATH):
        super().__init__(name='breaker-mujoco',daemon=True)
        self.use_viewer=use_viewer; self.show_panel=show_panel; self.model_path=model_path
        self.commands=queue.Queue(); self.updates=queue.Queue(maxsize=1)
        self.stop_event=threading.Event(); self.ready=threading.Event()
        self.backend=None; self.viewer=None; self.paused=False; self.error=''

    def submit(self,command): self.commands.put(command)
    def stop(self): self.stop_event.set()

    def emit(self,running=True):
        update=Update(self.backend.state() if self.backend else None,running,self.paused,self.error)
        try: self.updates.get_nowait()
        except queue.Empty: pass
        self.updates.put_nowait(update)

    def accept_commands(self):
        while True:
            try: command=self.commands.get_nowait()
            except queue.Empty: return
            try:
                if command=='pause':
                    self.paused=not self.paused
                    if self.paused: self.backend.cancel()
                elif command=='reset': self.backend.reset()
                elif command in ('on','off'):
                    if self.paused: raise RuntimeError('仿真已暂停，请按空格恢复。')
                    self.backend.set_power(command=='on')
                else: raise ValueError('未知操作：'+command)
                self.error=''
            except (RuntimeError,ValueError) as exc: self.error=str(exc)

    def loop(self,viewer):
        previous=time.monotonic(); accumulator=0.; last_emit=0.
        self.viewer=viewer; self.ready.set()
        while not self.stop_event.is_set() and (viewer is None or viewer.is_running()):
            now=time.monotonic(); elapsed=min(.05,max(0.,now-previous)); previous=now
            with viewer.lock() if viewer is not None else nullcontext():
                self.accept_commands()
                if self.paused: accumulator=0.
                else:
                    accumulator+=elapsed
                    steps=int(accumulator/self.backend.dt)
                    if steps:
                        self.backend.step(steps); accumulator-=steps*self.backend.dt
                if now-last_emit>=.025: self.emit(); last_emit=now
            if viewer is not None: viewer.sync()
            self.stop_event.wait(.008)

    def run(self):
        try:
            self.backend=BreakerBackend(self.model_path,self.show_panel)
            if self.use_viewer:
                import mujoco.viewer
                with mujoco.viewer.launch_passive(self.backend.model,self.backend.data,
                    key_callback=lambda key:self.submit('pause') if key==32 else None,
                    show_left_ui=False,show_right_ui=False) as viewer:
                    with viewer.lock():
                        viewer.cam.lookat[:]=[.014,0,.12]
                        viewer.cam.distance=.145; viewer.cam.azimuth=-35; viewer.cam.elevation=-20
                        viewer.opt.geomgroup[3]=0; viewer.opt.geomgroup[5]=0
                    self.loop(viewer)
            else: self.loop(None)
        except Exception as exc: self.error=str(exc)
        finally:
            self.ready.set(); self.viewer=None; self.emit(running=False)


class BreakerPanel:
    def __init__(self,root,use_viewer=True,show_panel=True,model_path=MODEL_PATH):
        self.root=root; self.closed=False; self.pending=False
        self.latest=Update(None,True,False); self.received=0.
        root.title('Circuit Breaker - MuJoCo Control')
        root.geometry('600x420+1050+130'); root.minsize(580,410); root.configure(bg='#edf1f4')
        families=font.families(root)
        family=next((f for f in ['Noto Sans CJK SC','WenQuanYi Micro Hei','DejaVu Sans'] if f in families),'sans-serif')
        style=ttk.Style(root); style.theme_use('clam')
        style.configure('Breaker.TFrame',background='#edf1f4')
        style.configure('Breaker.TLabel',background='#edf1f4',foreground='#24364a',font=(family,11))
        style.configure('Breaker.TButton',font=(family,12),padding=(10,12))
        frame=ttk.Frame(root,padding=24,style='Breaker.TFrame'); frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='空气开关通断控制',font=(family,21,'bold'),style='Breaker.TLabel').pack(anchor='w')
        ttk.Label(frame,text='MuJoCo  ·  双极联动  ·  上拨 ON / 下拨 OFF',style='Breaker.TLabel',foreground='#607389').pack(anchor='w',pady=(4,18))
        card=tk.Frame(frame,bg='white',padx=18,pady=15); card.pack(fill='x')
        self.power_text=tk.StringVar(value='正在加载模型…')
        self.power_label=tk.Label(card,textvariable=self.power_text,font=(family,23,'bold'),bg='white',fg='#627183'); self.power_label.pack(anchor='w')
        self.angle_text=tk.StringVar(value='拨杆角度  —')
        tk.Label(card,textvariable=self.angle_text,font=(family,12),bg='white',fg='#344b60').pack(anchor='w',pady=(7,0))
        self.status_text=tk.StringVar(value='正在启动 MuJoCo…')
        ttk.Label(frame,textvariable=self.status_text,style='Breaker.TLabel').pack(anchor='w',pady=(16,14))
        actions=ttk.Frame(frame,style='Breaker.TFrame'); actions.pack(fill='x')
        self.buttons={}
        for key,text in [('on','↑ 上拨通电'),('off','↓ 下拨断电'),('reset','初始化 / 复位')]:
            button=ttk.Button(actions,text=text,style='Breaker.TButton',command=lambda cmd=key:self.submit(cmd),state='disabled')
            button.pack(side='left',fill='x',expand=True,padx=(0,6)); self.buttons[key]=button
        self.default_note='默认从下拨 OFF 启动。三维窗口中按空格可暂停 / 恢复。' if use_viewer else '仅控制面板模式，MuJoCo 仿真在后台运行。'
        self.note=tk.StringVar(value=self.default_note)
        ttk.Label(frame,textvariable=self.note,style='Breaker.TLabel',foreground='#607389',wraplength=540).pack(anchor='w',pady=(16,0))
        self.worker=Worker(use_viewer,show_panel,model_path)
        root.protocol('WM_DELETE_WINDOW',self.close); self.worker.start(); root.after(30,self.poll)

    def submit(self,command):
        if self.pending or self.closed: return
        self.pending=True
        for button in self.buttons.values(): button.configure(state='disabled')
        self.worker.submit(command)

    def poll(self):
        if self.closed: return
        try:
            update=self.worker.updates.get_nowait(); self.latest=update
            self.received=time.monotonic(); self.pending=False
        except queue.Empty: pass
        update=self.latest; state=update.state
        fresh=bool(self.received) and time.monotonic()-self.received<.75
        if state is not None:
            self.power_text.set('ON  ·  通电' if state.powered else 'OFF  ·  断电')
            self.power_label.configure(fg='#ba492f' if state.powered else '#21806c')
            self.angle_text.set('拨杆角度  %+.1f°    /    −40° ～ +40°' % math.degrees(state.angle))
            self.status_text.set('● MuJoCo 已连接  ·  '+('已暂停' if update.paused else state.status))
        self.note.set('运行提示：'+update.error if update.error else self.default_note)
        if not update.running:
            self.status_text.set('● MuJoCo 已停止')
            if not update.error: self.note.set('三维窗口已关闭，请关闭面板后重新启动。')
        elif self.received and not fresh: self.status_text.set('● 等待仿真反馈…')
        allowed=fresh and update.running and state is not None and not state.busy and not update.paused and not self.pending
        for button in self.buttons.values(): button.configure(state='normal' if allowed else 'disabled')
        self.root.after(30,self.poll)

    def close(self):
        if self.closed: return
        self.closed=True; self.worker.stop(); self.worker.join(timeout=3.)
        self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description='MuJoCo 空气开关三维查看器与中文通断面板')
    parser.add_argument('--no-viewer',action='store_true',help='只打开控制面板，仿真在后台运行')
    parser.add_argument('--no-panel',action='store_true',help='隐藏三维局部柜面背景')
    parser.add_argument('--model',type=Path,default=MODEL_PATH)
    args=parser.parse_args()
    root=tk.Tk(); panel=BreakerPanel(root,not args.no_viewer,not args.no_panel,args.model)
    try: root.mainloop()
    except KeyboardInterrupt: pass
    finally:
        if not panel.closed: panel.close()


if __name__=='__main__': main()
