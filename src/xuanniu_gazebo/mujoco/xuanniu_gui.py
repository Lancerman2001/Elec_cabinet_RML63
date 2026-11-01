#!/usr/bin/env python3
"""Independent Qt client: no model loading, physics stepping or MuJoCo import."""
import argparse
from pathlib import Path
import signal
import sys

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'src'))
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QPushButton
from xuanniu_gazebo.gui import SelectorWindow
from transport import RemoteConnection


class MuJoCoControls(SelectorWindow):
    def __init__(self,connection,english=False):
        super().__init__(connection,ROOT,{'use_bundle':not english,'use_system':not english})
        buttons=QHBoxLayout()
        self.pause_button=QPushButton('Pause'); self.reset_button=QPushButton('Reset to Stop')
        buttons.addWidget(self.pause_button); buttons.addWidget(self.reset_button)
        self.layout().addLayout(buttons)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.reset_button.clicked.connect(self.reset)
        self.timer.setTimerType(Qt.PreciseTimer); self.timer.start(16)
        self.resize(590,760); self.tick()

    def tr(self,key):
        if key=='connected':
            state=self.connection.feedback()[0]
            if state and state.paused: return '仿真暂停' if self.chinese else 'Simulation paused'
            return '已连接 MuJoCo' if self.chinese else 'Connected to MuJoCo'
        return super().tr(key)

    def tick(self):
        if self.local_drag and not self.connection.command_ready():
            self.local_drag=False; self.dial.cancel_drag()
        super().tick()
        if hasattr(self,'pause_button'):
            import time
            state,received=self.connection.feedback()
            fresh=state is not None and time.monotonic()-received<=.5
            self.pause_button.setEnabled(fresh); self.reset_button.setEnabled(fresh)
            self.pause_button.setText('Resume' if state and state.paused else 'Pause')

    def toggle_pause(self):
        self.abandon_drag()
        state,_=self.connection.feedback()
        if state: self.connection.pause(not state.paused)

    def reset(self):
        self.local_drag=False; self.dial.cancel_drag(); self.pending_until=0
        self.connection.reset()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8767)
    parser.add_argument('--component',default='xuanniu')
    parser.add_argument('--english',action='store_true')
    parser.add_argument('--duration',type=float,default=0,help='Exit after N seconds (test helper)')
    parser.add_argument('--screenshot',type=Path,help='Save GUI after one second')
    args=parser.parse_args()
    app=QApplication(sys.argv[:1]); app.setApplicationName('Selector Switch Control')
    connection=RemoteConnection(args.host,args.port,args.component)
    window=MuJoCoControls(connection,args.english); window.show()
    signal.signal(signal.SIGINT,lambda *_:window.close())
    signal.signal(signal.SIGTERM,lambda *_:window.close())
    if args.duration: QTimer.singleShot(int(args.duration*1000),window.close)
    if args.screenshot:
        args.screenshot.parent.mkdir(parents=True,exist_ok=True)
        QTimer.singleShot(1000,lambda:window.grab().save(str(args.screenshot)))
    try: app.exec_()
    finally: connection.close()


if __name__=='__main__': main()
