"""Qt widgets driven by measured MuJoCo transport feedback."""
import math
from pathlib import Path
import time
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal, QEvent
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QFontMetrics, QPainter, QPainterPath, QPainterPathStroker, QPen, QPixmap, QTransform
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout
from .logic import HALF, adjacent, cursor_angle, drag_angle, snap
from .text import TEXT


def configure_font(app, package, use_bundle=True, use_system=True):
    """Never assumes a named Tk font or a particular system family exists."""
    families = []
    if use_bundle:
        font_id = QFontDatabase.addApplicationFont(str(package/'fonts/SelectorCJK.otf'))
        if font_id >= 0: families.extend(QFontDatabase.applicationFontFamilies(font_id))
    if use_system:
        available = set(QFontDatabase().families())
        families.extend(f for f in ('Noto Sans CJK SC','Noto Serif CJK SC','WenQuanYi Micro Hei','Microsoft YaHei','SimHei') if f in available)
    required = ''.join(value[0] for value in TEXT.values())
    for family in families:
        font = QFont(family, 11)
        metrics = QFontMetrics(font)
        if all(metrics.inFont(c) for c in required):
            app.setFont(font)
            return True, family
    font = QFont('sans-serif', 11); font.setStyleHint(QFont.SansSerif)
    app.setFont(font)
    return False, font.family()




class Dial(QWidget):
    drag_started = pyqtSignal(float)
    drag_changed = pyqtSignal(float)
    drag_finished = pyqtSignal(float)

    def __init__(self, package, tr, parent=None):
        super().__init__(parent)
        self.tr = tr; self.angle = 0.0; self.target = 0.0
        self.interactive = False; self.dragging = False; self.show_target = False
        self.plate = QPixmap(str(package/'materials/textures/nameplate.png'))
        self.setMinimumSize(350, 370)
        self.setMouseTracking(True)

    def geometry_values(self):
        scale = min((self.width()-42)/48, (self.height()-34)/49)
        return QPointF(self.width()/2, self.height()/2), scale

    def handle_path(self, scale):
        points = [(2,-20),(-2,-20),(-3,-6),(-5,-4),(-5,4),(-3,6),(-2,20),(2,20),(3,6),(5,4),(5,-4),(3,-6)]
        path = QPainterPath(QPointF(points[0][0]*scale, points[0][1]*scale))
        for x,y in points[1:]: path.lineTo(x*scale,y*scale)
        path.closeSubpath(); return path

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        centre, s = self.geometry_values(); p.translate(centre)
        p.setPen(Qt.NoPen); p.setBrush(QColor('#1e252d'))
        p.drawRoundedRect(QRectF(-24*s,-24.5*s,48*s,49*s),1.5*s,1.5*s)
        p.drawPixmap(QRectF(-22.5*s,-23*s,45*s,46*s),self.plate,QRectF(self.plate.rect()))
        p.setBrush(QColor('#242a32')); p.drawEllipse(QPointF(0,0),11*s,11*s)
        p.save(); p.rotate(-math.degrees(self.angle))
        p.setPen(QPen(QColor('#434b56'),1.5)); p.setBrush(QColor('#141a22'))
        p.drawPath(self.handle_path(s))
        marker=QPainterPath(QPointF(0,-18.2*s)); marker.lineTo(-1.2*s,-14.5*s); marker.lineTo(1.2*s,-14.5*s); marker.closeSubpath()
        p.setPen(Qt.NoPen); p.setBrush(QColor('#fff5d6')); p.drawPath(marker)
        p.setBrush(QColor('#bec7cd')); p.drawEllipse(QPointF(0,0),2.5*s,2.5*s)
        p.setPen(QPen(QColor('#56616c'),1.4)); p.drawLine(QPointF(-1.5*s,0),QPointF(1.5*s,0)); p.drawLine(QPointF(0,-1.5*s),QPointF(0,1.5*s))
        p.restore()
        if self.show_target:
            p.rotate(-math.degrees(self.target)); p.setPen(QPen(QColor('#007e95'),2,Qt.DashLine))
            p.drawLine(QPointF(0,-8*s),QPointF(0,-21*s))

    def mousePressEvent(self, event):
        if event.button()!=Qt.LeftButton or not self.interactive: return
        centre,s=self.geometry_values(); v=event.localPos()-centre
        if math.hypot(v.x(),v.y())<2.8*s: return
        transform=QTransform().rotate(math.degrees(self.angle)); local=transform.map(v)
        path=self.handle_path(s); stroke=QPainterPathStroker(); stroke.setWidth(10)
        if not path.united(stroke.createStroke(path)).contains(local): return
        self.initial_angle=self.angle; self.initial_cursor=cursor_angle(v.x(),v.y())
        self.target=self.angle; self.dragging=True; self.show_target=True
        self.grabMouse(); self.drag_started.emit(self.target); self.update()

    def mouseMoveEvent(self, event):
        if not self.dragging: return
        centre,s=self.geometry_values(); v=event.localPos()-centre
        if math.hypot(v.x(),v.y())<2*s: return
        self.target=drag_angle(self.initial_angle,self.initial_cursor,cursor_angle(v.x(),v.y()))
        self.drag_changed.emit(self.target); self.update()

    def mouseReleaseEvent(self, event):
        if not self.dragging or event.button()!=Qt.LeftButton: return
        self.mouseMoveEvent(event); target=self.target; self.cancel_drag(); self.drag_finished.emit(target)

    def cancel_drag(self):
        self.dragging=False; self.show_target=False
        if QWidget.mouseGrabber() is self: self.releaseMouse()
        self.update()


class SelectorWindow(QWidget):
    def __init__(self, connection, package=None, font_options=None):
        super().__init__()
        if package is None:
            import rospkg
            package = rospkg.RosPack().get_path('xuanniu_gazebo')
        package = Path(package)
        self.chinese,self.font_family = configure_font(QApplication.instance(),package,**(font_options or {}))
        self.connection=connection; self.sample=None; self.local_drag=False; self.drag_target=0
        self.pending_until=0; self.sent_at=0; self.expected=0; self.closed=False
        self.last_drag_send=0.0
        self.setWindowTitle('Selector Switch Control'); self.resize(580,690)
        self.setStyleSheet('QWidget { background:#edf2f5; color:#243746; } QPushButton { background:white; border:1px solid #ccd7df; border-radius:7px; padding:13px 18px; } QPushButton:disabled { color:#a5afb8; background:#e5eaee; } QPushButton:hover { border-color:#008398; }')
        outer=QVBoxLayout(self); outer.setContentsMargins(25,20,25,20); outer.setSpacing(8)
        self.heading=QLabel('Selector Switch Control'); self.heading.setStyleSheet('font-size:25px; font-weight:600;')
        outer.addWidget(self.heading); outer.addWidget(QLabel(self.tr('subtitle')))
        self.state_label=QLabel(self.tr('waiting')); self.state_label.setStyleSheet('font-size:20px; color:#00788b;')
        outer.addWidget(self.state_label)
        self.dial=Dial(package,self.tr); outer.addWidget(self.dial,1)
        legend=QLabel(self.tr('legend')); legend.setAlignment(Qt.AlignCenter); legend.setStyleSheet('font-size:11px; color:#687b89;'); outer.addWidget(legend)
        self.angle_label=QLabel(); self.angle_label.setAlignment(Qt.AlignCenter); outer.addWidget(self.angle_label)
        buttons=QHBoxLayout(); outer.addLayout(buttons)
        self.left_button=QPushButton(self.tr('left')); self.stop_button=QPushButton(self.tr('stop')); self.right_button=QPushButton(self.tr('right'))
        for b in (self.left_button,self.stop_button,self.right_button): buttons.addWidget(b)
        self.left_button.clicked.connect(lambda:self.step(True)); self.right_button.clicked.connect(lambda:self.step(False)); self.stop_button.clicked.connect(lambda:self.select(0))
        self.connection_label=QLabel(self.tr('waiting')); outer.addWidget(self.connection_label)
        self.note=QLabel(self.tr('hint')); self.note.setWordWrap(True); self.note.setStyleSheet('color:#657b89;'); outer.addWidget(self.note)
        self.dial.drag_started.connect(self.begin_drag); self.dial.drag_changed.connect(self.change_drag); self.dial.drag_finished.connect(self.finish_drag)
        self.timer=QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(33)
        self.tick()

    def tr(self,key): return TEXT.get(key,(key,key))[0 if self.chinese else 1]

    def ready(self):
        msg,received=self.connection.feedback()
        return msg is not None and time.monotonic()-received<=.5 and self.connection.command_ready() and msg.error!='NONFINITE_PHYSICS'

    def select(self,angle):
        if not self.ready(): return
        self.local_drag=False; self.dial.cancel_drag()
        self.expected=snap(angle); self.connection.send(self.expected); self.sent_at=time.monotonic(); self.pending_until=self.sent_at+.5
        self.tick()

    def step(self,left):
        if self.sample and self.sample.mode in ('MANUAL','STOP','AUTO') and not self.sample.moving:
            self.select(adjacent(self.sample.mode,left))

    def begin_drag(self,angle):
        self.local_drag=True; self.drag_target=angle; self.connection.send(angle,True)
        self.last_drag_send=time.monotonic()

    def change_drag(self,angle): self.drag_target=angle

    def finish_drag(self,angle): self.select(angle)

    def abandon_drag(self):
        if not self.local_drag: return
        self.local_drag=False; self.dial.cancel_drag()
        if self.sample and self.ready(): self.connection.send(snap(self.sample.angle_rad))

    def tick(self):
        msg,received=self.connection.feedback(); now=time.monotonic()
        fresh=msg is not None and now-received<=.5
        if not fresh: self.abandon_drag()
        self.sample=msg
        if self.pending_until and received>self.sent_at and msg and abs(msg.target_angle_rad-self.expected)<1e-6:
            self.pending_until=0
        pending=now<self.pending_until
        enabled=fresh and self.connection.command_ready() and msg.error!='NONFINITE_PHYSICS'
        if self.local_drag and enabled and now-self.last_drag_send>=.03:
            self.connection.send(self.drag_target,True); self.last_drag_send=now
        available=enabled and not pending and not self.local_drag and not msg.moving
        self.left_button.setEnabled(bool(available and msg.mode!='MANUAL'))
        self.right_button.setEnabled(bool(available and msg.mode!='AUTO'))
        self.stop_button.setEnabled(bool(enabled))
        self.dial.interactive=bool(available)
        if msg and math.isfinite(msg.angle_rad): self.dial.angle=msg.angle_rad
        self.dial.target=self.drag_target; self.dial.show_target=self.local_drag; self.dial.update()
        self.state_label.setText(self.tr(msg.mode) if fresh else self.tr('waiting'))
        angle=math.degrees(msg.angle_rad) if msg else 0; target=math.degrees(msg.target_angle_rad) if msg else 0
        self.angle_label.setText('%s: %+.1f°     %s: %+.1f°'%(self.tr('actual'),angle,self.tr('target'),target))
        self.connection_label.setText(self.tr('connected') if fresh else self.tr('stale' if msg else 'waiting'))
        self.note.setText(self.tr(msg.error) if fresh and msg.error else self.tr('dragging' if self.local_drag else 'hint'))

    def event(self,event):
        if event.type()==QEvent.WindowDeactivate and hasattr(self,'local_drag'): self.abandon_drag()
        return super().event(event)

    def closeEvent(self,event):
        if not self.closed:
            self.abandon_drag(); self.timer.stop(); self.connection.close(); self.closed=True
        event.accept()


