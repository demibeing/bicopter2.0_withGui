#latest best code with optimized settings

import sys
import serial
import serial.tools.list_ports
from collections import deque
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# Configuration
SERIAL_PORT = "COM7" 
BAUD_RATE = 115200
GRAPH_LEN = 300

# =============================================================================
# Custom Stylesheet
# =============================================================================
DARK_STYLESHEET = """
QMainWindow { background-color: #121212; }
QWidget { color: #e0e0e0; font-family: 'Segoe UI', Arial; font-size: 11px; }
QGroupBox { 
    border: 1px solid #333; border-radius: 6px; margin-top: 15px; 
    font-weight: bold; color: #00bcd4; background-color: #1a1a1a;
}
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
QLineEdit { background-color: #222; color: #00ff00; border: 1px solid #444; padding: 2px; }
QProgressBar {
    border: 1px solid #333; border-radius: 3px; background-color: #000; text-align: center; height: 18px;
}
QProgressBar::chunk { background-color: #007acc; }
"""

# =============================================================================
# Flight Instruments
# =============================================================================
class ArtificialHorizon(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(180, 180)
        self.roll = 0; self.pitch = 0
    def set_attitude(self, r, p): self.roll = r; self.pitch = p; self.update()
    def paintEvent(self, event):
        p = QtGui.QPainter(self); p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height(); center = QtCore.QPoint(w//2, h//2); rad = min(w,h)//2-5
        path = QtGui.QPainterPath(); path.addEllipse(center, rad, rad); p.setClipPath(path)
        p.translate(center); p.rotate(-self.roll); p.translate(0, self.pitch*2)
        p.fillRect(-w, -h*2, w*2, h*2, QtGui.QColor(30, 100, 180)) # Sky
        p.fillRect(-w, 0, w*2, h*2, QtGui.QColor(70, 40, 20))      # Ground
        p.resetTransform(); p.setClipping(False)
        p.setPen(QtGui.QPen(QtGui.QColor(150,150,150), 2)); p.drawEllipse(center, rad, rad)
        p.setPen(QtGui.QPen(Qt.yellow, 3)); p.drawLine(w//2-30, h//2, w//2-10, h//2); p.drawLine(w//2+10, h//2, w//2+30, h//2)

class CompassWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(140, 140); self.yaw = 0
    def set_yaw(self, y): self.yaw = y; self.update()
    def paintEvent(self, event):
        p = QtGui.QPainter(self); p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height(); center = QtCore.QPoint(w//2, h//2); rad = min(w,h)//2-10
        p.setBrush(QtGui.QColor(10, 10, 10)); p.drawEllipse(center, rad, rad)
        p.translate(center)
        for i in range(0, 360, 45):
            p.save(); p.rotate(i - self.yaw)
            p.setPen(Qt.white if i%90!=0 else Qt.red); p.drawLine(0, -rad+2, 0, -rad+10)
            p.restore()
        p.setPen(Qt.white); p.drawText(-20, 10, 40, 20, Qt.AlignCenter, f"{int(self.yaw)}°")

# =============================================================================
# Serial Thread
# =============================================================================
class SerialThread(QtCore.QThread):
    data_sig = QtCore.pyqtSignal(list)
    def __init__(self):
        super().__init__(); self.running = True; self.ser = None
    def run(self):
        try: self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except: pass
        while self.running:
            if self.ser and self.ser.is_open and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    parts = line.split(",")
                    if parts[0] == "ATT": self.data_sig.emit(["ATT", float(parts[1]), float(parts[2]), float(parts[3])])
                    elif parts[0] == "RC": self.data_sig.emit(["RC", [int(float(x)) for x in parts[1:5]]])
                    elif parts[0] == "SERVO": self.data_sig.emit(["SERVO", int(float(parts[1])), int(float(parts[2]))])
                except: pass
    def stop(self):
        self.running = False
        if self.ser: self.ser.close()

# =============================================================================
# Main GUI
# =============================================================================
class BicopterGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bicopter Ground Control System")
        self.resize(1400, 900); self.setStyleSheet(DARK_STYLESHEET)
        
        # State
        self.roll = 0; self.pitch = 0; self.yaw = 0
        self.target_roll = 0; self.target_pitch = 0
        self.servo_l = 1500; self.servo_r = 1500

        # UI LAYOUT
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        # Header
        header = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("FLASH PID SETTINGS")
        self.btn_save.setStyleSheet("background-color: #800; padding: 8px; color: white; border-radius: 3px;")
        header.addWidget(self.btn_save); header.addStretch()
        main_layout.addLayout(header)

        # Top Row: Tuning and RC
        top_row = QtWidgets.QHBoxLayout()
        
        pid_grp = QtWidgets.QGroupBox("PID Config")
        pid_grid = QtWidgets.QGridLayout(pid_grp)
        for i, h in enumerate(["P","I","D"]): pid_grid.addWidget(QtWidgets.QLabel(h), 0, i+1, Qt.AlignCenter)
        for r, name in enumerate(["Roll", "Pitch", "Yaw"]):
            pid_grid.addWidget(QtWidgets.QLabel(name), r+1, 0)
            for c in range(3): pid_grid.addWidget(QtWidgets.QLineEdit("0.0"), r+1, c+1)
        top_row.addWidget(pid_grp, 1)

        rc_grp = QtWidgets.QGroupBox("Radio (RC) Feedback")
        rc_form = QtWidgets.QFormLayout(rc_grp)
        self.rc_bars = {}
        for n in ["ROLL", "PITCH", "THR", "YAW"]:
            b = QtWidgets.QProgressBar(); b.setRange(1000, 2000); b.setValue(1500)
            rc_form.addRow(n, b); self.rc_bars[n] = b
        top_row.addWidget(rc_grp, 1)

        servo_grp = QtWidgets.QGroupBox("Servo Feedback")
        servo_v = QtWidgets.QVBoxLayout(servo_grp)
        self.s1_lbl = QtWidgets.QLabel("L-Servo: 1500"); self.s1_bar = QtWidgets.QProgressBar()
        self.s2_lbl = QtWidgets.QLabel("R-Servo: 1500"); self.s2_bar = QtWidgets.QProgressBar()
        for b in [self.s1_bar, self.s2_bar]: b.setRange(1000, 2000); b.setValue(1500)
        servo_v.addWidget(self.s1_lbl); servo_v.addWidget(self.s1_bar)
        servo_v.addWidget(self.s2_lbl); servo_v.addWidget(self.s2_bar)
        top_row.addWidget(servo_grp, 1)
        
        main_layout.addLayout(top_row)

        # Middle Row: Dual Graphs
        graph_row = QtWidgets.QHBoxLayout()
        self.roll_plot = pg.PlotWidget(title="Roll: Target vs Actual")
        self.roll_plot.addLegend(); self.roll_plot.setBackground('#0a0a0a')
        self.curve_roll_t = self.roll_plot.plot(pen=pg.mkPen('r', style=Qt.DashLine), name="Target")
        self.curve_roll_a = self.roll_plot.plot(pen=pg.mkPen('c', width=2), name="Actual")
        
        self.pitch_plot = pg.PlotWidget(title="Pitch: Target vs Actual")
        self.pitch_plot.addLegend(); self.pitch_plot.setBackground('#0a0a0a')
        self.curve_pitch_t = self.pitch_plot.plot(pen=pg.mkPen('r', style=Qt.DashLine), name="Target")
        self.curve_pitch_a = self.pitch_plot.plot(pen=pg.mkPen('y', width=2), name="Actual")
        
        graph_row.addWidget(self.roll_plot); graph_row.addWidget(self.pitch_plot)
        main_layout.addLayout(graph_row, 2)

        # Bottom Row: 3D Visualization and Instruments
        bottom_row = QtWidgets.QHBoxLayout()
        
        self.view3d = gl.GLViewWidget()
        self.view3d.opts['distance'] = 25
        self.view3d.addItem(gl.GLGridItem(size=QtGui.QVector3D(40,40,0)))
        
        # Diagnostic Model (Larger sizes to ensure they appear)
        self.body = gl.GLBoxItem(size=QtGui.QVector3D(2, 4, 1), color=(100, 100, 100, 255))
        self.arms = gl.GLBoxItem(size=QtGui.QVector3D(10, 0.5, 0.5), color=(50, 50, 50, 255))
        self.mot_l = gl.GLBoxItem(size=QtGui.QVector3D(1, 1, 0.5), color=(0, 200, 255, 255))
        self.mot_r = gl.GLBoxItem(size=QtGui.QVector3D(1, 1, 0.5), color=(255, 120, 0, 255))
        
        # Explicitly add items
        for item in [self.body, self.arms, self.mot_l, self.mot_r]:
            self.view3d.addItem(item)

        bottom_row.addWidget(self.view3d, 2)
        
        inst_v = QtWidgets.QVBoxLayout()
        self.horizon = ArtificialHorizon(); self.compass = CompassWidget()
        inst_v.addWidget(self.horizon); inst_v.addWidget(self.compass)
        bottom_row.addLayout(inst_v, 1)
        
        main_layout.addLayout(bottom_row, 3)

        # Buffers
        self.hist_r_a = deque([0]*GRAPH_LEN, maxlen=GRAPH_LEN); self.hist_r_t = deque([0]*GRAPH_LEN, maxlen=GRAPH_LEN)
        self.hist_p_a = deque([0]*GRAPH_LEN, maxlen=GRAPH_LEN); self.hist_p_t = deque([0]*GRAPH_LEN, maxlen=GRAPH_LEN)

        self.update_3d() # Force first render
        self.thread = SerialThread()
        self.thread.data_sig.connect(self.process_packet)
        self.thread.start()

    def process_packet(self, pkt):
        typ = pkt[0]
        if typ == "ATT":
            self.roll, self.pitch, self.yaw = pkt[1:]
            self.horizon.set_attitude(self.roll, self.pitch); self.compass.set_yaw(self.yaw)
            # Update Roll Graph
            self.hist_r_a.append(self.roll); self.hist_r_t.append(self.target_roll)
            self.curve_roll_a.setData(list(self.hist_r_a)); self.curve_roll_t.setData(list(self.hist_r_t))
            # Update Pitch Graph
            self.hist_p_a.append(self.pitch); self.hist_p_t.append(self.target_pitch)
            self.curve_pitch_a.setData(list(self.hist_p_a)); self.curve_pitch_t.setData(list(self.hist_p_t))
            self.update_3d()
        elif typ == "RC":
            for i, n in enumerate(["ROLL","PITCH","THR","YAW"]): self.rc_bars[n].setValue(pkt[1][i])
            self.target_roll = (pkt[1][0] - 1500) / 500.0 * 30.0
            self.target_pitch = (pkt[1][1] - 1500) / 500.0 * 30.0
        elif typ == "SERVO":
            # Map values and update UI
            self.servo_l, self.servo_r = pkt[1], pkt[2]
            self.s1_bar.setValue(self.servo_l); self.s1_lbl.setText(f"L-Servo: {self.servo_l}")
            self.s2_bar.setValue(self.servo_r); self.s2_lbl.setText(f"R-Servo: {self.servo_r}")

    def update_3d(self):
        # Base Transformation Matrix
        m = QtGui.QMatrix4x4()
        m.rotate(self.yaw, 0, 0, 1)
        m.rotate(self.pitch, 0, 1, 0)
        m.rotate(self.roll, 1, 0, 0)

        # Body: Centered
        mb = QtGui.QMatrix4x4(m); mb.translate(-1, -2, -0.5); self.body.setTransform(mb)
        
        # Arms: Centered
        ma = QtGui.QMatrix4x4(m); ma.translate(-5, -0.25, -0.25); self.arms.setTransform(ma)

        # Motors: Tilt based on servo PWM
        ml = QtGui.QMatrix4x4(m); ml.translate(-5, 0, 0.25)
        ml.rotate((self.servo_l - 1500)/10.0, 0, 1, 0); ml.translate(-0.5, -0.5, 0)
        self.mot_l.setTransform(ml)

        mr = QtGui.QMatrix4x4(m); mr.translate(5, 0, 0.25)
        mr.rotate((self.servo_r - 1500)/10.0, 0, 1, 0); mr.translate(-0.5, -0.5, 0)
        self.mot_r.setTransform(mr)

    def closeEvent(self, event):
        self.thread.stop(); super().closeEvent(event)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = BicopterGUI()
    window.show()
    sys.exit(app.exec_())