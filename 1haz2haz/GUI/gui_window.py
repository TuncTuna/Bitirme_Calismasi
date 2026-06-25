# =============================================================================
# GUI_WINDOW.PY  —  RPRR Robot Simülasyonu  v5.0
# =============================================================================

import time
import numpy as np
from PyQt6.QtWidgets import (
    QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout,
    QWidget, QLineEdit, QLabel, QMessageBox, QGroupBox, QTabWidget,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QSpinBox, QDoubleSpinBox, QCheckBox, QFrame,
    QSlider, QRadioButton, QButtonGroup, QScrollArea, QSplitter,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSlot
from PyQt6.QtGui import QFont, QColor, QImage, QPixmap
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from config_init import L2, L3, L_GRIP, H1, H2, OBSTACLES, HOME_Position, WAIT_Position
from config_init import LIMITS
from config_init import cam_to_robot
from VISION.vision_config import load_config, save_all
from VISION.vision_worker import VisionWorker
from VISION.pick_place   import PickPlaceWorker


# =============================================================================
# TEMA
# =============================================================================
class THEME:
    BG_DEEP    = "#f0f0eb"   # Kırık beyaz zemin
    BG_PANEL   = "#e8e8e2"   # Panel arka planı (hafif gri-bej)
    BG_CARD    = "#ffffff"   # Kartlar saf beyaz
    BG_INPUT   = "#f5f5f0"   # Input alanları
    BG_ROW_ALT = "#f8f8f5"   # Tablo alternatif satır
    ACCENT     = "#2563eb"   # Mavi aksant
    ACCENT2    = "#059669"   # Yeşil aksant
    DANGER     = "#dc2626"   # Kırmızı
    WARNING    = "#d97706"   # Amber
    DARK       = "#475569"   # Robot gövde rengi (orta koyu gri-mavi)
    TEXT       = "#1f2937"   # Ana metin
    TEXT_DIM   = "#6b7280"   # Soluk metin
    BORDER     = "#d1d5db"   # Kenarlık
    BORDER_ACT = "#2563eb"   # Aktif kenarlık
    SUBTLE     = "#94a3b8"   # İkincil butonlar
    FONT_PT    = 9
    BTN_H      = 26
    INPUT_H    = 22
    RADIUS     = "5px"


# =============================================================================
# MAINWINDOW
# =============================================================================
class MainWindow(QMainWindow):

    def __init__(self, calculate_callback):
        super().__init__()
        self.calculate_callback = calculate_callback
        self.setWindowTitle("AGRION_v.1  —  PRRR Robot Simulation")
        self.resize(1920, 1080)

        self.current_x   = 575.0
        self.current_y   = 0.0
        self.current_z   = 0.0
        self.current_phi = 0.0
        self.gripper_cmd = 0.0
        self.last_joint_traj = None
        self.last_t_vals     = None

        self._teach_points  = []
        self._tp_running    = False
        self._tp_step_index = 0
        self.tp_send_mode: int = 0

        self._tp_timer = QTimer()
        self._tp_timer.setSingleShot(True)
        self._tp_timer.timeout.connect(self._tp_execute_next)

        self._vision_worker       = None
        self._pick_worker         = None
        self.ack_callback         = None   # set by main.py
        self.uart_send            = None   # set by main.py
        self._locked_targets      = []
        self._vision_cfg          = load_config()
        self._vis_auto_start_pending = False

        # Animation
        self._anim_mode  = False
        self._anim_frame = 0
        self._anim_traj  = None   # (x,y,z,d1,Q2,Q3,Q4) full lists
        self._anim_timer = QTimer()
        self._anim_timer.setInterval(40)   # ~25 fps
        self._anim_timer.timeout.connect(self._anim_tick)

        self._apply_global_style()
        self._build_ui()

    def _apply_global_style(self):
        T = THEME
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background: {T.BG_DEEP};
                color: {T.TEXT};
                font-family: 'Segoe UI', 'SF Pro Display', sans-serif;
                font-size: {T.FONT_PT}pt;
                font-weight: 500;
            }}
            QGroupBox {{
                border: 1px solid {T.BORDER};
                border-radius: {T.RADIUS};
                margin-top: 10px;
                padding-top: 6px;
                background: {T.BG_CARD};
                font-size: {T.FONT_PT}pt;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: {T.ACCENT};
                font-weight: 700;
            }}
            QLineEdit, QSpinBox, QDoubleSpinBox {{
                background: {T.BG_INPUT};
                color: {T.TEXT};
                border: 1px solid {T.BORDER};
                border-radius: 4px;
                padding: 2px 6px;
                font-size: {T.FONT_PT}pt;
                font-weight: 600;
            }}
            QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
                border: 1px solid {T.BORDER_ACT};
            }}
            QPushButton {{
                background: {T.BG_CARD};
                color: {T.TEXT};
                border: 1px solid {T.BORDER};
                border-radius: 4px;
                padding: 4px 10px;
                font-size: {T.FONT_PT}pt;
                font-weight: 600;
            }}
            QPushButton:hover  {{ background: #eef2ff; border-color: {T.ACCENT}; }}
            QPushButton:pressed {{ background: #dbeafe; }}
            QTabWidget::pane {{
                border: 1px solid {T.BORDER};
                background: {T.BG_CARD};
                border-radius: {T.RADIUS};
            }}
            QTabBar::tab {{
                background: {T.BG_PANEL};
                color: {T.TEXT_DIM};
                padding: 6px 16px;
                border: 1px solid {T.BORDER};
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                margin-right: 2px;
                font-weight: 600;
            }}
            QTabBar::tab:selected {{
                background: {T.BG_CARD};
                color: {T.ACCENT};
                border-bottom: 2px solid {T.ACCENT};
                font-weight: 700;
            }}
            QLabel {{ color: {T.TEXT}; font-size: {T.FONT_PT}pt; font-weight: 600; }}
            QCheckBox {{ color: {T.TEXT}; font-weight: 600; }}
            QRadioButton {{ color: {T.TEXT}; font-weight: 600; }}
            QSlider::groove:horizontal {{
                background: {T.BORDER};
                height: 4px;
                border-radius: 2px;
            }}
            QSlider::handle:horizontal {{
                background: {T.ACCENT};
                width: 12px;
                height: 12px;
                margin: -4px 0;
                border-radius: 6px;
            }}
            QSlider::sub-page:horizontal {{
                background: {T.ACCENT};
                border-radius: 2px;
            }}
            QTableWidget {{
                background: {T.BG_CARD};
                color: {T.TEXT};
                gridline-color: {T.BORDER};
                font-size: {T.FONT_PT}pt;
                border: 1px solid {T.BORDER};
                font-weight: 500;
            }}
            QTableWidget::item:selected {{ background: #dbeafe; color: {T.TEXT}; }}
            QTableWidget::item:alternate {{ background: {T.BG_ROW_ALT}; }}
            QHeaderView::section {{
                background: {T.BG_PANEL};
                color: {T.ACCENT};
                font-size: {T.FONT_PT}pt;
                font-weight: 700;
                padding: 4px;
                border: none;
                border-right: 1px solid {T.BORDER};
                border-bottom: 1px solid {T.BORDER};
            }}
            QScrollBar:vertical {{
                background: {T.BG_DEEP};
                width: 8px;
                border-radius: 4px;
            }}
            QScrollBar::handle:vertical {{
                background: {T.BORDER};
                border-radius: 4px;
            }}
            QScrollBar:horizontal {{
                background: {T.BG_DEEP};
                height: 8px;
                border-radius: 4px;
            }}
            QScrollBar::handle:horizontal {{
                background: {T.BORDER};
                border-radius: 4px;
            }}
        """)

    def _build_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        main_layout.addWidget(self._build_left_panel(), stretch=0)
        main_layout.addWidget(self._build_right_panel(), stretch=1)
        root = QWidget()
        root.setLayout(main_layout)
        root.setStyleSheet(f"background:{THEME.BG_DEEP};")
        self.setCentralWidget(root)

    def _build_left_panel(self) -> QWidget:
        T = THEME
        self.x_input    = self._make_input("575")
        self.y_input    = self._make_input("0")
        self.z_input    = self._make_input("0")
        self.phi_input  = self._make_input("0")
        self.time_input = self._make_input("2.0")

        coord_box = QGroupBox("Target Coordinates")
        coord_box.setStyleSheet(f"QGroupBox {{ background:{T.BG_PANEL}; }}")
        cv = QVBoxLayout(); cv.setSpacing(4)
        for lbl, w in [("X (mm)",self.x_input),("Y (mm)",self.y_input),
                       ("Z (mm)",self.z_input),("Phi (°)",self.phi_input),
                       ("Süre (s)",self.time_input)]:
            cv.addWidget(self._dim_label(lbl)); cv.addWidget(w)
        coord_box.setLayout(cv)

        self.btn_home  = self._btn("⌂  HOME",            T.ACCENT,  lambda: self._call("HOME"))
        self.btn_wait  = self._btn("◎  Waiting Position", T.SUBTLE,  self._on_wait)
        self.btn_calc  = self._btn("⟳  Calculate",        "#2a4a2a", lambda: self._call("CALCULATE"))
        self.btn_send  = self._btn("▶  Send (UART)",      "#2a3a5a", lambda: self._call("SEND"))
        self.btn_open  = self._btn("○  Open Gripper",     "#3a3a1a", lambda: self._set_gripper(1.0))
        self.btn_close = self._btn("●  Close Gripper",    "#3a1a1a", lambda: self._set_gripper(2.0))
        self.btn_anim  = self._btn("▷  Animate",          "#1a3a4a", self._toggle_anim)

        btn_box = QGroupBox("Control")
        btn_box.setStyleSheet(f"QGroupBox {{ background:{T.BG_PANEL}; }}")
        bv = QVBoxLayout(); bv.setSpacing(4)
        for b in (self.btn_home,self.btn_wait,self.btn_calc,
                  self.btn_send,self.btn_open,self.btn_close,self.btn_anim):
            bv.addWidget(b)
        btn_box.setLayout(bv)

        self.d1_label   = QLabel("d1 : —")
        self.Q2_label   = QLabel("Q2 : —")
        self.Q3_label   = QLabel("Q3 : —")
        self.Q4_label   = QLabel("Q4 : —")
        self.grip_label = QLabel("Gripper : Idle")

        joint_box = QGroupBox("Joint Values")
        joint_box.setStyleSheet(f"QGroupBox {{ background:{T.BG_PANEL}; }}")
        jv = QVBoxLayout(); jv.setSpacing(3)
        for lbl in (self.d1_label,self.Q2_label,self.Q3_label,
                    self.Q4_label,self.grip_label):
            lbl.setStyleSheet(f"color:{T.TEXT_DIM}; font-family:monospace; font-size:8pt; font-weight:600;")
            jv.addWidget(lbl)
        joint_box.setLayout(jv)

        panel = QVBoxLayout(); panel.setSpacing(8)
        panel.addWidget(coord_box)
        panel.addWidget(btn_box)
        panel.addWidget(joint_box)
        panel.addStretch()

        w = QWidget()
        w.setLayout(panel)
        w.setFixedWidth(230)
        w.setStyleSheet(f"background:{T.BG_PANEL}; border-right:1px solid {T.BORDER};")
        return w

    def _check_limits(self, x, y, z, phi) -> bool:
        try:
            limits = LIMITS
            if not (limits['x'][0] <= x <= limits['x'][1]):
                self.show_error(f"X out of bounds! ({limits['x'][0]} to {limits['x'][1]} mm)")
                return False
            if not (limits['y'][0] <= y <= limits['y'][1]):
                self.show_error(f"Y out of bounds! ({limits['y'][0]} to {limits['y'][1]} mm)")
                return False
            if not (limits['z'][0] <= z <= limits['z'][1]):
                self.show_error(f"Z out of bounds! ({limits['z'][0]} to {limits['z'][1]} mm)")
                return False
            if not (limits['phi'][0] <= phi <= limits['phi'][1]):
                self.show_error("Phi out of bounds!")
                return False
            return True
        except Exception as e:
            self.show_error(f"Limit check error: {e}")
            return False

    def _build_right_panel(self) -> QWidget:
        self.tabs = QTabWidget()
        self.fig_3d    = Figure(facecolor=THEME.BG_DEEP)
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.tabs.addTab(self._wrap(self.canvas_3d), "  End-Effector Trajectory  ")
        self.fig_time    = Figure(facecolor=THEME.BG_DEEP)
        self.canvas_time = FigureCanvas(self.fig_time)
        self.tabs.addTab(self._wrap(self.canvas_time), "  Time Graphs  ")
        self.tabs.addTab(self._build_teachpad_tab(), "  🎯 TeachPad  ")
        self.tabs.addTab(self._build_vision_tab(),   "  🍅 Autonomus Harvesting  ")
        w = QWidget()
        v = QVBoxLayout(); v.setContentsMargins(0,0,0,0); v.addWidget(self.tabs)
        w.setLayout(v)
        return w


    # ==========================================================================
    # TEACHPAD
    # ==========================================================================
    def _build_teachpad_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(12,12,12,12)
        layout.setSpacing(10)
        layout.addWidget(self._tp_header())
        layout.addWidget(self._tp_input_panel())
        layout.addWidget(self._tp_step_panel())
        layout.addWidget(self._tp_table_panel())
        layout.addWidget(self._tp_run_panel())
        layout.addStretch()
        tab.setLayout(layout)
        return tab

    def _tp_header(self) -> QLabel:
        lbl = QLabel("🎯  TeachPad  —  Waypoint Programming")
        lbl.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        lbl.setStyleSheet(
            f"color:{THEME.ACCENT}; background:{THEME.BG_PANEL};"
            f"padding:8px 12px; border-radius:5px;"
            f"border-left:3px solid {THEME.ACCENT};"
        )
        return lbl

    def _tp_input_panel(self) -> QGroupBox:
        box = QGroupBox("New Waypoint")
        row = QHBoxLayout(); row.setSpacing(8)
        self.tp_x   = self._make_input(self.x_input.text(),    "X mm")
        self.tp_y   = self._make_input(self.y_input.text(),    "Y mm")
        self.tp_z   = self._make_input(self.z_input.text(),    "Z mm")
        self.tp_phi = self._make_input(self.phi_input.text(),  "Phi °")
        self.tp_t   = self._make_input(self.time_input.text(), "Time s")
        self.tp_grip= self._make_input("0", "0/1/2")
        self.tp_lbl = self._make_input("",  "Label")
        for caption, widget in [
            ("X",self.tp_x),("Y",self.tp_y),("Z",self.tp_z),
            ("Phi °",self.tp_phi),("Time",self.tp_t),
            ("Gripper",self.tp_grip),("Label",self.tp_lbl),
        ]:
            widget.setMaximumWidth(82)
            col = QVBoxLayout(); col.setSpacing(2)
            col.addWidget(self._dim_label(caption))
            col.addWidget(widget)
            row.addLayout(col)
        side = QVBoxLayout(); side.setSpacing(6)
        side.addWidget(self._btn("◀ Fill from Manual", THEME.ACCENT, self._tp_fill_from_manual))
        side.addWidget(self._btn("＋ Add Waypoint", "#2a5a2a", self._tp_add_point))
        side.addStretch()
        row.addLayout(side)
        box.setLayout(row)
        return box

    def _tp_step_panel(self) -> QGroupBox:
        box = QGroupBox("Step-by-Step Movement")
        row = QHBoxLayout(); row.setSpacing(6)
        self.tp_step_spin = QSpinBox()
        self.tp_step_spin.setRange(1,999)
        self.tp_step_spin.setValue(1)
        self.tp_step_spin.setFixedWidth(55)
        row.addWidget(self._dim_label("Multiplier"))
        row.addWidget(self.tp_step_spin)
        self.tp_step_size_x   = self._make_input("10"); self.tp_step_size_x.setFixedWidth(50)
        self.tp_step_size_y   = self._make_input("10"); self.tp_step_size_y.setFixedWidth(50)
        self.tp_step_size_z   = self._make_input("10"); self.tp_step_size_z.setFixedWidth(50)
        self.tp_step_size_phi = self._make_input("5");  self.tp_step_size_phi.setFixedWidth(50)
        for axis, label, size_w in [
            ("x","X",self.tp_step_size_x),("y","Y",self.tp_step_size_y),
            ("z","Z",self.tp_step_size_z),("phi","Φ",self.tp_step_size_phi),
        ]:
            sep = QFrame(); sep.setFrameShape(QFrame.Shape.VLine)
            sep.setStyleSheet(f"color:{THEME.BORDER};")
            row.addWidget(sep)
            row.addWidget(self._dim_label(f"{label}:"))
            row.addWidget(size_w)
            row.addWidget(self._btn("▲", THEME.ACCENT,
                lambda _=None, a=axis: self._step_move(a,+1), w=36))
            row.addWidget(self._btn("▼", THEME.DANGER,
                lambda _=None, a=axis: self._step_move(a,-1), w=36))
        row.addStretch()
        box.setLayout(row)
        return box

    def _tp_table_panel(self) -> QGroupBox:
        box = QGroupBox("Waypoint List")
        v = QVBoxLayout()
        self.tp_table = QTableWidget(0,8)
        self.tp_table.setHorizontalHeaderLabels(
            ["#","Label","X","Y","Z","Phi °","Time s","Gripper"])
        self.tp_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.tp_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.tp_table.setAlternatingRowColors(True)
        self.tp_table.setMaximumHeight(200)
        self.tp_table.verticalHeader().setVisible(False)
        v.addWidget(self.tp_table)
        bar = QHBoxLayout(); bar.setSpacing(6)
        for label, color, fn in [
            ("🗑 Delete",THEME.DANGER, self._tp_delete_selected),
            ("✖ Clear",  "#5a2020",   self._tp_clear_all),
            ("⬆ Up",     THEME.ACCENT,self._tp_move_up),
            ("⬇ Down",   THEME.ACCENT,self._tp_move_down),
            ("✏ Edit",   "#5a5a20",  self._tp_edit_selected),
        ]:
            bar.addWidget(self._btn(label,color,fn))
        bar.addStretch()
        v.addLayout(bar)
        box.setLayout(v)
        return box

    def _tp_run_panel(self) -> QGroupBox:
        box = QGroupBox("Program Execution")
        row = QHBoxLayout(); row.setSpacing(10)
        self.tp_delay_spin = QDoubleSpinBox()
        self.tp_delay_spin.setRange(0.0,60.0)
        self.tp_delay_spin.setValue(0.5)
        self.tp_delay_spin.setSuffix(" s")
        self.tp_delay_spin.setFixedWidth(85)
        self.tp_loop_check = QCheckBox("Loop")
        self.btn_tp_run       = self._btn("▶  Execute",   "#1a5a2a", self._tp_run_program,  bold=True)
        self.btn_tp_step_exec = self._btn("⏭  Next Step", THEME.ACCENT, self._tp_step_exec, bold=True)
        self.btn_tp_stop      = self._btn("⏹  Stop",      THEME.DANGER, self._tp_stop_program, bold=True)
        self.tp_status_label  = QLabel("Waiting to run...")
        self.tp_status_label.setStyleSheet(
            f"color:{THEME.ACCENT2}; background:{THEME.BG_PANEL};"
            f"padding:4px 12px; border-radius:4px;"
            f"border:1px solid {THEME.ACCENT2}; font-family:monospace; font-weight:700;"
        )
        row.addWidget(self._dim_label("Delay:"))
        row.addWidget(self.tp_delay_spin)
        row.addWidget(self.tp_loop_check)
        row.addSpacing(8)
        row.addWidget(self.btn_tp_run)
        row.addWidget(self.btn_tp_step_exec)
        row.addWidget(self.btn_tp_stop)
        row.addStretch()
        row.addWidget(self.tp_status_label)
        box.setLayout(row)
        return box

    def _tp_fill_from_manual(self):
        self.tp_x.setText(self.x_input.text())
        self.tp_y.setText(self.y_input.text())
        self.tp_z.setText(self.z_input.text())
        self.tp_phi.setText(self.phi_input.text())
        self.tp_t.setText(self.time_input.text())
        self.tp_grip.setText({0.0:"0",1.0:"1",2.0:"2"}.get(self.gripper_cmd,"0"))

    def _tp_add_point(self):
        try:
            point = {
                "x":       float(self.tp_x.text()),
                "y":       float(self.tp_y.text()),
                "z":       float(self.tp_z.text()),
                "phi":     float(self.tp_phi.text()),
                "t":       float(self.tp_t.text()),
                "gripper": float(self.tp_grip.text() or "0"),
                "label":   self.tp_lbl.text().strip() or f"P{len(self._teach_points)+1}",
            }
            if not self._check_limits(point["x"],point["y"],point["z"],point["phi"]):
                return
        except ValueError:
            self.show_error("Unrecognized input!")
            return
        self._teach_points.append(point)
        self._tp_refresh_table()
        self.tp_lbl.clear()

    def _tp_refresh_table(self):
        GRIP_TXT = {0.0:"—",1.0:"Open",2.0:"Close"}
        self.tp_table.setRowCount(0)
        for i, p in enumerate(self._teach_points):
            self.tp_table.insertRow(i)
            vals = [str(i+1),p["label"],
                    f"{p['x']:.1f}",f"{p['y']:.1f}",f"{p['z']:.1f}",
                    f"{p['phi']:.1f}",f"{p['t']:.2f}",
                    GRIP_TXT.get(p["gripper"],str(p["gripper"]))]
            for col, val in enumerate(vals):
                item = QTableWidgetItem(val)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                if col == 0:
                    item.setForeground(QColor(THEME.ACCENT))
                self.tp_table.setItem(i,col,item)

    def _tp_delete_selected(self):
        for r in sorted({i.row() for i in self.tp_table.selectedItems()},reverse=True):
            if 0 <= r < len(self._teach_points):
                self._teach_points.pop(r)
        self._tp_refresh_table()

    def _tp_clear_all(self):
        self._teach_points.clear()
        self._tp_refresh_table()

    def _tp_move_up(self):
        rows = sorted({i.row() for i in self.tp_table.selectedItems()})
        if not rows or rows[0] == 0: return
        for r in rows:
            self._teach_points[r-1],self._teach_points[r] = \
                self._teach_points[r],self._teach_points[r-1]
        self._tp_refresh_table()
        for r in rows: self.tp_table.selectRow(r-1)

    def _tp_move_down(self):
        rows = sorted({i.row() for i in self.tp_table.selectedItems()},reverse=True)
        if not rows or rows[0] >= len(self._teach_points)-1: return
        for r in rows:
            self._teach_points[r],self._teach_points[r+1] = \
                self._teach_points[r+1],self._teach_points[r]
        self._tp_refresh_table()
        for r in rows: self.tp_table.selectRow(r+1)

    def _tp_edit_selected(self):
        rows = list({i.row() for i in self.tp_table.selectedItems()})
        if len(rows) != 1: return
        p = self._teach_points.pop(rows[0])
        self.tp_x.setText(str(p["x"]));   self.tp_y.setText(str(p["y"]))
        self.tp_z.setText(str(p["z"]));   self.tp_phi.setText(str(p["phi"]))
        self.tp_t.setText(str(p["t"]));   self.tp_grip.setText(str(int(p["gripper"])))
        self.tp_lbl.setText(p["label"])
        self._tp_refresh_table()

    def _step_move(self, axis: str, sign: int):
        MAP = {
            "x":   (self.tp_step_size_x,  self.x_input,   self.tp_x),
            "y":   (self.tp_step_size_y,   self.y_input,   self.tp_y),
            "z":   (self.tp_step_size_z,   self.z_input,   self.tp_z),
            "phi": (self.tp_step_size_phi, self.phi_input, self.tp_phi),
        }
        size_w, manual_w, teach_w = MAP[axis]
        try:
            delta = float(size_w.text()) * self.tp_step_spin.value() * sign
            new   = float(manual_w.text()) + delta
        except ValueError:
            return
        manual_w.setText(f"{new:.2f}")
        teach_w.setText(f"{new:.2f}")

    def _tp_run_program(self):
        if not self._teach_points:
            self.show_error("Waypoint list is empty!"); return
        if self._tp_running: return
        self._tp_running    = True
        self._tp_step_index = 0
        self.btn_tp_run.setEnabled(False)
        self._tp_set_status("▶ Running…", THEME.ACCENT2)
        self._tp_execute_next()

    def _tp_stop_program(self):
        self._tp_running = False
        self._tp_timer.stop()
        self.btn_tp_run.setEnabled(True)
        self._tp_set_status("⏹ Stopped", THEME.WARNING)
        self._tp_highlight_row(-1)

    def _tp_step_exec(self):
        if not self._teach_points:
            self.show_error("Waypoint list is empty!"); return
        idx = self._tp_step_index % len(self._teach_points)
        self._tp_send_point(idx)
        self._tp_step_index = (idx+1) % len(self._teach_points)
        self._tp_set_status(f"⏭ Step {idx+1}/{len(self._teach_points)}", THEME.ACCENT)

    def _tp_execute_next(self):
        if not self._tp_running: return
        n = len(self._teach_points)
        if self._tp_step_index >= n:
            if self.tp_loop_check.isChecked():
                self._tp_step_index = 0
            else:
                self._tp_running = False
                self.btn_tp_run.setEnabled(True)
                self._tp_set_status("✅ Completed", THEME.ACCENT2)
                self._tp_highlight_row(-1)
                return
        idx = self._tp_step_index
        self._tp_send_point(idx)
        self._tp_set_status(
            f"▶ {idx+1}/{n}  {self._teach_points[idx]['label']}", THEME.ACCENT2)
        self._tp_highlight_row(idx)
        self._tp_step_index += 1
        self._tp_timer.start(int(self.tp_delay_spin.value() * 1000))

    def _tp_send_point(self, idx: int):
        p = self._teach_points[idx]
        self.x_input.setText(str(p["x"]))
        self.y_input.setText(str(p["y"]))
        self.z_input.setText(str(p["z"]))
        self.phi_input.setText(str(p["phi"]))
        self.time_input.setText(str(p["t"]))
        self._set_gripper(p["gripper"])
        self.calculate_callback(
            "CALCULATE",str(p["x"]),str(p["y"]),str(p["z"]),
            str(p["phi"]),str(p["t"]),self)
        if not self._check_limits(p["x"],p["y"],p["z"],p["phi"]): return
        self.tp_send_mode = 1
        self.calculate_callback(
            "SEND",str(p["x"]),str(p["y"]),str(p["z"]),
            str(p["phi"]),str(p["t"]),self)
        self.tp_send_mode = 0

    def _tp_highlight_row(self, idx: int):
        self.tp_table.clearSelection()
        if 0 <= idx < self.tp_table.rowCount():
            self.tp_table.selectRow(idx)
            self.tp_table.scrollToItem(self.tp_table.item(idx,0))

    def _tp_set_status(self, text: str, color: str = THEME.ACCENT2):
        self.tp_status_label.setText(text)
        self.tp_status_label.setStyleSheet(
            f"color:{color}; background:{THEME.BG_PANEL};"
            f"padding:4px 12px; border-radius:4px;"
            f"border:1px solid {color}; font-family:monospace; font-weight:700;")


    # ==========================================================================
    # MANUEL KONTROL
    # ==========================================================================
    def _on_wait(self):
        p = WAIT_Position
        for w, v in [(self.x_input,str(p["x"])),(self.y_input,str(p["y"])),
                     (self.z_input,str(p["z"])),(self.phi_input,str(p["phi"]))]:
            w.setText(v)
        self._set_gripper(0.0)
        self._call("WAIT")

    def _set_gripper(self, cmd: float, update_label: bool = True):
        self.gripper_cmd = float(cmd)
        if update_label:
            txt = {0.0:"Idle",1.0:"Open",2.0:"Close"}.get(cmd,str(cmd))
            self.grip_label.setText(f"Gripper : {txt}")

    def _call(self, mode: str):
        try:
            x   = float(self.x_input.text())
            y   = float(self.y_input.text())
            z   = float(self.z_input.text())
            phi = float(self.phi_input.text())
        except ValueError:
            self.show_error("Invalid numeric input!"); return
        if not self._check_limits(x,y,z,phi): return
        self.calculate_callback(mode,str(x),str(y),str(z),
                                str(phi),self.time_input.text(),self)


    # ==========================================================================
    # OTONOM HASAT SEKMESİ
    # ==========================================================================
    def _build_vision_tab(self) -> QWidget:
        T   = THEME
        cfg = self._vision_cfg
        tab = QWidget()

        main_split = QSplitter(Qt.Orientation.Horizontal)
        main_split.setStyleSheet(
            f"QSplitter::handle {{ background:{T.BORDER}; width:2px; }}")

        # ── SOL TARAF ────────────────────────────────────────────────────────
        left_w   = QWidget()
        left_lay = QVBoxLayout()
        left_lay.setContentsMargins(8,8,4,8)
        left_lay.setSpacing(8)

        # Durum çubuğu
        status_bar = QWidget()
        status_bar.setStyleSheet(
            f"background:{T.BG_PANEL}; border:1px solid {T.BORDER}; border-radius:5px;")
        sb_lay = QHBoxLayout(); sb_lay.setContentsMargins(8,4,8,4)
        self.vis_cam_status = QLabel("⚪  CAMERA OFF")
        self.vis_cam_status.setStyleSheet(
            f"color:{T.TEXT_DIM}; font-family:monospace; font-weight:700;")
        self.vis_robot_status = QLabel("🤖  ROBOT: —")
        self.vis_robot_status.setStyleSheet(
            f"color:{T.TEXT_DIM}; font-family:monospace; font-weight:600;")
        self.vis_scan_info = QLabel("")
        self.vis_scan_info.setStyleSheet(
            f"color:{T.TEXT_DIM}; font-family:monospace; font-size:8pt; font-weight:600;")
        sb_lay.addWidget(self.vis_cam_status)
        sb_lay.addSpacing(20)
        sb_lay.addWidget(self.vis_robot_status)
        sb_lay.addStretch()
        sb_lay.addWidget(self.vis_scan_info)
        status_bar.setLayout(sb_lay)
        left_lay.addWidget(status_bar)

        # Kamera önizleme
        cam_box = QGroupBox("Camera Preview")
        cam_lay = QVBoxLayout(); cam_lay.setContentsMargins(4,4,4,4)
        self.vis_cam_label = QLabel()
        self.vis_cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.vis_cam_label.setMinimumSize(400,380)
        self.vis_cam_label.setStyleSheet(
            f"background:{T.BG_DEEP}; border:1px solid {T.BORDER}; border-radius:4px;")
        self.vis_cam_label.setText("Camera not initialized")
        cam_lay.addWidget(self.vis_cam_label)
        cam_box.setLayout(cam_lay)
        left_lay.addWidget(cam_box, stretch=1)

        # Kilitli hedef listesi + Sabit konumlar YAN YANA
        mid_row = QHBoxLayout(); mid_row.setSpacing(8)

        # Sol: Kilitli hedef listesi
        tgt_box = QGroupBox("Locked Target List (Robot Frame, cm)")
        tgt_lay = QVBoxLayout(); tgt_lay.setContentsMargins(4,4,4,4)
        self.vis_target_table = QTableWidget(0,6)
        self.vis_target_table.setHorizontalHeaderLabels(
            ["#","ID","Class","X (cm)","Y (cm)","Z (cm)"])
        self.vis_target_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.vis_target_table.setAlternatingRowColors(True)
        self.vis_target_table.setMinimumHeight(120)
        self.vis_target_table.verticalHeader().setVisible(False)
        self.vis_target_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers)
        self.vis_target_table.verticalHeader().setDefaultSectionSize(26)
        self.vis_target_table.horizontalHeader().setMinimumSectionSize(45)
        tgt_lay.addWidget(self.vis_target_table, stretch=1)
        self.btn_vis_unlock = self._btn(
            "🔓  Unlock (Scan Again)", T.WARNING, self._vis_unlock)
        self.btn_vis_unlock.setEnabled(False)
        tgt_lay.addWidget(self.btn_vis_unlock)
        tgt_box.setLayout(tgt_lay)
        left_lay.addWidget(tgt_box)

        # Sağ: Sabit konumlar
        pos_box = QGroupBox("Fixed Positions")
        pos_lay = QVBoxLayout(); pos_lay.setSpacing(6)

        a_row = QHBoxLayout()
        a_row.addWidget(self._dim_label("Start:"))
        self.vis_ax   = self._make_input(str(cfg["position_A"]["x"]), "X"); self.vis_ax.setFixedWidth(65)
        self.vis_ay   = self._make_input(str(cfg["position_A"]["y"]), "Y"); self.vis_ay.setFixedWidth(65)
        self.vis_az   = self._make_input(str(cfg["position_A"]["z"]), "Z"); self.vis_az.setFixedWidth(65)
        self.vis_aphi = self._make_input(str(cfg["position_A"]["phi"]), "Φ"); self.vis_aphi.setFixedWidth(55)
        for lbl, w in [("X", self.vis_ax), ("Y", self.vis_ay), ("Z", self.vis_az), ("Φ", self.vis_aphi)]:
            a_row.addWidget(self._dim_label(lbl)); a_row.addWidget(w)
        a_row.addWidget(self._btn("📍", T.SUBTLE, self._vis_get_current_a, w=30))
        pos_lay.addLayout(a_row)

        b_row = QHBoxLayout()
        b_row.addWidget(self._dim_label("Basket / Pool:"))
        self.vis_bx   = self._make_input(str(cfg["position_B"]["x"]), "X"); self.vis_bx.setFixedWidth(65)
        self.vis_by   = self._make_input(str(cfg["position_B"]["y"]), "Y"); self.vis_by.setFixedWidth(65)
        self.vis_bz   = self._make_input(str(cfg["position_B"]["z"]), "Z"); self.vis_bz.setFixedWidth(65)
        self.vis_bphi = self._make_input(str(cfg["position_B"]["phi"]), "Φ"); self.vis_bphi.setFixedWidth(55)
        for lbl, w in [("X", self.vis_bx), ("Y", self.vis_by), ("Z", self.vis_bz), ("Φ", self.vis_bphi)]:
            b_row.addWidget(self._dim_label(lbl)); b_row.addWidget(w)
        b_row.addWidget(self._btn("📍", T.SUBTLE, self._vis_get_current_b, w=30))
        pos_lay.addLayout(b_row)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self._btn("💾 Save",        T.ACCENT2, self._vis_save_config, bold=True))
        btn_row.addWidget(self._btn("▶ Start",     T.ACCENT,  self._vis_goto_a))
        btn_row.addWidget(self._btn("▶ Basket / Pool",         T.ACCENT,  self._vis_goto_b))
        pos_lay.addLayout(btn_row)

        pos_lay.addStretch()
        pos_box.setLayout(pos_lay)
        mid_row.addWidget(pos_box, stretch=1)

        left_lay.addLayout(mid_row)

        # Ayarlar + Kontrol
        bottom_row = QVBoxLayout(); bottom_row.setSpacing(6)

        settings_box = QGroupBox("Settings")
        set_lay = QVBoxLayout(); set_lay.setSpacing(8)

        scan_row = QHBoxLayout()
        scan_row.addWidget(self._dim_label("Scan Time:"))
        self.vis_scan_spin = QDoubleSpinBox()
        self.vis_scan_spin.setRange(1.5,10.0)
        self.vis_scan_spin.setSingleStep(0.1)
        self.vis_scan_spin.setValue(cfg.get("scan_timeout",2.5))
        self.vis_scan_spin.setSuffix(" s")
        self.vis_scan_spin.setFixedWidth(80)
        scan_row.addWidget(self.vis_scan_spin)
        scan_row.addStretch()
        set_lay.addLayout(scan_row)

        conf_row = QHBoxLayout()
        conf_row.addWidget(self._dim_label("Confidence Threshold:"))
        self.vis_conf_slider = QSlider(Qt.Orientation.Horizontal)
        self.vis_conf_slider.setRange(30,95)
        self.vis_conf_slider.setValue(int(cfg.get("confidence",0.65)*100))
        self.vis_conf_slider.setFixedWidth(120)
        self.vis_conf_val = QLabel(f"{cfg.get('confidence',0.65):.2f}")
        self.vis_conf_val.setStyleSheet(f"color:{T.ACCENT}; font-family:monospace;")
        self.vis_conf_slider.valueChanged.connect(self._vis_on_conf_changed)
        conf_row.addWidget(self.vis_conf_slider)
        conf_row.addWidget(self.vis_conf_val)
        conf_row.addStretch()
        set_lay.addLayout(conf_row)
        settings_box.setLayout(set_lay)
        bottom_row.addWidget(settings_box)

        ctrl_box = QGroupBox("Control")
        ctrl_lay = QVBoxLayout(); ctrl_lay.setSpacing(6)

        cam_btn_row = QHBoxLayout()
        self.btn_vis_cam_start = self._btn("▶  Start Camera","#2a4a2a",self._vis_start_camera)
        self.btn_vis_cam_stop  = self._btn("■  Stop Camera", T.DANGER, self._vis_stop_camera)
        self.btn_vis_cam_stop.setEnabled(False)
        cam_btn_row.addWidget(self.btn_vis_cam_start)
        cam_btn_row.addWidget(self.btn_vis_cam_stop)
        ctrl_lay.addLayout(cam_btn_row)

        mode_row = QHBoxLayout()
        mode_row.addWidget(self._dim_label("Mode:"))
        self.vis_mode_auto = QRadioButton("Full Autonomous")
        self.vis_mode_step = QRadioButton("Step by Step")
        self.vis_mode_auto.setChecked(cfg.get("run_mode","full_auto") == "full_auto")
        self.vis_mode_step.setChecked(cfg.get("run_mode","full_auto") == "step_by_step")
        mode_group = QButtonGroup(self)
        mode_group.addButton(self.vis_mode_auto)
        mode_group.addButton(self.vis_mode_step)
        mode_row.addWidget(self.vis_mode_auto)
        mode_row.addWidget(self.vis_mode_step)
        mode_row.addStretch()
        ctrl_lay.addLayout(mode_row)

        self.btn_vis_start = self._btn(
            "🍅  START AUTONOMOUS HARVEST","#1a5a1a",
            self._vis_start_harvest, bold=True)
        ctrl_lay.addWidget(self.btn_vis_start)

        action_row = QHBoxLayout()
        self.btn_vis_pause        = self._btn("⏸  Pause", T.WARNING, self._vis_pause)
        self.btn_vis_estop        = self._btn("🛑  EMERGENCY STOP", T.DANGER,  self._vis_estop, bold=True)
        self.btn_vis_step_confirm = self._btn("⏭  Continue", T.ACCENT,  self._vis_step_confirm)
        self.btn_vis_pause.setEnabled(False)
        self.btn_vis_estop.setEnabled(False)
        self.btn_vis_step_confirm.setEnabled(False)
        action_row.addWidget(self.btn_vis_pause)
        action_row.addWidget(self.btn_vis_estop)
        action_row.addWidget(self.btn_vis_step_confirm)
        ctrl_lay.addLayout(action_row)

        ctrl_box.setLayout(ctrl_lay)
        bottom_row.addWidget(ctrl_box)

        left_lay.addStretch()
        left_w.setLayout(left_lay)

        # ── SAĞ TARAF — İşlem basamağı ───────────────────────────────────────
        right_w   = QWidget()
        right_lay = QVBoxLayout()
        right_lay.setContentsMargins(4,8,8,8)
        right_lay.setSpacing(6)

        state_header = QLabel("📋  Process Steps")
        state_header.setFont(QFont("Segoe UI",10,QFont.Weight.Bold))
        state_header.setStyleSheet(
            f"color:{T.ACCENT}; padding:6px 10px;"
            f"border-left:3px solid {T.ACCENT};"
            f"background:{T.BG_PANEL}; border-radius:4px;")
        right_lay.addWidget(state_header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(
            f"QScrollArea {{ border:1px solid {T.BORDER};"
            f"background:{T.BG_CARD}; border-radius:4px; }}")
        self.vis_log_widget = QWidget()
        self.vis_log_layout = QVBoxLayout()
        self.vis_log_layout.setContentsMargins(8,8,8,8)
        self.vis_log_layout.setSpacing(3)
        self.vis_log_layout.addStretch()
        self.vis_log_widget.setLayout(self.vis_log_layout)
        scroll.setWidget(self.vis_log_widget)
        right_lay.addWidget(scroll, stretch=1)

        # Ayarlar + Kontrol sağ tarafa taşındı
        right_lay.addLayout(bottom_row)

        self.vis_scan_timer_label = QLabel("⏱  —")
        self.vis_scan_timer_label.setStyleSheet(
            f"color:{T.TEXT_DIM}; font-family:monospace; font-size:8pt; font-weight:600;"
            f"padding:4px; border:1px solid {T.BORDER}; border-radius:4px;")
        right_lay.addWidget(self.vis_scan_timer_label)

        right_w.setLayout(right_lay)
        right_w.setMinimumWidth(320)
        right_w.setMaximumWidth(420)

        main_split.addWidget(left_w)
        main_split.addWidget(right_w)
        main_split.setSizes([850,380])

        tab_lay = QVBoxLayout()
        tab_lay.setContentsMargins(0,0,0,0)
        tab_lay.addWidget(main_split)
        tab.setLayout(tab_lay)

        self._vis_scan_start_time = None
        self._vis_scan_timer = QTimer()
        self._vis_scan_timer.setInterval(100)
        self._vis_scan_timer.timeout.connect(self._vis_update_scan_timer)

        return tab

    # ── Vision yardımcı metodlar ──────────────────────────────────────────────

    @pyqtSlot(int)
    def _vis_on_conf_changed(self, v: int):
        self.vis_conf_val.setText(f"{v/100:.2f}")
        if self._vision_worker:
            self._vision_worker.set_confidence(v/100)

    def _vis_set_cam_status(self, text: str, color: str):
        self.vis_cam_status.setText(text)
        self.vis_cam_status.setStyleSheet(
            f"color:{color}; font-family:monospace; font-weight:600;")

    def _vis_set_robot_status(self, text: str):
        self.vis_robot_status.setText(f"🤖  {text}")

    def _vis_log_add(self, t_no: int, step_no: int, label: str, status: str):
        T = THEME
        color_map = {'active':T.WARNING,'done':T.ACCENT2,'skipped':T.TEXT_DIM,'pending':T.TEXT_DIM}
        icon_map  = {'active':'●','done':'✓','skipped':'↷','pending':'○'}
        color = color_map.get(status, T.TEXT_DIM)
        icon  = icon_map.get(status, '○')
        text  = f"  {icon} {t_no}.{step_no} {label}" if t_no > 0 else f"  {icon} {label}"
        lbl   = QLabel(text)
        lbl.setStyleSheet(
            f"color:{color}; font-family:monospace; font-size:9pt; font-weight:600; padding:1px 4px;")
        count = self.vis_log_layout.count()
        self.vis_log_layout.takeAt(count-1)
        self.vis_log_layout.addWidget(lbl)
        self.vis_log_layout.addStretch()

    def _vis_log_add_separator(self, text: str):
        T = THEME
        lbl = QLabel(f"── {text} ──")
        lbl.setStyleSheet(
            f"color:{T.ACCENT}; font-family:monospace; font-size:9pt;"
            f"font-weight:600; padding:4px 4px 2px 4px;")
        count = self.vis_log_layout.count()
        self.vis_log_layout.takeAt(count-1)
        self.vis_log_layout.addWidget(lbl)
        self.vis_log_layout.addStretch()

    def _vis_log_clear(self):
        while self.vis_log_layout.count():
            item = self.vis_log_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.vis_log_layout.addStretch()

    def _vis_update_scan_timer(self):
        if self._vis_scan_start_time is None:
            return
        elapsed = time.time() - self._vis_scan_start_time
        timeout = self.vis_scan_spin.value()
        self.vis_scan_timer_label.setText(f"⏱  {elapsed:.1f} / {timeout:.1f} s")
        self.vis_scan_info.setText(f"{len(self._locked_targets)} stable targets")
        if elapsed >= timeout:
            self._vis_do_lock()

    def _vis_do_lock(self):
        self._vis_scan_timer.stop()
        self._vis_scan_start_time = None

        if not self._locked_targets:
            self._vis_set_cam_status("⚪  NO TARGETS — SCANNING", THEME.TEXT_DIM)
            self.vis_scan_timer_label.setText("⏱  No targets found, scanning continues")
            self._vis_scan_start_time = time.time()
            self._vis_scan_timer.start()
            return

        if self._vision_worker:
            self._vision_worker.set_locked(True)
        self._vis_set_cam_status(
            f"🟡  LOCKED · {len(self._locked_targets)} targets", THEME.WARNING)
        self.btn_vis_unlock.setEnabled(True)
        self.btn_vis_start.setEnabled(True)
        self.vis_scan_timer_label.setText(
            f"⏱  Locked · {len(self._locked_targets)} targets")
        self._vis_log_add(0,0,"Scanning completed",'done')
        self._vis_log_add(0,0,f"Locked: {len(self._locked_targets)} targets",'done')

        # Otonom hasat bekliyorsa otomatik başlat
        if getattr(self,'_vis_auto_start_pending',False):
            self._vis_auto_start_pending = False
            self._vis_start_harvest()

    def _vis_get_current_a(self):
        self.vis_ax.setText(f"{self.current_x:.1f}")
        self.vis_ay.setText(f"{self.current_y:.1f}")
        self.vis_az.setText(f"{self.current_z:.1f}")
        self.vis_aphi.setText(f"{self.current_phi:.1f}")

    def _vis_get_current_b(self):
        self.vis_bx.setText(f"{self.current_x:.1f}")
        self.vis_by.setText(f"{self.current_y:.1f}")
        self.vis_bz.setText(f"{self.current_z:.1f}")
        self.vis_bphi.setText(f"{self.current_phi:.1f}")

    def _vis_save_config(self):
        try:
            pos_a = {"x":float(self.vis_ax.text()),"y":float(self.vis_ay.text()),
                     "z":float(self.vis_az.text()),"phi":float(self.vis_aphi.text())}
            pos_b = {"x":float(self.vis_bx.text()),"y":float(self.vis_by.text()),
                     "z":float(self.vis_bz.text()),"phi":float(self.vis_bphi.text())}
            run_mode = "full_auto" if self.vis_mode_auto.isChecked() else "step_by_step"
            ok = save_all(pos_a,pos_b,
                          self.vis_scan_spin.value(),
                          self.vis_conf_slider.value()/100,
                          run_mode)
            if ok:
                self._vision_cfg.update({
                    "position_A":pos_a,"position_B":pos_b,
                    "scan_timeout":self.vis_scan_spin.value(),
                    "confidence":self.vis_conf_slider.value()/100,
                    "run_mode":run_mode,
                })
                self.show_info("Settings saved ✓")
            else:
                self.show_error("Failed to save settings!")
        except ValueError:
            self.show_error("Invalid coordinate values!")

    def _vis_goto_a(self):
        p = self._vision_cfg["position_A"]
        self.x_input.setText(str(p["x"])); self.y_input.setText(str(p["y"]))
        self.z_input.setText(str(p["z"])); self.phi_input.setText(str(p["phi"]))
        self._call("CALCULATE"); self._call("SEND")

    def _vis_goto_b(self):
        p = self._vision_cfg["position_B"]
        self.x_input.setText(str(p["x"])); self.y_input.setText(str(p["y"]))
        self.z_input.setText(str(p["z"])); self.phi_input.setText(str(p["phi"]))
        self._call("CALCULATE"); self._call("SEND")

    def _vis_start_camera(self):
        if self._vision_worker and self._vision_worker.isRunning():
            return
        confidence = self.vis_conf_slider.value() / 100
        self._vision_worker = VisionWorker(confidence=confidence, mock_mode=False)
        self._vision_worker.frame_ready.connect(self._vis_on_frame)
        self._vision_worker.targets_updated.connect(self._vis_on_targets)
        self._vision_worker.status_changed.connect(
            lambda t, c: self._vis_set_cam_status(f"🟢  {t}", c))
        self._vision_worker.error_occurred.connect(self.show_error)
        self._vision_worker.start()
        self._vis_scan_start_time = time.time()
        self._vis_scan_timer.start()
        self._vis_set_cam_status("🟢  SCANNING", THEME.ACCENT2)
        self._vis_set_robot_status("Ready")
        self.btn_vis_cam_start.setEnabled(False)
        self.btn_vis_cam_stop.setEnabled(True)
        self._vis_log_clear()
        self._vis_log_add(0,0,"HOME",'done')
        self._vis_log_add(0,0,"Camera started",'done')
        self._vis_log_add(0,0,f"Scanning: {self.vis_scan_spin.value():.1f}s",'active')

    def _vis_stop_camera(self):
        if self._vision_worker:
            self._vision_worker.stop()
            self._vision_worker.wait(2000)
            self._vision_worker = None
        self._vis_scan_timer.stop()
        self._vis_scan_start_time = None
        # Kamera durdurulunca listeyi temizle
        self._locked_targets = []
        self._vis_update_target_table([])
        self._vis_set_cam_status("⚪  CAMERA OFF", THEME.TEXT_DIM)
        self.btn_vis_cam_start.setEnabled(True)
        self.btn_vis_cam_stop.setEnabled(False)
        self.btn_vis_start.setEnabled(False)
        self.vis_cam_label.setText("Camera not started")

    def _vis_unlock(self):
        if self._vision_worker:
            self._vision_worker.set_locked(False)
        self._locked_targets = []
        self._vis_update_target_table([])
        self.btn_vis_unlock.setEnabled(False)
        self.btn_vis_start.setEnabled(False)
        self._vis_set_cam_status("🟢  SCANNING", THEME.ACCENT2)
        self._vis_scan_start_time = time.time()
        self._vis_scan_timer.start()
        # Log panelini sıfırla ve baştan yaz
        self._vis_log_clear()
        self._vis_log_add(0, 0, "HOME", 'done')
        self._vis_log_add(0, 0, "Camera started", 'done')
        self._vis_log_add(0, 0, f"Scanning: {self.vis_scan_spin.value():.1f}s", 'active')

    @pyqtSlot(object)
    def _vis_on_frame(self, frame):
        try:
            import cv2
            rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch*w, QImage.Format.Format_RGB888)
            pix  = QPixmap.fromImage(qimg)
            self.vis_cam_label.setPixmap(
                pix.scaled(
                    self.vis_cam_label.width(),
                    self.vis_cam_label.height(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                ))
        except Exception:
            pass

    @pyqtSlot(list)
    def _vis_on_targets(self, targets: list):
        self._locked_targets = targets
        self._vis_update_target_table(targets)

    def _vis_update_target_table(self, targets: list):
        T = THEME
        CLASS_COLORS = {'fully':T.DANGER,'half':T.WARNING,'green':T.ACCENT2}
        CLASS_LABELS = {'fully':'OLGUN','half':'YARI','green':'HAM'}
        self.vis_target_table.setRowCount(0)
        for i, t in enumerate(targets):
            self.vis_target_table.insertRow(i)

            # coords kamera frame'inde, cm cinsinden
            # → mm'ye çevir → robot frame'ine çevir → tekrar cm'ye böl
            x_cam_mm = float(t['coords'][0]) * 10.0
            y_cam_mm = float(t['coords'][1]) * 10.0
            z_cam_mm = float(t['coords'][2]) * 10.0
            x_rob_mm, y_rob_mm, z_rob_mm = cam_to_robot(x_cam_mm, y_cam_mm, z_cam_mm)
            x_rob_cm = x_rob_mm / 10.0
            y_rob_cm = y_rob_mm / 10.0
            z_rob_cm = z_rob_mm / 10.0

            vals = [str(i+1),t['id'],
                    CLASS_LABELS.get(t['class'],t['class']),
                    f"{x_rob_cm:.1f}",
                    f"{y_rob_cm:.1f}",
                    f"{z_rob_cm:.1f}"]
            color = CLASS_COLORS.get(t['class'],T.TEXT)
            for col, val in enumerate(vals):
                item = QTableWidgetItem(val)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if col == 2:
                    item.setForeground(QColor(color))
                self.vis_target_table.setItem(i,col,item)

    def _vis_start_harvest(self):
        # Kamera kapalıysa otomatik başlat
        if self._vision_worker is None or not self._vision_worker.isRunning():
            self._vis_start_camera()

        # Henüz kilitli hedef yoksa tara ve bekle
        if not self._locked_targets:
            self._vis_auto_start_pending = True
            self._vis_set_cam_status("🟢  SCANNING — Harvest pending", THEME.ACCENT2)
            self._vis_log_add(0,0,"Harvest pending...",'active')
            return

        self._vis_auto_start_pending = False

        cfg      = self._vision_cfg
        run_mode = "full_auto" if self.vis_mode_auto.isChecked() else "step_by_step"

        self._pick_worker = PickPlaceWorker(
            targets      = self._locked_targets,
            pos_a        = cfg["position_A"],
            pos_b        = cfg["position_B"],
            run_mode     = run_mode,
            ack_callback = self.ack_callback,
            uart_send    = self.uart_send,
        )
        self._pick_worker.state_changed.connect(self._vis_set_robot_status)
        self._pick_worker.log_entry.connect(self._vis_on_log_entry)
        self._pick_worker.step_request.connect(self._vis_on_step_request)
        self._pick_worker.gripper_request.connect(self._set_gripper)
        self._pick_worker.cycle_done.connect(self._vis_on_cycle_done)
        self._pick_worker.error_occurred.connect(self.show_error)
        self._pick_worker.ack_received.connect(self._vis_on_ack_received)
        self._pick_worker.start()

        self.btn_vis_start.setEnabled(False)
        self.btn_vis_pause.setEnabled(True)
        self.btn_vis_estop.setEnabled(True)
        if run_mode == "step_by_step":
            self.btn_vis_step_confirm.setEnabled(True)

        olgun = [t for t in self._locked_targets if t['class'] == 'fully']
        for idx, t in enumerate(olgun):
            self._vis_log_add_separator(f"Target {idx+1}/{len(olgun)}: {t['id']} (FULLY)")
            for sno, slbl in enumerate(["Go to target","Grab","Carry to pool","Place"],1):
                self._vis_log_add(idx+1,sno,slbl,'pending')
        self._vis_log_add(0,0,"Return to start position",'pending')

    @pyqtSlot(int, int, str, str)
    def _vis_on_log_entry(self, t_no: int, step_no: int, label: str, status: str):
        T = THEME
        color_map = {'active':T.WARNING,'done':T.ACCENT2,'skipped':T.TEXT_DIM,'pending':T.TEXT_DIM}
        icon_map  = {'active':'●','done':'✓','skipped':'↷','pending':'○'}
        color  = color_map.get(status, T.TEXT_DIM)
        icon   = icon_map.get(status, '○')
        search = f"{t_no}.{step_no} {label}" if t_no > 0 else label
        for i in range(self.vis_log_layout.count()):
            item = self.vis_log_layout.itemAt(i)
            if item and item.widget() and isinstance(item.widget(), QLabel):
                if search in item.widget().text():
                    item.widget().setText(f"  {icon} {search}")
                    item.widget().setStyleSheet(
                        f"color:{color}; font-family:monospace;"
                        f"font-size:9pt; font-weight:600; padding:1px 4px;")
                    break

    @pyqtSlot(str)
    def _vis_on_ack_received(self, msg: str):
        """STM32 DONE sinyali geldiğinde Process Steps'e yeni satır ekler."""
        T = THEME
        lbl = QLabel(f"  ✓ {msg}")
        lbl.setStyleSheet(
            f"color:{T.ACCENT2}; font-family:monospace;"
            f"font-size:9pt; font-weight:700; padding:1px 4px;")
        self.vis_log_layout.addWidget(lbl)

    @pyqtSlot(str, float, float, float, float, float)
    def _vis_on_step_request(self, mode: str,
                              x: float, y: float, z: float,
                              phi: float, t: float):
        self.x_input.setText(f"{x:.2f}")
        self.y_input.setText(f"{y:.2f}")
        self.z_input.setText(f"{z:.2f}")
        self.phi_input.setText(f"{phi:.2f}")
        self.time_input.setText(f"{t:.2f}")
        self.calculate_callback(mode,str(x),str(y),str(z),str(phi),str(t),self)

    def _vis_on_cycle_done(self):
        self._vis_set_robot_status("HOME — Cycle completed")
        self.btn_vis_start.setEnabled(False)
        self.btn_vis_pause.setEnabled(False)
        self.btn_vis_estop.setEnabled(False)
        self.btn_vis_step_confirm.setEnabled(False)
        self._vis_unlock()

    def _vis_pause(self):
        if self._pick_worker:
            if self._pick_worker._paused:
                self._pick_worker.resume()
                self.btn_vis_pause.setText("⏸  Duraklat")
            else:
                self._pick_worker.pause()
                self.btn_vis_pause.setText("▶  Devam")

    def _vis_estop(self):
        if self._pick_worker:
            self._pick_worker.emergency_stop()
            self._pick_worker = None
        if self._vision_worker:
            self._vision_worker.stop()
            self._vision_worker = None
        self._vis_scan_timer.stop()
        self._vis_scan_start_time = None
        self._locked_targets = []
        self._vis_update_target_table([])
        self._vis_auto_start_pending = False
        self._vis_set_cam_status("🔴  EMERGENCY STOP — Press Camera Start to reset system",
                                  THEME.DANGER)
        self._vis_set_robot_status("ESTOP")
        self.btn_vis_cam_start.setEnabled(True)   # tekrar başlatılabilir
        self.btn_vis_cam_stop.setEnabled(False)
        self.btn_vis_start.setEnabled(False)
        self.btn_vis_pause.setEnabled(False)
        self.btn_vis_estop.setEnabled(False)
        self.btn_vis_step_confirm.setEnabled(False)
        self.btn_vis_pause.setText("⏸  Duraklat")  # buton metnini sıfırla
        self._vis_log_add(0, 0, "EMERGENCY STOP — system stopped", 'skipped')

    def _vis_step_confirm(self):
        if self._pick_worker:
            self._pick_worker.confirm_step()


    # ==========================================================================
    # PLOT / DISPLAY
    # ==========================================================================
    def plot_path_and_robot(self, x_traj, y_traj, z_traj, d1, Q2, Q3, Q4):
        if not x_traj:
            return

        # Store full trajectory for animation
        if self.last_joint_traj:
            d1_l, Q2_l, Q3_l, Q4_l, _ = self.last_joint_traj
            self._anim_traj = (list(x_traj), list(y_traj), list(z_traj),
                               list(d1_l), list(Q2_l), list(Q3_l), list(Q4_l))

        if self._anim_mode and self._anim_traj:
            self._anim_frame = 0
            if not self._anim_timer.isActive():
                self._anim_timer.start()
            return

        self._draw_traj_static(x_traj, y_traj, z_traj, d1, Q2, Q3, Q4)

    def _draw_traj_static(self, x_traj, y_traj, z_traj, d1, Q2, Q3, Q4):
        self.fig_3d.clear()
        ax = self.fig_3d.add_subplot(111, projection="3d")
        ax.set_facecolor(THEME.BG_CARD)
        self.fig_3d.patch.set_facecolor(THEME.BG_DEEP)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_facecolor(THEME.BG_ROW_ALT)
            pane.set_edgecolor(THEME.BORDER)
            pane.set_alpha(0.6)
        ax.plot(x_traj,y_traj,z_traj,color=THEME.ACCENT,label="Path",linewidth=1.8)
        ax.scatter(x_traj[0],y_traj[0],z_traj[0],color=THEME.ACCENT2,s=80,label="Start",zorder=5)
        ax.scatter(x_traj[-1],y_traj[-1],z_traj[-1],color=THEME.WARNING,s=80,label="Target",zorder=5)
        self._draw_robot(ax,d1,Q2,Q3,Q4)
        self._setup_3d_axes(ax)
        self.canvas_3d.draw()

    def _setup_3d_axes(self, ax):
        for spine in ["x","y","z"]:
            ax.tick_params(axis=spine,colors=THEME.TEXT_DIM,labelsize=8)
        ax.set_xlabel("X (mm)",color=THEME.TEXT,fontsize=9,fontweight='semibold')
        ax.set_ylabel("Y (mm)",color=THEME.TEXT,fontsize=9,fontweight='semibold')
        ax.set_zlabel("Z (mm)",color=THEME.TEXT,fontsize=9,fontweight='semibold')
        ax.set_title("End-Effector Trajectory + Robot Pose | Z{-360/360} Y{-800/800} X{0/800}",
                     color=THEME.ACCENT,fontsize=10,fontweight='semibold',pad=10)
        ax.set_xlim(-1000,1000); ax.set_ylim(-1000,1000); ax.set_zlim(-1000,1000)
        ax.legend(loc="upper left",bbox_to_anchor=(-1.1,1.05),
                  fontsize=9,facecolor=THEME.BG_CARD,labelcolor=THEME.TEXT,
                  edgecolor=THEME.BORDER)

    def _toggle_anim(self):
        self._anim_mode = not self._anim_mode
        if self._anim_mode:
            self.btn_anim.setText("■  Stop Animate")
            if self._anim_traj:
                self._anim_frame = 0
                self._anim_timer.start()
        else:
            self._anim_timer.stop()
            self.btn_anim.setText("▷  Animate")
            # Redraw static with last trajectory
            if self._anim_traj:
                x_t, y_t, z_t, d1_l, Q2_l, Q3_l, Q4_l = self._anim_traj
                self._draw_traj_static(x_t, y_t, z_t, d1_l[-1], Q2_l[-1], Q3_l[-1], Q4_l[-1])

    def _anim_tick(self):
        if self._anim_traj is None:
            self._anim_timer.stop()
            return
        x_t, y_t, z_t, d1_l, Q2_l, Q3_l, Q4_l = self._anim_traj
        n = len(d1_l)
        if self._anim_frame >= n:
            self._anim_timer.stop()
            return

        idx = self._anim_frame
        self.fig_3d.clear()
        ax = self.fig_3d.add_subplot(111, projection="3d")
        ax.set_facecolor(THEME.BG_CARD)
        self.fig_3d.patch.set_facecolor(THEME.BG_DEEP)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.set_facecolor(THEME.BG_ROW_ALT)
            pane.set_edgecolor(THEME.BORDER)
            pane.set_alpha(0.6)
        # Full path (faint)
        ax.plot(x_t, y_t, z_t, color=THEME.ACCENT, linewidth=1.2, alpha=0.25, label="Path")
        # Traversed portion
        ax.plot(x_t[:idx+1], y_t[:idx+1], z_t[:idx+1],
                color=THEME.ACCENT, linewidth=2.0, label="Traversed")
        # Current end-effector position
        ax.scatter([x_t[idx]], [y_t[idx]], [z_t[idx]],
                   color=THEME.WARNING, s=120, zorder=6, label="EE")
        # Robot at current frame
        self._draw_robot(ax, d1_l[idx], Q2_l[idx], Q3_l[idx], Q4_l[idx])
        self._setup_3d_axes(ax)
        self.canvas_3d.draw()

        self._anim_frame += 3   # step 3 frames per tick → smooth at 25fps

    def plot_time_graphs_xyz(self, t_vals, x_traj, y_traj, z_traj,
                              x_vel, y_vel, z_vel, x_acc, y_acc, z_acc):
        self.fig_time.clear()
        self.fig_time.patch.set_facecolor(THEME.BG_DEEP)
        colors = [THEME.ACCENT,THEME.ACCENT2,THEME.WARNING]
        for ax_idx, data_set, title in [
            (311,[(x_traj,"X"),(y_traj,"Y"),(z_traj,"Z")],"Position (mm)"),
            (312,[(x_vel,"Xv"),(y_vel,"Yv"),(z_vel,"Zv")],"Velocity (mm/s)"),
            (313,[(x_acc,"Xa"),(y_acc,"Ya"),(z_acc,"Za")],"Acceleration (mm/s²)"),
        ]:
            ax = self.fig_time.add_subplot(ax_idx)
            ax.set_facecolor(THEME.BG_CARD)
            ax.grid(True, color=THEME.BORDER, linewidth=0.5, alpha=0.8)
            for (data,lbl),c in zip(data_set,colors):
                ax.plot(t_vals,data,label=lbl,color=c,linewidth=1.4)
            ax.set_title(title,color=THEME.ACCENT,fontsize=9,fontweight='semibold',pad=4)
            ax.tick_params(colors=THEME.TEXT_DIM,labelsize=8)
            ax.legend(loc="center left",bbox_to_anchor=(1.01,0.5),
                      fontsize=8,facecolor=THEME.BG_CARD,labelcolor=THEME.TEXT,
                      edgecolor=THEME.BORDER)
            for spine in ax.spines.values():
                spine.set_edgecolor(THEME.BORDER)
        self.fig_time.subplots_adjust(hspace=0.6,right=0.88)
        self.canvas_time.draw()

    def update_joint_display(self, d1_list, Q2_list, Q3_list, Q4_list):
        if not d1_list: return
        for lbl, name, lst in [
            (self.d1_label,"d1",d1_list),(self.Q2_label,"Q2",Q2_list),
            (self.Q3_label,"Q3",Q3_list),(self.Q4_label,"Q4",Q4_list),
        ]:
            lbl.setText(f"{name}: {lst[0]:.3f} → {lst[-1]:.3f}")

    def show_error(self, message: str):
        dlg = QMessageBox(self)
        dlg.setIcon(QMessageBox.Icon.Critical)
        dlg.setText(message)
        dlg.setWindowTitle("Error")
        dlg.setStyleSheet(f"background:{THEME.BG_CARD}; color:{THEME.TEXT};")
        dlg.exec()

    def show_info(self, message: str):
        dlg = QMessageBox(self)
        dlg.setIcon(QMessageBox.Icon.Information)
        dlg.setText(message)
        dlg.setWindowTitle("Bilgi")
        dlg.setStyleSheet(f"background:{THEME.BG_CARD}; color:{THEME.TEXT};")
        dlg.exec()


    # ==========================================================================
    # YARDIMCI FACTORY METODLAR
    # ==========================================================================
    @staticmethod
    def _make_input(default: str = "", placeholder: str = "") -> QLineEdit:
        w = QLineEdit(default)
        w.setPlaceholderText(placeholder)
        w.setMaximumHeight(THEME.INPUT_H)
        return w

    @staticmethod
    def _dim_label(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{THEME.TEXT_DIM}; font-size:8pt; font-weight:600;")
        return lbl

    @staticmethod
    def _btn(text: str, color: str, fn, bold: bool = False,
             tip: str = "", w: int = 0) -> QPushButton:
        b  = QPushButton(text)
        fw = "600" if bold else "400"
        b.setStyleSheet(
            f"QPushButton {{ background:{color}; color:#ffffff;"
            f"border:none; border-radius:4px; padding:4px 10px; font-weight:{fw}; }}"
            f"QPushButton:hover  {{ background:{color}dd; border:1px solid {THEME.BORDER_ACT}; }}"
            f"QPushButton:pressed {{ background:{color}88; }}"
        )
        b.setMaximumHeight(THEME.BTN_H)
        b.clicked.connect(fn)
        if tip: b.setToolTip(tip)
        if w:   b.setFixedWidth(w)
        return b

    @staticmethod
    def _wrap(widget: QWidget) -> QWidget:
        container = QWidget()
        lay = QVBoxLayout()
        lay.setContentsMargins(0,0,0,0)
        lay.addWidget(widget)
        container.setLayout(lay)
        return container

    def _draw_aabb(self, ax, box, color=THEME.DANGER):
        corners = np.array([
            [box.xmin,box.ymin,box.zmin],[box.xmax,box.ymin,box.zmin],
            [box.xmax,box.ymax,box.zmin],[box.xmin,box.ymax,box.zmin],
            [box.xmin,box.ymin,box.zmax],[box.xmax,box.ymin,box.zmax],
            [box.xmax,box.ymax,box.zmax],[box.xmin,box.ymax,box.zmax],
        ],dtype=float)
        for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
            ax.plot(*zip(corners[a],corners[b]),color=color,linewidth=1.2)

    def _draw_obstacles(self, ax):
        for obs in OBSTACLES:
            self._draw_aabb(ax,obs)

    def _draw_robot(self, ax, d1, Q2, Q3, Q4):
        z  = float(d1) 
        p0 = np.array([0.,0.,0.])
        pb = np.array([0.,0.,z])
        p1 = pb + np.array([L2*np.cos(Q2),L2*np.sin(Q2),0])
        p2 = p1 + np.array([L3*np.cos(Q2+Q3),L3*np.sin(Q2+Q3),0])
        ph = Q2+Q3+Q4
        p3 = p2 + np.array([L_GRIP*np.cos(ph),L_GRIP*np.sin(ph),0])
        for a,b,lw,lbl in [(p0,pb,5,"Dikey Eksen"),(pb,p1,4,None),(p1,p2,4,None),(p2,p3,3,"Gripper")]:
            kw = dict(linewidth=10,color=THEME.DARK,solid_capstyle='round')
            if lbl: kw["label"] = lbl
            ax.plot(*zip(a,b),**kw)
        pts = np.array([p0,pb,p1,p2,p3])
        ax.scatter(pts[:,0],pts[:,1],pts[:,2],s=150,color=THEME.ACCENT,zorder=5,edgecolors='white',linewidths=1.5)
