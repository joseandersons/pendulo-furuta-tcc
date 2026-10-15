import csv
import time
import math
import threading
import collections
from datetime import datetime

from PySide6.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGroupBox,
    QFormLayout, QDoubleSpinBox, QPushButton, QLabel, QTextEdit,
    QComboBox, QMessageBox, QScrollArea, QGridLayout, QSizePolicy,
    QLineEdit
)
from PySide6.QtCore import QTimer, Qt, QObject, Signal, QThread, Slot
import pyqtgraph as pg

from config_store import DATA_DIR, load_config, save_config
from telemetry import decode_telemetry
from serial_comm import (
    serial_is_open, list_serial_ports, open_serial, close_serial,
    drain, send_line, read_until_prefix, read_line
)
from lqr_design import compute_lqr_gain
from observer_matrix import observer_matrices_from_poles

WIN_SEC = 20.0
HZ = 50
GUI_HZ = 20

# Calibracao do zero do pendulo no topo
TOP_CALIB_SEC = 5.0          # tempo de media com o pendulo equilibrado
TOP_CALIB_MAX_STD_DEG = 1.0  # rejeita se alpha variar demais (pendulo nao estava parado)
TOP_CALIB_MAX_MEAN_DEG = 5.0 # maior ajuste aceito (igual ao firmware)

# Mesmos limites usados no firmware fuzzy.cpp para |alpha_dot|.
# Esses valores desenham as funções de pertinência trapezoidais
# baixa, média e alta no gráfico 2 da interface.
FUZZY_ALPHA_DOT_LOW_FULL_DEG_S = 15.0
FUZZY_ALPHA_DOT_LOW_END_DEG_S = 40.0
FUZZY_ALPHA_DOT_MED_START_DEG_S = 25.0
FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S = 50.0
FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S = 70.0
FUZZY_ALPHA_DOT_MED_END_DEG_S = 95.0
FUZZY_ALPHA_DOT_HIGH_START_DEG_S = 80.0
FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S = 110.0
FUZZY_ALPHA_DOT_MAX_DEG_S = 150.0


def hud_text(lines):
    return "\n".join(lines)

class SerialCommandWorker(QObject):
    log = Signal(str)
    finished = Signal(bool, str)

    def __init__(self, commands, serial_lock):
        super().__init__()
        self.commands = commands
        self.serial_lock = serial_lock

    def run(self):
        try:
            with self.serial_lock:
                drain()

                for cmd, prefix in self.commands:
                    self.log.emit(f"[ENVIO] {cmd}")
                    send_line(cmd)

                    resp = read_until_prefix(prefix)

                    if resp:
                        self.log.emit(f"[RESP] {resp}")

                        if resp.startswith("ERR") or resp.endswith(" ERR"):
                            raise RuntimeError(resp)
                    else:
                        self.log.emit(f"[WARN] sem confirmação para {prefix}")

            self.finished.emit(True, "OK")

        except Exception as e:
            self.finished.emit(False, str(e))


class LQRTunerWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LQR Tuner - Pêndulo de Furuta")

        self.buf_t = collections.deque(maxlen=int(WIN_SEC * HZ))
        self.buf = {
            k: collections.deque(maxlen=int(WIN_SEC * HZ))
            for k in ["A", "TH", "AD", "TD", "U", "VEL", "EN"]
        }

        self.log_buf = collections.deque(maxlen=500)
        self.stop_reader = False

        self.serial_lock = threading.RLock()
        self.serial_command_active = threading.Event()
        self.cmd_thread = None
        self.cmd_worker = None

        self.is_collecting = False
        self.collect_file = None
        self.collect_writer = None
        self.collect_start_t = None
        self.collect_meta = {}

        self.theta_ref_applied_deg = 0.0
        self.accel_max_rad_s2 = 30.0

        self.top_calib_lock = threading.Lock()
        self.top_calib_active = False
        self.top_calib_samples = []

        self._cmd_on_success = None
        self._cmd_error_title = ""

        self.last_lqr_cmd = ""
        self.last_obs_label = "manual"

        self._build_ui()
        self._load_initial_config()
        self._refresh_serial_ports()
        self._start_reader_thread()
        self._start_timer()

    def _clamp01(self, x):
        if x < 0.0:
            return 0.0

        if x > 1.0:
            return 1.0

        return x

    def _trapmf(self, x, a, b, c, d):
        if x <= a:
            return 1.0 if a == b else 0.0

        if x < b:
            return self._clamp01((x - a) / (b - a))

        if x <= c:
            return 1.0

        if x < d:
            return self._clamp01((d - x) / (d - c))

        return 1.0 if c == d else 0.0

    def _update_fuzzy_alpha_dot_plot(self):
        x_values = [float(x) for x in range(0, 161)]

        y_low = [
            self._trapmf(
                x,
                0.0,
                0.0,
                FUZZY_ALPHA_DOT_LOW_FULL_DEG_S,
                FUZZY_ALPHA_DOT_LOW_END_DEG_S,
            )
            for x in x_values
        ]

        y_medium = [
            self._trapmf(
                x,
                FUZZY_ALPHA_DOT_MED_START_DEG_S,
                FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S,
                FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S,
                FUZZY_ALPHA_DOT_MED_END_DEG_S,
            )
            for x in x_values
        ]

        y_high = [
            self._trapmf(
                x,
                FUZZY_ALPHA_DOT_HIGH_START_DEG_S,
                FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S,
                FUZZY_ALPHA_DOT_MAX_DEG_S,
                FUZZY_ALPHA_DOT_MAX_DEG_S,
            )
            for x in x_values
        ]

        self.cur_fuzzy_alpha_dot_low.setData(x_values, y_low)
        self.cur_fuzzy_alpha_dot_medium.setData(x_values, y_medium)
        self.cur_fuzzy_alpha_dot_high.setData(x_values, y_high)

    def _build_ui(self):
        main = QHBoxLayout(self)
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(10)

        pg.setConfigOptions(antialias=False)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setMinimumWidth(760)
        main.addWidget(self.plot_widget, stretch=3)

        self.p1 = self.plot_widget.addPlot(row=0, col=0, title="Ângulos: pêndulo α, braço θ e referência θ")
        self.p1.setLabel("left", "ângulo", units="deg")
        self.p1.setLabel("bottom", "tempo", units="s")
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.p1.setYRange(-180, 180)
        self.p1.disableAutoRange(axis="y")
        self.p1.addLegend()

        self.cur_alpha = self.p1.plot(
            pen=pg.mkPen(color=(0, 180, 255), width=2),
            name="α pêndulo"
        )

        self.cur_theta = self.p1.plot(
            pen=pg.mkPen(color=(255, 170, 0), width=2),
            name="θ braço"
        )

        self.cur_theta_ref = self.p1.plot(
            pen=pg.mkPen(color=(255, 255, 255), width=2, style=Qt.DashLine),
            name="θ ref"
        )

        self.hud_p1 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p1.setZValue(10)
        self.p1.addItem(self.hud_p1)

        self.p2 = self.plot_widget.addPlot(row=1, col=0, title="Fuzzy trapezoidal - |α_dot|")
        self.p2.setLabel("left", "pertinência")
        self.p2.setLabel("bottom", "|α_dot|", units="deg/s")
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.p2.setXRange(0.0, 160.0)
        self.p2.setYRange(0.0, 1.05)
        self.p2.disableAutoRange(axis="x")
        self.p2.disableAutoRange(axis="y")
        self.p2.addLegend()

        self.cur_fuzzy_alpha_dot_low = self.p2.plot(
            pen=pg.mkPen(color=(0, 220, 120), width=2),
            name="baixa"
        )

        self.cur_fuzzy_alpha_dot_medium = self.p2.plot(
            pen=pg.mkPen(color=(255, 200, 0), width=2),
            name="média"
        )

        self.cur_fuzzy_alpha_dot_high = self.p2.plot(
            pen=pg.mkPen(color=(255, 100, 100), width=2),
            name="alta"
        )

        self.line_alpha_dot_now = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color=(255, 255, 255), width=2, style=Qt.DashLine)
        )
        self.p2.addItem(self.line_alpha_dot_now)

        self._update_fuzzy_alpha_dot_plot()

        self.hud_p2 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p2.setZValue(10)
        self.p2.addItem(self.hud_p2)

        self.p3 = self.plot_widget.addPlot(row=2, col=0, title="Controle LQR")
        self.p3.setLabel("left", "u / vel")
        self.p3.setLabel("bottom", "tempo", units="s")
        self.p3.showGrid(x=True, y=True, alpha=0.3)
        self.p3.setYRange(-120, 120)
        self.p3.disableAutoRange(axis="y")
        self.p3.addLegend()

        self.cur_u = self.p3.plot(
            pen=pg.mkPen(color=(200, 100, 255), width=2),
            name="u rad/s²"
        )

        self.cur_vel = self.p3.plot(
            pen=pg.mkPen(color=(120, 255, 120), width=2, style=Qt.DashLine),
            name="vel rad/s"
        )

        self.hud_p3 = pg.TextItem(
            text="",
            anchor=(0, 0),
            color=(255, 255, 255),
            fill=pg.mkBrush(0, 0, 0, 170),
            border=pg.mkPen(255, 255, 255, 120)
        )
        self.hud_p3.setZValue(10)
        self.p3.addItem(self.hud_p3)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setMinimumWidth(520)
        side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        main.addWidget(side_scroll, stretch=2)

        side_widget = QWidget()
        side_scroll.setWidget(side_widget)

        side = QVBoxLayout(side_widget)
        side.setContentsMargins(6, 6, 6, 6)
        side.setSpacing(8)

        def setup_spin(spn, minimum, maximum, decimals, step, read_only=False):
            spn.setRange(minimum, maximum)
            spn.setDecimals(decimals)
            spn.setSingleStep(step)
            spn.setMinimumWidth(105)
            spn.setAlignment(Qt.AlignRight)
            spn.setKeyboardTracking(False)

            if read_only:
                spn.setReadOnly(True)
                spn.setButtonSymbols(QDoubleSpinBox.NoButtons)
                spn.setFocusPolicy(Qt.NoFocus)

            return spn

        def label(text):
            lbl = QLabel(text)
            lbl.setMinimumWidth(92)
            return lbl

        def add_grid_spin(grid, row, col, text, spn):
            box = QWidget()
            lay = QHBoxLayout(box)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(6)
            lay.addWidget(label(text))
            lay.addWidget(spn, stretch=1)
            grid.addWidget(box, row, col)

        grp_serial = QGroupBox("Conexão serial")
        serial_layout = QGridLayout(grp_serial)
        serial_layout.setHorizontalSpacing(8)
        serial_layout.setVerticalSpacing(6)

        self.cmb_port = QComboBox()
        self.btn_refresh_ports = QPushButton("Atualizar")
        self.btn_connect = QPushButton("Conectar")
        self.btn_disconnect = QPushButton("Desconectar")
        self.lbl_serial_status = QLabel("Desconectado")

        self.cmb_port.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_refresh_ports.setMinimumWidth(110)

        serial_layout.addWidget(QLabel("Porta"), 0, 0)
        serial_layout.addWidget(self.cmb_port, 0, 1)
        serial_layout.addWidget(self.btn_refresh_ports, 0, 2)
        serial_layout.addWidget(QLabel("Ação"), 1, 0)
        serial_layout.addWidget(self.btn_connect, 1, 1)
        serial_layout.addWidget(self.btn_disconnect, 1, 2)
        serial_layout.addWidget(QLabel("Status"), 2, 0)
        serial_layout.addWidget(self.lbl_serial_status, 2, 1, 1, 2)

        side.addWidget(grp_serial)

        btn_row1 = QHBoxLayout()
        self.btn_start = QPushButton("INICIAR")
        self.btn_stop = QPushButton("STOP")
        self.btn_rth = QPushButton("Reset θ")
        self.btn_calibrate_alpha = QPushButton("Calibrar α")
        self.btn_calibrate_top = QPushButton("Calibrar α topo")
        self.btn_calibrate_top.setToolTip(
            "Ajuste manual (opcional). O firmware já faz isso sozinho\n"
            "(autozero) ao entrar no equilíbrio e após cada mudança de posição.\n"
            "Com o pêndulo EQUILIBRADO em cima, mede o α médio por "
            f"{TOP_CALIB_SEC:.0f} s e corrige o zero.\n"
            "O novo zero é gravado na flash ao apertar STOP."
        )

        btn_row1.addWidget(self.btn_start)
        btn_row1.addWidget(self.btn_stop)
        btn_row1.addWidget(self.btn_rth)
        btn_row1.addWidget(self.btn_calibrate_alpha)
        btn_row1.addWidget(self.btn_calibrate_top)

        side.addLayout(btn_row1)

        grp_qr = QGroupBox("Sintonia LQR: Q e R")
        grid_qr = QGridLayout(grp_qr)
        grid_qr.setHorizontalSpacing(10)
        grid_qr.setVerticalSpacing(6)

        self.spn_q_theta = setup_spin(QDoubleSpinBox(), 0.000001, 1000000.0, 6, 5.0)
        self.spn_q_alpha = setup_spin(QDoubleSpinBox(), 0.000001, 1000000.0, 6, 50.0)
        self.spn_q_theta_dot = setup_spin(QDoubleSpinBox(), 0.000001, 1000000.0, 6, 1.0)
        self.spn_q_alpha_dot = setup_spin(QDoubleSpinBox(), 0.000001, 1000000.0, 6, 5.0)
        self.spn_r = setup_spin(QDoubleSpinBox(), 0.000001, 1000000.0, 6, 0.1)
        self.btn_apply_lqr = QPushButton("Calcular + Aplicar LQR")

        add_grid_spin(grid_qr, 0, 0, "q_theta", self.spn_q_theta)
        add_grid_spin(grid_qr, 0, 1, "q_alpha", self.spn_q_alpha)
        add_grid_spin(grid_qr, 1, 0, "q_theta_dot", self.spn_q_theta_dot)
        add_grid_spin(grid_qr, 1, 1, "q_alpha_dot", self.spn_q_alpha_dot)
        add_grid_spin(grid_qr, 2, 0, "R", self.spn_r)
        grid_qr.addWidget(self.btn_apply_lqr, 2, 1)

        side.addWidget(grp_qr)

        grp_lqr = QGroupBox("Ganhos LQR calculados")
        grid_lqr = QGridLayout(grp_lqr)
        grid_lqr.setHorizontalSpacing(10)
        grid_lqr.setVerticalSpacing(6)

        self.spn_k_theta = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 6, 0.1, read_only=True)
        self.spn_k_alpha = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 6, 0.1, read_only=True)
        self.spn_k_theta_dot = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 6, 0.1, read_only=True)
        self.spn_k_alpha_dot = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 6, 0.1, read_only=True)

        add_grid_spin(grid_lqr, 0, 0, "k_theta", self.spn_k_theta)
        add_grid_spin(grid_lqr, 0, 1, "k_alpha", self.spn_k_alpha)
        add_grid_spin(grid_lqr, 1, 0, "k_theta_dot", self.spn_k_theta_dot)
        add_grid_spin(grid_lqr, 1, 1, "k_alpha_dot", self.spn_k_alpha_dot)

        side.addWidget(grp_lqr)

        grp_theta_ref = QGroupBox("Referência do braço θ")
        grid_theta_ref = QGridLayout(grp_theta_ref)
        grid_theta_ref.setHorizontalSpacing(10)
        grid_theta_ref.setVerticalSpacing(6)

        self.spn_theta_target_deg = setup_spin(QDoubleSpinBox(), -180.0, 180.0, 3, 5.0)
        self.spn_theta_target_deg.setValue(0.0)
        self.btn_apply_theta_ref = QPushButton("Aplicar θ ref")

        add_grid_spin(grid_theta_ref, 0, 0, "θ alvo (graus)", self.spn_theta_target_deg)
        grid_theta_ref.addWidget(self.btn_apply_theta_ref, 0, 1)

        side.addWidget(grp_theta_ref)

        grp_microsteps = QGroupBox("Microsteps do LQR")
        grid_microsteps = QGridLayout(grp_microsteps)
        grid_microsteps.setHorizontalSpacing(10)
        grid_microsteps.setVerticalSpacing(6)

        self.btn_microstep_auto = QPushButton("Auto microstep: ON")
        self.btn_microstep_auto.setCheckable(True)
        self.btn_microstep_auto.setChecked(True)

        self.cmb_manual_microsteps = QComboBox()
        self.cmb_manual_microsteps.addItems(["8", "16", "32", "64", "128", "256"])
        self.cmb_manual_microsteps.setCurrentText("256")

        self.btn_apply_microstep = QPushButton("Aplicar microstep")

        grid_microsteps.addWidget(QLabel("Controle"), 0, 0)
        grid_microsteps.addWidget(self.btn_microstep_auto, 0, 1, 1, 2)
        grid_microsteps.addWidget(QLabel("Manual"), 1, 0)
        grid_microsteps.addWidget(self.cmb_manual_microsteps, 1, 1)
        grid_microsteps.addWidget(self.btn_apply_microstep, 1, 2)

        side.addWidget(grp_microsteps)

        grp_obs_poles = QGroupBox("Observador reduzido: polos")
        grid_obs_poles = QGridLayout(grp_obs_poles)
        grid_obs_poles.setHorizontalSpacing(10)
        grid_obs_poles.setVerticalSpacing(6)

        self.spn_obs_p1 = setup_spin(QDoubleSpinBox(), -500.0, -1.0, 3, 1.0)
        self.spn_obs_p2 = setup_spin(QDoubleSpinBox(), -500.0, -1.0, 3, 1.0)
        self.btn_apply_observer = QPushButton("Calcular + Aplicar OBS")

        add_grid_spin(grid_obs_poles, 0, 0, "p1", self.spn_obs_p1)
        add_grid_spin(grid_obs_poles, 0, 1, "p2", self.spn_obs_p2)
        grid_obs_poles.addWidget(self.btn_apply_observer, 1, 0, 1, 2)

        side.addWidget(grp_obs_poles)

        grp_obs = QGroupBox("Matrizes OBS calculadas")
        grid_obs = QGridLayout(grp_obs)
        grid_obs.setHorizontalSpacing(8)
        grid_obs.setVerticalSpacing(6)

        self.spn_lr00 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_lr01 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_lr10 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_lr11 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)

        self.spn_f00 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_f01 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_f10 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_f11 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)

        self.spn_g00 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_g01 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_g10 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_g11 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)

        self.spn_h0 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)
        self.spn_h1 = setup_spin(QDoubleSpinBox(), -1000000.0, 1000000.0, 8, 0.01, read_only=True)

        grid_obs.addWidget(QLabel("matriz"), 0, 0)
        grid_obs.addWidget(QLabel("[0,0]"), 0, 1)
        grid_obs.addWidget(QLabel("[0,1]"), 0, 2)
        grid_obs.addWidget(QLabel("[1,0]"), 0, 3)
        grid_obs.addWidget(QLabel("[1,1]"), 0, 4)

        grid_obs.addWidget(QLabel("Lr"), 1, 0)
        grid_obs.addWidget(self.spn_lr00, 1, 1)
        grid_obs.addWidget(self.spn_lr01, 1, 2)
        grid_obs.addWidget(self.spn_lr10, 1, 3)
        grid_obs.addWidget(self.spn_lr11, 1, 4)

        grid_obs.addWidget(QLabel("F"), 2, 0)
        grid_obs.addWidget(self.spn_f00, 2, 1)
        grid_obs.addWidget(self.spn_f01, 2, 2)
        grid_obs.addWidget(self.spn_f10, 2, 3)
        grid_obs.addWidget(self.spn_f11, 2, 4)

        grid_obs.addWidget(QLabel("G"), 3, 0)
        grid_obs.addWidget(self.spn_g00, 3, 1)
        grid_obs.addWidget(self.spn_g01, 3, 2)
        grid_obs.addWidget(self.spn_g10, 3, 3)
        grid_obs.addWidget(self.spn_g11, 3, 4)

        grid_obs.addWidget(QLabel("H"), 4, 0)
        grid_obs.addWidget(QLabel("[0]"), 4, 1)
        grid_obs.addWidget(self.spn_h0, 4, 2)
        grid_obs.addWidget(QLabel("[1]"), 4, 3)
        grid_obs.addWidget(self.spn_h1, 4, 4)

        side.addWidget(grp_obs)

        btn_row3 = QHBoxLayout()
        self.btn_start_collect = QPushButton("Iniciar coleta")
        self.btn_stop_collect = QPushButton("Parar coleta")
        self.btn_stop_collect.setEnabled(False)

        btn_row3.addWidget(self.btn_start_collect)
        btn_row3.addWidget(self.btn_stop_collect)

        side.addLayout(btn_row3)

        self.btn_save_json = QPushButton("Salvar JSON")
        side.addWidget(self.btn_save_json)

        side.addWidget(QLabel("Log da serial:"))

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMinimumHeight(120)
        side.addWidget(self.txt_log, stretch=1)

        side.addWidget(QLabel("Comando manual:"))

        terminal_row = QHBoxLayout()
        self.edt_terminal_command = QLineEdit()
        self.edt_terminal_command.setPlaceholderText("Digite um comando serial, por exemplo: MS?")
        self.btn_send_terminal = QPushButton("Enviar")

        terminal_row.addWidget(self.edt_terminal_command, stretch=1)
        terminal_row.addWidget(self.btn_send_terminal)

        side.addLayout(terminal_row)
        side.addStretch(1)

        self.btn_disconnect.setEnabled(False)

        self.btn_refresh_ports.clicked.connect(self._refresh_serial_ports)
        self.btn_connect.clicked.connect(self.on_connect_serial)
        self.btn_disconnect.clicked.connect(self.on_disconnect_serial)
        self.btn_apply_lqr.clicked.connect(self.on_apply_lqr)
        self.btn_apply_theta_ref.clicked.connect(self.on_apply_theta_ref)
        self.btn_apply_microstep.clicked.connect(self.on_apply_microstep_control)
        self.btn_microstep_auto.clicked.connect(self.on_apply_microstep_control)
        self.btn_apply_observer.clicked.connect(self.on_apply_observer)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_rth.clicked.connect(self.on_rth)
        self.btn_calibrate_alpha.clicked.connect(self.on_calibrate_alpha)
        self.btn_calibrate_top.clicked.connect(self.on_calibrate_alpha_top)
        self.btn_send_terminal.clicked.connect(self.on_send_terminal_command)
        self.edt_terminal_command.returnPressed.connect(self.on_send_terminal_command)
        self.btn_save_json.clicked.connect(self.on_save_json)
        self.btn_start_collect.clicked.connect(self.on_start_collect)
        self.btn_stop_collect.clicked.connect(self.on_stop_collect)

    def _refresh_serial_ports(self):
        current = self.cmb_port.currentText()
        ports = list_serial_ports()

        self.cmb_port.clear()
        self.cmb_port.addItems(ports)

        if current and current in ports:
            self.cmb_port.setCurrentText(current)

        if not ports:
            self.lbl_serial_status.setText("Nenhuma porta encontrada")
        elif not serial_is_open():
            self.lbl_serial_status.setText("Desconectado")

    def _load_initial_config(self):
        cfg = load_config()
        lqr_qr = cfg.get("lqr_qr", {})
        lqr = cfg.get("lqr", {})
        theta_ref = cfg.get("theta_ref", {})
        microsteps = cfg.get("microsteps", {})
        observer = cfg.get("observer_poles", {})
        lim = cfg.get("limits", {})

        self.spn_q_theta.setValue(float(lqr_qr.get("q_theta", 5.0)))
        self.spn_q_alpha.setValue(float(lqr_qr.get("q_alpha", 300.0)))
        self.spn_q_theta_dot.setValue(float(lqr_qr.get("q_theta_dot", 1.0)))
        self.spn_q_alpha_dot.setValue(float(lqr_qr.get("q_alpha_dot", 20.0)))
        self.spn_r.setValue(float(lqr_qr.get("r", 1.0)))

        self.spn_k_theta.setValue(float(lqr.get("k_theta", -2.2038)))
        self.spn_k_alpha.setValue(float(lqr.get("k_alpha", 228.1777)))
        self.spn_k_theta_dot.setValue(float(lqr.get("k_theta_dot", -3.0616)))
        self.spn_k_alpha_dot.setValue(float(lqr.get("k_alpha_dot", 38.1739)))

        self.spn_obs_p1.setValue(float(observer.get("p1", -18.0)))
        self.spn_obs_p2.setValue(float(observer.get("p2", -28.0)))

        self.accel_max_rad_s2 = float(lim.get("accel_max_rad_s2", 30.0))

        theta_target_deg = float(
            theta_ref.get(
                "theta_target_deg",
                cfg.get("theta_target_deg", 0.0)
            )
        )

        self.spn_theta_target_deg.setValue(theta_target_deg)
        self.theta_ref_applied_deg = 0.0

        microstep_auto_enabled = bool(microsteps.get("auto_enabled", True))
        microstep_manual = int(microsteps.get("manual", 256))

        if str(microstep_manual) not in ["8", "16", "32", "64", "128", "256"]:
            microstep_manual = 256

        self.btn_microstep_auto.setChecked(microstep_auto_enabled)
        self.cmb_manual_microsteps.setCurrentText(str(microstep_manual))
        self._sync_microstep_ui()

        try:
            self._calculate_lqr_only()
            self._calculate_observer_only()
        except Exception as e:
            self.log_buf.append(f"[WARN] erro ao calcular valores iniciais: {e}")

    def _sample_is_valid(self, sample):
        checks = [
            "alpha_deg",
            "theta_deg",
            "alpha_dot_deg_s",
            "theta_dot_deg_s",
            "u_rad_s2",
            "vel_rad_s",
        ]

        for key in checks:
            value = sample.get(key, math.nan)

            if not math.isfinite(value):
                return False

        if abs(sample["alpha_deg"]) > 360.0:
            return False

        if abs(sample["theta_deg"]) > 720.0:
            return False

        if abs(sample["alpha_dot_deg_s"]) > 5000.0:
            return False

        if abs(sample["theta_dot_deg_s"]) > 5000.0:
            return False

        if abs(sample["u_rad_s2"]) > 1000.0:
            return False

        if abs(sample["vel_rad_s"]) > 100.0:
            return False

        return True

    def _start_reader_thread(self):
        def reader():
            while not self.stop_reader:
                if not serial_is_open():
                    time.sleep(0.1)
                    continue

                try:
                    if self.serial_command_active.is_set():
                        time.sleep(0.005)
                        continue

                    with self.serial_lock:
                        line = read_line()

                    if not line:
                        continue

                    if line.startswith("T,"):
                        sample = decode_telemetry(line)

                        if sample is None:
                            continue

                        if not self._sample_is_valid(sample):
                            continue

                        self.buf_t.append(sample["t_s"])
                        self.buf["A"].append(sample["alpha_deg"])
                        self.buf["TH"].append(sample["theta_deg"])
                        self.buf["AD"].append(sample["alpha_dot_deg_s"])
                        self.buf["TD"].append(sample["theta_dot_deg_s"])
                        self.buf["U"].append(sample["u_rad_s2"])
                        self.buf["VEL"].append(sample["vel_rad_s"])
                        self.buf["EN"].append(sample["enabled"])

                        if self.top_calib_active:
                            with self.top_calib_lock:
                                self.top_calib_samples.append(
                                    (sample["alpha_deg"], sample["enabled"])
                                )

                        if self.is_collecting and self.collect_writer is not None:
                            if self.collect_start_t is None:
                                self.collect_start_t = sample["t_s"]

                            self.collect_writer.writerow([
                                sample["t_s"] - self.collect_start_t,
                                sample["alpha_deg"],
                                sample["theta_deg"],
                                sample["alpha_dot_deg_s"],
                                sample["theta_dot_deg_s"],
                                sample["u_rad_s2"],
                                sample["vel_rad_s"],
                                sample["enabled"],
                                self.collect_meta.get("q_theta", math.nan),
                                self.collect_meta.get("q_alpha", math.nan),
                                self.collect_meta.get("q_theta_dot", math.nan),
                                self.collect_meta.get("q_alpha_dot", math.nan),
                                self.collect_meta.get("r", math.nan),
                                self.collect_meta.get("k_theta", math.nan),
                                self.collect_meta.get("k_alpha", math.nan),
                                self.collect_meta.get("k_theta_dot", math.nan),
                                self.collect_meta.get("k_alpha_dot", math.nan),
                                self.collect_meta.get("theta_target_deg", math.nan),
                                self.collect_meta.get("microstep_auto_enabled", math.nan),
                                self.collect_meta.get("microstep_manual", math.nan),
                                self.collect_meta.get("observer_p1", math.nan),
                                self.collect_meta.get("observer_p2", math.nan),
                            ])

                            self.collect_file.flush()

                    else:
                        if len(self.log_buf) < 500:
                            self.log_buf.append(line)

                except Exception:
                    time.sleep(0.01)

        threading.Thread(target=reader, daemon=True).start()

    def _start_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(1000 / GUI_HZ))

    def _current_vals(self):
        return {
            "q_theta": self.spn_q_theta.value(),
            "q_alpha": self.spn_q_alpha.value(),
            "q_theta_dot": self.spn_q_theta_dot.value(),
            "q_alpha_dot": self.spn_q_alpha_dot.value(),
            "r": self.spn_r.value(),

            "k_theta": self.spn_k_theta.value(),
            "k_alpha": self.spn_k_alpha.value(),
            "k_theta_dot": self.spn_k_theta_dot.value(),
            "k_alpha_dot": self.spn_k_alpha_dot.value(),

            "theta_target_deg": self.spn_theta_target_deg.value(),

            "microstep_auto_enabled": self.btn_microstep_auto.isChecked(),
            "microstep_manual": int(self.cmb_manual_microsteps.currentText()),

            "observer_p1": self.spn_obs_p1.value(),
            "observer_p2": self.spn_obs_p2.value(),

            "accel_max_rad_s2": self.accel_max_rad_s2,
        }

    def _log(self, text):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_buf.append(f"[{ts}] {text}")

    def _calculate_lqr_only(self):
        k = compute_lqr_gain(
            self.spn_q_theta.value(),
            self.spn_q_alpha.value(),
            self.spn_q_theta_dot.value(),
            self.spn_q_alpha_dot.value(),
            self.spn_r.value(),
        )

        self.spn_k_theta.setValue(float(k[0]))
        self.spn_k_alpha.setValue(float(k[1]))
        self.spn_k_theta_dot.setValue(float(k[2]))
        self.spn_k_alpha_dot.setValue(float(k[3]))

        self.last_lqr_cmd = (
            f"LQR {k[0]:.6f} {k[1]:.6f} "
            f"{k[2]:.6f} {k[3]:.6f}"
        )

        return k

    def _calculate_observer_only(self):
        matrices = observer_matrices_from_poles(
            self.spn_obs_p1.value(),
            self.spn_obs_p2.value(),
        )

        self.spn_lr00.setValue(float(matrices.Lr[0, 0]))
        self.spn_lr01.setValue(float(matrices.Lr[0, 1]))
        self.spn_lr10.setValue(float(matrices.Lr[1, 0]))
        self.spn_lr11.setValue(float(matrices.Lr[1, 1]))

        self.spn_f00.setValue(float(matrices.F[0, 0]))
        self.spn_f01.setValue(float(matrices.F[0, 1]))
        self.spn_f10.setValue(float(matrices.F[1, 0]))
        self.spn_f11.setValue(float(matrices.F[1, 1]))

        self.spn_g00.setValue(float(matrices.G[0, 0]))
        self.spn_g01.setValue(float(matrices.G[0, 1]))
        self.spn_g10.setValue(float(matrices.G[1, 0]))
        self.spn_g11.setValue(float(matrices.G[1, 1]))

        self.spn_h0.setValue(float(matrices.H[0]))
        self.spn_h1.setValue(float(matrices.H[1]))

        self.last_obs_label = matrices.label

        return matrices

    def _sync_microstep_ui(self):
        auto_enabled = self.btn_microstep_auto.isChecked()
        self.btn_microstep_auto.setText(
            "Auto microstep: ON" if auto_enabled else "Auto microstep: OFF"
        )
        self.cmb_manual_microsteps.setEnabled(not auto_enabled)

    def _set_command_buttons_enabled(self, enabled):
        self.btn_apply_lqr.setEnabled(enabled)
        self.btn_apply_theta_ref.setEnabled(enabled)
        self.btn_apply_observer.setEnabled(enabled)
        self.btn_apply_microstep.setEnabled(enabled)
        self.btn_microstep_auto.setEnabled(enabled)
        self.btn_start.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)
        self.btn_rth.setEnabled(enabled)
        self.btn_calibrate_alpha.setEnabled(enabled)
        self.btn_calibrate_top.setEnabled(enabled and not self.top_calib_active)
        self.btn_send_terminal.setEnabled(enabled)
        self.edt_terminal_command.setEnabled(enabled)
        self.btn_save_json.setEnabled(enabled)

        if enabled:
            self._sync_microstep_ui()
        else:
            self.cmb_manual_microsteps.setEnabled(False)

    def _run_serial_commands_async(self, commands, error_title, on_success=None):
        if self.cmd_thread is not None:
            QMessageBox.warning(
                self,
                "Comando em andamento",
                "Aguarde o comando atual terminar."
            )
            return

        self.serial_command_active.set()
        self._set_command_buttons_enabled(False)

        self.cmd_thread = QThread(self)
        self.cmd_worker = SerialCommandWorker(commands, self.serial_lock)

        self.cmd_worker.moveToThread(self.cmd_thread)

        self.cmd_thread.started.connect(self.cmd_worker.run)
        self.cmd_worker.log.connect(self._log)

        # O resultado precisa ser tratado na thread da interface: conectar a
        # metodos da janela (e nao a funcoes locais) faz o Qt entregar o sinal
        # na thread principal. Antes, o callback rodava na thread serial e
        # abrir uma janela (QMessageBox) ali derrubava o programa.
        self._cmd_on_success = on_success
        self._cmd_error_title = error_title

        self.cmd_worker.finished.connect(self._on_cmd_finished)
        self.cmd_worker.finished.connect(self.cmd_worker.deleteLater)
        self.cmd_thread.finished.connect(self.cmd_thread.deleteLater)
        self.cmd_thread.finished.connect(self._on_cmd_thread_finished)

        self.cmd_thread.start()

    @Slot(bool, str)
    def _on_cmd_finished(self, ok, message):
        self.serial_command_active.clear()
        self._set_command_buttons_enabled(True)

        on_success = self._cmd_on_success
        error_title = self._cmd_error_title
        self._cmd_on_success = None

        if self.cmd_thread is not None:
            self.cmd_thread.quit()

        if ok:
            if on_success is not None:
                on_success()
        else:
            QMessageBox.critical(self, error_title, message)

    @Slot()
    def _on_cmd_thread_finished(self):
        self.cmd_thread = None
        self.cmd_worker = None

    def _send_and_log(self, cmd, prefix):
        self._log(f"[ENVIO] {cmd}")
        with self.serial_lock:
            send_line(cmd)

        with self.serial_lock:
            resp = read_until_prefix(prefix)

        if resp:
            self._log(f"[RESP] {resp}")
        else:
            self._log(f"[WARN] sem confirmação para {prefix}")

        return resp

    def on_connect_serial(self):
        port = self.cmb_port.currentText().strip()

        if not port:
            QMessageBox.warning(self, "Serial", "Selecione uma porta serial.")
            return

        try:
            open_serial(port)
            self.lbl_serial_status.setText(f"Conectado em {port}")
            self._log(f"[INFO] conectado em {port}")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)

        except Exception as e:
            self.lbl_serial_status.setText("Erro ao conectar")
            QMessageBox.critical(
                self,
                "Serial",
                f"Não foi possível abrir a porta {port}.\n\n{e}"
            )

    def on_disconnect_serial(self):
        if self.cmd_thread is not None:
            QMessageBox.warning(
                self,
                "Serial",
                "Aguarde o comando atual terminar antes de desconectar."
            )
            return

        try:
            if self.is_collecting:
                self.on_stop_collect()

            port = self.cmb_port.currentText().strip()
            close_serial()

            self.lbl_serial_status.setText("Desconectado")
            self._log(
                f"[INFO] desconectado de {port}"
                if port
                else "[INFO] serial desconectada"
            )
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)

        except Exception as e:
            QMessageBox.critical(self, "Serial", f"Erro ao desconectar.\n\n{e}")

    def on_apply_lqr(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            k = self._calculate_lqr_only()
            vals = self._current_vals()

            self._log(
                "[CALC] Q=diag("
                f"{vals['q_theta']:.3f}, {vals['q_alpha']:.3f}, "
                f"{vals['q_theta_dot']:.3f}, {vals['q_alpha_dot']:.3f}), "
                f"R={vals['r']:.3f}"
            )
            self._log(
                f"[CALC] K=[{k[0]:.6f}, {k[1]:.6f}, {k[2]:.6f}, {k[3]:.6f}]"
            )

            def after_success():
                save_config(self._current_vals())

            self._run_serial_commands_async(
                [(self.last_lqr_cmd, "LQR")],
                "Erro LQR",
                after_success
            )

        except Exception as e:
            QMessageBox.critical(self, "Erro LQR", str(e))

    def on_apply_observer(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            matrices = self._calculate_observer_only()
            vals = self._current_vals()

            self._log(f"[CALC] OBS {matrices.label}")

            commands = [
                (matrices.cmd_olr(), "OLR"),
                (matrices.cmd_of(), "OF"),
                (matrices.cmd_og(), "OG"),
                (matrices.cmd_oh(), "OH"),
                ("OAPPLY", "OAPPLY"),
            ]

            def after_success():
                self._log(
                    "[TROCA] OBS aplicado manualmente | "
                    f"{matrices.label}"
                )
                save_config(vals)

            self._run_serial_commands_async(
                commands,
                "Erro OBS",
                after_success
            )

        except Exception as e:
            QMessageBox.critical(self, "Erro OBS", str(e))

    def on_apply_theta_ref(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            vals = self._current_vals()
            theta_deg = vals["theta_target_deg"]
            cmd = f"TH {theta_deg:.3f}"

            def after_success():
                self.theta_ref_applied_deg = theta_deg
                # A coleta registra o alvo atual, nao o do inicio da coleta
                if self.is_collecting:
                    self.collect_meta["theta_target_deg"] = theta_deg
                save_config(vals)

            self._run_serial_commands_async(
                [(cmd, "TH")],
                "Erro",
                after_success
            )

        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_apply_microstep_control(self):
        self._sync_microstep_ui()

        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            vals = self._current_vals()
            auto_enabled = bool(vals["microstep_auto_enabled"])
            manual_microsteps = int(vals["microstep_manual"])

            commands = [
                (f"MSMAN {manual_microsteps}", "MSMAN"),
                (f"MSAUTO {1 if auto_enabled else 0}", "MSAUTO"),
                ("MS?", "MS"),
            ]

            def after_success():
                save_config(vals)
                self._sync_microstep_ui()
                self._log(
                    "[TROCA] microstep aplicado | "
                    f"auto={1 if auto_enabled else 0} manual={manual_microsteps}"
                )

            self._run_serial_commands_async(
                commands,
                "Erro microstep",
                after_success
            )

        except Exception as e:
            QMessageBox.critical(self, "Erro microstep", str(e))

    def on_start(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        try:
            self._calculate_lqr_only()
            matrices = self._calculate_observer_only()
            vals = self._current_vals()

            manual_microsteps = int(vals["microstep_manual"])
            auto_enabled = bool(vals["microstep_auto_enabled"])

            commands = [
                (f"MSMAN {manual_microsteps}", "MSMAN"),
                (f"MSAUTO {1 if auto_enabled else 0}", "MSAUTO"),
                (self.last_lqr_cmd, "LQR"),
                (matrices.cmd_olr(), "OLR"),
                (matrices.cmd_of(), "OF"),
                (matrices.cmd_og(), "OG"),
                (matrices.cmd_oh(), "OH"),
                ("OAPPLY", "OAPPLY"),
                ("START", "START"),
            ]

            def after_success():
                save_config(vals)
                self.theta_ref_applied_deg = 0.0

            self._run_serial_commands_async(
                commands,
                "Erro",
                after_success
            )

        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_stop(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        self._run_serial_commands_async(
            [("STOP", "STOP")],
            "Erro"
        )

    def on_rth(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        def after_success():
            self.theta_ref_applied_deg = 0.0

        self._run_serial_commands_async(
            [("RTH", "RTH")],
            "Erro",
            after_success
        )

    def on_calibrate_alpha(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        answer = QMessageBox.question(
            self,
            "Calibração do pêndulo",
            "Coloque o pêndulo parado na posição inferior (180°).\n\n"
            "A calibração interrompe o controle e salva o novo zero absoluto. "
            "Depois, pressione INICIAR novamente.\n\n"
            "Deseja calibrar agora?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if answer != QMessageBox.Yes:
            return

        def after_success():
            self._log("[CALIB] zero absoluto do pêndulo salvo")

        self._run_serial_commands_async(
            [("CALIB", "CALIB")],
            "Erro de calibração",
            after_success
        )

    def on_calibrate_alpha_top(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        if self.top_calib_active:
            return

        en = list(self.buf["EN"])[-10:]

        if not en or not all(int(e) == 1 for e in en):
            QMessageBox.warning(
                self,
                "Calibração no topo",
                "O pêndulo precisa estar equilibrado pelo LQR (controle ativo).\n\n"
                "Aperte INICIAR, deixe o pêndulo equilibrar e tente de novo."
            )
            return

        with self.top_calib_lock:
            self.top_calib_samples = []
            self.top_calib_active = True

        self.btn_calibrate_top.setEnabled(False)
        self._log(
            f"[CALIB TOPO] medindo α médio por {TOP_CALIB_SEC:.0f} s... "
            "não toque no pêndulo"
        )

        QTimer.singleShot(
            int(TOP_CALIB_SEC * 1000),
            self._finish_top_calibration
        )

    def _finish_top_calibration(self):
        with self.top_calib_lock:
            samples = list(self.top_calib_samples)
            self.top_calib_samples = []
            self.top_calib_active = False

        self.btn_calibrate_top.setEnabled(serial_is_open())

        expected = TOP_CALIB_SEC * HZ
        alphas = [a for a, _ in samples]
        enabled = [int(e) for _, e in samples]

        def fail(msg):
            self._log(f"[CALIB TOPO] cancelada: {msg}")
            QMessageBox.warning(self, "Calibração no topo", msg)

        if len(alphas) < 0.6 * expected:
            fail(
                f"Poucas amostras de telemetria ({len(alphas)} de ~{expected:.0f}). "
                "Verifique a conexão e tente de novo."
            )
            return

        if not all(e == 1 for e in enabled):
            fail("O controle desligou durante a medição (pêndulo caiu?).")
            return

        n = len(alphas)
        mean_deg = sum(alphas) / n
        std_deg = math.sqrt(sum((a - mean_deg) ** 2 for a in alphas) / n)

        if std_deg > TOP_CALIB_MAX_STD_DEG:
            fail(
                f"α variou demais (desvio {std_deg:.2f}°). "
                "Espere o pêndulo estabilizar e não toque nele."
            )
            return

        if abs(mean_deg) > TOP_CALIB_MAX_MEAN_DEG:
            fail(
                f"α médio de {mean_deg:.2f}° é grande demais para um ajuste. "
                "Faça primeiro a calibração por baixo (Calibrar α)."
            )
            return

        self._log(
            f"[CALIB TOPO] α médio={mean_deg:+.4f}°  desvio={std_deg:.3f}°  "
            f"amostras={n}"
        )

        def after_success():
            self._log(
                f"[CALIB TOPO] zero ajustado em {mean_deg:+.4f}°. "
                "O controle passa ao novo zero aos poucos; "
                "aperte STOP para gravar na flash."
            )
            QMessageBox.information(
                self,
                "Calibração no topo",
                f"Zero ajustado em {mean_deg:+.3f}° (desvio {std_deg:.3f}°, {n} amostras).\n\n"
                "O novo zero entra aos poucos (~0,2°/s).\n"
                "Ele será gravado na memória do ESP32 quando você apertar STOP.\n\n"
                "Se quiser refinar, espere alguns segundos e calibre de novo."
            )

        self._run_serial_commands_async(
            [(f"ZTOP {mean_deg:.4f}", "ZTOP")],
            "Erro na calibração no topo",
            after_success
        )

    def on_send_terminal_command(self):
        if not serial_is_open():
            QMessageBox.warning(self, "Serial", "Conecte a porta serial primeiro.")
            return

        if self.cmd_thread is not None:
            QMessageBox.warning(
                self,
                "Comando em andamento",
                "Aguarde o comando atual terminar."
            )
            return

        command = self.edt_terminal_command.text().strip()

        if not command:
            return

        try:
            self._log(f"[TERMINAL] > {command}")

            with self.serial_lock:
                send_line(command)

            self.edt_terminal_command.clear()

        except Exception as e:
            QMessageBox.critical(self, "Erro no terminal", str(e))

    def on_save_json(self):
        try:
            self._calculate_lqr_only()
            self._calculate_observer_only()
            save_config(self._current_vals())
            self._log("[INFO] configuração salva.")
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def on_start_collect(self):
        if self.is_collecting:
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = DATA_DIR / f"coleta_lqr_{ts}.csv"

        try:
            self._calculate_lqr_only()
            self._calculate_observer_only()

            self.collect_meta = self._current_vals()
            self.collect_file = filepath.open("w", newline="", encoding="utf-8")
            self.collect_writer = csv.writer(self.collect_file)

            self.collect_writer.writerow([
                "t_s",
                "alpha_deg",
                "theta_deg",
                "alpha_dot_deg_s",
                "theta_dot_deg_s",
                "u_rad_s2",
                "vel_rad_s",
                "enabled",
                "q_theta",
                "q_alpha",
                "q_theta_dot",
                "q_alpha_dot",
                "r",
                "k_theta",
                "k_alpha",
                "k_theta_dot",
                "k_alpha_dot",
                "theta_target_deg",
                "microstep_auto_enabled",
                "microstep_manual",
                "observer_p1",
                "observer_p2",
            ])

            self.collect_start_t = None
            self.is_collecting = True

            self.btn_start_collect.setEnabled(False)
            self.btn_stop_collect.setEnabled(True)

            self._log(f"[COLETA] iniciada: {filepath.name}")

        except Exception as e:
            self.is_collecting = False
            self.collect_writer = None
            self.collect_file = None
            self.collect_start_t = None
            self.collect_meta = {}

            QMessageBox.critical(
                self,
                "Erro",
                f"Não foi possível iniciar a coleta.\n\n{e}"
            )

    def on_stop_collect(self):
        if not self.is_collecting:
            return

        try:
            self.is_collecting = False
            self.collect_writer = None

            if self.collect_file is not None:
                self.collect_file.close()

            self.collect_file = None
            self.collect_start_t = None
            self.collect_meta = {}

            self.btn_start_collect.setEnabled(True)
            self.btn_stop_collect.setEnabled(False)

            self._log("[COLETA] finalizada")

        except Exception as e:
            QMessageBox.critical(self, "Erro", f"Erro ao finalizar coleta.\n\n{e}")

    def _hud_pos(self, plot):
        (xmin, xmax), (ymin, ymax) = plot.viewRange()

        xr = (xmax - xmin) if (xmax > xmin) else 1.0
        yr = (ymax - ymin) if (ymax > ymin) else 1.0

        return xmin + 0.02 * xr, ymax - 0.05 * yr

    def on_timer(self):
        if self.buf_t:
            t1 = self.buf_t[-1]
            t0 = max(self.buf_t[0], t1 - WIN_SEC)

            for p in [self.p1, self.p3]:
                p.setXRange(t0, t1, padding=0.0)

            t_list = list(self.buf_t)
            alpha = list(self.buf["A"])
            theta = list(self.buf["TH"])
            alpha_dot = list(self.buf["AD"])
            theta_dot = list(self.buf["TD"])
            u = list(self.buf["U"])
            vel = list(self.buf["VEL"])
            en = list(self.buf["EN"])

            theta_ref = self.theta_ref_applied_deg
            theta_ref_list = [theta_ref] * len(t_list)

            self.cur_alpha.setData(t_list, alpha)
            self.cur_theta.setData(t_list, theta)
            self.cur_theta_ref.setData(t_list, theta_ref_list)

            self.cur_u.setData(t_list, u)
            self.cur_vel.setData(t_list, vel)

            alpha_last = alpha[-1] if alpha else math.nan
            theta_last = theta[-1] if theta else math.nan
            alpha_dot_last = alpha_dot[-1] if alpha_dot else math.nan
            theta_dot_last = theta_dot[-1] if theta_dot else math.nan
            u_last = u[-1] if u else math.nan
            vel_last = vel[-1] if vel else math.nan
            en_last = en[-1] if en else 0

            x1, y1 = self._hud_pos(self.p1)
            self.hud_p1.setPos(x1, y1)
            self.hud_p1.setText(
                hud_text([
                    f"α = {alpha_last: .2f}°",
                    f"θ = {theta_last: .2f}°",
                    f"θref = {theta_ref: .2f}°",
                ])
            )

            alpha_dot_abs_last = (
                abs(alpha_dot_last)
                if math.isfinite(alpha_dot_last)
                else 0.0
            )

            alpha_dot_low = self._trapmf(
                alpha_dot_abs_last,
                0.0,
                0.0,
                FUZZY_ALPHA_DOT_LOW_FULL_DEG_S,
                FUZZY_ALPHA_DOT_LOW_END_DEG_S,
            )

            alpha_dot_medium = self._trapmf(
                alpha_dot_abs_last,
                FUZZY_ALPHA_DOT_MED_START_DEG_S,
                FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S,
                FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S,
                FUZZY_ALPHA_DOT_MED_END_DEG_S,
            )

            alpha_dot_high = self._trapmf(
                alpha_dot_abs_last,
                FUZZY_ALPHA_DOT_HIGH_START_DEG_S,
                FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S,
                FUZZY_ALPHA_DOT_MAX_DEG_S,
                FUZZY_ALPHA_DOT_MAX_DEG_S,
            )

            self.line_alpha_dot_now.setPos(alpha_dot_abs_last)

            x2, y2 = self._hud_pos(self.p2)
            self.hud_p2.setPos(x2, y2)
            self.hud_p2.setText(
                hud_text([
                    f"|α_dot| = {alpha_dot_abs_last: .2f}°/s",
                    f"baixa = {alpha_dot_low: .2f}",
                    f"média = {alpha_dot_medium: .2f}",
                    f"alta = {alpha_dot_high: .2f}",
                    f"θ_dot = {theta_dot_last: .2f}°/s",
                ])
            )

            x3, y3 = self._hud_pos(self.p3)
            self.hud_p3.setPos(x3, y3)
            self.hud_p3.setText(
                hud_text([
                    f"u = {u_last: .2f} rad/s²",
                    f"vel = {vel_last: .2f} rad/s",
                    f"OBS = {self.last_obs_label}",
                    f"EN = {en_last}",
                ])
            )

        flush = 0

        while self.log_buf and flush < 8:
            self.txt_log.append(self.log_buf.popleft())
            flush += 1

    def closeEvent(self, event):
        self.stop_reader = True

        if self.is_collecting:
            try:
                self.on_stop_collect()
            except Exception:
                pass

        if self.cmd_thread is not None:
            self.serial_command_active.clear()
            self.cmd_thread.quit()
            self.cmd_thread.wait(1000)

        close_serial()
        event.accept()


if __name__ == "__main__":
    app = QApplication([])
    w = LQRTunerWindow()
    w.resize(1500, 900)
    w.show()
    app.exec()
