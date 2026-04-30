#!/usr/bin/env python3
"""
EDFA Lab Bench GUI
PyQt5 interface for FiberLabs Desktop Optical Fibre Amplifier

Architecture:
    The serial port lives on a dedicated worker thread (EDFAWorker).
    The GUI never blocks on I/O. User actions are posted to the worker's
    command queue; monitoring polls run on the worker's own QTimer.
    Results come back to the GUI via Qt signals.

Requires:
    PyQt5, pyserial, edfa_controller.py (same directory)
"""

import sys
import queue
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

from PyQt5.QtCore import Qt, QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QComboBox,
    QDoubleSpinBox,
    QLineEdit,
    QStatusBar,
    QMessageBox,
)

from .edfa_controller import EDFAController, DrivingMode

# ==================================================================
# Worker-thread command objects
# ==================================================================


@dataclass
class _Cmd:
    """Base class for commands posted to the worker."""

    pass


@dataclass
class CmdConnect(_Cmd):
    port: str
    baudrate: int
    delimiter: str


@dataclass
class CmdDisconnect(_Cmd):
    pass


@dataclass
class CmdSetOutput(_Cmd):
    active: bool


@dataclass
class CmdSetMode(_Cmd):
    channel: int
    mode: DrivingMode


@dataclass
class CmdSetALC(_Cmd):
    channel: int
    level_dbm: float


@dataclass
class CmdSetACC(_Cmd):
    channel: int
    current_ma: float


@dataclass
class CmdRefreshModes(_Cmd):
    pass


@dataclass
class CmdStop(_Cmd):
    """Terminate the worker thread."""

    pass


# ==================================================================
# Worker: owns the serial port, runs on its own thread
# ==================================================================


class EDFAWorker(QObject):
    """
    Runs on a QThread. Owns the EDFAController exclusively — no other thread
    ever touches the serial port. Communication happens via:

      - inbound: `post(cmd)` pushes a _Cmd onto an internal queue
      - outbound: Qt signals emitted when work completes or readings arrive

    The worker keeps a QTimer for 1 Hz monitoring polls. Both the poll
    tick and the command-queue drain happen on this thread, so they can't
    collide — no mutex needed.
    """

    # Outbound signals (all cross-thread, delivered to GUI via event loop)
    connected = pyqtSignal(str)  # idn string
    disconnected = pyqtSignal()
    error = pyqtSignal(str, str)  # title, message
    output_state_changed = pyqtSignal(bool)
    mode_updated = pyqtSignal(int, object)  # channel, DrivingMode
    status_message = pyqtSignal(str, int)  # msg, timeout_ms
    telemetry = pyqtSignal(dict)  # see _poll()
    alarms = pyqtSignal(dict)  # {'OUT': bool, ...}

    POLL_INTERVAL_MS = 1000
    CMD_DRAIN_INTERVAL_MS = 50

    def __init__(self, num_channels: int = 2):
        super().__init__()
        self._num_channels = num_channels
        self._edfa: Optional[EDFAController] = None
        self._queue: "queue.Queue[_Cmd]" = queue.Queue()
        self._poll_timer: Optional[QTimer] = None
        self._drain_timer: Optional[QTimer] = None

    # -----------------------------------------------------------------
    # Lifecycle — called once the thread's event loop is running
    # -----------------------------------------------------------------

    @pyqtSlot()
    def start(self):
        self._drain_timer = QTimer()
        self._drain_timer.timeout.connect(self._drain_queue)
        self._drain_timer.start(self.CMD_DRAIN_INTERVAL_MS)

        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._poll)
        # Poll timer is started only after a successful connection

    def post(self, cmd: _Cmd):
        """Thread-safe: just pushes onto the internal queue."""
        self._queue.put(cmd)

    # -----------------------------------------------------------------
    # Command queue drain
    # -----------------------------------------------------------------

    def _drain_queue(self):
        while True:
            try:
                cmd = self._queue.get_nowait()
            except queue.Empty:
                return
            self._handle(cmd)

    def _handle(self, cmd: _Cmd):
        try:
            if isinstance(cmd, CmdConnect):
                self._do_connect(cmd)
            elif isinstance(cmd, CmdDisconnect):
                self._do_disconnect()
            elif isinstance(cmd, CmdSetOutput):
                self._do_set_output(cmd.active)
            elif isinstance(cmd, CmdSetMode):
                self._do_set_mode(cmd.channel, cmd.mode)
            elif isinstance(cmd, CmdSetALC):
                self._do_set_alc(cmd.channel, cmd.level_dbm)
            elif isinstance(cmd, CmdSetACC):
                self._do_set_acc(cmd.channel, cmd.current_ma)
            elif isinstance(cmd, CmdRefreshModes):
                self._do_refresh_modes()
            elif isinstance(cmd, CmdStop):
                self._do_stop()
        except Exception as e:
            self.error.emit(type(cmd).__name__, str(e))

    # -----------------------------------------------------------------
    # Command implementations (all run on worker thread)
    # -----------------------------------------------------------------

    def _do_connect(self, cmd: CmdConnect):
        if self._edfa is not None:
            self._do_disconnect()

        edfa = EDFAController(
            port=cmd.port, baudrate=cmd.baudrate, delimiter=cmd.delimiter
        )
        if not edfa.connect():
            self.error.emit(
                "Connection failed",
                f"Could not open {cmd.port} at {cmd.baudrate} baud.",
            )
            return

        # Probe with *IDN? — if this fails, baud or delimiter is likely wrong
        try:
            idn = edfa.get_identification()
        except Exception as e:
            edfa.disconnect()
            self.error.emit(
                "No response from device",
                f"Connected to {cmd.port} but no reply.\n\n{e}\n\n"
                "Check baud rate and delimiter (CR vs LF).",
            )
            return

        self._edfa = edfa
        self.connected.emit(idn)
        self._do_refresh_modes()
        self._poll()
        if self._poll_timer is not None:
            self._poll_timer.start(self.POLL_INTERVAL_MS)

    def _do_disconnect(self):
        if self._poll_timer is not None:
            self._poll_timer.stop()
        if self._edfa is not None:
            try:
                self._edfa.set_output_active(False)
            except Exception:
                pass
            self._edfa.disconnect()
            self._edfa = None
        self.disconnected.emit()

    def _do_set_output(self, active: bool):
        if self._edfa is None:
            return
        self._edfa.set_output_active(active)
        self.output_state_changed.emit(active)
        self.status_message.emit("Output ACTIVE" if active else "Output off", 3000)

    def _do_set_mode(self, channel: int, mode: DrivingMode):
        if self._edfa is None:
            return
        self._edfa.set_driving_mode(channel, mode)
        # Per manual, changing mode turns output OFF
        self.output_state_changed.emit(False)
        self.mode_updated.emit(channel, mode)
        self.status_message.emit(f"Channel {channel} → {mode.name}", 3000)

    def _do_set_alc(self, channel: int, level_dbm: float):
        if self._edfa is None:
            return
        self._edfa.set_alc_output_level(channel, level_dbm)
        self.status_message.emit(f"Channel {channel} ALC = {level_dbm:.2f} dBm", 3000)

    def _do_set_acc(self, channel: int, current_ma: float):
        if self._edfa is None:
            return
        self._edfa.set_acc_current(channel, current_ma)
        self.status_message.emit(f"Channel {channel} ACC = {current_ma:.1f} mA", 3000)

    def _do_refresh_modes(self):
        if self._edfa is None:
            return
        for ch in range(1, self._num_channels + 1):
            try:
                mode = self._edfa.get_driving_mode(ch)
                self.mode_updated.emit(ch, mode)
            except Exception:
                # Channel not installed — silently skip
                pass

    def _do_stop(self):
        self._do_disconnect()
        if self._poll_timer is not None:
            self._poll_timer.stop()
        if self._drain_timer is not None:
            self._drain_timer.stop()

    # -----------------------------------------------------------------
    # Polling (worker thread)
    # -----------------------------------------------------------------

    def _poll(self):
        if self._edfa is None:
            return

        data: Dict[str, Any] = {
            "output_levels": [None] * self._num_channels,
            "ld_currents": [None] * self._num_channels,
            "ld_temps": [None] * self._num_channels,
            "case_temp": None,
        }

        # Each read wrapped individually so one missing optional monitor
        # doesn't abort the whole poll.
        try:
            data["output_levels"] = _pad(
                self._edfa.get_output_level(), self._num_channels
            )
        except Exception:
            pass

        try:
            currs = self._edfa.get_ld_current()
            if not isinstance(currs, list):
                currs = [currs]
            data["ld_currents"] = _pad(currs, self._num_channels)
        except Exception:
            pass

        try:
            temps = self._edfa.get_ld_temperature()
            if not isinstance(temps, list):
                temps = [temps]
            data["ld_temps"] = _pad(temps, self._num_channels)
        except Exception:
            pass

        try:
            data["case_temp"] = self._edfa.get_case_temperature()
        except Exception:
            pass

        self.telemetry.emit(data)

        try:
            self.alarms.emit(self._edfa.get_alarm_status())
        except Exception:
            self.alarms.emit({})  # GUI will show 'unknown'


def _pad(lst: List, n: int) -> List:
    """Pad/truncate lst to length n with None."""
    out = list(lst)[:n]
    while len(out) < n:
        out.append(None)
    return out


# ==================================================================
# Widgets
# ==================================================================


class StatusLED(QLabel):
    """Coloured indicator. Green = OK, red = alarm, grey = unknown."""

    COLOURS = {"ok": "#2ecc71", "alarm": "#e74c3c", "unknown": "#95a5a6"}

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self._label = label
        self.setFixedSize(80, 40)
        self.setAlignment(Qt.AlignCenter)
        self.set_state("unknown")

    def set_state(self, state: str):
        colour = self.COLOURS.get(state, self.COLOURS["unknown"])
        self.setStyleSheet(
            f"background-color: {colour};"
            "color: white;"
            "border-radius: 6px;"
            "font-weight: bold;"
            "padding: 4px;"
        )
        self.setText(self._label)


class ChannelWidget(QGroupBox):
    """Single pump-LD channel panel."""

    mode_change_requested = pyqtSignal(int, DrivingMode)
    setpoint_change_requested = pyqtSignal(int, DrivingMode, float)

    def __init__(self, channel: int, parent=None):
        super().__init__(f"Channel {channel}", parent)
        self.channel = channel
        self._suppress_signals = True

        layout = QGridLayout()
        layout.setColumnMinimumWidth(1, 120)

        layout.addWidget(QLabel("Mode:"), 0, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["ALC", "ACC"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        layout.addWidget(self.mode_combo, 0, 1)

        layout.addWidget(QLabel("Setpoint:"), 1, 0)
        self.setpoint_spin = QDoubleSpinBox()
        self.setpoint_spin.setRange(-30.0, 30.0)
        self.setpoint_spin.setSuffix(" dBm")
        self.setpoint_spin.setDecimals(2)
        layout.addWidget(self.setpoint_spin, 1, 1)

        self.apply_btn = QPushButton("Apply setpoint")
        self.apply_btn.clicked.connect(self._on_apply_clicked)
        layout.addWidget(self.apply_btn, 1, 2)

        mono = QFont("Monospace")
        mono.setStyleHint(QFont.TypeWriter)

        layout.addWidget(QLabel("LD current:"), 2, 0)
        self.current_lbl = QLabel("—  mA")
        self.current_lbl.setFont(mono)
        layout.addWidget(self.current_lbl, 2, 1)

        layout.addWidget(QLabel("LD temp:"), 3, 0)
        self.temp_lbl = QLabel("—  °C")
        self.temp_lbl.setFont(mono)
        layout.addWidget(self.temp_lbl, 3, 1)

        layout.addWidget(QLabel("Output:"), 4, 0)
        self.output_lbl = QLabel("—  dBm")
        self.output_lbl.setFont(mono)
        layout.addWidget(self.output_lbl, 4, 1)

        self.setLayout(layout)
        self._suppress_signals = False

    def _current_mode(self) -> DrivingMode:
        return (
            DrivingMode.ALC if self.mode_combo.currentIndex() == 0 else DrivingMode.ACC
        )

    def _on_mode_changed(self):
        if self._suppress_signals:
            return
        mode = self._current_mode()
        self._configure_setpoint_for_mode(mode)
        self.mode_change_requested.emit(self.channel, mode)

    def _on_apply_clicked(self):
        self.setpoint_change_requested.emit(
            self.channel, self._current_mode(), self.setpoint_spin.value()
        )

    def _configure_setpoint_for_mode(self, mode: DrivingMode):
        self.setpoint_spin.blockSignals(True)
        if mode == DrivingMode.ALC:
            self.setpoint_spin.setRange(-30.0, 30.0)
            self.setpoint_spin.setSuffix(" dBm")
            self.setpoint_spin.setDecimals(2)
            self.setpoint_spin.setSingleStep(0.5)
        else:
            self.setpoint_spin.setRange(0.0, 1500.0)
            self.setpoint_spin.setSuffix(" mA")
            self.setpoint_spin.setDecimals(1)
            self.setpoint_spin.setSingleStep(10.0)
        self.setpoint_spin.blockSignals(False)

    def update_mode(self, mode: DrivingMode):
        if self._current_mode() == mode:
            return
        self._suppress_signals = True
        self.mode_combo.setCurrentIndex(0 if mode == DrivingMode.ALC else 1)
        self._configure_setpoint_for_mode(mode)
        self._suppress_signals = False

    def update_readouts(self, current_ma, temp_c, output_dbm):
        self.current_lbl.setText(
            f"{current_ma:7.1f} mA" if current_ma is not None else "—  mA"
        )
        self.temp_lbl.setText(f"{temp_c:7.1f} °C" if temp_c is not None else "—  °C")
        self.output_lbl.setText(
            f"{output_dbm:7.2f} dBm" if output_dbm is not None else "—  dBm"
        )

    def set_enabled_controls(self, enabled: bool):
        self.mode_combo.setEnabled(enabled)
        self.setpoint_spin.setEnabled(enabled)
        self.apply_btn.setEnabled(enabled)


# ==================================================================
# Main window
# ==================================================================


class EDFAMainWindow(QMainWindow):

    def __init__(self, num_channels: int = 4):
        super().__init__()
        self.setWindowTitle("EDFA Lab Bench")
        self.num_channels = num_channels
        self._output_active = False
        self._connected = False

        self.channel_width = 240
        self.setMaximumWidth(self.channel_width * self.num_channels + 60)

        self._build_ui()
        self._start_worker()

    # -----------------------------------------------------------------
    # Worker thread setup
    # -----------------------------------------------------------------

    def _start_worker(self):
        self.thread = QThread(self)
        self.worker = EDFAWorker(num_channels=self.num_channels)
        self.worker.moveToThread(self.thread)

        # Start worker's timers once the thread's event loop is running
        self.thread.started.connect(self.worker.start)

        # Wire worker signals to GUI slots
        self.worker.connected.connect(self._on_worker_connected)
        self.worker.disconnected.connect(self._on_worker_disconnected)
        self.worker.error.connect(self._on_worker_error)
        self.worker.output_state_changed.connect(self._on_output_state_changed)
        self.worker.mode_updated.connect(self._on_mode_updated)
        self.worker.status_message.connect(
            lambda m, t: self.status_bar.showMessage(m, t)
        )
        self.worker.telemetry.connect(self._on_telemetry)
        self.worker.alarms.connect(self._on_alarms)

        self.thread.start()

    # -----------------------------------------------------------------
    # UI construction
    # -----------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout()
        main_layout.addWidget(self._build_connection_panel())
        main_layout.addWidget(self._build_master_controls())
        main_layout.addWidget(self._build_system_readouts())
        main_layout.addWidget(self._build_alarm_panel())
        main_layout.addWidget(self._build_channels_panel(), stretch=1)
        central.setLayout(main_layout)
        self.setCentralWidget(central)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Disconnected")

        self._set_controls_enabled(False)

    def _build_connection_panel(self) -> QGroupBox:
        box = QGroupBox("Connection")
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Port:"))
        self.port_edit = QLineEdit("COM6")
        self.port_edit.setMaximumWidth(160)
        layout.addWidget(self.port_edit)

        layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600"])
        self.baud_combo.setCurrentText("57600")
        layout.addWidget(self.baud_combo)

        layout.addWidget(QLabel("Delimiter:"))
        self.delim_combo = QComboBox()
        self.delim_combo.addItems(["CR", "LF"])
        layout.addWidget(self.delim_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        self.idn_lbl = QLabel("—")
        self.idn_lbl.setStyleSheet("color: #7f8c8d; font-style: italic;")
        layout.addWidget(self.idn_lbl, stretch=1)

        box.setLayout(layout)
        return box

    def _build_master_controls(self) -> QGroupBox:
        box = QGroupBox("Master")
        layout = QHBoxLayout()

        self.output_btn = QPushButton("OUTPUT OFF")
        self.output_btn.setCheckable(True)
        self.output_btn.setMinimumHeight(50)
        self.output_btn.setStyleSheet(self._output_btn_style(False))
        self.output_btn.clicked.connect(self._on_output_toggled)
        layout.addWidget(self.output_btn, stretch=4)

        self.emergency_btn = QPushButton("EMERGENCY OFF")
        self.emergency_btn.setMinimumHeight(50)
        self.emergency_btn.setStyleSheet(
            "background-color: #c0392b; color: white;"
            "font-weight: bold; font-size: 14px;"
        )
        self.emergency_btn.clicked.connect(self._on_emergency)
        layout.addWidget(self.emergency_btn, stretch=1)

        box.setLayout(layout)
        return box

    def _build_system_readouts(self) -> QGroupBox:
        box = QGroupBox("System")
        layout = QHBoxLayout()
        mono = QFont("Monospace")
        mono.setStyleHint(QFont.TypeWriter)
        layout.addWidget(QLabel("Case temp:"))
        self.case_temp_lbl = QLabel("—  °C")
        self.case_temp_lbl.setFont(mono)
        layout.addWidget(self.case_temp_lbl)
        layout.addStretch()
        box.setLayout(layout)
        return box

    def _build_alarm_panel(self) -> QGroupBox:
        box = QGroupBox("Alarms")
        layout = QHBoxLayout()
        self.alarm_leds = {
            "OUT": StatusLED("OUT"),
            "LDC": StatusLED("LDC"),
            "IN": StatusLED("IN"),
            "TEMP": StatusLED("TEMP"),
        }
        for led in self.alarm_leds.values():
            layout.addWidget(led)
        layout.addStretch()
        box.setLayout(layout)
        return box

    def _build_channels_panel(self) -> QGroupBox:
        box = QGroupBox("Channels")
        layout = QHBoxLayout()
        self.channel_widgets = []
        for ch in range(1, self.num_channels + 1):
            w = ChannelWidget(ch)
            w.mode_change_requested.connect(self._on_mode_change_requested)
            w.setpoint_change_requested.connect(self._on_setpoint_change_requested)
            self.channel_widgets.append(w)
            layout.addWidget(w)
        box.setLayout(layout)
        return box

    @staticmethod
    def _output_btn_style(active: bool) -> str:
        if active:
            return (
                "background-color: #27ae60; color: white;"
                "font-weight: bold; font-size: 14px;"
            )
        return (
            "background-color: #34495e; color: white;"
            "font-weight: bold; font-size: 14px;"
        )

    # -----------------------------------------------------------------
    # User actions (GUI → worker)
    # -----------------------------------------------------------------

    def _on_connect_clicked(self):
        if self._connected:
            self.worker.post(CmdDisconnect())
        else:
            self.worker.post(
                CmdConnect(
                    port=self.port_edit.text().strip(),
                    baudrate=int(self.baud_combo.currentText()),
                    delimiter=self.delim_combo.currentText(),
                )
            )
            self.status_bar.showMessage("Connecting…")

    def _on_output_toggled(self):
        if not self._connected:
            return
        self.worker.post(CmdSetOutput(active=self.output_btn.isChecked()))

    def _on_emergency(self):
        if not self._connected:
            return
        self.worker.post(CmdSetOutput(active=False))
        self.status_bar.showMessage("EMERGENCY STOP requested", 5000)

    def _on_mode_change_requested(self, channel: int, mode: DrivingMode):
        if not self._connected:
            return
        if self._output_active:
            reply = QMessageBox.question(
                self,
                "Mode change will disable output",
                f"Switching channel {channel} driving mode will turn the "
                "optical output OFF (per the FiberLabs manual).\n\nContinue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                # Ask worker to re-push actual modes so the combo snaps back
                self.worker.post(CmdRefreshModes())
                return
        self.worker.post(CmdSetMode(channel=channel, mode=mode))

    def _on_setpoint_change_requested(
        self, channel: int, mode: DrivingMode, value: float
    ):
        if not self._connected:
            return
        if mode == DrivingMode.ALC:
            self.worker.post(CmdSetALC(channel=channel, level_dbm=value))
        else:
            self.worker.post(CmdSetACC(channel=channel, current_ma=value))

    # -----------------------------------------------------------------
    # Worker signals → GUI state
    # -----------------------------------------------------------------

    @pyqtSlot(str)
    def _on_worker_connected(self, idn: str):
        self._connected = True
        self.idn_lbl.setText(idn)
        self.connect_btn.setText("Disconnect")
        self._set_controls_enabled(True)
        self.status_bar.showMessage(f"Connected: {idn}")

    @pyqtSlot()
    def _on_worker_disconnected(self):
        self._connected = False
        self._output_active = False
        self._update_output_button()
        self.idn_lbl.setText("—")
        self.connect_btn.setText("Connect")
        self._set_controls_enabled(False)
        self.case_temp_lbl.setText("—  °C")
        for w in self.channel_widgets:
            w.update_readouts(None, None, None)
        for led in self.alarm_leds.values():
            led.set_state("unknown")
        self.status_bar.showMessage("Disconnected")

    @pyqtSlot(str, str)
    def _on_worker_error(self, title: str, msg: str):
        QMessageBox.warning(self, title, msg)
        self.status_bar.showMessage(f"{title}: {msg}", 5000)

    @pyqtSlot(bool)
    def _on_output_state_changed(self, active: bool):
        self._output_active = active
        self._update_output_button()

    @pyqtSlot(int, object)
    def _on_mode_updated(self, channel: int, mode: DrivingMode):
        for w in self.channel_widgets:
            if w.channel == channel:
                w.update_mode(mode)
                break

    @pyqtSlot(dict)
    def _on_telemetry(self, data: dict):
        case = data.get("case_temp")
        if case is not None:
            self.case_temp_lbl.setText(f"{case:5.1f} °C")
        else:
            self.case_temp_lbl.setText("—  °C")

        outs = data.get("output_levels", [])
        currs = data.get("ld_currents", [])
        temps = data.get("ld_temps", [])

        for i, w in enumerate(self.channel_widgets):
            w.update_readouts(
                currs[i] if i < len(currs) else None,
                temps[i] if i < len(temps) else None,
                outs[i] if i < len(outs) else None,
            )

    @pyqtSlot(dict)
    def _on_alarms(self, alarms: dict):
        for name, led in self.alarm_leds.items():
            if not alarms:
                led.set_state("unknown")
            else:
                led.set_state("alarm" if alarms.get(name) else "ok")

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------

    def _update_output_button(self):
        active = self._output_active
        self.output_btn.blockSignals(True)
        self.output_btn.setChecked(active)
        self.output_btn.blockSignals(False)
        self.output_btn.setText("OUTPUT ON" if active else "OUTPUT OFF")
        self.output_btn.setStyleSheet(self._output_btn_style(active))

    def _set_controls_enabled(self, enabled: bool):
        self.output_btn.setEnabled(enabled)
        self.emergency_btn.setEnabled(enabled)
        for w in getattr(self, "channel_widgets", []):
            w.set_enabled_controls(enabled)

    # -----------------------------------------------------------------
    # Clean shutdown
    # -----------------------------------------------------------------

    def closeEvent(self, event):
        # Post stop, then tear down the thread
        self.worker.post(CmdStop())
        self.thread.quit()
        # Wait up to 3 s — covers a pending serial read with default 1 s timeout
        if not self.thread.wait(3000):
            self.thread.terminate()
            self.thread.wait(1000)
        super().closeEvent(event)


# ==================================================================
# Entry point
# ==================================================================


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = EDFAMainWindow(num_channels=2)
    window.resize(1100, 600)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
