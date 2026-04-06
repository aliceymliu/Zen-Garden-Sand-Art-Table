"""
- application main window for Zen Garden Sand Art Table PC GUI
- includes connection panel, mode selector, action buttons, sand canvas preview, status bar, and debug log
- communicates with Arduino via SerialThread, which runs in the background and emits signals for received lines
"""

import sys
from datetime import datetime
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QPushButton,
    QComboBox,
    QLabel,
    QSizePolicy,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor
 
from serial_thread import SerialThread
from sand_canvas import SandCanvas
 
 
#### UI STYLE SETTINGS ####
 
WINDOW_TITLE = "Zen Garden Sand Art Table"
WINDOW_MIN_W = 960
WINDOW_MIN_H = 640

STYLE_SHEET = """
QMainWindow {
    background-color: #F5F0E8;
}
QGroupBox {
    font-family: 'Segoe UI', sans-serif;
    font-size: 11pt;
    font-weight: 600;
    color: #5A4520;
    border: 1px solid #D4C4A8;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}
QPushButton {
    font-family: 'Segoe UI', sans-serif;
    font-size: 10pt;
    background-color: #E8DCC8;
    color: #4A3A1A;
    border: 1px solid #C4B490;
    border-radius: 4px;
    padding: 6px 14px;
    min-height: 28px;
}
QPushButton:hover {
    background-color: #DDD0B8;
    border-color: #A89870;
}
QPushButton:pressed {
    background-color: #C8BC9F;
}
QPushButton:checked {
    background-color: #8B7355;
    color: #F5F0E8;
    border-color: #6B5535;
}
QPushButton:disabled {
    background-color: #EDE8DD;
    color: #B0A890;
    border-color: #D8D0C0;
}
QPushButton#connectBtn {
    background-color: #6B8E5A;
    color: white;
    border-color: #5A7A4A;
}
QPushButton#connectBtn:hover {
    background-color: #7AA068;
}
QPushButton#disconnectBtn {
    background-color: #C0392B;
    color: white;
    border-color: #962D22;
}
QPushButton#disconnectBtn:hover {
    background-color: #E74C3C;
}
QPushButton#eraseBtn {
    background-color: #A0845C;
    color: white;
    border-color: #7A6040;
}
QPushButton#eraseBtn:hover {
    background-color: #B89468;
}
QPushButton#eraseBtn:disabled {
    background-color: #EDE8DD;
    color: #B0A890;
    border-color: #D8D0C0;
}
QComboBox {
    font-family: 'Segoe UI', sans-serif;
    font-size: 10pt;
    background-color: #FFFFFF;
    border: 1px solid #C4B490;
    border-radius: 4px;
    padding: 4px 8px;
    min-height: 26px;
}
QLabel {
    font-family: 'Segoe UI', sans-serif;
    font-size: 10pt;
    color: #5A4520;
}
QLabel#statusLabel {
    font-family: 'Consolas', monospace;
    font-size: 10pt;
    color: #6B5535;
    padding: 4px 8px;
    background-color: #EDE8DD;
    border: 1px solid #D4C4A8;
    border-radius: 4px;
}
"""
 
 
class MainWindow(QMainWindow):

    # Direction strings the Arduino sends in joystick mode
    _DIRECTIONS = {
        "RIGHT", "UPRIGHT30", "UPRIGHT45", "UPRIGHT60",
        "UP",
        "UPLEFT60", "UPLEFT45", "UPLEFT30",
        "LEFT",
        "DOWNLEFT30", "DOWNLEFT45", "DOWNLEFT60",
        "DOWN",
        "DOWNRIGHT60", "DOWNRIGHT45", "DOWNRIGHT30",
        "NEUTRAL",
    }
 
    def __init__(self):
        # Initialize the main window, build the UI, and connect signals
        super().__init__()
        self.setWindowTitle(WINDOW_TITLE)
        self.setMinimumSize(WINDOW_MIN_W, WINDOW_MIN_H)
        self.setStyleSheet(STYLE_SHEET)
 
        # Serial thread for communicating with the Arduino in the background
        self._serial       = SerialThread(self)
        self._connected    = False
        self._current_mode = None
        self._erasing      = False                  #true while erase is running on Arduino
 
        # Build UI
        central     = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(12)
 
        # Create canvas early — left panel buttons reference it
        self._canvas = SandCanvas()
        self._canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
 
        # --- Left panel: controls ---
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)
 
        left_panel.addWidget(self._build_connection_group())
        left_panel.addWidget(self._build_mode_group())
        left_panel.addWidget(self._build_actions_group())
        left_panel.addStretch()
 
        left_container = QWidget()
        left_container.setLayout(left_panel)
        left_container.setFixedWidth(260)
 
        # --- Right panel: canvas + status ---
        right_panel = QVBoxLayout()
        right_panel.setSpacing(8)
 
        canvas_group  = QGroupBox("Sand table preview")
        canvas_layout = QVBoxLayout(canvas_group)
        canvas_layout.addWidget(self._canvas)
 
        # Status labels for connection status, position, joystick direction, and other info from Arduino
        self._status_label = QLabel("Disconnected")
        self._status_label.setObjectName("statusLabel")
 
        self._pos_label = QLabel("Position: — , —")
        self._pos_label.setObjectName("statusLabel")
 
        self._joy_label = QLabel("Joystick: — , —")
        self._joy_label.setObjectName("statusLabel")
 
        status_row = QHBoxLayout()
        status_row.addWidget(self._status_label, stretch=2)
        status_row.addWidget(self._pos_label,    stretch=2)
        status_row.addWidget(self._joy_label,    stretch=2)
 
        right_panel.addWidget(canvas_group, stretch=1)
        right_panel.addLayout(status_row)
 
        right_container = QWidget()
        right_container.setLayout(right_panel)
 
        root_layout.addWidget(left_container)
        root_layout.addWidget(right_container, stretch=1)
 
        # --- Connect signals ---
        self._connect_signals()
 
        # --- Refresh timer for port list ---
        self._port_timer = QTimer(self)
        self._port_timer.timeout.connect(self._refresh_ports)
        self._port_timer.start(3000)
        self._refresh_ports()
 
    #### UI BUILDERS ####
    def _build_connection_group(self) -> QGroupBox:
        # Connection group with COM port selector, refresh button, and connect/disconnect buttons
        group  = QGroupBox("Connection")
        layout = QVBoxLayout(group)
 
        port_row = QHBoxLayout()
        self._port_combo = QComboBox()
        self._port_combo.setPlaceholderText("Select COM port...")
        self._refresh_btn = QPushButton("↻")
        self._refresh_btn.setToolTip("Refresh port list")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(self._port_combo, stretch=1)
        port_row.addWidget(self._refresh_btn)
        layout.addLayout(port_row)
 
        btn_row = QHBoxLayout()
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setObjectName("connectBtn")
        self._connect_btn.clicked.connect(self._on_connect)
        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.setObjectName("disconnectBtn")
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        btn_row.addWidget(self._connect_btn)
        btn_row.addWidget(self._disconnect_btn)
        layout.addLayout(btn_row)
 
        return group
 
    def _build_mode_group(self) -> QGroupBox:
        # Mode selector group with buttons for Clock, Joystick, and Coordinate modes

        group  = QGroupBox("Operating mode")
        layout = QVBoxLayout(group)
 
        # Mode buttons match the three modes in Two_Modes_Clock_PyCompatible.ino
        self._mode_buttons: dict[str, QPushButton] = {}
        for label, cmd in [
            ("Clock",      "SETMODE,CLOCK"),
            ("Joystick",   "SETMODE,JOYSTICK"),
            ("Coordinate", "SETMODE,COORDINATE"),
        ]:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setEnabled(False)
            btn.clicked.connect(lambda checked, c=cmd: self._on_mode_select(c))
            layout.addWidget(btn)
            self._mode_buttons[cmd] = btn
 
        return group
 
    def _build_actions_group(self) -> QGroupBox:
        # Action buttons for Home (calibrate), Erase (raster scan), and Clear preview (canvas only)
        group  = QGroupBox("Actions")
        layout = QVBoxLayout(group)
 
        # Home button: sends HOME command to Arduino, which runs home() on the firmware to calibrate position to (0,0)
        self._home_btn = QPushButton("Home (calibrate)")
        self._home_btn.setEnabled(False)
        self._home_btn.clicked.connect(self._on_home)
        layout.addWidget(self._home_btn)
 
        # Erase button: sends ERASE command to Arduino, which runs eraseCanvas() (full boustrophedon raster scan) on the firmware
        self._erase_btn = QPushButton("Erase sand")
        self._erase_btn.setObjectName("eraseBtn")
        self._erase_btn.setEnabled(False)
        self._erase_btn.setToolTip(
            "Raster-scan the full sandbox to wipe the current drawing"
        )
        self._erase_btn.clicked.connect(self._on_erase)
        layout.addWidget(self._erase_btn)
 
        # Clear canvas preview only (no Arduino command)
        self._clear_btn = QPushButton("Clear canvas preview")
        self._clear_btn.setToolTip(
            "Clears the trail on screen only — does not move the gantry"
        )
        self._clear_btn.clicked.connect(self._canvas.clear_trail)
        layout.addWidget(self._clear_btn)
 
        return group

    #### SIGNAL CONNECTIONS FOR SERIAL THREAD ####
    def _connect_signals(self):
        self._serial.line_received.connect(self._on_line_received)
        self._serial.connection_changed.connect(self._on_connection_changed)
 
    #### SIGNAL HANDLERS FOR ARDUINO RESPONSES ####
    def _on_line_received(self, line: str):
        # Handle every line received from the Arduino and update the UI accordingly
        upper = line.upper()
 
        if upper == "READY":
            self._status_label.setText("Status: READY — send Home to calibrate")
            self._home_btn.setEnabled(True)
            self._erase_btn.setEnabled(True)
            for btn in self._mode_buttons.values():
                btn.setEnabled(True)
 
        elif upper.startswith("POS,"):
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    self._canvas.update_position(x, y)
                    self._pos_label.setText(f"Position: {x:.1f}, {y:.1f}")
                except ValueError:
                    pass
 
        elif upper.startswith("OK,MODE,"):
            mode = line.split(",")[2].upper()
            self._set_active_mode_btn("SETMODE," + mode)
            self._canvas.clear_trail()
            if mode == "CLOCK":
                self._status_label.setText("Status: CLOCK mode")
            elif mode == "JOYSTICK":
                self._status_label.setText("Status: JOYSTICK mode")
            elif mode == "COORDINATE":
                self._status_label.setText("Status: COORDINATE mode")
 
        elif upper == "OK,HOME":
            self._status_label.setText("Status: IDLE — homed at (0,0)")
            self._canvas.clear_trail()
            self._set_actions_enabled(True)
 
        elif upper == "OK,STOP":
            self._status_label.setText("Status: STOPPED")
            self._erasing = False
            self._erase_btn.setEnabled(True)
 
        elif upper == "OK,ERASE":
            self._erasing = False
            self._erase_btn.setEnabled(True)
            self._status_label.setText("Status: Erase complete")
 
        elif upper.startswith("STATUS,"):
            parts = line.split(",")
            if len(parts) == 5:
                self._status_label.setText(
                    f"Status: {parts[4]}  pos known: {parts[1]}"
                )
                self._pos_label.setText(f"Position: {parts[2]}, {parts[3]}")
                # Enable buttons if they were disabled waiting for READY
                self._home_btn.setEnabled(True)
                self._erase_btn.setEnabled(True)
                for btn in self._mode_buttons.values():
                    btn.setEnabled(True)
 
        elif upper.startswith("INFO,"):
            msg = line[5:]
            self._status_label.setText(f"Status: {msg.replace('_', ' ').title()}")
            if msg.upper() == "ERASING":
                self._erasing = True
                self._erase_btn.setEnabled(False)
                self._canvas.clear_trail()
            elif msg.upper() == "HOMING":
                self._set_actions_enabled(False)
 
        elif upper.startswith("LIMIT,"):
            self._status_label.setText(f"Limit hit: {line[6:]} — home required")
            if self._erasing:
                self._erasing = False
                self._erase_btn.setEnabled(True)
 
        elif upper.startswith("ERR,"):
            self._status_label.setText(f"Error: {line[4:]}")
            if self._erasing:
                self._erasing = False
                self._erase_btn.setEnabled(True)
 
        elif upper in self._DIRECTIONS:
            self._joy_label.setText(f"Dir: {line}")
 
    def _on_connection_changed(self, connected: bool):
        # Update the UI when connection status changes, enabling/disabling buttons as appropriate
        self._connected = connected
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        # Keep action/mode buttons disabled until READY is received
        self._home_btn.setEnabled(False)
        self._erase_btn.setEnabled(False)

        for btn in self._mode_buttons.values():
            btn.setEnabled(False)
 
        if connected:
            self._status_label.setText("Status: Connected — waiting for READY...")
        else:
            self._status_label.setText("Status: Disconnected")
            self._current_mode = None
            self._erasing      = False
            for btn in self._mode_buttons.values():
                btn.setChecked(False)
 
    #### UI EVENT HANDLERS ####
    def _refresh_ports(self):
        # Refresh the COM port list, trying to keep the current selection if possible
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = SerialThread.list_ports()
        self._port_combo.addItems(ports)
        idx = self._port_combo.findText(current)
        if idx >= 0:
            self._port_combo.setCurrentIndex(idx)
 
    def _on_connect(self):
        # Connect to the selected COM port and start the serial thread
        port = self._port_combo.currentText()
        if not port:
            return
        if self._serial.connect_to_port(port):
            self._serial.start()
            # Arduino resets on DTR; query status after it boots to catch READY if missed
            QTimer.singleShot(3000, lambda: self._serial.send_command("STATUS"))
 
    def _on_disconnect(self):
        self._serial.disconnect()
 
    def _on_mode_select(self, cmd: str):
        self._send(cmd)
        self._current_mode = cmd
 
    def _on_home(self):
        self._send("HOME")
        self._set_actions_enabled(False)                #disabled until OK,HOME arrives
 
    def _on_erase(self):
        self._send("ERASE")
        self._erase_btn.setEnabled(False)               #re-enabled on completion or error
        self._canvas.clear_trail()
 
    #### HELPER METHODS FOR UI STATE MANAGEMENT ####
    def _send(self, cmd: str):
        self._serial.send_command(cmd)
 
    def _set_active_mode_btn(self, active_cmd: str):
        # Update the mode buttons to reflect the currently active mode, making sure only one is checked at a time
        for cmd, btn in self._mode_buttons.items():
            btn.setChecked(cmd == active_cmd)
        self._current_mode = active_cmd
 
    def _set_actions_enabled(self, enabled: bool):
        # Enable or disable action buttons (Home, Erase) based on the current state (e.g. disable Home while homing, disable Erase while erasing)
        self._home_btn.setEnabled(enabled and self._connected)
        self._erase_btn.setEnabled(enabled and self._connected)
        for btn in self._mode_buttons.values():
            btn.setEnabled(enabled and self._connected)
 
    def closeEvent(self, event):
        if self._connected:
            self._serial.disconnect()
        event.accept()
 
 
#### APPLICATION ENTRY POINT #### 
def main():
    app = QApplication(sys.argv)                #Create the application instance
    app.setStyle("Fusion")                      
 
    window = MainWindow()                       #Create and show the main window (build UI and start serial thread)
    window.show()                               #Start the event loop (handles UI events and signals until the window is closed)
 
    sys.exit(app.exec())
 
 
if __name__ == "__main__":
    main()