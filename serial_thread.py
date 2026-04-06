"""
- USB serial communication with the arduino in background to avoid blocking the GUI thread
- uses plain newline-terminated text commands and responses
- PC sends commands like:  HOME, MOVE,x,y, SETMODE,COORDINATE,
- Arduino sends responses like: READY, POS,x,y, OK,MOVE,x,y, STATUS 
"""

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal


class SerialThread(QThread):

    line_received      = Signal(str)                    #take every line from Arduino, unparsed
    connection_changed = Signal(bool)                   #True = connected, False = disconnected

    def __init__(self, parent=None):
        super().__init__(parent)                        
        self._serial  = None                            
        self._running = False

    def connect_to_port(self, port: str) -> bool:
        try:
            self._serial = serial.Serial(port, baudrate=115200, timeout=0.1)
            self._serial.dtr = True                     #required for R4 Minima
            self._running = True
            self.connection_changed.emit(True)          #display UI connection status
            return True
        except serial.SerialException as e:             #display connection errors in the UI
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:       #close the serial port if it's open
            self._serial.close()
        self._serial = None
        self.connection_changed.emit(False)

    def send_command(self, cmd: str):
        if self._serial and self._serial.is_open:
            self._serial.write((cmd + "\n").encode("ascii"))

    @staticmethod
    def list_ports() -> list[str]:
        # Helper to list available serial ports for the UI dropdown
        return [p.device for p in serial.tools.list_ports.comports()]

    def run(self):
        # Background thread loop to read lines from the serial port and emit signals
        buf = ""
        while self._running:
            try:
                data = self._serial.read(self._serial.in_waiting or 1)                  #read all available data or wait for at least one byte
                if data:                                                                #if we got any data, decode it and split into lines
                    buf += data.decode("ascii", errors="replace")                       
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)                                  
                        line = line.strip()                                             
                        if line:                                                        #ignore empty lines
                            self.line_received.emit(line)
            except serial.SerialException:
                self.connection_changed.emit(False)
                self._running = False