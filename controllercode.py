##############################################################################################
# MIE438 FINAL PROJECT - PYTHON HOST CONTROLLER
# 
# PURPOSE:
# Host-side interface for rail gantry, communicates with Arduino motor controller over serial.
#
# ROLE:
# 1) Manual command execution
# 2) Preset motion maths for automated drawing
# 3) User-facing interface for operations
##############################################################################################

import serial
import time
from datetime import datetime

PORT = "COM5"
BAUD = 115200
TIMEOUT = 2

##############################################################################################
# PRESET DRAWINGS
# 
# Predefined xy coordinate paths for automated drawing mode or selected drawing mode.
# Each drawing is a list of (x,y) waypoints.
##############################################################################################
PRESET_DRAWINGS = {
    "square": [(2,2), (10,2), (10,10), (2,10), (2,2)],
    "triangle": [(5,2), (12,12), (2,12), (5,2)],
    "zigzag": [(2,2), (5,5), (2,8), (5,11), (2,14)],
}

##############################################################################################
# CONTROLLER CLASS
#
# All serial communication and host-side gantry commands.
##############################################################################################
class Controller:
    def __init__(self, port, baud, timeout=2):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)
        self.flush_startup()
        
    def flush_startup(self):                            # read and print any startup messages from Arduino code, prevent parsing
        #read startup messages
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                print("[ARDUINO]", line)
                
    def send_command(self, cmd):                        # sends a new line string command to Arduino to be parsed
        print(f"[PC -> ARDUINO] {cmd}")
        self.ser.write((cmd + "\n").encode())
        
    def read_lines(self, duration = 0.5):               # read and return status messages from Arduino
        end_time = time.time() + duration
        lines = []
        
        while time.time() < end_time:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore")
                if line:
                    print("[ARDUINO]", line)
                    lines.append(line)
                else:
                    time.sleep(0.01)
        return lines
    
    def read_response_until_done(self, timeout=10):     # reads serial responses, waiting for requests to be completed
        start = time.time()
        responses = []
        
        while time.time()-start < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    print("[ARDUINO]", line)
                    responses.append(line)
                    
                    if line.startswith("OK,MOVE") or line.startswith("OK,HOME") or line.startswith("OK,MODE"):
                        return responses
                    if line.startswith("ERR") or line.startswith("LIMIT"):
                        return responses
            else:
                time.sleep(0.01)
        return responses

    def home(self):         # perform homing routine
        self.send_command("HOME")
        return self.read_response_until_done(timeout=20)
    
    def move_to(self,x,y):  # move to (x,y) location
        self.send_command(f"MOVE,{x},{y}")
        return self.read_response_until_done(timeout=10)
    
    def get_position(self): # request current xy position
        self.send_command("GETPOS")
        lines = self.read_lines(0.5)
        
        for line in lines:
            if line.startswith("POS,"):
                _,x,y = line.split(",")
                return int(x), int(y)
        return None
    
    def get_status(self):   # request status
        self.send_command("STATUS")
        lines = self.read_lines(0.5)
        
        for line in lines:
            if line.startswith("STATUS,"):
                return line
        return None
    
    def stop(self):         # sends stop request, emergency interruption
        self.send_command("STOP")
        return self.read_lines(0.5)

    def set_mode_coordinate(self):                      # switches to coordinate mode
        self.send_command("SETMODE,COORDINATE")
        return self.read_response_until_done(timeout=5)
    
    def set_mode_joystick(self):                        # switches to joystick mode
        self.send_command("SETMODE,JOYSTICK")
        return self.read_response_until_done(timeout=5)
        
    def run_path(self, path, delay_between_points=1):   # executing move commands based on a list of points
        self.set_mode_coordinate()
        
        for point in path:
            x,y = point
            responses = self.move_to(x,y)
            
            if any(r.startswith("ERR") or r.startswith("LIMIT") for r in responses):
                print("Path interrupted due to error/limit switch.")
                return False #CHANGE TO AUTO HOME
            
            time.sleep(delay_between_points)
        return True
    
    def close(self):        # close and end serial port connection
        self.ser.close()

def preset_drawing_mode(gantry):            # preset drawing mode that lets user select a predefined motion path
    print("\nAvailable drawings:")
    for name in PRESET_DRAWINGS.keys():
        print(f" - {name}")
    
    choice = input("Enter drawing name: ").strip().lower()
    
    if choice not in PRESET_DRAWINGS:
        print("Invalid drawing.")
        return
    
    print(f"Running preset drawing: {choice}")
    gantry.run_path(PRESET_DRAWINGS[choice])
    
def auto_cycle_mode(gantry):                # continuously cycles through all preset drawings until interrupted
    print("\nAuto-cycle mode started.")
    print("This will continuously cycle through all preset drawings.")
    print("Press Ctrl+C to stop.\n")
    
    try:
        gantry.set_mode_coordinate()
        
        while True:
            for name, path in PRESET_DRAWINGS.items():
                print(f"\n--- Running drawing: {name} ---")
                success = gantry.run_path(path)

                if not success:
                    print("Cycle stopped due to gantry error.")
                    return

                time.sleep(1) 
                       
    except KeyboardInterrupt:
        print("\nAuto-cycle stopped by user.")
        
def joystick_mode(gantry):                  # switch to joystick control mode
    print("\nSwitching to JOYSTICK mode...")
    gantry.set_mode_joystick()
    print("JOYSTICK mode active. Press Enter to return to menu.")
    input()
    
    print("Switching back to COORDINATE mode...")
    gantry.set_mode_coordinate()

def clock_mode():                           # placeholder for clock mode where time based patterns are generated
    print("\nClock mode not yet implemented.")
    now = datetime.now()
    print(f"Current time: {now.strftime('%H:%M:%S')}")

##############################################################################################
# MAIN
#
# Host-side user interface loop.
##############################################################################################    
def main():
    gantry = Controller(PORT,BAUD,TIMEOUT)
    
    try:
        while True:
            print("Z E N   G A R D E N : M O D E   S E L E C T")
            print("1. Home")
            print("2. Select Drawing")
            print("3. Auto Drawing")
            print("4. Joystick Control")
            print("5. Clock Mode")
            print("6. Get Position")
            print("7. Get Status")
            print("8. Stop Motion")
            print("9. Exit")
        
            choice = input("Select option: ").strip()
        
            if choice == "1":
                gantry.home()
        
            elif choice == "2":
                preset_drawing_mode(gantry)
            
            elif choice == "3":
                auto_cycle_mode(gantry)
            
            elif choice == "4":
                joystick_mode(gantry)
            
            elif choice == "5":
                clock_mode()
            
            elif choice == "6":
                pos = gantry.get_position()
                print("Position:",pos)
            
            elif choice == "7":
                status = gantry.get_status()
                print("Status:",status)
        
            elif choice == "8":
                gantry.stop()
            
            elif choice == "9":   
                print("Exiting.")
                break
        
            else:
                print("Invalid.")
            
    finally:
        gantry.close()
        
if __name__ == "__main__":
    main()