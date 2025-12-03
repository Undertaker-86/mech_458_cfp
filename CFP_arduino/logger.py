import serial
import time
import csv
import threading
import sys

# ==== CONFIGURATION ====
SERIAL_PORT = 'COM10'  # <--- CHANGE THIS to your Arduino Port!
BAUD_RATE = 115200

# ==== SETUP ====
print("--- Motor Controller & Logger ---")
filename_input = input("Enter log filename (e.g. 'test1'): ")
if not filename_input.endswith(".csv"):
    filename_input += ".csv"

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error: {e}")
    print("Check your USB cable and COM port.")
    sys.exit()

stop_event = threading.Event()

# ==== READING THREAD (Saves Data) ====
def read_from_serial():
    with open(filename_input, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time_ms", "Target", "M1_Pos", "M2_Pos", "M1_RPM", "M2_RPM", "PWM1", "PWM2"])
        
        while not stop_event.is_set():
            try:
                if ser.in_waiting > 0:
                    raw = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "," in raw:
                        data = raw.split(',')
                        if len(data) == 8: 
                            writer.writerow(data)
                            file.flush() # Ensure data is written immediately for live plotter
                    elif raw:
                        print(f"[Arduino]: {raw}")
            except Exception:
                break

# ==== WRITING THREAD (Sends Commands) ====
def write_to_serial():
    print("\n=== CONTROLS ===")
    print("  q / a : Jog Motor 1 (Up/Down)")
    print("  w / s : Jog Motor 2 (Up/Down)")
    print("  z     : SET ZERO (Required first)")
    print("  Number: Move to height (e.g. 5000)")
    print("  stop  : EMERGENCY HALT & QUIT")
    print("================\n")
    
    while not stop_event.is_set():
        user_input = input() 
        
        if user_input.lower() in ["stop", "exit"]:
            print("!!! EMERGENCY HALT SENT !!!")
            ser.write(b'h\n') # Send Halt command
            time.sleep(0.2)
            stop_event.set()
            break
            
        if user_input:
            ser.write((user_input + '\n').encode('utf-8'))

if __name__ == "__main__":
    t1 = threading.Thread(target=read_from_serial, daemon=True)
    t1.start()
    try:
        write_to_serial()
    except KeyboardInterrupt:
        ser.write(b'h\n')
        stop_event.set()
    ser.close()
    sys.exit()