import RPi.GPIO as GPIO
import time
import csv
from datetime import datetime
from pymavlink import mavutil

# --------------------------------------------
# CONFIGURATION
# --------------------------------------------

# Define sensor GPIO pins: {position: (TRIG, ECHO)}
SENSORS = {
    "front": (4, 17),
    "back": (27, 22),
    "left": (7, 8),
    "right": (13, 19),
    "top": (20, 21)
}

# CSV output file
csv_file = "drone_dat.csv"

# Serial port to Pixhawk (check your RPi port, typically /dev/serial0 or /dev/ttyAMA0)
PIXHAWK_PORT = '/dev/serial0'
PIXHAWK_BAUD = 57600

# --------------------------------------------
# INITIALIZATION
# --------------------------------------------

GPIO.setmode(GPIO.BCM)
for trig, echo in SENSORS.values():
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
master.wait_heartbeat()
print(f"Connected to system {master.target_system} component {master.target_component}")

# Request ATTITUDE data stream at 5Hz
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    5,
    1
)

# Prepare CSV file
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    header = ["timestamp", "front", "back", "left", "right", "top", "roll", "pitch", "yaw"]
    writer.writerow(header)

# --------------------------------------------
# FUNCTION: Measure Distance (US-100 GPIO mode)
# --------------------------------------------

def measure_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.05)

    GPIO.output(trig, True)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(trig, False)

    timeout = time.time() + 1
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    timeout = time.time() + 1
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # cm
    return round(distance, 2)

# --------------------------------------------
# MAIN LOOP
# --------------------------------------------

try:
    while True:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        distances = {}

        # Get ultrasonic readings
        for position, (trig, echo) in SENSORS.items():
            dist = measure_distance(trig, echo)
            distances[position] = dist if dist is not None else "NA"

        # Get roll, pitch, yaw from Pixhawk
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            roll = round(msg.roll, 4)
            pitch = round(msg.pitch, 4)
            yaw = round(msg.yaw, 4)
        else:
            roll, pitch, yaw = "NA", "NA", "NA"

        # Print to console
        print(f"{timestamp} | US100: {distances} | RPY: {roll}, {pitch}, {yaw}")

        # Save to CSV
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp,
                distances["front"],
                distances["back"],
                distances["left"],
                distances["right"],
                distances["top"],
                roll, pitch, yaw
            ])

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
