import time
import numpy as np
import RPi.GPIO as GPIO
from pymavlink import mavutil
from stable_baselines3 import PPO

# ============================================================
# CONFIG
# ============================================================

SAFE_THRESHOLD_CM = 25.0      # distance threshold (cm)
LOOP_DT = 0.1                 # loop time (s), ~10 Hz

# ML influence (kept small for stability)
ASSIST_SCALE = 10.0           # scale PPO |raw| -> % stick
MIN_ASSIST = 3.0              # minimum % when obstacle exists
MAX_ASSIST = 10.0             # absolute limit on % stick
MAX_STEP = 2.0                # max change per loop in % (anti-jerk)

SMOOTHING = 0.6               # 0..1, higher = smoother / slower

MODEL_PATH = "ppo_drone_robust_8input.zip"
ENABLE_MAVLINK_CONTROL = True   # False = debug only (no RC override)

# ============================================================
# GPIO ULTRASONIC SETUP
# ============================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

SENSORS = {
    "front": (4, 17),
    "back":  (27, 22),
    "left":  (7, 8),
    "right": (13, 19),
    "top":   (20, 21)
}

for trig, echo in SENSORS.values():
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)


def read_ultrasonic(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    timeout = start + 0.02

    while GPIO.input(echo) == 0 and time.time() < timeout:
        start = time.time()
    while GPIO.input(echo) == 1 and time.time() < timeout:
        end = time.time()

    try:
        duration = end - start
    except UnboundLocalError:
        return 999.0

    distance = duration * 17150.0
    return max(0.0, min(distance, 400.0))

# ============================================================
# MAVLINK SETUP
# ============================================================

print("Connecting to Pixhawk /dev/serial0 ...")
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
master.wait_heartbeat()
print("MAVLink heartbeat received.")

def read_imu():
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
    if msg is None:
        return 0.0, 0.0, 0.0
    return float(msg.roll), float(msg.pitch), float(msg.yaw)

def percent_to_pwm(percent, center=1500, max_delta=400):
    percent = max(-100.0, min(100.0, float(percent)))
    return int(center + (percent / 100.0) * max_delta)

def send_rc_override(roll=None, pitch=None, yaw=None):
    if not ENABLE_MAVLINK_CONTROL:
        return

    ch1 = percent_to_pwm(roll) if roll is not None else 0
    ch2 = percent_to_pwm(pitch) if pitch is not None else 0
    ch3 = 0   # throttle untouched
    ch4 = percent_to_pwm(yaw) if yaw is not None else 0

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        ch1, ch2, ch3, ch4, 0, 0, 0, 0
    )

# ============================================================
# LOAD PPO MODEL
# ============================================================

print("Loading PPO model:", MODEL_PATH)
model = PPO.load(MODEL_PATH)
print("Model loaded successfully.")

def ppo_predict(obs_vec):
    action, _ = model.predict(obs_vec, deterministic=True)
    action = np.array(action).flatten()
    if action.shape[0] != 3:
        raise ValueError(f"Expected action shape (3,), got {action.shape}")
    return float(action[0]), float(action[1]), float(action[2])  # roll_raw, pitch_raw, yaw_raw

def limit_step(prev, target, max_step):
    """Limit how fast the command can change between loops."""
    if target > prev + max_step:
        return prev + max_step
    if target < prev - max_step:
        return prev - max_step
    return target

def closest_obstacle_direction(distances):
    """Return one of 'FRONT','BACK','LEFT','RIGHT' or None."""
    cand = {}
    for name in ["front", "back", "left", "right"]:
        d = distances[name]
        if 0 < d < SAFE_THRESHOLD_CM:
            cand[name] = d
    if not cand:
        return None
    # choose min distance
    key = min(cand, key=cand.get)
    return key.upper()

# ============================================================
# MAIN LOOP
# ============================================================

prev_roll = 0.0
prev_pitch = 0.0

try:
    while True:

        # ------------ READ SENSORS ------------
        distances = {name: read_ultrasonic(*pins) for name, pins in SENSORS.items()}
        imu_roll, imu_pitch, imu_yaw = read_imu()

        front = distances["front"]
        back = distances["back"]
        left = distances["left"]
        right = distances["right"]
        top = distances["top"]

        print("\n--- SENSOR + IMU READINGS ---")
        print(f"Front={front:.1f}cm  Back={back:.1f}cm  Left={left:.1f}cm  Right={right:.1f}cm  Top={top:.1f}cm")
        print(f"IMU: roll={imu_roll:.3f} rad, pitch={imu_pitch:.3f} rad, yaw={imu_yaw:.3f} rad")

        # ------------ BUILD OBS (same as training) ------------
        obs = np.array([
            front, back, left, right, top,
            imu_roll, imu_pitch, imu_yaw
        ], dtype=np.float32)

        min_dist = min(distances.values())
        obs_dir = closest_obstacle_direction(distances)

        # ======================================================
        # NO OBSTACLE: full manual control
        # ======================================================
        if obs_dir is None:
            print(f"No obstacle < {SAFE_THRESHOLD_CM}cm → MANUAL, no override.")
            send_rc_override(None, None, None)
            prev_roll = prev_pitch = 0.0
            time.sleep(LOOP_DT)
            continue

        # ======================================================
        # OBSTACLE NEAR: use PPO magnitude + safe direction
        # ======================================================
        roll_raw, pitch_raw, yaw_raw = ppo_predict(obs)
        print(f"PPO RAW OUTPUTS: roll_raw={roll_raw:.3f}, pitch_raw={pitch_raw:.3f}, yaw_raw={yaw_raw:.3f}")
        print(f"Closest obstacle at: {obs_dir}, dist={min_dist:.1f}cm")

        # Get magnitudes from PPO (ignore sign, we choose direction ourselves)
        roll_mag  = min(MAX_ASSIST, max(MIN_ASSIST, abs(roll_raw)  * ASSIST_SCALE))
        pitch_mag = min(MAX_ASSIST, max(MIN_ASSIST, abs(pitch_raw) * ASSIST_SCALE))

        roll_target = 0.0
        pitch_target = 0.0

        # IMPORTANT: sign mapping assumes:
        # +roll  -> drone tilts RIGHT
        # -roll  -> drone tilts LEFT
        # +pitch -> nose DOWN (forward)
        # -pitch -> nose UP   (backward)

        if obs_dir == "FRONT":
            movement = "MOVE BACKWARD"
            pitch_target = -pitch_mag   # back
        elif obs_dir == "BACK":
            movement = "MOVE FORWARD"
            pitch_target = +pitch_mag   # forward
        elif obs_dir == "LEFT":
            movement = "MOVE RIGHT"
            roll_target = +roll_mag     # right
        elif obs_dir == "RIGHT":
            movement = "MOVE LEFT"
            roll_target = -roll_mag     # left
        else:
            movement = "UNKNOWN"
            roll_target = pitch_target = 0.0

        # Limit step change
        roll_cmd  = limit_step(prev_roll,  roll_target,  MAX_STEP)
        pitch_cmd = limit_step(prev_pitch, pitch_target, MAX_STEP)

        # Smooth
        roll_cmd  = SMOOTHING * prev_roll  + (1 - SMOOTHING) * roll_cmd
        pitch_cmd = SMOOTHING * prev_pitch + (1 - SMOOTHING) * pitch_cmd

        prev_roll, prev_pitch = roll_cmd, pitch_cmd

        print(f"OBSTACLE DIR={obs_dir} → {movement}")
        print(f"FINAL RC COMMANDS: roll={roll_cmd:.1f}%  pitch={pitch_cmd:.1f}%")

        send_rc_override(roll_cmd, pitch_cmd, None)

        time.sleep(LOOP_DT)

except KeyboardInterrupt:
    print("Stopping script...")

finally:
    send_rc_override(None, None, None)
    GPIO.cleanup()
    print("Overrides cleared, GPIO cleaned up.")
