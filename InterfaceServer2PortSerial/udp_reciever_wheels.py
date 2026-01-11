import socket
import threading
from datetime import datetime
import time
import serial
import sys
import math
import pygame

# === Initialize Joystick ===
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()


joystick_state = [0.0] * joystick.get_numaxes()

# === Shared values ===
latest_position = None
latest_rotation = None
z_baseline = None
z_inverted = False
recording = False
current_record = []
record_count = 21
lock = threading.Lock()

# === Port settings ===
SERIAL_PORT = r"\\.\COM13"
BAUD = 115200
SERIAL_TIMEOUT = 10
SERIAL_WRITE_TIMEOUT = 20
RETRY_DELAY = 0.1
MAX_WRITE_RETRIES = 3

# === Filters ===
class ScalarKalman:
    def __init__(self, R=0.5, Q=0.01):
        self.R = R
        self.Q = Q
        self.A = 1
        self.C = 1
        self.cov = None
        self.x = None

    def filter(self, z):
        if self.x is None:
            self.x = z
            self.cov = 1.0
        predX = self.A * self.x
        predCov = self.A * self.cov * self.A + self.Q
        K = predCov * self.C / (self.C * predCov * self.C + self.R)
        self.x = predX + K * (z - self.C * predX)
        self.cov = predCov - K * self.C * predCov
        return self.x

class YawUnwrapper:
    def __init__(self):
        self.last_yaw = None
        self.continuous_yaw = 0.0
        self.offset = None

    def update(self, current_yaw):
        if current_yaw is None or math.isnan(current_yaw):
            return self.continuous_yaw
        if self.offset is None:
            self.offset = current_yaw
        normalized_yaw = current_yaw - self.offset
        if self.last_yaw is None:
            self.last_yaw = normalized_yaw
            return self.continuous_yaw
        delta = normalized_yaw - self.last_yaw
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        self.continuous_yaw += delta
        self.last_yaw = normalized_yaw
        return self.continuous_yaw

kf_x, kf_y, kf_z = ScalarKalman(), ScalarKalman(), ScalarKalman()
kf_rx, kf_ry, kf_rz = ScalarKalman(), ScalarKalman(), ScalarKalman()
yaw_unwrapper = YawUnwrapper()

def rotate_135_degrees(x, y, clockwise=True):
    angle = -3 * math.pi / 4 if clockwise else 3 * math.pi / 4
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    x_rot = x * cos_a - y * sin_a
    y_rot = x * sin_a + y * cos_a
    return x_rot, y_rot

def parse_clean_parts(decoded, expected_count, port):
    cleaned = decoded.replace('\x00', '').strip(', \n\r')
    parts = [p.strip() for p in cleaned.split(',') if p.strip()]
    if len(parts) != expected_count:
        print(f"[{port}] ⚠️ Unexpected count: {len(parts)} parts → {parts}")
        return None
    return parts

def listen_position():
    global latest_position
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 9000))
    print("📡 Position listener on port 9000")
    while True:
        data, _ = sock.recvfrom(1024)
        parts = parse_clean_parts(data.decode().strip(), 4, 9000)
        if parts:
            try:
                ts, x, y, z = map(float, parts)
                with lock:
                    latest_position = (ts, x, y, z)
            except:
                pass

def listen_rotation():
    global latest_rotation
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 8000))
    print("📡 Rotation listener on port 8000")
    while True:
        data, _ = sock.recvfrom(1024)
        parts = parse_clean_parts(data.decode().strip(), 3, 8000)
        if parts:
            try:
                rx, ry, rz = map(float, parts)
                with lock:
                    latest_rotation = (rx, ry, rz)
            except:
                pass

def open_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=SERIAL_TIMEOUT, write_timeout=SERIAL_WRITE_TIMEOUT)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        return ser
    except Exception as e:
        print(f"[Error] Failed to open {SERIAL_PORT}: {e}", file=sys.stderr)
        
    sys.exit(1)

def write_with_retries(ser, payload):
    for _ in range(MAX_WRITE_RETRIES):
        try:
            ser.write(payload)
            ser.flush()
            return True
        except (serial.SerialTimeoutException, OSError):
            time.sleep(RETRY_DELAY)
    return False

def combine_loop(ser):
    global joystick_state, z_baseline, z_inverted, recording, current_record, record_count
    last_sent_values = [None] * 10
    deadzone = [0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 0.02, 0.02, 0.02, 0.02]
    last_valid_position = None
    last_send_time = 0
    send_interval = 1 / 30

    while True:
        with lock:
            now = time.time()
            if now - last_send_time < send_interval:
                time.sleep(0.001)
                continue
            last_send_time = now

            if latest_rotation:
                rx, ry, rz = latest_rotation
                rx_f = kf_rx.filter(rx)
                ry_f = kf_ry.filter(ry)
                rz_unwrapped = yaw_unwrapper.update(rz)
                rz_f = kf_rz.filter(rz_unwrapped)

                if latest_position:
                    ts, x, y, z = latest_position
                    x_f = kf_x.filter(x)
                    y_f = kf_y.filter(y)
                    z_f = kf_z.filter(z)
                    last_valid_position = (ts, x_f, y_f, z_f)
                elif last_valid_position:
                    ts, x_f, y_f, z_f = last_valid_position
                else:
                    ts = time.time()
                    x_f = y_f = z_f = 0.0

                x_rot, y_rot = rotate_135_degrees(x_f, y_f)
                pitch_rot, roll_rot = rotate_135_degrees(rx_f, ry_f)
                z_display = 0.0 if not z_inverted else -(z_f - z_baseline if z_baseline else 0)

                x_out = round(max(min(x_rot * 3, 25), -25), 2)
                y_out = round(max(min(-y_rot * 3, 25), -25), 2)
                z_out = round(max(min(z_display, 20), -14), 2)
                rx_out = round(max(min(pitch_rot * 30, 7), -7), 2)
                ry_out = round(max(min(roll_rot * 30, 7), -7), 2)
                rz_out = round(max(min(-rz_f * 12, 40), -40), 2)

                xW, yW, turnR, turnL = joystick_state[0], -joystick_state[1], joystick_state[4], joystick_state[5]
                values = [x_out, y_out, z_out, rx_out, ry_out, rz_out, xW, yW, turnR, turnL]

                if any(last is None or abs(v - last) > dz for v, last, dz in zip(values, last_sent_values, deadzone)):
                    last_sent_values = values.copy()
                    output = ", ".join(f"{v:.2f}" for v in values)
                    print(f"🧭 {datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]} → {output}")
                    write_with_retries(ser, (output + '\n').encode('utf-8'))

                    if recording:
                        current_record.append(f"{datetime.now().strftime('%Y%m%d_%H%M%S.%f')[:-3]}, {output}")

        time.sleep(0.001)

def listen_for_input():
    global recording, current_record, record_count, z_inverted, z_baseline
    print("⌨️ Commands: 'r' to record, 'z' to baseline Z")
    while True:
        cmd = input().strip().lower()
        with lock:
            if cmd == 'r':
                if not recording:
                    recording = True
                    current_record = []
                    print(f"🔴 Started recording movement_{record_count:02}.txt")
                else:
                    recording = False
                    with open(f"movement_{record_count:02}.txt", "w") as f:
                        f.writelines(line + "\n" for line in current_record)
                    print(f"✅ Saved recording: movement_{record_count:02}.txt")
                    record_count += 1
            elif cmd == 'z':
                z_inverted = True
                if latest_position:
                    z_baseline = latest_position[3]
                    print(f"✅ Z baseline updated to {z_baseline:.4f}")
                else:
                    print("⚠️ No Z position available to baseline.")


def main():
    global joystick_state
    ser = open_serial()
    threading.Thread(target=listen_position, daemon=True).start()
    threading.Thread(target=listen_rotation, daemon=True).start()
    threading.Thread(target=listen_for_input, daemon=True).start()
    threading.Thread(target=combine_loop, args=(ser,), daemon=True).start()

    while True:
        pygame.event.pump()
        joystick_state = [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]
        time.sleep(0.01)

if __name__ == "__main__":
    main()
