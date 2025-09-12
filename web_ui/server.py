#!/usr/bin/env python3
"""Simple Flask web UI for Raspberry Pi sensors (gyro + ultrasonic).

Features:
 - Polls MPU6050 gyro (or mock) and integrates yaw rate into a heading in degrees (0-359)
 - Polls two HC-SR04 ultrasonic sensors (or mock) for distances in cm
 - Exposes HTTP API: GET /api/status and POST /api/reset_heading
 - Serves a small single-page UI at /

This is designed to be safe to run on a non-RPi machine (mock sensors used when libraries are missing).
"""
import time
import threading
import random
import math
from math import fmod
from flask import Flask, jsonify, request, send_from_directory

app = Flask(__name__, static_folder="static", static_url_path="")

try:
    from mpu6050 import mpu6050
    HAS_MPU = True
except Exception:
    HAS_MPU = False

# Prefer direct smbus2 access to avoid heavy scipy/numpy deps from some mpu6050 packages
try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except Exception:
    HAS_SMBUS = False

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False


class GyroSensor:
    """Minimal wrapper around MPU6050 to provide Z-axis rotation rate in deg/s.
    Falls back to a mock that returns 0.0 when device is unavailable.
    """
    def __init__(self, bus=1, address=0x68):
        self._use_smbus = False
        self.dev = None
        self._bus_num = bus
        self._address = address

        if HAS_SMBUS:
            try:
                self._bus = SMBus(self._bus_num)
                # wake up device
                self._bus.write_byte_data(self._address, 0x6B, 0)
                self._use_smbus = True
                print("INFO: MPU6050 accessed via smbus2")
            except Exception as e:
                print(f"WARN: smbus2 access to MPU6050 failed: {e}")
                self._use_smbus = False

        if not self._use_smbus:
            # no library/device available; leave dev as None so callers know
            self.dev = None

    def get_gyro_z(self):
        return self.get_gyro('z')

    def get_gyro_x(self):
        return self.get_gyro('x')

    def get_gyro_y(self):
        return self.get_gyro('y')

    def get_gyro(self, axis='z'):
        a = axis.lower()
        if self._use_smbus:
            try:
                data = self._bus.read_i2c_block_data(self._address, 0x43, 6)
                def to_signed(h, l):
                    v = (h << 8) | l
                    return v - 65536 if v & 0x8000 else v
                gx = to_signed(data[0], data[1])
                gy = to_signed(data[2], data[3])
                gz = to_signed(data[4], data[5])
                raw = {'x': gx, 'y': gy, 'z': gz}
                scale = 131.0
                return float(raw.get(a, 0.0)) / scale
            except Exception:
                return 0.0

        if self.dev:
            try:
                g = self.dev.get_gyro_data()
                return float(g.get(a, 0.0))
            except Exception:
                return 0.0

        return 0.0


class UltrasonicSensor:
    """Wrapper for HC-SR04. If RPi.GPIO isn't available, returns mock distances.
    Initialize with TRIG and ECHO BCM pin numbers.
    """
    def __init__(self, trig_pin=None, echo_pin=None):
        self.trig = trig_pin
        self.echo = echo_pin
        self.mock = not HAS_GPIO or self.trig is None or self.echo is None
        if not self.mock:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
        else:
            pass

    def get_distance_cm(self):
        if self.mock:
            return None

        GPIO.output(self.trig, False)
        time.sleep(0.0002)
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        start = time.time()
        timeout = start + 0.02
        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            start = time.time()

        stop = time.time()
        timeout = stop + 0.02
        while GPIO.input(self.echo) == 1 and time.time() < timeout:
            stop = time.time()

        elapsed = stop - start
        distance = elapsed * 34300.0 / 2.0
        return distance


class HeadingTracker:
    """Integrate gyro z-rate to produce a heading in degrees within [0, 360).
    Supports reset() to declare the current heading as 0.
    """
    def __init__(self, gyro_sensor, poll_interval=0.05, sign=1, axis='z', min_rate_thresh=0.5, drift_correction_dps=-0.1):
        self.gyro = gyro_sensor
        self.poll = poll_interval
        self._raw = 0.0
        self._offset = 0.0
        # sign: 1 keeps gyro sign as-is, -1 inverts rotation direction used for integration
        self.sign = 1 if sign >= 0 else -1
        # axis: 'x','y' or 'z' - which gyro axis to integrate
        self.axis = axis.lower()
        # minimum absolute gyro rate (deg/s) required to integrate; prevents slow drift from noise
        self.min_rate_thresh = float(min_rate_thresh)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        # apply a small constant correction (deg/s) to counteract slow gyro drift
        # default is -0.1 deg/s (subtract 0.1 degrees every second)
        self.drift_correction_dps = float(drift_correction_dps)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        last = time.time()
        while self._running:
            now = time.time()
            dt = now - last
            last = now
            # pick the correct gyro axis getter
            getter = getattr(self.gyro, f'get_gyro_{self.axis}', None)
            if getter is None:
                # fallback to z
                rate = self.gyro.get_gyro('z')
            else:
                try:
                    rate = getter()
                except Exception:
                    rate = 0.0

            # if gyro appears to be a mock/unavailable, or rate is tiny, skip integration to avoid drift
            is_gyro_mock = (getattr(self.gyro, 'dev', None) is None) and (not getattr(self.gyro, '_use_smbus', False))
            if rate is None:
                rate = 0.0
            if is_gyro_mock or abs(rate) < self.min_rate_thresh:
                # nothing to integrate; just sleep and continue
                time.sleep(max(0.0, self.poll - 0.0))
                last = time.time()
                continue

            # apply configurable sign so sensor wrapper stays untouched
            delta = (self.sign * rate) * dt
            # small constant drift correction (deg/s * dt)
            correction_delta = (self.drift_correction_dps) * dt
            with self._lock:
                self._raw = fmod((self._raw + delta + correction_delta), 360.0)
                if self._raw < 0:
                    self._raw += 360.0
            time.sleep(max(0.0, self.poll - 0.0))

    def get_heading(self):
        with self._lock:
            h = (self._raw - self._offset) % 360.0
            if h < 0:
                h += 360.0
            return h

    def reset(self):
        with self._lock:
            # make current raw heading become 0
            self._offset = self._raw


# Instantiate sensors using BCM pin numbers from your pin map (see raspberry-pins.txt)
gyro = GyroSensor()
sensor_front = UltrasonicSensor(trig_pin=15, echo_pin=14)  # front TRIG=BCM15, ECHO=BCM14
sensor_right = UltrasonicSensor(trig_pin=23, echo_pin=24)  # right TRIG=BCM23, ECHO=BCM24

# Invert gyro sign to match physical orientation of the sensor.
# Keep only a single HeadingTracker for the Z axis (heading) as requested.
# Apply a stronger drift correction (-0.5 deg/s) and allow correction even when
# the measured rate is very small by setting min_rate_thresh=0.0.
tracker_z = HeadingTracker(gyro, sign=-1, axis='z', drift_correction_dps=-0.476, min_rate_thresh=0.0)
tracker_z.start()

# Log sensor modes so it's obvious on startup whether real GPIO is used
print(f'INFO: sensor_front mock={sensor_front.mock}, sensor_right mock={sensor_right.mock}, HAS_GPIO={HAS_GPIO}, HAS_MPU={HAS_MPU}')
print(f'INFO: gyro mock={(gyro.dev is None)}')

# --- Motor controller for L298N (uses BCM pins from raspberry-pins.txt) ---
class MotorController:
    """Simple motor controller for four motors (L298N). Expects BCM pin numbers.
    Motor naming: front_left, back_left, front_right, back_right
    Each motor uses two GPIO pins (A,B). We'll use A for PWM control and B as direction flip.
    This is a simple abstraction sufficient for forward/backward/rotate.
    """
    def __init__(self, pins=None, pwm_freq=1000):
        # pins: dict(name -> (a_pin, b_pin))
        default = {
            'front_left': (13, 6),
            'back_left': (26, 19),
            'front_right': (27, 22),
            'back_right': (4, 17)
        }
        self.pins = pins or default
        self.pwms = {}
        # If your wiring makes positive speeds drive backwards, set this True to flip direction
        self.invert_direction = True
        self.enabled = HAS_GPIO
        if self.enabled:
            GPIO.setmode(GPIO.BCM)
            for name, (a, b) in self.pins.items():
                # initialize direction pin to a safe known state BEFORE enabling PWM
                GPIO.setup(a, GPIO.OUT)
                GPIO.setup(b, GPIO.OUT)
                GPIO.output(b, GPIO.LOW)
                pwm = GPIO.PWM(a, pwm_freq)
                # start PWM with 0 duty to ensure motor is stopped
                pwm.start(0.0)
                self.pwms[name] = (pwm, b)
        else:
            print('INFO: GPIO not available, MotorController will operate in mock mode.')

    def set_motor(self, name, speed):
        """Set motor speed in range [-1.0, 1.0]. Positive = forward.
        Speed is applied as PWM duty cycle (0-100) on pin A; pin B controls direction.
        """
        speed = max(-1.0, min(1.0, float(speed)))
        if not self.enabled:
            print(f'MOCK motor {name} set to {speed:.2f}')
            return
        pwm, b_pin = self.pwms[name]
        duty = abs(speed) * 100.0
        # apply global inversion if wiring is reversed
        dir_forward = True if speed >= 0 else False
        if getattr(self, 'invert_direction', False):
            dir_forward = not dir_forward
        GPIO.output(b_pin, GPIO.HIGH if dir_forward else GPIO.LOW)
        pwm.ChangeDutyCycle(duty)

    def stop_all(self):
        if not self.enabled:
            print('MOCK stop_all')
            return
        for name in self.pwms:
            pwm, b = self.pwms[name]
            pwm.ChangeDutyCycle(0.0)
            # leave direction pins in safe (LOW) state
            try:
                GPIO.output(b, GPIO.LOW)
            except Exception:
                pass


# Instantiate motor controller (safe mock if no GPIO) and ensure stopped at startup
motors = MotorController()
motors.stop_all()

# Helper: rotate in place to a relative heading (degrees can be positive or negative).
def _rotate_in_place(degrees, rot_speed=0.4, tol_deg=4.0, timeout=8.0):
    """Rotate the robot in place by `degrees` (signed). Blocks until done or aborted.
    Returns True on success, False if aborted via _stop_event or timeout.
    """
    # normalize target heading
    start = tracker_z.get_heading()
    target = (start + float(degrees)) % 360.0

    def rotate_clockwise(s):
        # left forward, right backward
        motors.set_motor('front_left', s)
        motors.set_motor('back_left', s)
        motors.set_motor('front_right', -s)
        motors.set_motor('back_right', -s)

    def rotate_ccw(s):
        rotate_clockwise(-s)

    def shortest_angle_diff(a, b):
        d = (b - a + 180.0) % 360.0 - 180.0
        return d

    start_t = time.time()
    print(f"INFO: rotate_in_place start: start_heading={start:.2f} target={target:.2f} degrees={degrees}")
    try:
        initial = tracker_z.get_heading()
        last_diff = None
        while True:
            if _stop_event.is_set():
                motors.stop_all()
                print("INFO: rotate_in_place aborted due to _stop_event")
                return False
            now = tracker_z.get_heading()
            diff = shortest_angle_diff(now, target)
            # Stoppen, wenn Ziel überschritten wurde (Richtungswechsel im diff)
            if last_diff is not None and (diff == 0 or (last_diff > 0 and diff < 0) or (last_diff < 0 and diff > 0)):
                print(f"INFO: rotate_in_place überschritten: now={now:.2f}, target={target:.2f}, diff={diff:.2f}")
                break
            if abs(diff) <= tol_deg:
                break
            # rotate towards target via shortest path
            if diff > 0:
                rotate_ccw(rot_speed)
            else:
                rotate_clockwise(rot_speed)
            last_diff = diff
            time.sleep(0.05)
        print(f"INFO: rotate_in_place reached/überschritten target (now={tracker_z.get_heading():.2f})")
        return True
    finally:
        motors.stop_all()


# Worker to run the corner-handling sequence (ecken_handling)
_collect_lock = threading.Lock()
_stop_event = threading.Event()
COOLDOWN_SECONDS = 14.0  # Cooldown-Mechanismus für automatische Drehung (8s)
_last_auto_turn_time = 0.0
def ecken_handling_sequence():
    """Simplified corner handling: when a front obstacle is detected while
    driving forward, perform exactly three steps:
      1) rotate clockwise for 1s
      2) drive backward for 1s
      3) rotate to (start_heading - 90°) relative to the heading when the
         sequence started

    The function respects _stop_event and ensures motors are stopped on exit.
    """
    global _last_auto_turn_time
    try:
        speed = 0.6

        def forward(s):
            motors.set_motor('front_left', s)
            motors.set_motor('back_left', s)
            motors.set_motor('front_right', s)
            motors.set_motor('back_right', s)

        def rotate_clockwise(s):
            motors.set_motor('front_left', s)
            motors.set_motor('back_left', s)
            motors.set_motor('front_right', -s)
            motors.set_motor('back_right', -s)

        def rotate_ccw(s):
            rotate_clockwise(-s)

        def shortest_angle_diff(a, b):
            d = (b - a + 180.0) % 360.0 - 180.0
            return d

        def rotate_towards_target(target, rot_speed=0.4, tol_deg=5.0, timeout=5.0):
            start_t = time.time()
            while True:
                if _stop_event.is_set():
                    motors.stop_all()
                    return False
                h = tracker_z.get_heading()
                diff = shortest_angle_diff(h, target)
                if abs(diff) <= tol_deg:
                    break
                if diff > 0:
                    rotate_clockwise(rot_speed)
                else:
                    rotate_ccw(rot_speed)
                time.sleep(0.05)
                motors.stop_all()
                if time.time() - start_t > timeout:
                    break
            motors.stop_all()
            return True

        course_heading = tracker_z.get_heading()

        def set_course_to_current():
            nonlocal course_heading
            try:
                course_heading = tracker_z.get_heading()
            except Exception:
                pass

        def apply_heading_hold(speed, kp=0.02):
            try:
                cur = tracker_z.get_heading()
            except Exception:
                cur = course_heading
            err = shortest_angle_diff(cur, course_heading)
            corr = kp * err
            left = max(-1.0, min(1.0, speed + corr))
            right = max(-1.0, min(1.0, speed - corr))
            motors.set_motor('front_left', left)
            motors.set_motor('back_left', left)
            motors.set_motor('front_right', right)
            motors.set_motor('back_right', right)

        while True:
            if _stop_event.is_set():
                motors.stop_all()
                return

            start_heading = tracker_z.get_heading()
            course_heading = start_heading

            # 1) Drive forward until front sensor sees <=5cm
            while True:
                if _stop_event.is_set():
                    motors.stop_all()
                    return
                d = sensor_front.get_distance_cm()
                if d is not None and d <= 5.0:
                    break
                apply_heading_hold(speed, kp=0.04)
                time.sleep(0.05)

            # brief stop before executing the three-step maneuver
            motors.stop_all()
            time.sleep(0.1)

            # COOLDOWN: Prüfen, ob seit letzter automatischer Drehung COOLDOWN_SECONDS vergangen sind
            now = time.time()
            if now - _last_auto_turn_time < COOLDOWN_SECONDS:
                print(f"INFO: Auto-Turn Cooldown aktiv, warte {COOLDOWN_SECONDS - (now - _last_auto_turn_time):.1f}s")
                time.sleep(COOLDOWN_SECONDS - (now - _last_auto_turn_time))
            _last_auto_turn_time = time.time()

            # 2) rotate clockwise 1s
            rotate_clockwise(speed)
            start = time.time()
            while time.time() - start < 1.0:
                if _stop_event.is_set():
                    motors.stop_all()
                    return
                time.sleep(0.02)
            motors.stop_all()
            time.sleep(0.1)

            # 3) backward 1s
            forward(-speed)
            start = time.time()
            while time.time() - start < 1.0:
                if _stop_event.is_set():
                    motors.stop_all()
                    return
                time.sleep(0.02)
            motors.stop_all()
            time.sleep(0.1)

            # 4) rotate to absolute heading (start_heading - 90°)
            target = (start_heading - 90.0) % 360.0
            rotate_towards_target(target, rot_speed=0.4, tol_deg=5.0, timeout=5.0)

            time.sleep(0.15)
    finally:
        motors.stop_all()


@app.route('/api/stop_motors', methods=['POST'])
def api_stop_motors():
    # set the stop event and immediately stop motors
    _stop_event.set()
    try:
        motors.stop_all()
    except Exception:
        pass
    return jsonify({'status': 'stopped'})


@app.route('/api/turn_left', methods=['POST'])
def api_turn_left():
    """Immediately rotate -90 degrees (left/CCW) without backing up.
    This will signal any running collect sequence to stop, perform the turn,
    then return the result.
    """
    # remember if the collect (ecken_handling) sequence was running so we
    # can resume it after the manual rotation
    try:
        was_collect_running = _collect_lock.locked()
    except Exception:
        was_collect_running = False

    # request immediate stop of any background sequence
    _stop_event.set()
    # wait briefly for a running collect thread to exit and release the lock
    wait_start = time.time()
    while time.time() - wait_start < 1.5:
        # try to acquire the collect lock briefly to test if it's held
        acquired = _collect_lock.acquire(blocking=False)
        if acquired:
            # no collector was running (or it released): release and continue
            _collect_lock.release()
            break
        time.sleep(0.05)

    # clear stop so the rotation helper won't immediately abort
    _stop_event.clear()
    try:
        ok = _rotate_in_place(-90.0, rot_speed=0.45, tol_deg=4.0, timeout=8.0)
    except Exception:
        ok = False

    # If the collect sequence was running before the manual turn, restart it
    if was_collect_running:
        # Try to start the collect worker (same logic as api_start_collect)
        if _collect_lock.acquire(blocking=False):
            def _worker():
                try:
                    ecken_handling_sequence()
                finally:
                    _collect_lock.release()

            t = threading.Thread(target=_worker, daemon=True)
            t.start()

    return jsonify({'status': 'ok' if ok else 'failed'})


@app.route('/api/turn_right', methods=['POST'])
def api_turn_right():
    """Immediately rotate +90 degrees (right/CW) without backing up.
    """
    try:
        was_collect_running = _collect_lock.locked()
    except Exception:
        was_collect_running = False

    _stop_event.set()
    wait_start = time.time()
    while time.time() - wait_start < 1.5:
        acquired = _collect_lock.acquire(blocking=False)
        if acquired:
            _collect_lock.release()
            break
        time.sleep(0.05)
    _stop_event.clear()
    try:
        ok = _rotate_in_place(90.0, rot_speed=0.45, tol_deg=4.0, timeout=8.0)
    except Exception:
        ok = False

    if was_collect_running:
        if _collect_lock.acquire(blocking=False):
            def _worker():
                try:
                    ecken_handling_sequence()
                finally:
                    _collect_lock.release()

            t = threading.Thread(target=_worker, daemon=True)
            t.start()

    return jsonify({'status': 'ok' if ok else 'failed'})


@app.route('/api/start_collect', methods=['POST'])
def api_start_collect():
    # start the ecken_handling sequence in a background thread; prevent concurrent runs
    if not _collect_lock.acquire(blocking=False):
        return jsonify({'status': 'already_running'})

    def _worker():
        try:
            ecken_handling_sequence()
        finally:
            _collect_lock.release()

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    return jsonify({'status': 'started'})


@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/control')
def control():
    """Serve a minimal full-screen control page with two big buttons (left/right)."""
    return send_from_directory(app.static_folder, 'control.html')


@app.route('/api/status')
def api_status():
    try:
        front = sensor_front.get_distance_cm()
    except Exception:
        front = None
    try:
        right = sensor_right.get_distance_cm()
    except Exception:
        right = None
    # get live gyro rates per axis (deg/s). Use try/except in case the sensor read fails
    try:
        gx = gyro.get_gyro_x()
    except Exception:
        gx = None
    try:
        gy = gyro.get_gyro_y()
    except Exception:
        gy = None
    try:
        gz = gyro.get_gyro_z()
    except Exception:
        gz = None

    # collect heading (Z axis) from the tracker
    hz = tracker_z.get_heading()

    # Cooldown für automatische Drehung anzeigen (Restzeit in Sekunden)
    global _last_auto_turn_time, COOLDOWN_SECONDS
    cooldown_left = max(0.0, COOLDOWN_SECONDS - (time.time() - _last_auto_turn_time))
    return jsonify({
        'distance_front_cm': None if front is None else round(front, 1),
        'distance_right_cm': None if right is None else round(right, 1),
        'front_is_mock': bool(sensor_front.mock),
        'right_is_mock': bool(sensor_right.mock),
        'timestamp': int(time.time()),
        'gyro_rate_z_dps': None if gz is None else round(gz, 2),
        'gyro_is_mock': bool(gyro.dev is None),
        # rotation direction (Z axis)
        'rotation_dir_z': (None if gz is None else ('CW' if (tracker_z.sign * gz) > 0 else ('CCW' if (tracker_z.sign * gz) < 0 else 'stopped'))),
        'heading_z_deg': round(hz, 2),
        'auto_turn_cooldown': round(cooldown_left, 1)
    })


@app.route('/api/reset_heading', methods=['POST'])
def api_reset_heading():
    tracker_z.reset()
    return jsonify({'status': 'ok', 'heading_z_deg': 0.0})


if __name__ == '__main__':
    # Use 0.0.0.0 so the UI is reachable from other devices on the LAN
    app.run(host='0.0.0.0', port=5000, debug=False)
