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

try:
    from mpu6050 import mpu6050
    HAS_MPU = True
except Exception:
    HAS_MPU = False

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

app = Flask(__name__, static_folder="static", static_url_path="")


class GyroSensor:
    """Minimal wrapper around MPU6050 to provide Z-axis rotation rate in deg/s.
    Falls back to a mock that returns a slow rotation for demo/testing.
    """
    def __init__(self, bus=1, address=0x68):
        if HAS_MPU:
            print("INFO: MPU6050 library found.")
            try:
                self.dev = mpu6050(address, bus)
                print("INFO: MPU6050 device initialized successfully.")
            except Exception as e:
                print(f"ERROR: Failed to initialize MPU6050 on bus {bus} at address {hex(address)}: {e}")
                self.dev = None
        else:
            print("WARNING: MPU6050 library not found. Using mock sensor.")
            self.dev = None
            # Mock parameters to make demo gyro more realistic and variable
            self._gyro_phase = random.random() * 2.0 * math.pi
            self._gyro_amp = 20.0  # deg/s peak
            self._gyro_freq = 0.08  # Hz (slow rotation)

    def get_gyro_z(self):
        """Return gyroscope Z rate in degrees/second (positive = clockwise when looking down).
        """
        if self.dev:
            try:
                g = self.dev.get_gyro_data()  # returns x/y/z in deg/s
                return g.get('z', 0.0)
            except Exception:
                return 0.0
        # Mock: slow sinusoidal rotation with small noise
        t = time.time()
        return self._gyro_amp * math.sin(2.0 * math.pi * self._gyro_freq * t + self._gyro_phase) + random.uniform(-1.0, 1.0)


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
            # Per-instance demo parameters so multiple mock sensors don't return identical values
            # Use pin numbers when available to derive deterministic differences
            seed = (int(self.trig or 0) * 37) ^ (int(self.echo or 0) * 17)
            rnd = random.Random(seed)
            self._base = 25.0 + (rnd.random() * 20.0)  # base distance between ~25..45 cm
            self._amp = 6.0 + (rnd.random() * 8.0)     # amplitude between ~6..14 cm
            self._freq = 0.08 + (rnd.random() * 0.12)  # frequency between ~0.08..0.2 Hz
            self._phase = rnd.random() * 2.0 * math.pi
            self._noise = 0.3 + rnd.random() * 1.2     # small random noise

    def get_distance_cm(self):
        if self.mock:
            # produce a smooth sinusoidal distance plus small random noise
            t = time.time()
            val = self._base + self._amp * math.sin(2.0 * math.pi * self._freq * t + self._phase)
            val += random.uniform(-self._noise, self._noise)
            return max(0.0, val)

        # Send trigger
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
        # speed of sound ~34300 cm/s, distance = elapsed * speed / 2
        distance = elapsed * 34300.0 / 2.0
        return distance


class HeadingTracker:
    """Integrate gyro z-rate to produce a heading in degrees within [0, 360).
    Supports reset() to declare the current heading as 0.
    """
    def __init__(self, gyro_sensor, poll_interval=0.05, sign=1):
        self.gyro = gyro_sensor
        self.poll = poll_interval
        self._raw = 0.0
        self._offset = 0.0
        # sign: 1 keeps gyro sign as-is, -1 inverts rotation direction used for integration
        self.sign = 1 if sign >= 0 else -1
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

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
            rate = self.gyro.get_gyro_z()  # deg/s
            # apply configurable sign so sensor wrapper stays untouched
            delta = (self.sign * rate) * dt
            with self._lock:
                self._raw = fmod((self._raw + delta), 360.0)
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
tracker = HeadingTracker(gyro, sign=-1)
tracker.start()

# Log sensor modes so it's obvious on startup whether real GPIO is used
print(f'INFO: sensor_front mock={sensor_front.mock}, sensor_right mock={sensor_right.mock}, HAS_GPIO={HAS_GPIO}, HAS_MPU={HAS_MPU}')
print(f'INFO: gyro mock={(gyro.dev is None)}')


@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')


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
    # get a live gyro rate (deg/s). Use try/except in case the sensor read fails
    try:
        gyro_rate = gyro.get_gyro_z()
    except Exception:
        gyro_rate = None

    heading = tracker.get_heading()
    return jsonify({
        'distance_front_cm': None if front is None else round(front, 1),
        'distance_right_cm': None if right is None else round(right, 1),
    'front_is_mock': bool(sensor_front.mock),
    'right_is_mock': bool(sensor_right.mock),
    'timestamp': int(time.time()),
        'gyro_rate_dps': None if gyro_rate is None else round(gyro_rate, 2),
        'gyro_is_mock': bool(gyro.dev is None),
        # rotation direction as used for heading integration: sign * rate
        'rotation_dir': (None if gyro_rate is None else ('CW' if (tracker.sign * gyro_rate) > 0 else ('CCW' if (tracker.sign * gyro_rate) < 0 else 'stopped'))),
        'heading_deg': round(heading, 2)
    })


@app.route('/api/reset_heading', methods=['POST'])
def api_reset_heading():
    tracker.reset()
    return jsonify({'status': 'ok', 'heading_deg': 0.0})


if __name__ == '__main__':
    # Use 0.0.0.0 so the UI is reachable from other devices on the LAN
    app.run(host='0.0.0.0', port=5000, debug=False)
