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

app = Flask(__name__, static_folder="static", static_url_path="")


class GyroSensor:
    """Minimal wrapper around MPU6050 to provide Z-axis rotation rate in deg/s.
    Falls back to a mock that returns a slow rotation for demo/testing.
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

        if not self._use_smbus and HAS_MPU:
            print("INFO: MPU6050 library found (fallback).")
            try:
                self.dev = mpu6050(address, bus)
                print("INFO: MPU6050 device initialized successfully via library.")
            except Exception as e:
                print(f"ERROR: Failed to initialize MPU6050 via library on bus {bus} at address {hex(address)}: {e}")
                self.dev = None
        else:
            print("WARNING: MPU6050 library not found. MPU6050 unavailable; gyro reads will return 0.0")
            self.dev = None
            # No demo/mock gyro signal anymore. When hardware is not present we return 0.0.

    def get_gyro_z(self):
        """Return gyroscope Z rate in degrees/second (positive = clockwise when looking down).
        """
        # keep compatibility
        return self.get_gyro('z')

    def get_gyro_x(self):
        return self.get_gyro('x')

    def get_gyro_y(self):
        return self.get_gyro('y')

    def get_gyro(self, axis='z'):
        """Return gyroscope rate for axis 'x','y' or 'z' in deg/s.
        Falls back to a mock signal when no device is present.
        """
        a = axis.lower()
        if self._use_smbus:
            try:
                # read 6 bytes starting at GYRO_XOUT_H (0x43)
                data = self._bus.read_i2c_block_data(self._address, 0x43, 6)
                def to_signed(h, l):
                    v = (h << 8) | l
                    return v - 65536 if v & 0x8000 else v
                gx = to_signed(data[0], data[1])
                gy = to_signed(data[2], data[3])
                gz = to_signed(data[4], data[5])
                raw = {'x': gx, 'y': gy, 'z': gz}
                # default FS = ±250 dps -> scale = 131 LSB/(deg/s)
                scale = 131.0
                return float(raw.get(a, 0.0)) / scale
            except Exception:
                return 0.0

        if self.dev:
            try:
                g = self.dev.get_gyro_data()  # returns x/y/z in deg/s
                return float(g.get(a, 0.0))
            except Exception:
                return 0.0

        # No mock/demo gyro signal: return 0.0 when no device is present.
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
            # GPIO not available or pins not specified. Do not generate demo distance values.
            # get_distance_cm() will return None to indicate the sensor is unavailable.
            pass

    def get_distance_cm(self):
        if self.mock:
            # No hardware available: indicate unavailable explicitly
            return None

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
    def __init__(self, gyro_sensor, poll_interval=0.05, sign=1, axis='z', min_rate_thresh=0.5):
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
            # Treat missing device OR missing smbus as a mock. Previously this used AND which
            # could mark the sensor as "real" when _use_smbus is True but dev is None,
            # leading to spurious small reads being integrated slowly.
            is_gyro_mock = (getattr(self.gyro, 'dev', None) is None) or (not getattr(self.gyro, '_use_smbus', False))
            if rate is None:
                rate = 0.0
            if is_gyro_mock or abs(rate) < self.min_rate_thresh:
                # nothing to integrate; just sleep and continue
                time.sleep(max(0.0, self.poll - 0.0))
                last = time.time()
                continue

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
# Keep only a single HeadingTracker for the Z axis (heading) as requested.
tracker_z = HeadingTracker(gyro, sign=-1, axis='z')
tracker_z.start()

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
    'heading_z_deg': round(hz, 2)
    })


@app.route('/api/reset_heading', methods=['POST'])
def api_reset_heading():
    tracker_z.reset()
    return jsonify({'status': 'ok', 'heading_z_deg': 0.0})


if __name__ == '__main__':
    # Use 0.0.0.0 so the UI is reachable from other devices on the LAN
    app.run(host='0.0.0.0', port=5000, debug=False)
