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
            print("WARNING: MPU6050 library not found. Using mock sensor.")
            self.dev = None
            # Mock parameters to make demo gyro more realistic and variable
            self._gyro_phase = random.random() * 2.0 * math.pi
            self._gyro_amp = 20.0  # deg/s peak
            self._gyro_freq = 0.08  # Hz (slow rotation)

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

        # Mock: produce smooth sinusoidal values with slightly different params per axis
        t = time.time()
        if not hasattr(self, '_mock_gyro_params'):
            # deterministic-ish per-axis mock parameters
            self._mock_gyro_params = {}
            base_amp = {'x': 10.0, 'y': 12.0, 'z': 20.0}
            base_freq = {'x': 0.11, 'y': 0.09, 'z': 0.08}
            for ax in ('x', 'y', 'z'):
                self._mock_gyro_params[ax] = {
                    'amp': base_amp[ax],
                    'freq': base_freq[ax],
                    'phase': random.random() * 2.0 * math.pi,
                    'noise': 1.0 + random.random() * 0.6,
                }

        p = self._mock_gyro_params.get(a, self._mock_gyro_params['z'])
        return p['amp'] * math.sin(2.0 * math.pi * p['freq'] * t + p['phase']) + random.uniform(-p['noise'], p['noise'])


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
    def __init__(self, gyro_sensor, poll_interval=0.05, sign=1, axis='z'):
        self.gyro = gyro_sensor
        self.poll = poll_interval
        self._raw = 0.0
        self._offset = 0.0
        # sign: 1 keeps gyro sign as-is, -1 inverts rotation direction used for integration
        self.sign = 1 if sign >= 0 else -1
        # axis: 'x','y' or 'z' - which gyro axis to integrate
        self.axis = axis.lower()
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
                rate = getter()
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
# Create a HeadingTracker per axis so we can test integration on x/y/z independently.
trackers = {
    'x': HeadingTracker(gyro, sign=-1, axis='x'),
    'y': HeadingTracker(gyro, sign=-1, axis='y'),
    'z': HeadingTracker(gyro, sign=-1, axis='z'),
}
for t in trackers.values():
    t.start()

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

    # collect headings per axis from their trackers
    hx = trackers['x'].get_heading()
    hy = trackers['y'].get_heading()
    hz = trackers['z'].get_heading()

    return jsonify({
        'distance_front_cm': None if front is None else round(front, 1),
        'distance_right_cm': None if right is None else round(right, 1),
    'front_is_mock': bool(sensor_front.mock),
    'right_is_mock': bool(sensor_right.mock),
    'timestamp': int(time.time()),
        'gyro_rate_x_dps': None if gx is None else round(gx, 2),
        'gyro_rate_y_dps': None if gy is None else round(gy, 2),
        'gyro_rate_z_dps': None if gz is None else round(gz, 2),
        'gyro_is_mock': bool(gyro.dev is None),
        # rotation directions per axis
        'rotation_dir_x': (None if gx is None else ('CW' if (trackers['x'].sign * gx) > 0 else ('CCW' if (trackers['x'].sign * gx) < 0 else 'stopped'))),
        'rotation_dir_y': (None if gy is None else ('CW' if (trackers['y'].sign * gy) > 0 else ('CCW' if (trackers['y'].sign * gy) < 0 else 'stopped'))),
        'rotation_dir_z': (None if gz is None else ('CW' if (trackers['z'].sign * gz) > 0 else ('CCW' if (trackers['z'].sign * gz) < 0 else 'stopped'))),
        'heading_x_deg': round(hx, 2),
        'heading_y_deg': round(hy, 2),
        'heading_z_deg': round(hz, 2)
    })


@app.route('/api/reset_heading', methods=['POST'])
def api_reset_heading():
    for t in trackers.values():
        t.reset()
    return jsonify({'status': 'ok', 'heading_x_deg': 0.0, 'heading_y_deg': 0.0, 'heading_z_deg': 0.0})


if __name__ == '__main__':
    # Use 0.0.0.0 so the UI is reachable from other devices on the LAN
    app.run(host='0.0.0.0', port=5000, debug=False)
