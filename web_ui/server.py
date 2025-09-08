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
from math import fmod
from flask import Flask, jsonify, request, send_from_directory

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

from .gyro_sensor import GyroSensor

app = Flask(__name__, static_folder="static", static_url_path="")


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

    def get_distance_cm(self):
        if self.mock:
            # simple mock value that changes slowly
            return 30.0 + 10.0 * (0.5 - (time.time() % 3) / 3)

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


# Instantiate sensors using BCM pin numbers from your pin map (see raspberry-pins.txt)
# GyroSensor arguments: address=0x68, use_raw_lsb=False by default
gyro = GyroSensor(use_raw_lsb=False, mock=False, calibrate_on_init=False)
sensor_front = UltrasonicSensor(trig_pin=15, echo_pin=14)  # front TRIG=BCM15, ECHO=BCM14
sensor_right = UltrasonicSensor(trig_pin=23, echo_pin=24)  # right TRIG=BCM23, ECHO=BCM24

# Start gyro integration thread
gyro.start()



@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')


# runtime sign multiplier to flip rotation sense when needed
GYRO_SIGN = 1


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

    # ensure we update integration once before reading
    try:
        angle = gyro.update_angle()
    except Exception:
        angle = gyro.get_current_angle()

    # apply sign multiplier for user convenience
    angle_signed = GYRO_SIGN * angle
    heading = (angle_signed - gyro.heading_offset) % 360.0
    return jsonify({
        'distance_front_cm': None if front is None else round(front, 1),
        'distance_right_cm': None if right is None else round(right, 1),
        'angle_z_deg': round(angle_signed, 2),
        'heading_deg': round(heading, 2),
        'gyro_calibrated': gyro.is_calibrated
    })


@app.route('/api/reset_heading', methods=['POST'])
def api_reset_heading():
    gyro.reset_angle()
    gyro.calibrate_heading()
    return jsonify({'status': 'ok', 'heading_deg': 0.0})


@app.route('/api/calibrate', methods=['POST'])
def api_calibrate():
    # optional JSON payload: { samples: int }
    data = request.get_json(silent=True) or {}
    samples = int(data.get('samples', 200))
    offset = gyro.calibrate(samples=samples)
    return jsonify({'status': 'ok', 'gyro_z_offset_deg_per_s': offset})


@app.route('/api/set_sign', methods=['POST'])
def api_set_sign():
    global GYRO_SIGN
    data = request.get_json(silent=True) or {}
    sign = int(data.get('sign', 1))
    GYRO_SIGN = 1 if sign >= 0 else -1
    return jsonify({'status': 'ok', 'sign': GYRO_SIGN})


if __name__ == '__main__':
    # Use 0.0.0.0 so the UI is reachable from other devices on the LAN
    app.run(host='0.0.0.0', port=5000, debug=False)
