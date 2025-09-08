#!/usr/bin/env python3
"""Ultrasonic sensor tester: choose 'front' or 'right' to test the corresponding HC-SR04.

Usage:
  python3 ultra_test.py --sensor front
  python3 ultra_test.py --sensor right

Pins (BCM):
  front: TRIG=15, ECHO=14
  right: TRIG=23, ECHO=24

The script prints continuous distance measurements (cm) until Ctrl+C.
"""
import time
import argparse

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

SENSOR_MAP = {
    'front': {'trig': 15, 'echo': 14},
    'right': {'trig': 23, 'echo': 24},
}

def measure_once(trig, echo):
    # perform one HC-SR04 measurement
    GPIO.output(trig, False)
    time.sleep(0.0002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    timeout = start + 0.02
    while GPIO.input(echo) == 0 and time.time() < timeout:
        start = time.time()

    stop = time.time()
    timeout = stop + 0.02
    while GPIO.input(echo) == 1 and time.time() < timeout:
        stop = time.time()

    elapsed = stop - start
    distance = elapsed * 34300.0 / 2.0
    return max(0.0, distance)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--sensor', choices=['front','right'], default='front')
    p.add_argument('--count', type=int, default=0, help='number of readings (0 = continuous)')
    args = p.parse_args()

    if args.sensor not in SENSOR_MAP:
        print('Unknown sensor')
        return

    trig = SENSOR_MAP[args.sensor]['trig']
    echo = SENSOR_MAP[args.sensor]['echo']

    if not HAS_GPIO:
        print('RPi.GPIO not available. Running in mock mode (no hardware).')
        # Simple mock: print varying values
        i = 0
        try:
            while args.count == 0 or i < args.count:
                val = 30.0 + 10.0 * (0.5 - (time.time() % 6) / 6.0) + (i % 5)
                print(f'{args.sensor}: {val:.1f} cm (mock)')
                time.sleep(1.0)
                i += 1
        except KeyboardInterrupt:
            pass
        return

    print(f'Testing sensor "{args.sensor}" (TRIG={trig}, ECHO={echo}) — Ctrl+C to stop')
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

    try:
        i = 0
        while args.count == 0 or i < args.count:
            d = measure_once(trig, echo)
            print(f'{args.sensor}: {d:.1f} cm')
            time.sleep(0.8)
            i += 1
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
