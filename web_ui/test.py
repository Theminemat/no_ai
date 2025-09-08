# ultra_test.py
import time
import RPi.GPIO as GPIO

TRIG = 15
ECHO = 14

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure():
    GPIO.output(TRIG, False)
    time.sleep(0.0002)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start = time.time()
    timeout = start + 0.02
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        start = time.time()
    stop = time.time()
    timeout = stop + 0.02
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        stop = time.time()
    elapsed = stop - start
    d = elapsed * 34300.0 / 2.0
    return d

try:
    for i in range(5):
        print(round(measure(),1), "cm")
        time.sleep(1.0)
finally:
    GPIO.cleanup()