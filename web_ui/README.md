RPi Sensors Web UI
===================

Small local web UI to view two HC-SR04 distances and current heading from MPU6050.

Files:
- `server.py` - Flask server + sensor wrappers
- `static/index.html` - single-page UI that polls the server
- `requirements.txt` - Python dependencies

How to run (on Raspberry Pi):

1. Create and activate a Python 3 venv (recommended).
2. Install dependencies:

   pip install -r requirements.txt

3. Run the server as root or with access to GPIO/I2C:

   python server.py

4. Open http://<raspberry-pi-ip>:5000/ in a browser on your LAN.

Notes and safety:
- The server uses BCM pin numbers matching `raspberry-pins.txt` in the repo. Update the pin numbers
  in `server.py` if your wiring differs.
- If `RPi.GPIO` or `mpu6050` aren't available, the server runs in mock mode and returns simulated values.
- For production use, consider running the Flask app under a proper WSGI server and adding authentication.
