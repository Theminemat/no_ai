import time
import threading
from typing import Optional

try:
    from mpu6050 import mpu6050
    HAS_MPU = True
except Exception:
    HAS_MPU = False


class GyroSensor:
    """Gyro wrapper implementing the algorithm you described.

    - Converts raw to °/s via raw/131.0 when use_raw_lsb=True
    - Otherwise expects dev.get_gyro_data()['z'] in °/s already
    - Calibrate to compute gyro_z_offset (°/s)
    - update_angle integrates over dt: angle_z += (angular_velocity) * dt
    - reset_angle sets angle_z = 0
    - get_heading returns normalized 0..359.999 of (angle_z - heading_offset)

    Notes/assumptions:
    - Many MPU libraries already return deg/s; to support boards that return raw LSBs,
      pass use_raw_lsb=True. If unsure, leave False. This file documents the assumption.
    """

    def __init__(self, address: int = 0x68, use_raw_lsb: bool = False, mock: bool = False,
                 poll_interval: float = 0.02, calibrate_on_init: bool = False,
                 calibrate_samples: int = 200):
        self.address = address
        self.use_raw_lsb = bool(use_raw_lsb)
        self.mock = bool(mock) or not HAS_MPU
        self.dev = None
        if not self.mock and HAS_MPU:
            try:
                self.dev = mpu6050(self.address)
            except Exception:
                self.dev = None

        # bias in °/s (to subtract from raw conversion)
        self.gyro_z_offset = 0.0

        # integrated angle in degrees (can be >360 or negative)
        self.angle_z = 0.0
        self.heading_offset = 0.0

        self._lock = threading.Lock()
        self._last_time = time.monotonic()
        self._poll = float(poll_interval)

        # background thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # mock rotation in deg/s (used when mock=True)
        self._mock_rot = 0.0

        if calibrate_on_init:
            try:
                self.calibrate(samples=calibrate_samples)
            except Exception:
                pass

    def _read_raw_z(self) -> float:
        """Read raw gyro Z value. If using a library that returns deg/s, this will be that value.
        If use_raw_lsb is True but low-level access is unavailable, we fall back to library value.
        Mock mode returns the configured mock rotation value.
        """
        if self.mock:
            return self._mock_rot

        if not self.dev:
            return 0.0

        try:
            g = self.dev.get_gyro_data()  # usually returns {'x':..,'y':..,'z':..} in deg/s
            raw = g.get('z', 0.0)
        except Exception:
            # fallback: try attribute names some wrappers provide
            try:
                rd = getattr(self.dev, 'get_raw_gyro', None)
                if callable(rd):
                    raw = rd()['z']
                else:
                    raw = 0.0
            except Exception:
                raw = 0.0

        return raw

    def get_angular_velocity_z(self) -> float:
        """Return angular velocity in °/s after converting and subtracting offset.

        If use_raw_lsb is True we apply raw/131.0; otherwise we assume raw already in °/s.
        """
        raw = self._read_raw_z()
        if self.use_raw_lsb:
            angular_velocity = (raw / 131.0) - self.gyro_z_offset
        else:
            angular_velocity = raw - self.gyro_z_offset
        return angular_velocity

    def calibrate(self, samples: int = 200, delay: float = 0.005) -> float:
        """Calibrate gyro_z_offset by averaging samples while sensor is still.
        Returns the computed offset (°/s).
        """
        total = 0.0
        count = 0
        for _ in range(samples):
            raw = self._read_raw_z()
            if self.use_raw_lsb:
                v = raw / 131.0
            else:
                v = raw
            total += v
            count += 1
            time.sleep(delay)

        avg = total / max(1, count)
        with self._lock:
            self.gyro_z_offset = avg
        return self.gyro_z_offset

    def reset_angle(self) -> None:
        """Set current integrated angle to 0 and reset time base."""
        with self._lock:
            self.angle_z = 0.0
            self._last_time = time.monotonic()

    def calibrate_heading(self) -> None:
        """Make the current integrated angle the heading zero (heading_offset)."""
        with self._lock:
            self.heading_offset = self.angle_z

    def get_current_angle(self) -> float:
        """Return integrated angle (may be >360 or negative)."""
        with self._lock:
            return float(self.angle_z)

    def get_heading(self) -> float:
        """Return heading normalized to [0,360)."""
        with self._lock:
            h = (self.angle_z - self.heading_offset) % 360.0
            if h < 0:
                h += 360.0
            return float(h)

    def update_angle(self) -> float:
        """Perform a single integration step and return the updated angle_z.

        Formula used (per your spec):
          angle_z += (raw/131.0 - gyro_z_offset) * dt   (when use_raw_lsb=True)
          angle_z += (raw - gyro_z_offset) * dt         (when use_raw_lsb=False)
        """
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_time
            self._last_time = now
        if dt <= 0:
            return self.get_current_angle()

        angular_velocity = self.get_angular_velocity_z()
        delta = angular_velocity * dt
        with self._lock:
            self.angle_z += delta
            return float(self.angle_z)

    def set_mock_rotation(self, deg_per_sec: float) -> None:
        self._mock_rot = float(deg_per_sec)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while self._running:
            self.update_angle()
            time.sleep(self._poll)

    @property
    def is_calibrated(self) -> bool:
        return abs(self.gyro_z_offset) > 0.0
