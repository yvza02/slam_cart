import time
import threading
from smbus import SMBus

MPU_ADDR = 0x68
bus = SMBus(1)

DEADBAND = 0.5

_offset_z = 0.0
_angle = 0.0
_running = False
_thread = None
_lock = threading.Lock()


def _read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 65536
    return val


def _read_gyro_z():
    raw = _read_word(0x47)
    return raw / 131.0


def init():
    last_err = None
    for _ in range(5):
        try:
            bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)  # wake up
            bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)  # gyro ±250 dps
            time.sleep(0.1)
            return
        except OSError as e:
            last_err = e
            print("Gyro init error:", e)
            time.sleep(0.1)

    raise last_err


def calibrate(samples=500):
    global _offset_z
    total = 0.0
    good = 0

    print("Calibrating gyro... keep robot still")

    while good < samples:
        try:
            total += _read_gyro_z()
            good += 1
            time.sleep(0.005)
        except OSError as e:
            print("Gyro calibrate read error:", e)
            time.sleep(0.02)

    _offset_z = total / good
    print("Gyro Z offset =", _offset_z)


def reset():
    global _angle
    with _lock:
        _angle = 0.0


def _worker():
    global _angle, _running
    last_time = time.time()

    while _running:
        try:
            now = time.time()
            dt = now - last_time
            last_time = now

            gz = _read_gyro_z() - _offset_z
            if abs(gz) < DEADBAND:
                gz = 0.0

            with _lock:
                _angle += gz * dt

        except OSError as e:
            print("Gyro I2C read error:", e)
            last_time = time.time()
            time.sleep(0.02)

        time.sleep(0.005)


def start():
    global _running, _thread
    if _running:
        return

    _running = True
    _thread = threading.Thread(target=_worker, daemon=True)
    _thread.start()


def stop():
    global _running, _thread
    _running = False
    if _thread is not None:
        _thread.join(timeout=1.0)
        _thread = None


def get_angle():
    with _lock:
        return int(_angle)