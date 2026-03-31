import serial
import time
import threading


class RobotSerial:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=1):
        self.port = port
        self.baud = baud
        self.timeout = timeout

        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2)

        self._forward = 0
        self._turn = 0
        self._running = True
        self._lock = threading.Lock()

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while self._running:
            with self._lock:
                cmd = f"M,{int(self._forward)},{int(self._turn)}\n"

            self.ser.write(cmd.encode())
            time.sleep(0.05)  # 20Hz sending

    def send(self, forward, turn):
        with self._lock:
            self._forward = forward
            self._turn = turn

    # high-level controls
    def forward(self, speed):
        self.send(speed, 0)

    def backward(self, speed):
        self.send(-speed, 0)

    def left(self, speed):
        self.send(0, -speed)

    def right(self, speed):
        self.send(0, speed)

    def stop(self):
        self.send(0, 0)

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.ser.close()
        print("Serial closed")