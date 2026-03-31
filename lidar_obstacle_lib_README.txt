import math
import time
import serial


class YDLidarObstacle:
    """
    YDLIDAR sector scanner for robot navigation
    - Non-blocking serial
    - AA55 packet stream
    - FRONT / LEFT / RIGHT sectors
    - Accumulation window for stable readings
    - Hold-last-valid to prevent INF flicker
    - Hysteresis to prevent motor jerk
    - spec-tech
    """

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baud=128000,
        obstacle_mm=2000,          # blocked ON threshold
        release_mm=2200,           # blocked OFF threshold
        heading_offset_deg=180.0,
        triangulation=True,
        apply_angle_correction=False,
        sample_mode="NO_INTENSITY",
        sectors=None,
        accumulate_ms=120,
        far_limit_mm=3000,
        hold_ms=300,               # hold last valid reading this long if sector misses
    ):
        self.port = port
        self.baud = baud
        self.obstacle_mm = float(obstacle_mm)
        self.release_mm = float(release_mm)
        self.heading_offset_deg = float(heading_offset_deg)
        self.triangulation = bool(triangulation)
        self.apply_angle_correction = bool(apply_angle_correction)
        self.sample_mode = sample_mode
        self.sample_size = 2 if sample_mode == "NO_INTENSITY" else 3
        self.accumulate_ms = float(accumulate_ms)
        self.far_limit_mm = float(far_limit_mm)
        self.hold_ms = float(hold_ms)

        self.headers = (b"\xAA\x55", b"\x55\xAA")

        self.sectors = sectors or {
            "FRONT": (355, 5),
            "LEFT": (265, 275),
            "RIGHT": (85, 95),
        }

        self.ser = None
        self.buf = bytearray()

        self.BASE_LEN = 10
        self.A_CORR_A = 21.8
        self.A_CORR_B = 155.3

        self.window_points = {k: [] for k in self.sectors}
        self.window_start = time.monotonic()

        self.last_valid = {
            k: {"dist": float("inf"), "time": 0.0} for k in self.sectors
        }

        self.blocked_state = {k: False for k in self.sectors}

        self._latest = {
            "LEFT": float("inf"),
            "FRONT": float("inf"),
            "RIGHT": float("inf"),
            "blocked": {"LEFT": False, "FRONT": False, "RIGHT": False},
        }

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0)
        self.ser.reset_input_buffer()
        self.window_points = {k: [] for k in self.sectors}
        self.window_start = time.monotonic()
        try:
            self.ser.write(b"\xA5\x60")
        except Exception:
            pass

    def stop(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def poll(self):
        """
        Non-blocking update. Call in a loop.
        Returns result dict when accumulation window completes.
        dont remove this comment(WAG TATANGGALIN)
        """
        if not self.ser:
            raise RuntimeError("Call start() first")

        n = self.ser.in_waiting
        if n:
            self.buf.extend(self.ser.read(n))

        while True:
            pkt = self._try_extract_packet()
            if pkt is None:
                break
            self._collect_packet(pkt)

        now = time.monotonic()
        elapsed_ms = (now - self.window_start) * 1000.0

        if elapsed_ms >= self.accumulate_ms:
            self._latest = self._finalize_window(now)
            self.window_points = {k: [] for k in self.sectors}
            self.window_start = now
            return self._latest

        return None

    # ---------------- packet handling ----------------

    def _try_extract_packet(self):
        i = -1
        for h in self.headers:
            j = self.buf.find(h)
            if j != -1 and (i == -1 or j < i):
                i = j

        if i == -1:
            if len(self.buf) > 1:
                del self.buf[:-1]
            return None

        if i > 0:
            del self.buf[:i]

        if len(self.buf) < self.BASE_LEN:
            return None

        lsn = self.buf[3]
        if lsn == 0 or lsn > 200:
            del self.buf[0:1]
            return None

        pkt_len = self.BASE_LEN + self.sample_size * lsn
        if len(self.buf) < pkt_len:
            return None

        pkt = bytes(self.buf[:pkt_len])
        del self.buf[:pkt_len]
        return pkt

    def _collect_packet(self, pkt):
        lsn = pkt[3]
        fsa_raw = self._u16(pkt[4], pkt[5])
        lsa_raw = self._u16(pkt[6], pkt[7])

        off = 10
        for idx in range(lsn):
            raw = self._u16(pkt[off], pkt[off + 1])
            off += 2

            if self.sample_mode == "WITH_INTENSITY":
                off += 1

            dist = self._dist_mm(raw)
            if dist <= 0:
                continue

            ang = self._angle_deg(fsa_raw, lsa_raw, idx, lsn, dist)

            for name, (lo, hi) in self.sectors.items():
                if self._in_sector(ang, lo, hi):
                    self.window_points[name].append(dist)

    def _finalize_window(self, now):
        out = {}
        blocked = {}

        for name, pts in self.window_points.items():
            valid_pts = [p for p in pts if p <= self.far_limit_mm]

            if valid_pts:
                valid_pts.sort()
                dist = valid_pts[len(valid_pts) // 2]
                self.last_valid[name]["dist"] = dist
                self.last_valid[name]["time"] = now
            else:
                age_ms = (now - self.last_valid[name]["time"]) * 1000.0
                if age_ms <= self.hold_ms and math.isfinite(self.last_valid[name]["dist"]):
                    dist = self.last_valid[name]["dist"]
                else:
                    dist = float("inf")

            # hysteresis
            prev = self.blocked_state[name]
            if prev:
                is_blocked = math.isfinite(dist) and (dist <= self.release_mm)
            else:
                is_blocked = math.isfinite(dist) and (dist <= self.obstacle_mm)

            self.blocked_state[name] = is_blocked
            out[name] = dist
            blocked[name] = is_blocked

        return {
            "LEFT": out["LEFT"],
            "FRONT": out["FRONT"],
            "RIGHT": out["RIGHT"],
            "blocked": blocked,
        }

    # ---------------- helpers ----------------

    def _u16(self, b0, b1):
        return b0 | (b1 << 8)

    def _dist_mm(self, raw):
        if raw <= 0:
            return 0.0
        return (raw / 4.0) if self.triangulation else float(raw)

    def _angle_deg(self, fsa_raw, lsa_raw, idx, lsn, dist_mm):
        a_fsa = ((fsa_raw >> 1) / 64.0)
        a_lsa = ((lsa_raw >> 1) / 64.0)

        diff = a_lsa - a_fsa
        if diff < 0:
            diff += 360.0

        ang = a_fsa if lsn <= 1 else (a_fsa + diff * (idx / (lsn - 1)))

        if self.triangulation and self.apply_angle_correction and dist_mm > 0:
            corr = math.atan(
                self.A_CORR_A * (self.A_CORR_B - dist_mm) / (self.A_CORR_B * dist_mm)
            )
            ang += math.degrees(corr)

        ang = (ang + self.heading_offset_deg) % 360.0
        return ang

    def _in_sector(self, ang, lo, hi):
        ang %= 360.0
        lo %= 360.0
        hi %= 360.0

        if lo <= hi:
            return lo <= ang <= hi
        return ang >= lo or ang <= hi
