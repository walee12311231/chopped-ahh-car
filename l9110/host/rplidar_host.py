import math
import socket
import struct
import threading
import time
from collections import deque

import numpy as np
import pygame

HOST = "192.168.4.1"
PORT = 8080

CELL_M       = 0.05
GRID_N       = 400
ROBOT_CELL   = GRID_N // 2
MAX_RANGE_M  = 6.0
WIN          = 800
PX_PER_CELL  = WIN // GRID_N

L_OCC, L_FREE, L_MIN, L_MAX = 0.85, -0.4, -5.0, 5.0
P_THRESH_OCC, P_THRESH_FREE = 0.65, 0.35

MAGIC = b"\xA5\x5A"

FRAME_SCAN = 0x01
FRAME_IMU  = 0x02

# Accel scale: MPU6050 ±2g => 16384 LSB/g. Used only for optional dead reckoning.
ACCEL_LSB_PER_G = 16384.0
G = 9.80665


class FrameStats:
    def __init__(self):
        self.ok = 0
        self.bad = 0
        self.last_n = 0
        self.imu = 0
        self.recent = deque(maxlen=120)


stats = FrameStats()
points_lock = threading.Lock()
pose_lock = threading.Lock()
points = []
grid_logodds = np.zeros((GRID_N, GRID_N), dtype=np.float32)
running = True

# Robot pose in world frame: x_m, y_m relative to grid center; yaw_rad CCW.
# Yaw comes from the IMU; x/y dead-reckoning from accel is included for
# completeness but DRIFTS BADLY without wheel odometry — treat as informational.
robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0,
              "vx": 0.0, "vy": 0.0,
              "last_t_ms": None, "have_imu": False}


def parse_stream(sock):
    """Yields ('scan', seq, t_ms, samples, truncated) and ('imu', seq, t_ms, yaw_cdeg, ax, ay, az)."""
    buf = bytearray()
    while running:
        try:
            chunk = sock.recv(4096)
        except OSError:
            return
        if not chunk:
            return
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 8192:
                    del buf[:-1]
                break
            if i > 0:
                del buf[:i]
            if len(buf) < 16:
                break
            ftype = buf[2]
            flags = buf[3]
            seq, t_ms = struct.unpack_from("<II", buf, 4)
            n, plen = struct.unpack_from("<HH", buf, 12)

            if ftype == FRAME_SCAN:
                # scan invariant: plen == n*5
                total = 16 + plen + 1
                if n > 720 or plen != n * 5:
                    del buf[:2]
                    stats.bad += 1
                    continue
            elif ftype == FRAME_IMU:
                # IMU invariant: n=1, plen=10
                total = 16 + plen + 1
                if not (n == 1 and plen == 10):
                    del buf[:2]
                    stats.bad += 1
                    continue
            else:
                del buf[:2]
                stats.bad += 1
                continue

            if len(buf) < total:
                break
            frame = bytes(buf[:total])
            del buf[:total]
            cs = 0
            for b in frame[:-1]:
                cs ^= b
            if cs != frame[-1]:
                stats.bad += 1
                stats.recent.append((time.time(), False))
                continue

            if ftype == FRAME_SCAN:
                samples = []
                off = 16
                for _ in range(n):
                    aq6, dmm, qv = struct.unpack_from("<HHB", frame, off)
                    off += 5
                    deg = (aq6 / 64.0) % 360.0
                    samples.append((deg, dmm, qv))
                stats.ok += 1
                stats.last_n = n
                stats.recent.append((time.time(), True))
                yield ("scan", seq, t_ms, samples, bool(flags & 0x02))
            else:  # IMU
                yaw_cdeg, ax, ay, az, _resv = struct.unpack_from("<hhhhH", frame, 16)
                stats.imu += 1
                yield ("imu", seq, t_ms, yaw_cdeg, ax, ay, az)


def update_pose_from_imu(t_ms, yaw_cdeg, ax_lsb, ay_lsb):
    """Yaw -> trusted; x/y dead reckoning -> noisy, kept for display only."""
    with pose_lock:
        robot_pose["yaw"] = math.radians(yaw_cdeg / 100.0)
        last = robot_pose["last_t_ms"]
        robot_pose["last_t_ms"] = t_ms
        robot_pose["have_imu"] = True
        if last is None:
            return
        dt = (t_ms - last) / 1000.0
        if dt <= 0 or dt > 0.5:
            return
        # Convert accel LSB to m/s^2, rotate body->world by yaw, integrate.
        ax_ms2 = (ax_lsb / ACCEL_LSB_PER_G) * G
        ay_ms2 = (ay_lsb / ACCEL_LSB_PER_G) * G
        cy, sy = math.cos(robot_pose["yaw"]), math.sin(robot_pose["yaw"])
        wx = cy * ax_ms2 - sy * ay_ms2
        wy = sy * ax_ms2 + cy * ay_ms2
        # Heavy decay on velocity: pure-accel odometry is unobservable, this
        # at least keeps the displayed x/y bounded.
        robot_pose["vx"] = robot_pose["vx"] * 0.90 + wx * dt
        robot_pose["vy"] = robot_pose["vy"] * 0.90 + wy * dt
        robot_pose["x"] += robot_pose["vx"] * dt
        robot_pose["y"] += robot_pose["vy"] * dt


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0); dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy


def integrate(samples):
    """Place samples in the world grid using current pose (yaw + x/y).
    Falls back to fixed-pose integration if no IMU has arrived yet."""
    with pose_lock:
        px_m = robot_pose["x"]; py_m = robot_pose["y"]
        yaw  = robot_pose["yaw"]
    cx = ROBOT_CELL + int(round(px_m / CELL_M))
    cy = ROBOT_CELL + int(round(py_m / CELL_M))
    if not (0 <= cx < GRID_N and 0 <= cy < GRID_N):
        return
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    for deg, mm, _q in samples:
        if mm == 0:
            continue
        d_m = mm / 1000.0
        if d_m < 0.10 or d_m > MAX_RANGE_M:
            continue
        # Lidar samples are in body frame (deg about lidar Z, CCW). Rotate by yaw
        # then translate by robot world position to get world cell.
        a = math.radians(deg)
        bx = d_m * math.cos(a)
        by = d_m * math.sin(a)
        wx = cos_y * bx - sin_y * by
        wy = sin_y * bx + cos_y * by
        gx = cx + int(round(wx / CELL_M))
        gy = cy + int(round(wy / CELL_M))
        if not (0 <= gx < GRID_N and 0 <= gy < GRID_N):
            continue
        last = (gx, gy)
        for px, py in bresenham(cx, cy, gx, gy):
            if (px, py) == last:
                break
            grid_logodds[py, px] += L_FREE
        grid_logodds[gy, gx] += L_OCC
    np.clip(grid_logodds, L_MIN, L_MAX, out=grid_logodds)


def confidence():
    if not points:
        return 0.0
    bins = np.zeros(36, dtype=bool)
    qsum = 0; qn = 0
    for deg, mm, q in points:
        if mm == 0:
            continue
        bins[int(deg // 10) % 36] = True
        qsum += q; qn += 1
    coverage = bins.sum() / 36.0
    qual = (qsum / qn / 47.0) if qn else 0.0
    qual = min(qual, 1.0)
    now = time.time()
    recent = [r for r in stats.recent if now - r[0] < 1.0]
    integ = (sum(1 for _, ok in recent if ok) / len(recent)) if recent else 0.0
    score = 0.5 * coverage + 0.3 * qual + 0.2 * integ
    return max(0.0, min(1.0, score))


def receiver():
    global points
    while running:
        try:
            sock = socket.create_connection((HOST, PORT), timeout=3)
            sock.settimeout(2.0)
            sock.sendall(b"s")
        except OSError as e:
            print(f"[net] connect failed: {e}; retry in 1s")
            time.sleep(1.0)
            continue
        print("[net] connected")
        try:
            for item in parse_stream(sock):
                kind = item[0]
                if kind == "scan":
                    _, _seq, _t, samples, _trunc = item
                    with points_lock:
                        points = samples
                    integrate(samples)
                elif kind == "imu":
                    _, _seq, t_ms, yaw_cdeg, ax, ay, _az = item
                    update_pose_from_imu(t_ms, yaw_cdeg, ax, ay)
        except OSError as e:
            print(f"[net] recv err: {e}")
        finally:
            try:
                sock.close()
            except Exception:
                pass
        if running:
            time.sleep(0.5)



def render():
    global running
    pygame.init()
    screen = pygame.display.set_mode((WIN, WIN))
    pygame.display.set_caption("RPLIDAR A1 - SLAM with IMU odometry")
    font = pygame.font.SysFont("Menlo", 14)
    clock = pygame.time.Clock()
    grid_surface = pygame.Surface((GRID_N, GRID_N))

    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False

        p = 1.0 - 1.0 / (1.0 + np.exp(grid_logodds))
        img = np.full((GRID_N, GRID_N, 3), 128, dtype=np.uint8)
        free = p < P_THRESH_FREE
        occ = p > P_THRESH_OCC
        img[free] = (240, 240, 240)
        img[occ]  = (20, 20, 20)
        pygame.surfarray.blit_array(grid_surface, np.transpose(img, (1, 0, 2)))
        screen.blit(pygame.transform.scale(grid_surface, (WIN, WIN)), (0, 0))

        with pose_lock:
            px_m = robot_pose["x"]; py_m = robot_pose["y"]
            yaw  = robot_pose["yaw"]; have_imu = robot_pose["have_imu"]
        rx_cell = ROBOT_CELL + int(round(px_m / CELL_M))
        ry_cell = ROBOT_CELL + int(round(py_m / CELL_M))
        cx_px = rx_cell * PX_PER_CELL
        cy_px = ry_cell * PX_PER_CELL

        with points_lock:
            pts = list(points)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for deg, mm, q in pts:
            if mm == 0:
                continue
            d_m = mm / 1000.0
            if d_m > MAX_RANGE_M:
                continue
            a = math.radians(deg)
            bx = d_m * math.cos(a)
            by = d_m * math.sin(a)
            wx = cos_y * bx - sin_y * by
            wy = sin_y * bx + cos_y * by
            x = cx_px + int(wx / CELL_M * PX_PER_CELL)
            y = cy_px + int(wy / CELL_M * PX_PER_CELL)
            if 0 <= x < WIN and 0 <= y < WIN:
                shade = 80 + min(175, q * 3)
                screen.set_at((x, y), (shade, 255, shade))

        # Robot: red dot + heading line.
        pygame.draw.circle(screen, (255, 64, 64), (cx_px, cy_px), 5)
        hx = cx_px + int(20 * math.cos(yaw))
        hy = cy_px + int(20 * math.sin(yaw))
        pygame.draw.line(screen, (255, 64, 64), (cx_px, cy_px), (hx, hy), 2)

        c = confidence()
        yaw_deg = math.degrees(yaw)
        hud = [
            f"frames ok={stats.ok} bad={stats.bad} imu={stats.imu}  n={stats.last_n}",
            f"confidence: {c:0.2f}",
            f"yaw: {yaw_deg:+7.2f} deg   pose: x={px_m:+.2f} y={py_m:+.2f} m   {'IMU' if have_imu else 'no-IMU'}",
        ]
        for i, line in enumerate(hud):
            screen.blit(font.render(line, True, (255, 255, 0)), (8, 8 + i * 16))
        pygame.draw.rect(screen, (60, 60, 60), (8, 60, 200, 10))
        pygame.draw.rect(screen, (0, 200, 0),   (8, 60, int(200 * c), 10))

        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


def main():
    global running
    t = threading.Thread(target=receiver, daemon=True)
    t.start()
    try:
        render()
    finally:
        running = False
        time.sleep(0.3)


if __name__ == "__main__":
    main()
