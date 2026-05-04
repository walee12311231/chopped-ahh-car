"""
slam_host.py — PC-side 2D SLAM for the ESP32 RPLidar + MPU6050 bridge.

Fusion pipeline (per scan)
--------------------------
  1. PREDICT  x/y  : motion prior from last N accepted GOOD translations
     PREDICT  yaw  : IMU gyro-Z integrated on firmware
  2. COARSE SEARCH : 5×5×5 grid search over ±CS_RANGE_M / ±CS_RANGE_RAD
                     around the predicted pose — finds the best ICP starting
                     basin without full iteration cost
  3. ICP REFINE    : point-to-point closed-form 2D, against the downsampled
                     rolling SUBMAP (last 10 GOOD scans)
  4. CLASSIFY      :
       GOOD   inliers≥60 AND err<0.08 m  → full pose, global map write, submap
       WEAK   inliers≥30 AND err<0.25 m  → half translation, NO global map,
                                           live overlay only (unless g toggled)
       REJECT everything else             → hold translation, IMU heading, no map
  5. MAP           : Bresenham ray-cast log-odds (GOOD only by default)

Coordinate frames
-----------------
  Robot : x forward, y left, origin at lidar centre.
  World : fixed at startup [0,0,0].
  Grid  : cell = GRID_C + round(world_m / CELL_M). GRID_C is a display
          offset only — not the logical origin.

Wire protocol
-------------
  0x01 Scan : header(16) + n×5 bytes + 1 XOR
  0x02 IMU  : header(16) + 10 bytes  + 1 XOR
    IMU LE payload: yaw_cdeg(i16) ax(i16) ay(i16) az(i16) reserved(u16)

Keys
----
  c  reset map + pose          r  reset pose only (keep map)
  p  pause / resume mapping    g  toggle WEAK scans → global map
  t  toggle submap overlay     q / ESC  quit
"""

import argparse
import math
import socket
import struct
import threading
import time
from collections import deque

import numpy as np
import pygame
from scipy.spatial import cKDTree


# ── network ───────────────────────────────────────────────────────────────────
PORT       = 8080
MAGIC      = b"\xA5\x5A"
FRAME_SCAN = 0x01
FRAME_IMU  = 0x02

# ── map ───────────────────────────────────────────────────────────────────────
CELL_M       = 0.03
GRID_N       = 800
GRID_C       = GRID_N // 2   # display offset — NOT the logical world origin
WIN_W, WIN_H = 1280, 720
PX_PER_M     = 100
MAX_RANGE_M  = 6.0
MIN_RANGE_M  = 0.15

# ── log-odds ──────────────────────────────────────────────────────────────────
L_OCC, L_FREE = 0.85, -0.4
L_MIN, L_MAX  = -5.0, 5.0
P_OCC_T, P_FREE_T = 0.65, 0.35

# ── ICP thresholds ────────────────────────────────────────────────────────────
ICP_MAX_ITERS     = 25
ICP_GATE_M        = 0.50      # max correspondence distance
ICP_EPS           = 1e-4      # convergence criterion
ICP_MIN_PTS       = 60        # GOOD: minimum inliers
ICP_ERR_GOOD      = 0.08      # GOOD: maximum mean error (m)
ICP_MIN_PTS_WEAK  = 30        # WEAK: minimum inliers
ICP_ERR_WEAK      = 0.25      # WEAK: maximum mean error (m)
REJECT_JUMP_M     = 1.0       # hard cap: translation jump
REJECT_JUMP_RAD   = math.radians(60)  # hard cap: rotation jump

# ── fusion weights ────────────────────────────────────────────────────────────
WEAK_TRANS_ALPHA  = 0.5       # fraction of ICP translation on WEAK
WEAK_L_OCC_SCALE  = 0.5       # l_occ multiplier for WEAK (when enabled by 'g')
IMU_W_GOOD        = 0.15      # IMU heading blend weight on GOOD ICP
IMU_W_WEAK        = 0.80      # IMU heading blend weight on WEAK ICP

# ── submap ────────────────────────────────────────────────────────────────────
SUBMAP_MAX_SCANS  = 10        # rolling window of GOOD scans
SUBMAP_ADD_DIST   = 0.10      # min translation to add a scan (m)
SUBMAP_ADD_ROT    = math.radians(5)
SUBMAP_VOXEL_M    = 0.05      # voxel size for downsampling stored scans

# ── coarse grid search ────────────────────────────────────────────────────────
CS_RANGE_M        = 0.25      # ±search range in x and y
CS_RANGE_RAD      = math.radians(15)
CS_STEPS          = 5         # per axis → 5³ = 125 candidates
CS_SUBSAMPLE      = 3         # use every Nth source point (speed/accuracy trade)

# ── motion prior ──────────────────────────────────────────────────────────────
MOTION_WIN        = 4         # accepted GOOD poses to track


# =====================================================================
#  Helpers
# =====================================================================

def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def downsample(pts, voxel=SUBMAP_VOXEL_M):
    """Voxel-grid downsample: keep one representative point per cell."""
    if len(pts) < 2:
        return pts
    keys = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def scan_to_xy(angle_q6, dist_mm, quality):
    """Polar scan → (M,2) float32 in robot frame."""
    keep = (dist_mm > 0) & (quality > 0)
    d_m  = dist_mm[keep].astype(np.float32) / 1000.0
    a    = np.deg2rad(angle_q6[keep].astype(np.float32) / 64.0)
    k2   = (d_m >= MIN_RANGE_M) & (d_m <= MAX_RANGE_M)
    return np.column_stack([d_m[k2] * np.cos(a[k2]),
                            d_m[k2] * np.sin(a[k2])]).astype(np.float32)


def transform_xy(xy, pose):
    """SE(2): robot frame → world frame."""
    c = math.cos(pose[2]); s = math.sin(pose[2])
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    return (xy @ R.T) + np.asarray(pose[:2], dtype=np.float32)


# =====================================================================
#  Motion prior — translation extrapolation from recent GOOD updates
# =====================================================================

class MotionPrior:
    """
    Tracks the last MOTION_WIN accepted (GOOD) poses with timestamps.
    Extrapolates x/y velocity from the two most recent entries to predict
    where the robot will be at the next scan.

    Heading prediction is NOT done here — IMU yaw handles that.
    """

    def __init__(self, window: int = MOTION_WIN):
        self._hist: deque = deque(maxlen=window)  # (t, x, y)

    def update(self, pose: np.ndarray) -> None:
        self._hist.append((time.monotonic(), float(pose[0]), float(pose[1])))

    def predict_xy(self, cur_pose: np.ndarray) -> np.ndarray:
        """Return predicted [x, y] one scan period ahead, or cur_pose[:2] if uncertain."""
        if len(self._hist) < 2:
            return cur_pose[:2].copy()
        t1, x1, y1 = self._hist[-1]
        t0, x0, y0 = self._hist[-2]
        dt = t1 - t0
        if dt <= 0 or dt > 1.5:          # stale — robot stopped
            return cur_pose[:2].copy()
        # Estimate scan-to-scan interval from actual elapsed time
        elapsed = time.monotonic() - t1
        pred_dt = min(elapsed, 0.4)       # cap at 0.4 s to avoid huge jumps
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        return np.array([cur_pose[0] + vx * pred_dt,
                         cur_pose[1] + vy * pred_dt], dtype=np.float64)

    def clear(self) -> None:
        self._hist.clear()


# =====================================================================
#  SubMap — rolling voxel-downsampled reference for ICP
# =====================================================================

class SubMap:
    """
    Rolling window of the last SUBMAP_MAX_SCANS GOOD-tier scans in world
    frame. Each stored cloud is voxel-downsampled to ~SUBMAP_VOXEL_M cells,
    which keeps the merged reference dense spatially without piling up
    redundant nearby points.

    Only GOOD scans are ever added. All mutations are inside W.lock.
    """

    def __init__(self, max_scans: int = SUBMAP_MAX_SCANS):
        self._scans: deque = deque(maxlen=max_scans)
        self._last_pose    = np.zeros(3, dtype=np.float64)

    def add(self, pts_world: np.ndarray, pose: np.ndarray) -> None:
        if len(self._scans) > 0:
            dt = np.linalg.norm(pose[:2] - self._last_pose[:2])
            dr = abs(wrap_angle(pose[2] - self._last_pose[2]))
            if dt < SUBMAP_ADD_DIST and dr < SUBMAP_ADD_ROT:
                return
        self._scans.append(downsample(pts_world))
        self._last_pose = pose.copy()

    def reference(self):
        """Merged (N,2) float32, or None if empty."""
        if not self._scans:
            return None
        return np.vstack(list(self._scans)).astype(np.float32)

    def clear(self) -> None:
        self._scans.clear()
        self._last_pose[:] = 0.0

    def __len__(self) -> int:
        return len(self._scans)

    def point_count(self) -> int:
        return sum(len(s) for s in self._scans)


# =====================================================================
#  Shared state
# =====================================================================

class World:
    def __init__(self):
        self.lock          = threading.Lock()
        self.pose          = np.zeros(3, dtype=np.float64)
        self.trail         = deque(maxlen=4000)
        self.live_world    = np.zeros((0, 2), dtype=np.float32)
        self.submap_display= np.zeros((0, 2), dtype=np.float32)  # for 't' overlay
        self.last_scan_raw = None
        self.submap        = SubMap()
        self.submap_size   = 0
        self.submap_pts    = 0
        self.grid          = np.zeros((GRID_N, GRID_N), dtype=np.float32)
        self.motion_prior  = MotionPrior()
        # IMU
        self.imu_yaw_rad   = 0.0
        self.have_imu      = False
        self.imu_frames    = 0
        # diagnostics
        self.frames_ok     = 0
        self.frames_bad    = 0
        self.icp_iters     = 0
        self.icp_inliers   = 0
        self.icp_err       = float("inf")
        self.icp_tier      = "BOOT"
        self.icp_rejected  = False
        self.imu_prior_used= False
        self.last_step_m   = 0.0    # translation magnitude of last accepted update
        self.map_written   = False  # was global grid written last step
        self.frame_log     = deque(maxlen=120)
        # rolling tier stats
        self.tier_counts   = {"GOOD": 0, "WEAK": 0, "REJECT": 0}
        self.total_scans   = 0
        # ui flags (toggled by key, read by slam_step + renderer)
        self.paused        = False
        self.show_submap   = False
        self.weak_to_map   = False  # 'g' key: allow WEAK scans to write global map

    def reset(self, pose_only: bool = False):
        with self.lock:
            self.pose[:] = 0
            self.trail.clear()
            self.live_world   = np.zeros((0, 2), dtype=np.float32)
            self.motion_prior.clear()
            self.icp_tier     = "BOOT"
            self.icp_rejected = False
            self.last_step_m  = 0.0
            self.tier_counts  = {"GOOD": 0, "WEAK": 0, "REJECT": 0}
            self.total_scans  = 0
            if not pose_only:
                self.submap.clear()
                self.submap_size   = 0
                self.submap_pts    = 0
                self.submap_display= np.zeros((0, 2), dtype=np.float32)
                self.grid.fill(0.0)
                self.imu_yaw_rad   = 0.0
                self.have_imu      = False


W       = World()
RUNNING = True


# =====================================================================
#  Frame parser
# =====================================================================

def parse_stream(sock):
    """
    Yields:
      ("scan", angle_q6, dist_mm, quality)
      ("imu",  yaw_cdeg, ax, ay, az, t_ms)
    """
    buf = bytearray()
    while RUNNING:
        try:
            chunk = sock.recv(8192)
        except OSError:
            return
        if not chunk:
            return
        buf += chunk

        while True:
            i = buf.find(MAGIC)
            if i < 0:
                if len(buf) > 16384:
                    del buf[:-1]
                break
            if i > 0:
                del buf[:i]
            if len(buf) < 16:
                break

            ftype    = buf[2]
            n, plen  = struct.unpack_from("<HH", buf, 12)

            if ftype == FRAME_SCAN:
                if n > 720 or plen != n * 5:
                    del buf[:2]; W.frames_bad += 1
                    W.frame_log.append((time.time(), False)); continue
            elif ftype == FRAME_IMU:
                if n != 1 or plen != 10:
                    del buf[:2]; W.frames_bad += 1; continue
            else:
                del buf[:2]; W.frames_bad += 1; continue

            total = 16 + plen + 1
            if len(buf) < total:
                break

            frame = bytes(buf[:total])
            del buf[:total]
            cs = 0
            for b in frame[:-1]:
                cs ^= b
            if cs != frame[-1]:
                W.frames_bad += 1
                W.frame_log.append((time.time(), False)); continue

            if ftype == FRAME_SCAN:
                arr      = np.frombuffer(frame, dtype=np.uint8, count=plen, offset=16)
                arr      = arr.reshape(n, 5)
                angle_q6 = arr[:, 0].astype(np.uint16) | (arr[:, 1].astype(np.uint16) << 8)
                dist_mm  = arr[:, 2].astype(np.uint16) | (arr[:, 3].astype(np.uint16) << 8)
                quality  = arr[:, 4]
                W.frames_ok += 1
                W.frame_log.append((time.time(), True))
                W.last_scan_raw = (angle_q6, dist_mm, quality)
                yield ("scan", angle_q6, dist_mm, quality)
            else:
                yaw_cdeg, ax, ay, az, _r = struct.unpack_from("<hhhhH", frame, 16)
                _, t_ms = struct.unpack_from("<II", frame, 8)
                yield ("imu", yaw_cdeg, ax, ay, az, t_ms)


# =====================================================================
#  ICP — point-to-point closed-form 2D, accepts pre-built tree
# =====================================================================

def icp_2d(src_robot, dst_world, init_pose, tree=None):
    """
    Align src_robot (Mx2, robot frame) to dst_world (Nx2, world frame).
    Pass a pre-built cKDTree as `tree` to avoid rebuilding it inside coarse_search.
    Returns (pose, mean_err, iters, inliers).
    """
    if src_robot.shape[0] < ICP_MIN_PTS or dst_world.shape[0] < ICP_MIN_PTS:
        return init_pose.copy(), float("inf"), 0, 0

    if tree is None:
        tree = cKDTree(dst_world)

    pose     = init_pose.copy().astype(np.float64)
    last_inl = 0
    last_err = float("inf")

    for k in range(ICP_MAX_ITERS):
        src_w      = transform_xy(src_robot, pose)
        dists, idx = tree.query(src_w, k=1, distance_upper_bound=ICP_GATE_M)
        valid      = np.isfinite(dists)
        last_inl   = int(valid.sum())
        if last_inl < ICP_MIN_PTS_WEAK:   # bail early — won't reach GOOD
            break

        P   = src_w[valid].astype(np.float64)
        Q   = dst_world[idx[valid]].astype(np.float64)
        muP = P.mean(0); muQ = Q.mean(0)
        dP  = P - muP;   dQ  = Q - muQ

        a  = (dP[:, 0] * dQ[:, 1] - dP[:, 1] * dQ[:, 0]).sum()
        b  = (dP[:, 0] * dQ[:, 0] + dP[:, 1] * dQ[:, 1]).sum()
        dt = math.atan2(a, b)
        c  = math.cos(dt); s = math.sin(dt)
        R  = np.array([[c, -s], [s, c]])
        t  = muQ - R @ muP

        cP = math.cos(pose[2]); sP = math.sin(pose[2])
        Rp = np.array([[cP, -sP], [sP, cP]])
        Rnew      = R @ Rp
        pose[0:2] = R @ pose[:2] + t
        pose[2]   = math.atan2(Rnew[1, 0], Rnew[0, 0])

        last_err = float(np.linalg.norm((R @ P.T).T + t - Q, axis=1).mean())
        if abs(dt) + np.linalg.norm(t) < ICP_EPS:
            return pose, last_err, k + 1, last_inl

    return pose, last_err, ICP_MAX_ITERS, last_inl


# =====================================================================
#  Coarse grid search — finds the best ICP starting basin
# =====================================================================

def coarse_search(src_robot, dst_world, init_pose, tree=None):
    """
    Evaluate CS_STEPS³ candidate poses in a grid around init_pose.
    Uses a subsampled source cloud (every CS_SUBSAMPLE-th point) for speed.
    Returns the candidate pose with the most inlier correspondences.

    Purpose: when the robot has translated since the last scan, ICP can
    fail from a stale starting pose. This coarse pass finds a better basin
    before the full ICP refinement.
    """
    src_sub = src_robot[::CS_SUBSAMPLE]
    if len(src_sub) < 5 or len(dst_world) < ICP_MIN_PTS_WEAK:
        return init_pose.copy()

    if tree is None:
        tree = cKDTree(dst_world)

    best_pose = init_pose.copy()
    best_inl  = -1

    xs = np.linspace(-CS_RANGE_M,   CS_RANGE_M,   CS_STEPS)
    ys = np.linspace(-CS_RANGE_M,   CS_RANGE_M,   CS_STEPS)
    ts = np.linspace(-CS_RANGE_RAD, CS_RANGE_RAD, CS_STEPS)

    for dth in ts:
        theta = wrap_angle(init_pose[2] + dth)
        c, s  = math.cos(theta), math.sin(theta)
        R     = np.array([[c, -s], [s, c]], dtype=np.float32)
        # Rotate source points once per theta candidate
        rotated = src_sub @ R.T
        for dx in xs:
            for dy in ys:
                tx = float(init_pose[0]) + dx
                ty = float(init_pose[1]) + dy
                pts = rotated + np.array([tx, ty], dtype=np.float32)
                dists, _ = tree.query(pts, k=1,
                                      distance_upper_bound=ICP_GATE_M)
                inl = int(np.isfinite(dists).sum())
                if inl > best_inl:
                    best_inl  = inl
                    best_pose = np.array([tx, ty, theta], dtype=np.float64)

    return best_pose


# =====================================================================
#  Map update
# =====================================================================

def integrate_scan(grid, pose, scan_world, l_occ=L_OCC):
    """Bresenham ray-cast. l_occ is parameterised for WEAK half-weight writes."""
    cx = GRID_C + int(round(pose[0] / CELL_M))
    cy = GRID_C + int(round(pose[1] / CELL_M))
    if not (0 <= cx < GRID_N and 0 <= cy < GRID_N):
        return
    for x_w, y_w in scan_world:
        gx = GRID_C + int(round(x_w / CELL_M))
        gy = GRID_C + int(round(y_w / CELL_M))
        if not (0 <= gx < GRID_N and 0 <= gy < GRID_N):
            continue
        dx = abs(gx - cx); sx = 1 if cx < gx else -1
        dy = -abs(gy - cy); sy = 1 if cy < gy else -1
        err = dx + dy
        x, y = cx, cy
        while not (x == gx and y == gy):
            grid[y, x] += L_FREE
            e2 = 2 * err
            if e2 >= dy: err += dy; x += sx
            if e2 <= dx: err += dx; y += sy
        grid[gy, gx] += l_occ
    np.clip(grid, L_MIN, L_MAX, out=grid)


# =====================================================================
#  SLAM step
# =====================================================================

def slam_step(angle_q6, dist_mm, quality):
    """
    Per-scan fusion:
      1. Motion prior  → predict x/y from recent accepted translations
      2. IMU prior     → predict heading from gyro-integrated yaw
      3. Coarse search → find best basin for ICP (125 candidates)
      4. ICP refine    → precise alignment against submap
      5. Classify tier → GOOD / WEAK / REJECT
      6. Fuse pose     → blend translation and heading per tier
      7. Update        → map (GOOD only by default), submap (GOOD only),
                         motion prior (GOOD only)
    """
    # Bail if paused — live overlay still drawn from W.live_world
    with W.lock:
        if W.paused:
            return

    scan_xy = scan_to_xy(angle_q6, dist_mm, quality)
    if scan_xy.shape[0] < ICP_MIN_PTS_WEAK:
        return

    # ── 1: snapshot ──────────────────────────────────────────────────
    with W.lock:
        cur_pose  = W.pose.copy()
        imu_yaw   = W.imu_yaw_rad
        have_imu  = W.have_imu
        weak_map  = W.weak_to_map
        ref_world = W.submap.reference()

    # ── 2: bootstrap ─────────────────────────────────────────────────
    if ref_world is None:
        pose_used  = cur_pose.copy()
        scan_world = transform_xy(scan_xy, pose_used)
        with W.lock:
            integrate_scan(W.grid, pose_used, scan_world)
            W.submap.add(scan_world, pose_used)
            W.motion_prior.update(pose_used)
            W.submap_size    = len(W.submap)
            W.submap_pts     = W.submap.point_count()
            W.submap_display = ref_world if ref_world is not None else scan_world
            W.live_world     = scan_world
            W.trail.append((pose_used[0], pose_used[1]))
            W.icp_tier       = "GOOD"
            W.icp_rejected   = False
            W.imu_prior_used = False
            W.map_written    = True
            W.last_step_m    = 0.0
            W.tier_counts["GOOD"] += 1
            W.total_scans   += 1
        return

    # ── 3: PREDICT — motion prior for x/y, IMU for heading ───────────
    pred_xy    = W.motion_prior.predict_xy(cur_pose)
    if have_imu:
        init_pose  = np.array([pred_xy[0], pred_xy[1], imu_yaw],
                               dtype=np.float64)
        prior_used = True
    else:
        init_pose  = np.array([pred_xy[0], pred_xy[1], cur_pose[2]],
                               dtype=np.float64)
        prior_used = False

    # ── 4: build KD-tree once, share between coarse search and ICP ───
    ref_tree = cKDTree(ref_world)

    # ── 5: coarse search — find best starting basin ───────────────────
    init_pose = coarse_search(scan_xy, ref_world, init_pose, tree=ref_tree)

    # ── 6: ICP refinement ─────────────────────────────────────────────
    new_pose, err, iters, inliers = icp_2d(scan_xy, ref_world, init_pose,
                                           tree=ref_tree)

    # ── 7: hard caps (before tier) ────────────────────────────────────
    trans_jump  = np.linalg.norm(new_pose[:2] - cur_pose[:2])
    rot_jump    = abs(wrap_angle(new_pose[2] - cur_pose[2]))
    hard_reject = (trans_jump > REJECT_JUMP_M or rot_jump > REJECT_JUMP_RAD)

    # ── 8: classify tier ──────────────────────────────────────────────
    if hard_reject:
        tier = "REJECT"
    elif inliers >= ICP_MIN_PTS and err < ICP_ERR_GOOD:
        tier = "GOOD"
    elif inliers >= ICP_MIN_PTS_WEAK and err < ICP_ERR_WEAK:
        tier = "WEAK"
    else:
        tier = "REJECT"

    # ── 9: compute pose_used ──────────────────────────────────────────
    if tier == "GOOD":
        imu_w = IMU_W_GOOD if have_imu else 0.0
        delta = wrap_angle(imu_yaw - new_pose[2]) if have_imu else 0.0
        theta = wrap_angle(new_pose[2] + imu_w * delta)
        pose_used = np.array([new_pose[0], new_pose[1], theta])

    elif tier == "WEAK":
        px    = cur_pose[0] + WEAK_TRANS_ALPHA * (new_pose[0] - cur_pose[0])
        py    = cur_pose[1] + WEAK_TRANS_ALPHA * (new_pose[1] - cur_pose[1])
        imu_w = IMU_W_WEAK if have_imu else 0.0
        delta = wrap_angle(imu_yaw - new_pose[2]) if have_imu else 0.0
        theta = wrap_angle(new_pose[2] + imu_w * delta)
        pose_used = np.array([px, py, theta])

    else:   # REJECT
        theta = imu_yaw if have_imu else cur_pose[2]
        pose_used = np.array([cur_pose[0], cur_pose[1], theta])

    step_m     = float(np.linalg.norm(pose_used[:2] - cur_pose[:2]))
    scan_world = transform_xy(scan_xy, pose_used)

    # ── 10: update shared state ───────────────────────────────────────
    with W.lock:
        W.live_world     = scan_world
        W.icp_iters      = iters
        W.icp_inliers    = inliers
        W.icp_err        = err
        W.icp_tier       = tier
        W.icp_rejected   = (tier == "REJECT")
        W.imu_prior_used = prior_used
        W.last_step_m    = step_m
        W.tier_counts[tier] += 1
        W.total_scans   += 1

        if tier in ("GOOD", "WEAK"):
            W.pose[:] = pose_used
            W.trail.append((pose_used[0], pose_used[1]))

        if tier == "GOOD":
            # Global map write + submap update + motion prior
            integrate_scan(W.grid, pose_used, scan_world)
            W.submap.add(scan_world, pose_used)
            W.motion_prior.update(pose_used)
            W.map_written = True

        elif tier == "WEAK":
            # Default: live overlay only — no permanent map mark.
            # If the user pressed 'g', write at half weight.
            if weak_map:
                integrate_scan(W.grid, pose_used, scan_world,
                               l_occ=L_OCC * WEAK_L_OCC_SCALE)
                W.map_written = True
            else:
                W.map_written = False
            # WEAK does NOT update submap or motion prior.

        else:   # REJECT
            W.map_written = False

        W.submap_size    = len(W.submap)
        W.submap_pts     = W.submap.point_count()
        ref = W.submap.reference()
        W.submap_display = ref if ref is not None else np.zeros((0, 2), np.float32)


# =====================================================================
#  IMU update
# =====================================================================

def update_imu(yaw_cdeg):
    """Firmware already integrates gyro-Z; we just store the latest value."""
    with W.lock:
        W.imu_yaw_rad = math.radians(yaw_cdeg / 100.0)
        W.have_imu    = True
        W.imu_frames += 1


# =====================================================================
#  Network thread
# =====================================================================

def network_loop(host):
    while RUNNING:
        try:
            sock = socket.create_connection((host, PORT), timeout=3)
            sock.settimeout(2.0)
            sock.sendall(b"s")
            print(f"[net] connected to {host}:{PORT}")
        except OSError as e:
            print(f"[net] connect failed: {e}; retry in 1s")
            time.sleep(1.0)
            continue

        try:
            for item in parse_stream(sock):
                if item[0] == "scan":
                    _, aq6, dmm, q = item
                    slam_step(aq6, dmm, q)
                elif item[0] == "imu":
                    _, yaw_cdeg, _ax, _ay, _az, _t = item
                    update_imu(yaw_cdeg)
        except OSError as e:
            print(f"[net] recv err: {e}")
        finally:
            try: sock.close()
            except Exception: pass
        if RUNNING:
            time.sleep(0.5)


# =====================================================================
#  Renderer
# =====================================================================

def confidence(scan_raw, frame_log):
    if scan_raw is None:
        return 0.0
    aq6, mm, q = scan_raw
    valid = mm > 0
    if not valid.any():
        return 0.0
    deg  = (aq6[valid].astype(np.float32) / 64.0) % 360.0
    bins = np.zeros(36, dtype=bool)
    bins[(deg // 10).astype(int) % 36] = True
    coverage = bins.sum() / 36.0
    qual     = min(float(q[valid].mean()) / 47.0, 1.0)
    now      = time.time()
    recent   = [r for r in frame_log if now - r[0] < 1.0]
    integ    = (sum(1 for _, ok in recent if ok) / len(recent)) if recent else 0.0
    return max(0.0, min(1.0, 0.5 * coverage + 0.3 * qual + 0.2 * integ))


def draw_heart(screen, x, y, size=2, color=(255, 105, 180)):
    pygame.draw.circle(screen, color, (x - size, y - size), size)
    pygame.draw.circle(screen, color, (x + size, y - size), size)
    pygame.draw.polygon(screen, color,
        [(x - 2*size, y - size), (x + 2*size, y - size), (x, y + 3*size)])


PINK_BG    = (255, 213, 236)
PINK_FREE  = (255, 240, 248)
PINK_OCC   = ( 80,  30,  60)
PINK_TRAIL = (255, 105, 180)
PINK_ROBOT = (200,  50, 120)
PINK_TEXT  = (140,  40,  90)
PINK_WARN  = (220,  60,  60)
PINK_PAUSE = (255, 165,   0)
SUBMAP_COL = ( 80, 160, 255)   # submap overlay colour ('t' key)


def render_loop(host):
    global RUNNING
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("RPLIDAR SLAM + IMU fusion")
    font  = pygame.font.SysFont("Consolas", 15)
    clock = pygame.time.Clock()

    cx_scr = WIN_W // 2
    cy_scr = WIN_H // 2
    half_w = int(WIN_W / (2 * PX_PER_M * CELL_M)) + 4
    half_h = int(WIN_H / (2 * PX_PER_M * CELL_M)) + 4

    while RUNNING:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                RUNNING = False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    RUNNING = False
                elif k == pygame.K_c:
                    W.reset(pose_only=False)
                    print("[ui] map + pose reset")
                elif k == pygame.K_r:
                    W.reset(pose_only=True)
                    print("[ui] pose reset (map kept)")
                elif k == pygame.K_p:
                    with W.lock:
                        W.paused = not W.paused
                    print(f"[ui] {'PAUSED' if W.paused else 'RESUMED'}")
                elif k == pygame.K_g:
                    with W.lock:
                        W.weak_to_map = not W.weak_to_map
                    print(f"[ui] WEAK→map {'ON' if W.weak_to_map else 'OFF'}")
                elif k == pygame.K_t:
                    with W.lock:
                        W.show_submap = not W.show_submap
                    print(f"[ui] submap overlay {'ON' if W.show_submap else 'OFF'}")

        # ── snapshot under lock ───────────────────────────────────────
        with W.lock:
            pose        = W.pose.copy()
            trail       = list(W.trail)
            live        = W.live_world.copy()
            sub_disp    = W.submap_display.copy() if W.show_submap else None
            f_ok        = W.frames_ok
            f_bad       = W.frames_bad
            iters       = W.icp_iters
            inl         = W.icp_inliers
            err         = W.icp_err
            tier        = W.icp_tier
            prior       = W.imu_prior_used
            have_imu    = W.have_imu
            imu_yaw     = W.imu_yaw_rad
            imu_frames  = W.imu_frames
            sub_sz      = W.submap_size
            sub_pts     = W.submap_pts
            step_m      = W.last_step_m
            map_wr      = W.map_written
            paused      = W.paused
            weak_map    = W.weak_to_map
            show_sub    = W.show_submap
            tc          = dict(W.tier_counts)
            total       = W.total_scans
            scan_raw    = W.last_scan_raw
            frame_log   = list(W.frame_log)

            rcx = GRID_C + int(round(pose[0] / CELL_M))
            rcy = GRID_C + int(round(pose[1] / CELL_M))
            x0 = max(0, rcx - half_w); x1 = min(GRID_N, rcx + half_w)
            y0 = max(0, rcy - half_h); y1 = min(GRID_N, rcy + half_h)
            patch = W.grid[y0:y1, x0:x1].copy()

        screen.fill(PINK_BG)

        # ── map patch ─────────────────────────────────────────────────
        if patch.size > 0:
            p   = 1.0 - 1.0 / (1.0 + np.exp(patch))
            img = np.full((patch.shape[0], patch.shape[1], 3),
                          PINK_BG, dtype=np.uint8)
            img[p < P_FREE_T] = PINK_FREE
            img[p > P_OCC_T]  = PINK_OCC
            ps = pygame.Surface((patch.shape[1], patch.shape[0]))
            pygame.surfarray.blit_array(ps, np.transpose(img, (1, 0, 2)))
            pw = int(round((x1 - x0) * CELL_M * PX_PER_M))
            ph = int(round((y1 - y0) * CELL_M * PX_PER_M))
            ps = pygame.transform.scale(ps, (pw, ph))
            bx = cx_scr + int(round(((x0 - GRID_C) * CELL_M - pose[0]) * PX_PER_M))
            by = cy_scr + int(round(((y0 - GRID_C) * CELL_M - pose[1]) * PX_PER_M))
            screen.blit(ps, (bx, by))

        def to_screen(xw, yw):
            return (cx_scr + int((xw - pose[0]) * PX_PER_M),
                    cy_scr + int((yw - pose[1]) * PX_PER_M))

        # ── trail ─────────────────────────────────────────────────────
        if len(trail) >= 2:
            pygame.draw.lines(screen, PINK_TRAIL, False,
                              [to_screen(x, y) for x, y in trail], 2)

        # ── submap overlay (t key) ────────────────────────────────────
        if show_sub and sub_disp is not None and len(sub_disp) > 0:
            for xw, yw in sub_disp:
                sx, sy = to_screen(xw, yw)
                if 0 <= sx < WIN_W and 0 <= sy < WIN_H:
                    screen.set_at((sx, sy), SUBMAP_COL)

        # ── live scan overlay ─────────────────────────────────────────
        for xw, yw in live:
            sx, sy = to_screen(xw, yw)
            if 2 <= sx < WIN_W - 2 and 2 <= sy < WIN_H - 2:
                draw_heart(screen, sx, sy, size=1)

        # ── robot dot + fused heading (red) + IMU heading (blue) ──────
        rx, ry = cx_scr, cy_scr
        hx = rx + int(28 * math.cos(pose[2]))
        hy = ry + int(28 * math.sin(pose[2]))
        pygame.draw.line(screen,   PINK_ROBOT, (rx, ry), (hx, hy), 3)
        pygame.draw.circle(screen, PINK_ROBOT, (rx, ry), 6)
        if have_imu:
            ihx = rx + int(18 * math.cos(imu_yaw))
            ihy = ry + int(18 * math.sin(imu_yaw))
            pygame.draw.line(screen, (100, 200, 255), (rx, ry), (ihx, ihy), 2)

        # ── PAUSED banner ─────────────────────────────────────────────
        if paused:
            txt = font.render("  ⏸ PAUSED  ", True, (0, 0, 0), PINK_PAUSE)
            screen.blit(txt, (WIN_W // 2 - txt.get_width() // 2, 8))

        # ── HUD ───────────────────────────────────────────────────────
        c        = confidence(scan_raw, frame_log)
        pct_good = f"{100*tc['GOOD']//total}%" if total else "--"
        pct_weak = f"{100*tc['WEAK']//total}%" if total else "--"
        pct_rej  = f"{100*tc['REJECT']//total}%" if total else "--"
        imu_str  = (f"yaw={math.degrees(imu_yaw):+.1f}°  "
                    f"frames={imu_frames}  prior={'yes' if prior else 'no'}"
                    if have_imu else "no frames yet")
        map_str  = "[MAP✓]" if map_wr else "[MAP✗]"
        weak_str = f"WEAK→map={'ON' if weak_map else 'off'}"
        sub_str  = f"{'[submap]' if show_sub else ''}"

        hud = [
            f"pose    x={pose[0]:+.3f}m  y={pose[1]:+.3f}m  θ={math.degrees(pose[2]):+.1f}°",
            f"icp     {tier}  iters={iters}  inliers={inl}  err={err:.3f}m  "
            f"step={step_m:.3f}m  {map_str}",
            f"submap  scans={sub_sz}/{SUBMAP_MAX_SCANS}  pts={sub_pts}  {sub_str}",
            f"imu     {imu_str}",
            f"stats   G={pct_good} W={pct_weak} R={pct_rej}  "
            f"frames ok={f_ok} bad={f_bad}  conf={c:.2f}  {weak_str}",
            "keys    c=reset  r=pose  p=pause  g=WEAK→map  t=submap  q=quit",
        ]
        for i, line in enumerate(hud):
            col = PINK_WARN if (i == 1 and tier == "REJECT") else PINK_TEXT
            screen.blit(font.render(line, True, col), (8, 8 + i * 17))

        bar_x, bar_y, bar_w, bar_h = 8, 8 + len(hud) * 17 + 4, 240, 10
        pygame.draw.rect(screen, (210, 170, 195), (bar_x, bar_y, bar_w, bar_h))
        pygame.draw.rect(screen, PINK_TRAIL,      (bar_x, bar_y, int(bar_w * c), bar_h))
        pygame.draw.rect(screen, PINK_TEXT,       (bar_x, bar_y, bar_w, bar_h), 1)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


# =====================================================================
#  main
# =====================================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.4.1")
    args = ap.parse_args()

    t = threading.Thread(target=network_loop, args=(args.host,), daemon=True)
    t.start()
    try:
        render_loop(args.host)
    finally:
        global RUNNING
        RUNNING = False
        time.sleep(0.3)


if __name__ == "__main__":
    main()
