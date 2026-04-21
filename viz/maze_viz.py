"""
SLAM Maze Visualizer & Solver

Connects to the ESP32 over WiFi, receives the exported occupancy grid,
renders it with pygame, and runs three search algorithms (BFS, DFS, A*)
to solve the maze.

Usage:
    python maze_viz.py              # connect to ESP32 at default IP
    python maze_viz.py 192.168.4.1  # connect to specific IP
    python maze_viz.py --file map.bin  # load a previously saved map file

Controls (when connected to ESP32):
    W / A / S / D  - drive (forward / left / backward / right) — empty stubs
    X              - stop motors
    LEFT / RIGHT   - step servo ±5° and capture a beam into the scan buffer
    C              - commit the buffered beams to SLAM
    E              - export map from ESP32 and visualize

Controls (visualization mode):
    Left-click   - set START point (green)
    Right-click  - set END point (red)
    SPACE        - run all 3 algorithms and show comparison
    R            - reset start/end points
    F2           - save map to file
    Q / ESC      - quit
"""

import sys
import socket
import struct
import time
import numpy as np
import pygame
from collections import deque
import heapq

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 8080
RECV_TIMEOUT = 10  # seconds

COL_BG         = (180, 180, 180)  # unknown cells
COL_FREE       = (255, 255, 255)  # free space
COL_WALL       = (0,   0,   0)    # occupied
COL_START      = (0,   200, 0)
COL_END        = (200, 0,   0)
COL_SOLUTION   = (0,   0,   0)    # thick solution path
COL_BFS_TRAIL  = (100, 160, 255)  # BFS explored (faint blue)
COL_DFS_TRAIL  = (255, 160, 100)  # DFS explored (faint orange)
COL_ASTAR_TRAIL = (160, 255, 100) # A* explored  (faint green)

WALL_THRESHOLD = 16384  # cell value below this → wall

MAZE_SIZE_INCHES = 36.0          # physical maze is 36 × 36 inches
INCH_TO_M        = 0.0254
COL_MAZE_BOUNDS  = (220, 40, 40) # red outline for the 36″ maze boundary


class SlamMapData:
    def __init__(self):
        self.size  = 0
        self.scale = 0.0
        self.pose  = (0.0, 0.0, 0.0)
        self.cells = None  # numpy uint16 array, shape (size, size)

    def from_binary(self, data: bytes):
        if data[:4] != b"SMAP":
            raise ValueError("Bad magic (expected SMAP)")

        off = 4
        self.size,  = struct.unpack_from("<H", data, off); off += 2
        self.scale, = struct.unpack_from("<f", data, off); off += 4
        px, py, pt  = struct.unpack_from("<fff", data, off); off += 12
        self.pose   = (px, py, pt)

        expected = self.size * self.size * 2
        cell_data = data[off:off + expected]
        if len(cell_data) < expected:
            raise ValueError(f"Truncated cell data: got {len(cell_data)}, need {expected}")

        self.cells = np.frombuffer(cell_data, dtype=np.uint16).reshape(
            (self.size, self.size))

    def to_binary_grid(self):
        """True = passable, False = wall."""
        return self.cells >= WALL_THRESHOLD

    def save(self, path: str):
        header  = b"SMAP"
        header += struct.pack("<H", self.size)
        header += struct.pack("<f", self.scale)
        header += struct.pack("<fff", *self.pose)
        with open(path, "wb") as f:
            f.write(header)
            f.write(self.cells.tobytes())

    def load(self, path: str):
        with open(path, "rb") as f:
            data = f.read()
        self.from_binary(data)


class ESP32Client:
    def __init__(self, ip: str, port: int):
        self.ip   = ip
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(RECV_TIMEOUT)
        print(f"Connecting to {self.ip}:{self.port}...")
        self.sock.connect((self.ip, self.port))
        msg = self._read_line()
        print(f"ESP32: {msg}")

    def send_cmd(self, cmd: str):
        self.sock.sendall(cmd.encode())

    def commit_scan(self):
        self.send_cmd("c")
        while True:
            line = self._read_line()
            print(f"ESP32: {line}")
            if "SCAN_DONE" in line or "SCAN_EMPTY" in line:
                break

    def servo_step(self, direction: str):
        """direction='l' or 'r' — step the servo one increment and capture a beam."""
        self.send_cmd(direction)
        try:
            line = self._read_line()
            print(f"ESP32: {line}")
        except Exception:
            pass

    def drive(self, wasd: str):
        self.send_cmd(wasd)

    def export_map(self) -> SlamMapData:
        self.send_cmd("e")
        raw = self._recv_exact(4)
        if raw != b"SMAP":
            raise RuntimeError(f"Expected SMAP magic, got {raw!r}")

        header = self._recv_exact(2 + 4 + 4 + 4 + 4)  # size + scale + pose(x,y,θ)
        size, = struct.unpack_from("<H", header, 0)
        cell_bytes = size * size * 2
        cell_data = self._recv_exact(cell_bytes)

        try:
            line = self._read_line()
            print(f"ESP32: {line}")
        except Exception:
            pass

        m = SlamMapData()
        m.from_binary(b"SMAP" + header + cell_data)
        return m

    def close(self):
        if self.sock:
            self.sock.close()

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Connection lost during receive")
            buf.extend(chunk)
        return bytes(buf)

    def _read_line(self) -> str:
        buf = bytearray()
        while True:
            b = self.sock.recv(1)
            if not b or b == b"\n":
                break
            buf.extend(b)
        return buf.decode(errors="replace").strip()


# Each search returns (path, explored, stats) where:
#   path     = list of (row, col) from start to end (empty if no path)
#   explored = list of (row, col) in exploration order
#   stats    = dict with 'steps', 'time_ms', 'path_length'

DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected

def bfs(grid, start, end):
    rows, cols = grid.shape
    t0 = time.perf_counter()

    visited = np.zeros_like(grid, dtype=bool)
    parent  = {}
    queue   = deque([start])
    visited[start] = True
    explored = []
    steps = 0

    while queue:
        node = queue.popleft()
        steps += 1
        explored.append(node)

        if node == end:
            break

        for dr, dc in DIRS:
            nr, nc = node[0] + dr, node[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] and not visited[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = node
                queue.append((nr, nc))

    elapsed = (time.perf_counter() - t0) * 1000

    path = []
    if end in parent or end == start:
        cur = end
        while cur != start:
            path.append(cur)
            cur = parent.get(cur)
            if cur is None:
                path = []
                break
        if path:
            path.append(start)
            path.reverse()

    return path, explored, {
        "steps": steps,
        "time_ms": elapsed,
        "path_length": len(path),
    }


def dfs(grid, start, end):
    rows, cols = grid.shape
    t0 = time.perf_counter()

    visited = np.zeros_like(grid, dtype=bool)
    parent  = {}
    stack   = [start]
    visited[start] = True
    explored = []
    steps = 0

    while stack:
        node = stack.pop()
        steps += 1
        explored.append(node)

        if node == end:
            break

        for dr, dc in DIRS:
            nr, nc = node[0] + dr, node[1] + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] and not visited[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = node
                stack.append((nr, nc))

    elapsed = (time.perf_counter() - t0) * 1000

    path = []
    if end in parent or end == start:
        cur = end
        while cur != start:
            path.append(cur)
            cur = parent.get(cur)
            if cur is None:
                path = []
                break
        if path:
            path.append(start)
            path.reverse()

    return path, explored, {
        "steps": steps,
        "time_ms": elapsed,
        "path_length": len(path),
    }


def astar(grid, start, end):
    rows, cols = grid.shape
    t0 = time.perf_counter()

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    g_score = {start: 0}
    parent  = {}
    open_set = [(heuristic(start, end), 0, start)]  # (f, tie-break, node)
    closed  = set()
    explored = []
    steps = 0
    counter = 1

    while open_set:
        _, _, node = heapq.heappop(open_set)

        if node in closed:
            continue
        closed.add(node)
        steps += 1
        explored.append(node)

        if node == end:
            break

        for dr, dc in DIRS:
            nr, nc = node[0] + dr, node[1] + dc
            nb = (nr, nc)
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] and nb not in closed:
                tentative_g = g_score[node] + 1
                if tentative_g < g_score.get(nb, float("inf")):
                    g_score[nb] = tentative_g
                    parent[nb] = node
                    f = tentative_g + heuristic(nb, end)
                    heapq.heappush(open_set, (f, counter, nb))
                    counter += 1

    elapsed = (time.perf_counter() - t0) * 1000

    path = []
    if end in parent or end == start:
        cur = end
        while cur != start:
            path.append(cur)
            cur = parent.get(cur)
            if cur is None:
                path = []
                break
        if path:
            path.append(start)
            path.reverse()

    return path, explored, {
        "steps": steps,
        "time_ms": elapsed,
        "path_length": len(path),
    }


ALGO_INFO = [
    ("BFS",   bfs,   COL_BFS_TRAIL),
    ("DFS",   dfs,   COL_DFS_TRAIL),
    ("A*",    astar, COL_ASTAR_TRAIL),
]

def render_map(surface, cells, cell_px):
    size = cells.shape[0]
    for r in range(size):
        for c in range(size):
            v = cells[r, c]
            if v < WALL_THRESHOLD:
                col = COL_WALL
            elif v > 49152:
                col = COL_FREE
            else:
                col = COL_BG
            pygame.draw.rect(surface, col,
                             (c * cell_px, r * cell_px, cell_px, cell_px))


def render_map_fast(surface, cells, cell_px):
    """numpy → surface blit; far faster than per-cell draws."""
    size = cells.shape[0]
    rgb = np.zeros((size, size, 3), dtype=np.uint8)

    wall_mask    = cells < WALL_THRESHOLD
    free_mask    = cells > 49152
    unknown_mask = ~wall_mask & ~free_mask

    rgb[wall_mask]    = COL_WALL
    rgb[free_mask]    = COL_FREE
    rgb[unknown_mask] = COL_BG

    if cell_px > 1:
        rgb = np.repeat(np.repeat(rgb, cell_px, axis=0), cell_px, axis=1)

    surf = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
    surface.blit(surf, (0, 0))


def draw_explored(surface, explored, colour, cell_px, alpha=80):
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    faint = (*colour, alpha)
    for (r, c) in explored:
        pygame.draw.rect(overlay, faint,
                         (c * cell_px, r * cell_px, cell_px, cell_px))
    surface.blit(overlay, (0, 0))


def draw_path(surface, path, colour, cell_px, width=3):
    if len(path) < 2:
        return
    points = [(c * cell_px + cell_px // 2, r * cell_px + cell_px // 2)
              for (r, c) in path]
    pygame.draw.lines(surface, colour, False, points, width)


def draw_maze_bounds(surface, map_size, scale, cell_px):
    """36×36 inch physical maze outline, centered on map origin (robot start)."""
    if scale <= 0:
        return
    maze_cells = int(round(MAZE_SIZE_INCHES * INCH_TO_M / scale))
    centre = map_size // 2
    half   = maze_cells // 2
    x = (centre - half) * cell_px
    y = (centre - half) * cell_px
    w = maze_cells * cell_px
    h = maze_cells * cell_px
    pygame.draw.rect(surface, COL_MAZE_BOUNDS, (x, y, w, h), 2)


def draw_marker(surface, pos, colour, cell_px):
    r, c = pos
    cx = c * cell_px + cell_px // 2
    cy = r * cell_px + cell_px // 2
    radius = max(cell_px // 2, 4)
    pygame.draw.circle(surface, colour, (cx, cy), radius)
    pygame.draw.circle(surface, (0, 0, 0), (cx, cy), radius, 1)


def draw_comparison_panel(surface, results, panel_y, panel_w):
    font = pygame.font.SysFont("consolas", 16)
    bold = pygame.font.SysFont("consolas", 18, bold=True)

    x_offset = 10
    col_width = panel_w // 3

    for i, (name, _, trail_col, path, explored, stats) in enumerate(results):
        x = x_offset + i * col_width
        y = panel_y + 10

        label = bold.render(name, True, trail_col)
        surface.blit(label, (x, y))
        y += 25

        lines = [
            f"Steps explored: {stats['steps']}",
            f"Time: {stats['time_ms']:.2f} ms",
            f"Path length: {stats['path_length']} cells",
        ]
        if path:
            lines.append(f"Solution: YES")
        else:
            lines.append(f"Solution: NO PATH")

        for line in lines:
            text = font.render(line, True, (0, 0, 0))
            surface.blit(text, (x, y))
            y += 20


def main():
    map_file = None
    esp_ip   = ESP32_IP

    args = sys.argv[1:]
    if "--file" in args:
        idx = args.index("--file")
        map_file = args[idx + 1]
    elif args:
        esp_ip = args[0]

    slam_map = SlamMapData()

    if map_file:
        print(f"Loading map from {map_file}...")
        slam_map.load(map_file)
    else:
        client = ESP32Client(esp_ip, ESP32_PORT)
        try:
            client.connect()
            print("Connected. Press C to scan, E to export in the pygame window.")
            print("(Doing initial export now...)")
            slam_map = client.export_map()
        except Exception as e:
            print(f"Connection failed: {e}")
            print("Starting with empty map. Use --file to load a saved map.")
            slam_map.size  = 256
            slam_map.scale = 0.008
            slam_map.pose  = (0, 0, 0)
            slam_map.cells = np.full((256, 256), 32768, dtype=np.uint16)
            client = None
        else:
            client_ref = client

    map_size = slam_map.size
    print(f"Map: {map_size}x{map_size}, scale={slam_map.scale:.4f}m/cell")

    pygame.init()

    # Fit the map in ~768 px
    cell_px = max(1, 768 // map_size)
    map_px  = map_size * cell_px
    panel_h = 120
    win_w   = map_px
    win_h   = map_px + panel_h

    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption("SLAM Maze Visualizer")

    start_pos = None
    end_pos   = None
    algo_results = []  # [(name, fn, colour, path, explored, stats), ...]

    grid = slam_map.to_binary_grid()

    running = True
    needs_redraw = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

                elif event.key == pygame.K_c and not map_file:
                    try:
                        print("Committing buffered scan to SLAM...")
                        client_ref.commit_scan()
                    except Exception as e:
                        print(f"Commit failed: {e}")

                elif event.key == pygame.K_e and not map_file:
                    try:
                        print("Exporting map...")
                        slam_map = client_ref.export_map()
                        grid = slam_map.to_binary_grid()
                        algo_results = []
                        needs_redraw = True
                        print("Map updated.")
                    except Exception as e:
                        print(f"Export failed: {e}")

                elif event.key == pygame.K_w and not map_file:
                    client_ref.drive('w')
                elif event.key == pygame.K_a and not map_file:
                    client_ref.drive('a')
                elif event.key == pygame.K_s and not map_file:
                    client_ref.drive('s')
                elif event.key == pygame.K_d and not map_file:
                    client_ref.drive('d')
                elif event.key == pygame.K_x and not map_file:
                    client_ref.drive('x')

                elif event.key == pygame.K_LEFT and not map_file:
                    client_ref.servo_step('l')
                elif event.key == pygame.K_RIGHT and not map_file:
                    client_ref.servo_step('r')

                elif event.key == pygame.K_SPACE:
                    if start_pos and end_pos:
                        algo_results = []
                        for name, fn, colour in ALGO_INFO:
                            path, explored, stats = fn(grid, start_pos, end_pos)
                            algo_results.append(
                                (name, fn, colour, path, explored, stats))
                            print(f"{name}: {stats['steps']} steps, "
                                  f"{stats['time_ms']:.2f}ms, "
                                  f"path={stats['path_length']} cells")
                        needs_redraw = True
                    else:
                        print("Set start (left-click) and end (right-click) first.")

                elif event.key == pygame.K_r:
                    start_pos = None
                    end_pos   = None
                    algo_results = []
                    needs_redraw = True

                elif event.key == pygame.K_F2:
                    fname = "slam_map.bin"
                    slam_map.save(fname)
                    print(f"Map saved to {fname}")

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                col = mx // cell_px
                row = my // cell_px
                if 0 <= row < map_size and 0 <= col < map_size:
                    if event.button == 1:  # left click = start
                        start_pos = (row, col)
                        algo_results = []
                        needs_redraw = True
                        print(f"Start: ({row}, {col})")
                    elif event.button == 3:  # right click = end
                        end_pos = (row, col)
                        algo_results = []
                        needs_redraw = True
                        print(f"End: ({row}, {col})")

        if needs_redraw:
            screen.fill(COL_BG)

            render_map_fast(screen, slam_map.cells, cell_px)
            draw_maze_bounds(screen, map_size, slam_map.scale, cell_px)

            for name, fn, colour, path, explored, stats in algo_results:
                draw_explored(screen, explored, colour, cell_px)

            for name, fn, colour, path, explored, stats in algo_results:
                if path:
                    draw_path(screen, path, colour, cell_px, width=2)

            # Shortest solution in thick black, on top of per-algo paths.
            solved = [r for r in algo_results if r[3]]
            if solved:
                best = min(solved, key=lambda r: r[5]["path_length"])
                draw_path(screen, best[3], COL_SOLUTION, cell_px, width=4)

            if start_pos:
                draw_marker(screen, start_pos, COL_START, cell_px)
            if end_pos:
                draw_marker(screen, end_pos, COL_END, cell_px)

            panel_rect = pygame.Rect(0, map_px, win_w, panel_h)
            pygame.draw.rect(screen, (240, 240, 240), panel_rect)
            pygame.draw.line(screen, (0, 0, 0), (0, map_px), (win_w, map_px), 2)

            if algo_results:
                draw_comparison_panel(screen, algo_results, map_px, win_w)
            else:
                font = pygame.font.SysFont("consolas", 14)
                hint = font.render(
                    "Click: start/end | SPACE: solve | R: reset | "
                    "F2: save | Q: quit",
                    True, (80, 80, 80))
                screen.blit(hint, (10, map_px + 10))
                if not map_file:
                    hint2 = font.render(
                        "WASD: drive  X: stop  ←/→: servo step  "
                        "C: commit scan  E: export map",
                        True, (80, 80, 80))
                    screen.blit(hint2, (10, map_px + 30))

            pygame.display.flip()
            needs_redraw = False

        pygame.time.wait(16)  # ~60 FPS cap

    pygame.quit()

    if not map_file:
        try:
            client_ref.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
