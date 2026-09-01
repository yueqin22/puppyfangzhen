"""Python mirror of cpp_src/puppy_nav_core/src/path_validator.cpp (M2, §6).

Faithful port: same grid semantics, same 10 validation conditions, same footprint
rasterization (half-cell inflation + 1-cell bbox expansion) and the same clearance
quantisation (Chebyshev ring -> (ring-0.5)*resolution). This guarantees the
Python and C++ validators agree field-by-field on identical inputs, which is what
the consistency test (test_consistency.py) verifies.

Grid value convention (must match OccupancyGrid.h):
    log_odds > 0.6   -> occupied (out of bounds counts as occupied)
    |log_odds| < 0.3 -> unknown  (out of bounds counts as NOT unknown)
    log_odds < -0.5  -> free
"""
import math

M_PI = math.pi


# ---------------------------------------------------------------------------
# OccupancyGrid (minimal mirror of puppy_nav_core::OccupancyGrid)
# ---------------------------------------------------------------------------
class OccupancyGrid:
    def __init__(self, width=160, height=120, resolution=0.1,
                 origin_x=-8.0, origin_y=-6.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.log_odds = [0.0] * (width * height)

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.resolution))
        gy = int(math.floor((y - self.origin_y) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def is_in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx, gy):
        if not self.is_in_bounds(gx, gy):
            return True
        return self.log_odds[gy * self.width + gx] > 0.6

    def is_unknown(self, gx, gy):
        if not self.is_in_bounds(gx, gy):
            return False
        return abs(self.log_odds[gy * self.width + gx]) < 0.3

    def is_free(self, gx, gy):
        if not self.is_in_bounds(gx, gy):
            return False
        return self.log_odds[gy * self.width + gx] < -0.5


# ---------------------------------------------------------------------------
# PathValidationCode (int values mirror the C++ enum exactly)
# ---------------------------------------------------------------------------
class PathValidationCode:
    OK = 0
    EMPTY_PATH = 1
    SINGLE_POINT = 2
    START_IN_OBSTACLE = 3
    GOAL_IN_OBSTACLE = 4
    START_OUT_OF_BOUNDS = 5
    GOAL_OUT_OF_BOUNDS = 6
    GOAL_TOO_FAR = 7
    PATH_TOO_SHORT = 8
    FOOTPRINT_COLLISION = 9
    SEGMENT_PENETRATION = 10
    INSUFFICIENT_CLEARANCE = 11

    _NAMES = {
        OK: "OK", EMPTY_PATH: "EMPTY_PATH", SINGLE_POINT: "SINGLE_POINT",
        START_IN_OBSTACLE: "START_IN_OBSTACLE", GOAL_IN_OBSTACLE: "GOAL_IN_OBSTACLE",
        START_OUT_OF_BOUNDS: "START_OUT_OF_BOUNDS", GOAL_OUT_OF_BOUNDS: "GOAL_OUT_OF_BOUNDS",
        GOAL_TOO_FAR: "GOAL_TOO_FAR", PATH_TOO_SHORT: "PATH_TOO_SHORT",
        FOOTPRINT_COLLISION: "FOOTPRINT_COLLISION", SEGMENT_PENETRATION: "SEGMENT_PENETRATION",
        INSUFFICIENT_CLEARANCE: "INSUFFICIENT_CLEARANCE",
    }

    @staticmethod
    def to_string(c):
        return PathValidationCode._NAMES.get(c, "UNKNOWN")


class PathValidationOptions:
    def __init__(self):
        self.footprint = [(-0.35, -0.25), (-0.35, 0.25), (0.35, 0.25), (0.35, -0.25)]
        self.min_path_clearance = 0.08
        self.goal_tolerance = 0.35
        self.sample_step = 0.025
        self.unknown_is_blocked = True
        self.min_path_length_ratio = 0.5


class PathValidationResult:
    def __init__(self):
        self.valid = False
        self.code = PathValidationCode.EMPTY_PATH
        self.reason = PathValidationCode.to_string(self.code)
        self.min_clearance = 0.0
        self.endpoint_error = 0.0
        self.collision_index = -1
        self.checked_samples = 0


# ---------------------------------------------------------------------------
# PathValidator
# ---------------------------------------------------------------------------
class PathValidator:
    def __init__(self, grid):
        self.grid = grid

    # -- geometry helpers --------------------------------------------------
    @staticmethod
    def world_to_local(wx, wy, cx, cy, theta):
        dx = wx - cx
        dy = wy - cy
        c = math.cos(-theta)
        s = math.sin(-theta)
        return (dx * c - dy * s, dx * s + dy * c)

    @staticmethod
    def point_in_polygon(lx, ly, poly):
        inside = False
        n = len(poly)
        for i in range(n):
            j = n - 1 if i == 0 else i - 1
            xi, yi = poly[i]
            xj, yj = poly[j]
            intersect = ((yi > ly) != (yj > ly)) and \
                (lx < (xj - xi) * (ly - yi) / (yj - yi) + xi)
            if intersect:
                inside = not inside
        return inside

    @staticmethod
    def footprint_max_extent(footprint):
        m = 0.0
        for (px, py) in footprint:
            d = math.hypot(px, py)
            if d > m:
                m = d
        return m

    @staticmethod
    def admissible_half_width(footprint, min_path_clearance):
        return PathValidator.footprint_max_extent(footprint) + min_path_clearance

    def cell_clearance_cells(self, gx, gy, cap_cells=30):
        W = self.grid.width
        H = self.grid.height
        if gx < 0 or gy < 0 or gx >= W or gy >= H:
            return 0.0
        if self.grid.is_occupied(gx, gy):
            return 0.0
        for r in range(1, cap_cells + 1):
            found = False
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx = gx + dx
                    ny = gy + dy
                    if nx < 0 or ny < 0 or nx >= W or ny >= H:
                        continue
                    if self.grid.is_occupied(nx, ny):
                        found = True
                        break
                if found:
                    break
            if found:
                return r
        return cap_cells

    def clearance_m(self, x, y):
        gx, gy = self.grid.world_to_grid(x, y)
        ring = self.cell_clearance_cells(gx, gy)
        return 0.0 if ring == 0 else (ring - 0.5) * self.grid.resolution

    def footprint_cells(self, cx, cy, theta, footprint):
        g = self.grid
        res = g.resolution
        # 1) rotated bbox in world coords
        minx = miny = float("inf")
        maxx = maxy = float("-inf")
        for (px, py) in footprint:
            wx = cx + (px * math.cos(theta) - py * math.sin(theta))
            wy = cy + (px * math.sin(theta) + py * math.cos(theta))
            if wx < minx:
                minx = wx
            if wx > maxx:
                maxx = wx
            if wy < miny:
                miny = wy
            if wy > maxy:
                maxy = wy
        gx0, gy0 = g.world_to_grid(minx, miny)
        gx1, gy1 = g.world_to_grid(maxx, maxy)
        gx0 -= 1
        gy0 -= 1
        gx1 += 1
        gy1 += 1
        h = 0.5 * res
        out = []
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                wcx, wcy = g.grid_to_world(gx, gy)
                inside = (
                    self.point_in_polygon(wcx, wcy, footprint)
                    or self.point_in_polygon(wcx - h, wcy, footprint)
                    or self.point_in_polygon(wcx + h, wcy, footprint)
                    or self.point_in_polygon(wcx, wcy - h, footprint)
                    or self.point_in_polygon(wcx, wcy + h, footprint)
                )
                if inside:
                    out.append((gx, gy))
        return out

    def footprint_collision_and_clearance(self, x, y, theta, footprint,
                                          unknown_is_blocked):
        g = self.grid
        cells = self.footprint_cells(x, y, theta, footprint)
        W = g.width
        H = g.height
        min_clr = float("inf")
        for (gx, gy) in cells:
            if gx < 0 or gy < 0 or gx >= W or gy >= H:
                return True, 0.0
            if g.is_occupied(gx, gy):
                return True, 0.0
            if unknown_is_blocked and g.is_unknown(gx, gy):
                return True, 0.0
            clr = self.cell_clearance_cells(gx, gy)
            clr = 0.0 if clr == 0 else (clr - 0.5) * g.resolution
            if clr < min_clr:
                min_clr = clr
        out = min_clr if min_clr != float("inf") else (30.0 * g.resolution)
        return False, out

    # -- main validation (§6.2 ten conditions) -----------------------------
    def validate(self, path, start_x, start_y, goal_x, goal_y, opt):
        g = self.grid
        res = g.resolution
        r = PathValidationResult()
        r.valid = False
        r.min_clearance = float("inf")
        r.collision_index = -1

        step = opt.sample_step
        if step <= 0.0 or step > res / 2.0:
            step = res / 2.0

        # condition 1: start validity
        sgx, sgy = g.world_to_grid(start_x, start_y)
        if sgx < 0 or sgy < 0 or sgx >= g.width or sgy >= g.height:
            r.code = PathValidationCode.START_OUT_OF_BOUNDS
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if g.is_occupied(sgx, sgy):
            r.code = PathValidationCode.START_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if opt.unknown_is_blocked and g.is_unknown(sgx, sgy):
            r.code = PathValidationCode.START_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r

        # condition 2: goal legality (§6.7)
        ggx, ggy = g.world_to_grid(goal_x, goal_y)
        if ggx < 0 or ggy < 0 or ggx >= g.width or ggy >= g.height:
            r.code = PathValidationCode.GOAL_OUT_OF_BOUNDS
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if g.is_occupied(ggx, ggy):
            r.code = PathValidationCode.GOAL_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if opt.unknown_is_blocked and g.is_unknown(ggx, ggy):
            r.code = PathValidationCode.GOAL_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r

        # condition 3: path non-empty
        if not path:
            r.code = PathValidationCode.EMPTY_PATH
            r.reason = PathValidationCode.to_string(r.code)
            return r

        full = [(start_x, start_y)] + list(path)

        # condition 2b: endpoint validity (last point)
        egx, egy = g.world_to_grid(full[-1][0], full[-1][1])
        if egx < 0 or egy < 0 or egx >= g.width or egy >= g.height:
            r.code = PathValidationCode.GOAL_OUT_OF_BOUNDS
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if g.is_occupied(egx, egy):
            r.code = PathValidationCode.GOAL_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r
        if opt.unknown_is_blocked and g.is_unknown(egx, egy):
            r.code = PathValidationCode.GOAL_IN_OBSTACLE
            r.reason = PathValidationCode.to_string(r.code)
            return r

        # condition 4: endpoint error
        ex = full[-1][0] - goal_x
        ey = full[-1][1] - goal_y
        r.endpoint_error = math.hypot(ex, ey)
        if r.endpoint_error > opt.goal_tolerance:
            r.code = PathValidationCode.GOAL_TOO_FAR
            r.reason = PathValidationCode.to_string(r.code)
            return r

        # condition 5: path length >= ratio * straight line
        straight = math.hypot(goal_x - start_x, goal_y - start_y)
        plen = 0.0
        for i in range(1, len(full)):
            plen += math.hypot(full[i][0] - full[i - 1][0],
                               full[i][1] - full[i - 1][1])
        if straight > 1e-9 and plen < opt.min_path_length_ratio * straight:
            r.code = PathValidationCode.PATH_TOO_SHORT
            r.reason = PathValidationCode.to_string(r.code)
            return r

        # conditions 6/7/8: per-sample footprint collision / penetration / clearance
        sample_idx = 0
        n = len(full)
        for i in range(n):
            cx, cy = full[i]
            if i + 1 < n:
                theta = math.atan2(full[i + 1][1] - cy, full[i + 1][0] - cx)
            else:
                theta = math.atan2(cy - full[i - 1][1], cx - full[i - 1][0])
            hit, clr = self.footprint_collision_and_clearance(
                cx, cy, theta, opt.footprint, opt.unknown_is_blocked)
            if hit:
                r.code = PathValidationCode.FOOTPRINT_COLLISION
                r.reason = PathValidationCode.to_string(r.code)
                r.collision_index = sample_idx
                r.checked_samples = sample_idx + 1
                if r.min_clearance == float("inf"):
                    r.min_clearance = 0.0
                return r
            if clr < r.min_clearance:
                r.min_clearance = clr

            if i + 1 < n:
                nx, ny = full[i + 1]
                seg = math.hypot(nx - cx, ny - cy)
                nsub = int(math.ceil(seg / step))
                if nsub < 1:
                    nsub = 1
                for s in range(1, nsub):
                    t = s / nsub
                    ix = cx + (nx - cx) * t
                    iy = cy + (ny - cy) * t
                    ith = math.atan2(ny - cy, nx - cx)
                    ihit, iclr = self.footprint_collision_and_clearance(
                        ix, iy, ith, opt.footprint, opt.unknown_is_blocked)
                    if ihit:
                        r.code = PathValidationCode.SEGMENT_PENETRATION
                        r.reason = PathValidationCode.to_string(r.code)
                        r.collision_index = sample_idx + s
                        r.checked_samples = sample_idx + s + 1
                        if r.min_clearance == float("inf"):
                            r.min_clearance = 0.0
                        return r
                    if iclr < r.min_clearance:
                        r.min_clearance = iclr
                    sample_idx += 1
            sample_idx += 1
        r.checked_samples = sample_idx

        # condition 8: min clearance >= threshold
        if r.min_clearance < opt.min_path_clearance:
            r.code = PathValidationCode.INSUFFICIENT_CLEARANCE
            r.reason = PathValidationCode.to_string(r.code)
            r.valid = False
            return r

        r.valid = True
        r.code = PathValidationCode.OK
        r.reason = PathValidationCode.to_string(r.code)
        return r
