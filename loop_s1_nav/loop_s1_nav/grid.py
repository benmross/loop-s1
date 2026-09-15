"""A 2D costmap over the course, and an A* planner on it."""

import heapq
import math

import numpy as np
from scipy import ndimage

SQRT2 = math.sqrt(2.0)
MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


class Costmap:
    """Occupancy of the course on a grid, in two layers.

    ``known`` is rasterised from the course description. ``sensed`` is what
    the lidar has hit that the description does not explain. The planner
    treats them the same way: a cell closer than ``inflation`` to either is
    blocked, and cells inside ``preferred_clearance`` cost extra, so plans
    keep off walls when there is room to.

    Arrays are indexed [row, col], rows along y and columns along x.
    """

    def __init__(self, course, resolution=0.1, inflation=0.6,
                 preferred_clearance=1.2, clearance_weight=3.0, pad=0.5):
        self.res = resolution
        self.inflation = inflation
        self.preferred = max(preferred_clearance, inflation + resolution)
        self.weight = clearance_weight
        xmin, xmax, ymin, ymax = course.bounds(pad)
        self.origin = (xmin, ymin)
        self.nx = int(math.ceil((xmax - xmin) / resolution))
        self.ny = int(math.ceil((ymax - ymin) / resolution))
        xs = xmin + (np.arange(self.nx) + 0.5) * resolution
        ys = ymin + (np.arange(self.ny) + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        self.known = course.occupied(grid_x, grid_y)
        self.known_dist = self._distance(self.known)
        self.sensed = np.zeros_like(self.known)
        self.hits = np.zeros(self.known.shape, dtype=np.int32)
        self.update()

    def _distance(self, occupied):
        return ndimage.distance_transform_edt(~occupied) * self.res

    def update(self):
        """Recompute distances and costs after either layer changes."""
        self.dist = self._distance(self.known | self.sensed)
        self.blocked = self.dist < self.inflation
        span = self.preferred - self.inflation
        self.penalty = self.weight * np.clip((self.preferred - self.dist) / span, 0.0, 1.0) ** 2
        # A* reads these a cell at a time; Python lists are far faster there.
        self._blocked_flat = self.blocked.ravel().tolist()
        self._penalty_flat = self.penalty.ravel().tolist()

    def cell(self, x, y):
        return (int(math.floor((y - self.origin[1]) / self.res)),
                int(math.floor((x - self.origin[0]) / self.res)))

    def point(self, i, j):
        return (self.origin[0] + (j + 0.5) * self.res, self.origin[1] + (i + 0.5) * self.res)

    def in_bounds(self, i, j):
        return 0 <= i < self.ny and 0 <= j < self.nx

    def clamp(self, i, j):
        return min(max(i, 0), self.ny - 1), min(max(j, 0), self.nx - 1)

    def is_blocked(self, x, y):
        i, j = self.cell(x, y)
        return not self.in_bounds(i, j) or bool(self.blocked[i, j])

    def clearance(self, x, y, known_only=False):
        i, j = self.cell(x, y)
        if not self.in_bounds(i, j):
            return 0.0
        return float((self.known_dist if known_only else self.dist)[i, j])

    def nearest_free(self, i, j, radius=2.0):
        """The closest unblocked cell within ``radius`` metres, or None."""
        r = int(radius / self.res)
        i0, i1 = max(0, i - r), min(self.ny, i + r + 1)
        j0, j1 = max(0, j - r), min(self.nx, j + r + 1)
        rows, cols = np.nonzero(~self.blocked[i0:i1, j0:j1])
        if rows.size == 0:
            return None
        k = int(np.argmin((rows + i0 - i) ** 2 + (cols + j0 - j) ** 2))
        return int(rows[k] + i0), int(cols[k] + j0)

    def reachable(self, x, y):
        """Mask of cells connected to (x, y) through unblocked space."""
        start = self.nearest_free(*self.clamp(*self.cell(x, y)))
        if start is None:
            return np.zeros_like(self.blocked)
        # 4-connected: A* refuses to cut corners, so diagonal-only links do
        # not count as a way through.
        labels, _ = ndimage.label(~self.blocked)
        return labels == labels[start]

    def plan(self, start, goal):
        """A* from ``start`` to ``goal`` (both x, y). A list of (x, y), or None.

        If either end sits inside the inflated zone, as a vehicle nudged off
        its path can, the search starts or ends at the nearest free cell.
        """
        s = self.nearest_free(*self.clamp(*self.cell(*start)))
        g = self.nearest_free(*self.clamp(*self.cell(*goal)))
        if s is None or g is None:
            return None
        nx, ny = self.nx, self.ny
        blocked, penalty = self._blocked_flat, self._penalty_flat
        source, target = s[0] * nx + s[1], g[0] * nx + g[1]
        gi, gj = g
        best = {source: 0.0}
        parent = {source: -1}
        heap = [(0.0, 0.0, source)]
        while heap:
            _, cost, cur = heapq.heappop(heap)
            if cur == target:
                break
            if cost > best[cur]:
                continue
            i, j = divmod(cur, nx)
            for di, dj, step in MOVES:
                ni, nj = i + di, j + dj
                if ni < 0 or nj < 0 or ni >= ny or nj >= nx:
                    continue
                n = ni * nx + nj
                if blocked[n]:
                    continue
                if di and dj and (blocked[i * nx + nj] or blocked[ni * nx + j]):
                    continue
                new_cost = cost + step * (1.0 + penalty[n])
                if new_cost < best.get(n, math.inf):
                    best[n] = new_cost
                    parent[n] = cur
                    a, b = abs(ni - gi), abs(nj - gj)
                    heuristic = a + b + (SQRT2 - 2.0) * min(a, b)
                    heapq.heappush(heap, (new_cost + heuristic, new_cost, n))
        else:
            return None

        cells = []
        cur = target
        while cur != -1:
            cells.append(divmod(cur, nx))
            cur = parent[cur]
        cells.reverse()
        path = [self.point(i, j) for i, j in cells]
        if not self.is_blocked(*goal):
            path[-1] = (float(goal[0]), float(goal[1]))
        return path

    def path_blocked(self, path, start_index=0):
        for x, y in path[start_index:]:
            if self.is_blocked(x, y):
                return True
        return False

    def add_hits(self, xs, ys, threshold=3, ignore_within=0.35):
        """Record lidar returns. Returns how many cells became obstacles.

        Returns within ``ignore_within`` of a known obstacle are that obstacle,
        seen with a little pose lag, and add nothing. A cell becomes an
        obstacle after ``threshold`` returns, so one stray ray cannot close a
        corridor.
        """
        j = np.floor((np.asarray(xs) - self.origin[0]) / self.res).astype(int)
        i = np.floor((np.asarray(ys) - self.origin[1]) / self.res).astype(int)
        inside = (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)
        i, j = i[inside], j[inside]
        unexplained = self.known_dist[i, j] >= ignore_within
        i, j = i[unexplained], j[unexplained]
        if i.size == 0:
            return 0
        np.add.at(self.hits, (i, j), 1)
        before = int(np.count_nonzero(self.sensed))
        fresh = (self.hits[i, j] >= threshold) & ~self.sensed[i, j]
        self.sensed[i[fresh], j[fresh]] = True
        return int(np.count_nonzero(self.sensed)) - before

    def occupancy_data(self):
        """Cells as nav_msgs/OccupancyGrid data, in RViz's costmap scale."""
        scaled = (self.penalty / max(self.weight, 1e-9) * 90).astype(np.int8)
        data = np.where(self.blocked, 99, scaled)
        data = np.where(self.known | self.sensed, 100, data)
        return data.astype(np.int8).ravel()
