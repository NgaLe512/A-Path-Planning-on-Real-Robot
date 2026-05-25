import numpy as np
import heapq
import math


class Node:
    def __init__(self, x, y, theta, g=0.0, h=0.0, parent=None, direction=1):
        self.x = x
        self.y = y
        self.theta = (theta + np.pi) % (2 * np.pi) - np.pi
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.direction = direction  # track motion direction for path quality

    def __lt__(self, other):
        return self.f < other.f


class HybridAStar:
    def __init__(self):
        self.res        = 0.15   # Hash bin size — slightly larger = faster but coarser
        self.theta_res  = 0.4    # ~23 degrees per bin
        self.move_step  = 0.2    # metres per expansion step
        self.wheelbase  = 0.3    # robot wheelbase (metres)
        self.steer_set  = [-0.6, -0.3, 0.0, 0.3, 0.6]  # wider steer set for maze
        self.robot_radius = 0.30  # metres — virtual robot radius for collision checking.
                                  # Increase to give more clearance at outside wall corners.

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan(self, start, goal, grid, w, h, ox, oy, res):
        """
        Returns a list of (x, y, theta) tuples, or None if no path found.

        Parameters
        ----------
        start, goal : (x, y, theta) in world metres
        grid        : flat list of OccupancyGrid data (0-100, -1=unknown)
        w, h        : grid width / height in cells
        ox, oy      : grid origin in world metres
        res         : cell resolution in metres/cell
        """

        # --- Guard: reject goals inside walls ---------------------------------
        goal_gx = int((goal[0] - ox) / res)
        goal_gy = int((goal[1] - oy) / res)
        if 0 <= goal_gx < w and 0 <= goal_gy < h:
            val = grid[goal_gy * w + goal_gx]
            if val > 65:
                print(f"[HybridAStar] Goal {goal[:2]} is inside a wall (val={val}). Aborting.")
                return None

        start_node = Node(start[0], start[1], start[2])
        start_node.h = self._heuristic(start_node, goal)
        start_node.f = start_node.g + start_node.h

        # open_set  : hash → best Node seen so far
        # closed_set: hash → Node already expanded
        open_set   = {self._hash(start_node): start_node}
        pq         = [(start_node.f, start_node)]
        closed_set = {}

        iterations = 0
        max_iter   = 80_000   # raised slightly; OOB waste is gone so budget counts

        while pq and iterations < max_iter:
            iterations += 1
            _, current = heapq.heappop(pq)

            # Skip stale entries (a better node was found for this hash)
            cur_hash = self._hash(current)
            if cur_hash in closed_set:
                continue

            # --- Goal check -----------------------------------------------
            # Use a tight threshold (just over one move_step) so the planner
            # actually reaches the goal cell instead of stopping far away.
            if math.hypot(current.x - goal[0], current.y - goal[1]) < 0.35:
                print(f"[HybridAStar] Path found in {iterations} iterations.")
                return self._reconstruct_path(current)

            closed_set[cur_hash] = current

            # --- Expand neighbours ----------------------------------------
            # FIX: each (steer, direction) pair produces one child.
            # The previous code had a nested loop that overwrote `child` so
            # only the BACKWARD child was ever evaluated. Fixed by moving the
            # validity check inside the inner loop.
            for steer in self.steer_set:
                for direction in [1, -1]:
                    child = self._step(current, steer, direction)

                    child_hash = self._hash(child)
                    if child_hash in closed_set:
                        continue

                    # FIX: out-of-bounds nodes are REJECTED, not silently
                    # accepted.  Accepting them wasted the entire 50k-iteration
                    # budget exploring outside the map.
                    if not self._is_valid(child, grid, w, h, ox, oy, res):
                        continue

                    child.h = self._heuristic(child, goal)
                    child.f = child.g + child.h

                    if child_hash not in open_set or child.g < open_set[child_hash].g:
                        open_set[child_hash] = child
                        heapq.heappush(pq, (child.f, child))

        print(f"[HybridAStar] Search exhausted after {iterations} iterations — no path.")
        return None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------
    def _step(self, node, steer, direction=1):
        """Kinematic bicycle-model step."""
        move = self.move_step * direction
        nx   = node.x + move * math.cos(node.theta)
        ny   = node.y + move * math.sin(node.theta)
        nt   = node.theta + (move / self.wheelbase) * math.tan(steer)

        # Penalty structure:
        #   • reversing costs extra (prefer forward arcs)
        #   • direction change costs extra (avoids oscillating fwd/rev)
        reverse_penalty = 0.5 if direction == -1 else 0.0
        switch_penalty  = 1.0 if direction != node.direction else 0.0

        return Node(
            nx, ny, nt,
            g         = node.g + abs(move) + reverse_penalty + switch_penalty,
            parent    = node,
            direction = direction,
        )

    def _is_valid(self, node, grid, w, h, ox, oy, res):
        """
        Returns True iff every grid cell within robot_radius of the node
        centre is free (occupancy ≤ 65 at the centre, ≤ 40 in the shell).

        Key fix vs the original square check:
          • inflate_cells is derived from self.robot_radius / map-resolution,
            so the clearance scales correctly regardless of map resolution.
          • The inner loop uses math.hypot to test circular containment rather
            than a square ±N box.  This matters most at OUTSIDE CORNERS where
            a square check lets the robot pass diagonally too close to the
            corner cell, while a circle keeps it uniformly robot_radius away.
        """
        gx = int((node.x - ox) / res)
        gy = int((node.y - oy) / res)

        # Reject nodes that fall outside the map entirely
        if not (0 <= gx < w and 0 <= gy < h):
            return False

        # Centre cell: hard obstacle threshold
        if grid[gy * w + gx] > 65:
            return False

        # Circular inflation shell derived from the physical robot radius.
        # Increase self.robot_radius in __init__ to widen corner clearance.
        inflate_cells = int(math.ceil(self.robot_radius / res))
        for di in range(-inflate_cells, inflate_cells + 1):
            for dj in range(-inflate_cells, inflate_cells + 1):
                # Skip cells outside the circular footprint
                if math.hypot(di, dj) > inflate_cells:
                    continue
                nx_, ny_ = gx + di, gy + dj
                if 0 <= nx_ < w and 0 <= ny_ < h:
                    if grid[ny_ * w + nx_] > 40:
                        return False

        return True

    def _heuristic(self, node, goal):
        """Euclidean distance + small angular penalty."""
        dist = math.hypot(node.x - goal[0], node.y - goal[1])
        # Penalise heading misalignment to bias toward goal-aligned expansions
        goal_angle = math.atan2(goal[1] - node.y, goal[0] - node.x)
        dtheta = abs(goal_angle - node.theta)
        dtheta = min(dtheta, 2 * math.pi - dtheta)   # wrap to [0, π]
        return dist + 0.1 * dtheta

    def _hash(self, node):
        """Discretise (x, y, θ) into a hashable tuple."""
        return (
            int(node.x     / self.res),
            int(node.y     / self.res),
            int(node.theta / self.theta_res),
        )

    def _reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y, node.theta))
            node = node.parent
        return path[::-1]