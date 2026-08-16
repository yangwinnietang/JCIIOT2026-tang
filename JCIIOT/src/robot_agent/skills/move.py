"""Move skill — navigate the robot base to a target via A* + backend.

Collision avoidance strategy
----------------------------
Collision *detection* lives in the (read-only) environment layer and the
base ``follow_path`` does not halt on contact. We therefore **prevent**
collisions at the planning level.

A plain binary inflation of the occupancy grid turned out to over-block the
factory's tight corridors (the only A*-feasible routes graze obstacle cells,
so any inflation makes planning fail). Instead we run a **clearance-aware
cost-weighted A***: cells stay passable exactly as in the original grid (so a
path is always found when the baseline can find one), but the step cost is
inflated for cells close to obstacles, biasing the route toward the centre of
corridors and maximising the minimum clearance to hard obstacles. Diagonal
moves are forbidden when they would "cut a corner" between two obstacles.
If scipy is unavailable or the custom planner throws, we fall back to the
core binary A* so planning never regresses.
"""

from __future__ import annotations

import heapq
import logging
import math
import re

import numpy as np

from robot_agent.core.types import ExecutionContext, SkillResult
from robot_agent.skills.base import BaseSkill
from robot_agent.skills._log import log_step, step_timer

logger = logging.getLogger(__name__)

# Weight on the clearance penalty in the A* step cost. Higher = stronger
# preference for corridor centres (at the cost of longer paths).
DEFAULT_CLEARANCE_WEIGHT = 6.0
# Below this clearance (metres) a cell is considered "tight" and penalised
# extra heavily to push the path away from walls.
DEFAULT_TIGHT_CLEARANCE_M = 0.30


class MoveSkill(BaseSkill):
    """Navigate the mobile base to a named station or world coordinate.

    Requires a backend, scene context, and occupancy grid — no mock fallback.
    """

    def __init__(
        self,
        *,
        backend,
        scene_context,
        grid: np.ndarray,
        path_spacing: float = 0.35,
        clearance_weight: float = DEFAULT_CLEARANCE_WEIGHT,
        tight_clearance_m: float = DEFAULT_TIGHT_CLEARANCE_M,
    ) -> None:
        super().__init__(
            name="move",
            description="Move to a specified location",
            keywords=(
                "move", "go", "navigate", "travel", "drive", "approach",
            ),
        )
        self._backend = backend
        self._scene = scene_context
        self._grid = grid
        self._path_spacing = path_spacing
        self._clearance_weight = clearance_weight
        self._tight_clearance_m = tight_clearance_m

    # ── public API ──────────────────────────────────────────

    def run(self, context: ExecutionContext) -> SkillResult:
        target: str = (
            context.metadata.get("inputs", {}).get("target")
            or context.task
        )

        with step_timer(self.name, "move") as _t:
            goal_xy = self._resolve_target(target)
            if goal_xy is None:
                log_step(self.name, "move", ok=False, target=target, reason="unresolved_target")
                return SkillResult(
                    skill_name=self.name,
                    success=False,
                    message=f"Cannot resolve target location: {target}",
                    payload={"action": "move", "target": target},
                )

            start_xy, start_yaw = self._backend.get_base_pose()
            path = self._plan(start_xy, goal_xy)
            if path is None:
                log_step(self.name, "move", ok=False, target=target, reason="planning_failed")
                return SkillResult(
                    skill_name=self.name,
                    success=False,
                    message=f"A* planning failed: {target}",
                    payload={"action": "move", "target": target, "start": start_xy.tolist()},
                )

            # Re-orient the base (when carrying) so the carried object's
            # sweep corridor along THIS path stays clear of every other
            # material object — see _ensure_carry_facing.
            self._ensure_carry_facing(path)

            reached = self._backend.follow_path(path)
            final_xy, final_yaw = self._backend.get_base_pose()
            # Best-effort collision surfacing: nav cannot stop on collision, but
            # expose whether any contact occurred so the planner/log can react.
            had_collision = bool(self._detect_collision_flag())
        log_step(
            self.name, "move", ok=bool(reached), target=target,
            waypoints=(len(path) if path is not None else 0),
            reached=bool(reached), had_collision=had_collision,
            elapsed=round(_t.elapsed, 3),
        )
        return SkillResult(
            skill_name=self.name,
            success=reached,
            message=f"Moved to: {target}" if reached else f"Failed to reach: {target}",
            payload={
                "action": "move",
                "target": target,
                "goal_xy": goal_xy.tolist(),
                "start_base_pose": {
                    "xy": start_xy.tolist(),
                    "yaw": float(start_yaw),
                    "robot_base_pos": [float(start_xy[0]), float(start_xy[1]), 0.0],
                    "robot_base_ori": [0.0, 0.0, float(start_yaw)],
                },
                "final_base_pose": {
                    "xy": final_xy.tolist(),
                    "yaw": float(final_yaw),
                    "robot_base_pos": [float(final_xy[0]), float(final_xy[1]), 0.0],
                    "robot_base_ori": [0.0, 0.0, float(final_yaw)],
                },
                "waypoints": len(path),
                "reached": reached,
                "had_collision": had_collision,
            },
        )

    # ── internal ────────────────────────────────────────────

    def _ensure_carry_facing(self, path) -> None:
        """Re-orient the base so the carried object's corridor along *path*
        stays clear of every other material object.

        While carrying, the tote rides at the attachment's ``relative_xy``
        (lever ~0.9m) at a FIXED base-frame angle — the holonomic base keeps
        one yaw for the whole leg, so the tote sweeps a corridor offset from
        the path by that lever.  The tote collision walls are ~0.17m taller
        than their visual rims, so a corridor that merely misses the visual
        meshes can still shove a neighbour off (L2/L3/L4/L5 knock-offs all
        happened on the first leg after a grasp).  For each candidate yaw
        delta we simulate (a) the swing of the re-orienting turn itself and
        (b) the tote's positions along the path, requiring
        ``navigation.carry_clear_dist`` from every other object.  The
        smallest |delta| that passes is applied with the guarded
        ``_drive_base_to`` turn; if none passes we keep the current yaw and
        let the drive object-guard be the safety net.
        """
        try:
            env = getattr(self._backend, "env", None)
            if env is None or not path or len(path) < 2:
                return
            from robosuite.environments.factory_sorting.transport_attachment import (
                TRANSPORT_ATTACHMENT_ATTR,
            )
            attachment = getattr(env, TRANSPORT_ATTACHMENT_ATTR, None)
            if attachment is None or not attachment.get("active"):
                return
            rel = attachment.get("relative_xy")
            if rel is None:
                return
            lever = float(np.hypot(rel[0], rel[1]))
            phi = float(np.arctan2(rel[1], rel[0]))
            held = attachment.get("object_name")
            objs = []
            for on in getattr(env, "material_objects", []) or []:
                if on == held:
                    continue
                for sfx in ("_joint0", "_free"):
                    try:
                        q = env.sim.data.get_joint_qpos(f"{on}{sfx}")
                        objs.append((float(q[0]), float(q[1])))
                        break
                    except Exception:
                        continue
            if not objs:
                return
            _nav_rp = getattr(self._backend, "_rp", {}).get("navigation", {}) or {}
            clear = float(_nav_rp.get("carry_clear_dist", 0.74))
            base_xy, yaw0 = self._backend.get_base_pose()
            base_xy = np.asarray(base_xy, dtype=float)[:2]
            # Densify the path (~0.1m spacing) — checking only waypoints
            # would miss corridor crossings mid-segment (L2: the straight
            # leg passed 0.39m from the lower tote between two "clear"
            # endpoints).
            pts: list[np.ndarray] = []
            raw_pts = [np.asarray(p, dtype=float)[:2] for p in path]
            for i in range(len(raw_pts) - 1):
                a, b = raw_pts[i], raw_pts[i + 1]
                seg = float(np.linalg.norm(b - a))
                n = max(1, int(seg / 0.1))
                for k in range(n):
                    pts.append(a + (b - a) * (k / n))
            if raw_pts:
                pts.append(raw_pts[-1])

            def _corridor_ok(theta: float) -> bool:
                # (a) swing of the re-orienting turn around the start pose
                b0 = yaw0 + phi
                d = (theta - yaw0 + np.pi) % (2 * np.pi) - np.pi
                n = max(2, int(abs(d) / 0.2) + 1)
                for a in np.linspace(0.0, d, n):
                    tp = base_xy + lever * np.array([np.cos(b0 + a), np.sin(b0 + a)])
                    for ox, oy in objs:
                        if (tp[0] - ox) ** 2 + (tp[1] - oy) ** 2 < clear * clear:
                            return False
                # (b) tote positions along the path at the fixed yaw
                off = lever * np.array([np.cos(theta + phi), np.sin(theta + phi)])
                for p in pts:
                    tp = p + off
                    for ox, oy in objs:
                        if (tp[0] - ox) ** 2 + (tp[1] - oy) ** 2 < clear * clear:
                            return False
                return True

            import math as _m
            chosen = None
            for d in (0.0, _m.radians(45), -_m.radians(45), _m.radians(90),
                      -_m.radians(90), _m.radians(135), -_m.radians(135), _m.pi):
                if _corridor_ok(yaw0 + d):
                    chosen = d
                    break
            if chosen is None:
                print(f"[CARRY_FACING] no clear facing along path (keeping yaw {yaw0:.2f})", flush=True)
                logger.warning("carry_facing: no clear facing along path (keeping yaw %.2f)", yaw0)
                return
            if abs(chosen) < 1e-3:
                print(f"[CARRY_FACING] keep yaw {yaw0:.2f} (corridor clear)", flush=True)
                return
            target_yaw = yaw0 + chosen
            print(f"[CARRY_FACING] re-orient yaw {yaw0:.2f} -> {target_yaw:.2f} (lever {lever:.2f}m)", flush=True)
            logger.info("carry_facing: re-orient yaw %.2f -> %.2f (lever %.2fm)",
                        yaw0, target_yaw, lever)
            from robosuite.environments.factory_sorting.load_factory_sorting_evalization import (
                _drive_base_to,
            )
            from robosuite.environments.factory_sorting.transport_attachment import (
                sync_transport_attachment,
            )
            def _turn_cb():
                # Keep the carried object welded through the turn — without
                # this it free-falls for the turn's duration and snaps back
                # afterwards (a visible drop+teleport defect, L2 t≈83s).
                try:
                    sync_transport_attachment(env)
                except Exception:
                    pass
                self._backend._record_trajectory_frame()
            _drive_base_to(
                env, env.robots[0], base_xy, yaw=target_yaw, max_steps=300,
                step_callback=_turn_cb,
            )
        except Exception as exc:
            logger.warning("carry_facing failed (non-fatal): %s", exc)


    def _resolve_target(self, target: str) -> np.ndarray | None:
        """Convert a target description to a (2,) world xy position.

        Resolution order:
        1. Known station name via ``SceneContext.approach_xy()`` (longest-name
           first so ``input_10`` is not shadowed by ``input_1``).
        2. Direct (x, y) tuple in the target string
        """
        # 1) named station — longest match wins to avoid input_1 ⊂ input_10
        for name in sorted(self._scene.all_port_names(), key=len, reverse=True):
            if name in target:
                return self._scene.approach_xy(name)

        # 2) numeric "x, y"
        nums = re.findall(r"[-+]?\d*\.?\d+", target)
        if len(nums) >= 2:
            try:
                return np.array([float(nums[0]), float(nums[1])], dtype=float)
            except ValueError:
                pass

        return None

    def _plan(
        self, start_xy: np.ndarray, goal_xy: np.ndarray,
    ) -> list[np.ndarray] | None:
        """Plan a world-frame path; clearance-aware A* with core fallback."""
        scene_dict = {
            "bounds": self._scene.bounds,
            "resolution": self._scene.resolution,
        }
        # Primary: clearance-aware cost-weighted A*.
        try:
            path = _plan_clearance_aware(
                scene_dict, self._grid, start_xy, goal_xy,
                min_spacing=self._path_spacing,
                clearance_weight=self._clearance_weight,
                tight_clearance_m=self._tight_clearance_m,
            )
            if path:
                return path
            logger.warning("clearance-aware A* returned empty; falling back to core A*")
        except Exception:
            logger.exception("clearance-aware A* failed; falling back to core A*")

        # Fallback: core binary A* (never regress below baseline).
        try:
            from robot_agent.core.map_loader import plan_world_path
            return plan_world_path(
                scene_dict, self._grid, start_xy, goal_xy,
                min_spacing=self._path_spacing,
            )
        except Exception:
            logger.exception("core A* planning failed")
            return None

    def _detect_collision_flag(self) -> bool:
        """Best-effort: read any collision flag the backend exposes.

        Never raises — every access is guarded so a logging/diagnostic
        failure cannot break navigation.
        """
        try:
            for attr in ("_last_nav_collision", "had_collision", "_collision_flag"):
                val = getattr(self._backend, attr, None)
                if val:
                    return True
            # Fallback: scan the tail of the recorded trajectory for has_collision.
            traj = getattr(self._backend, "get_trajectory", lambda: None)()
            if isinstance(traj, list) and traj:
                for frame in traj[-50:]:
                    if isinstance(frame, dict) and frame.get("has_collision"):
                        return True
        except Exception:
            logger.exception("collision-flag detection failed; assuming no collision")
            return False
        return False


# ── clearance-aware path planning (module-level helpers) ──────

# 8-connected neighbours: (drow, dcol, base_step_cost, is_diagonal)
_NEIGHBOURS = [
    (-1, 0, 1.0, False), (1, 0, 1.0, False),
    (0, -1, 1.0, False), (0, 1, 1.0, False),
    (-1, -1, math.sqrt(2.0), True), (-1, 1, math.sqrt(2.0), True),
    (1, -1, math.sqrt(2.0), True), (1, 1, math.sqrt(2.0), True),
]


def _clearance_field(grid: np.ndarray) -> np.ndarray:
    """Distance (in cells) from every cell to the nearest OBSTACLE (==1) cell.

    Uses scipy's Euclidean distance transform; falls back to a large constant
    if scipy is missing (the planner then degrades to ordinary A*).
    """
    g = np.asarray(grid)
    free = (g != 1)
    try:
        from scipy.ndimage import distance_transform_edt
        if free.any():
            return distance_transform_edt(free).astype(np.float64)
        return np.zeros(g.shape, dtype=np.float64)
    except Exception:
        return np.where(free, 64.0, 0.0)


def _is_passable(grid: np.ndarray, cell: tuple[int, int]) -> bool:
    """Same passability rule as core navigation (FREE/APPROACH/ROBOT)."""
    FREE, APPROACH, ROBOT = 0, 3, 4
    r, c = cell
    if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
        return False
    return int(grid[r, c]) in (FREE, APPROACH, ROBOT)


def _plan_clearance_aware(
    scene: dict,
    grid: np.ndarray,
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    *,
    min_spacing: float,
    clearance_weight: float,
    tight_clearance_m: float,
) -> list[np.ndarray]:
    """A* on the original grid with a clearance-based cost penalty.

    - Passability is identical to the core planner (never blocks a route the
      baseline could take).
    - Step cost = base_step * (1 + w * tight_penalty(clearance)). Cells within
      ``tight_clearance_m`` of an obstacle are penalised so the path prefers
      corridor centres.
    - Diagonal moves are disallowed when either orthogonal neighbour is an
      obstacle (no corner-cutting).
    """
    from robot_agent.core.navigation import (
        nearest_passable_cell, simplify_path, world_to_grid, grid_to_world,
    )

    bounds = scene["bounds"]
    resolution = float(scene["resolution"])
    g = np.asarray(grid)

    start_cell = nearest_passable_cell(g, world_to_grid(float(start_xy[0]), float(start_xy[1]), bounds, resolution))
    goal_cell = nearest_passable_cell(g, world_to_grid(float(goal_xy[0]), float(goal_xy[1]), bounds, resolution))

    clear = _clearance_field(g)
    tight_cells = max(1, int(round(tight_clearance_m / resolution)))

    def step_cost(clearance_cells: float, base: float) -> float:
        # Smooth penalty that grows as clearance shrinks below the tight band.
        if clearance_cells >= tight_cells:
            return base
        # 1 at the band edge, up to (1 + w) at zero clearance.
        ratio = 1.0 - (clearance_cells / tight_cells)
        return base * (1.0 + clearance_weight * ratio)

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap = [(heuristic(start_cell, goal_cell), 0.0, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start_cell: 0.0}

    while open_heap:
        _, cost, current = heapq.heappop(open_heap)
        if current == goal_cell:
            # reconstruct
            path_cells = [current]
            while current in came_from:
                current = came_from[current]
                path_cells.append(current)
            path_cells.reverse()
            world_path = [grid_to_world(r, c, bounds, resolution) for r, c in path_cells]
            return simplify_path(world_path, min_spacing=min_spacing)
        if cost > g_score.get(current, float("inf")):
            continue
        cr, cc = current
        for dr, dc, base, diag in _NEIGHBOURS:
            nxt = (cr + dr, cc + dc)
            if not _is_passable(g, nxt):
                continue
            if diag:
                # No corner-cutting: both orthogonal sides must be free.
                if not _is_passable(g, (cr + dr, cc)) or not _is_passable(g, (cr, cc + dc)):
                    continue
            clearance = float(clear[nxt[0], nxt[1]])
            tentative = cost + step_cost(clearance, base)
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                heapq.heappush(open_heap, (tentative + heuristic(nxt, goal_cell), tentative, nxt))

    return []  # no path — caller falls back to core A*
