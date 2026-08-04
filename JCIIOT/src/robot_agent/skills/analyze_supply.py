"""Analyze-supply skill — pick the best source for a target output station.

When the user says "3号出料口需要物料", this skill:
1. Scans inventory to find available crates at input stations
2. Picks the closest (or first available) source
3. Executes the full workflow: move→pick→move→place
"""

from __future__ import annotations

import logging

import numpy as np

from robot_agent.core.scene_context import SceneContext
from robot_agent.core.types import ExecutionContext, SkillResult
from robot_agent.skills.base import BaseSkill
from robot_agent.skills.move import MoveSkill, DEFAULT_CLEARANCE_WEIGHT, DEFAULT_TIGHT_CLEARANCE_M
from robot_agent.skills.pick_up import PickUpSkill
from robot_agent.skills.place_down import PlaceDownSkill
from robot_agent.skills._log import log_step, step_timer

logger = logging.getLogger(__name__)


class AnalyzeSupplySkill(BaseSkill):
    """Analyze a supply request and execute the full pick-and-place workflow.

    Given a target output station (e.g. ``output_3``), automatically:
    1. Find the best available source crate
    2. Navigate → pick → navigate → place
    """

    def __init__(
        self,
        *,
        backend,
        scene_context: SceneContext,
        grid: np.ndarray,
        path_spacing: float = 0.35,
        clearance_weight: float = DEFAULT_CLEARANCE_WEIGHT,
        tight_clearance_m: float = DEFAULT_TIGHT_CLEARANCE_M,
        abort_on_move_fail: bool = False,
    ) -> None:
        super().__init__(
            name="analyze_supply",
            description="Analyze supply/demand and auto-execute transport (given target, auto-select source)",
            keywords=(
                "analyze", "supply", "replenish", "demand", "need", "dispatch",
                "transport", "move", "carry",
            ),
        )
        self._backend = backend
        self._scene = scene_context
        self._abort_on_move_fail = abort_on_move_fail
        self._move = MoveSkill(
            backend=backend, scene_context=scene_context,
            grid=grid, path_spacing=path_spacing,
            clearance_weight=clearance_weight,
            tight_clearance_m=tight_clearance_m,
        )
        self._pick = PickUpSkill(backend=backend, scene_context=scene_context)
        self._place = PlaceDownSkill(backend=backend, scene_context=scene_context)

    def run(self, context: ExecutionContext) -> SkillResult:
        raw_target: str = (
            context.metadata.get("inputs", {}).get("target")
            or context.task
        )

        # Resolve target to output station
        from robot_agent.skills.pick_up import _resolve_station_name
        target = _resolve_station_name(raw_target, self._scene)
        logger.info("analyze_supply: target=%r (from %r)", target, raw_target)

        # Scan available crates
        available = self._backend.get_available_crates()
        logger.info("analyze_supply: available crates: %s", sorted(available.keys()))

        if not available:
            return SkillResult(
                skill_name=self.name,
                success=False,
                message=f"No materials available (all input stations are empty)",
                payload={"target": target, "available": []},
            )

        # Pick the best source (closest to robot, or first available)
        source = self._pick_best_source(available)
        logger.info("analyze_supply: selected source=%s", source)

        # Execute full workflow. Grasp success is the score gate (grasp_success
        # gates both the "departure" and "arrival" halves), so if the pick
        # fails we short-circuit immediately — continuing to move+place would
        # only waste time (hurts the time tiebreak) and risk a collision frame
        # (-5) for zero score gain. Move-to-source failure is short-circuited
        # only on opt-in (abort_on_move_fail) or when the base truly did not
        # reach, since a soft miss can still allow a successful grasp.
        steps_ok = 0
        steps_total = 0
        resolved_object_name: str | None = None
        had_collision = False

        def child_metadata(
            step_target: str,
            grasp_initial_base_pose: dict | None = None,
            object_name_override: str | None = None,
        ) -> dict:
            metadata = dict(context.metadata)
            parent_inputs = metadata.get("inputs", {})
            if not isinstance(parent_inputs, dict):
                parent_inputs = {}
            inputs = dict(parent_inputs)
            inputs["target"] = step_target
            if grasp_initial_base_pose is not None:
                inputs["grasp_initial_base_pose"] = grasp_initial_base_pose
            object_keys = ("object_name", "obj_name", "object", "target_object")
            if object_name_override:
                inputs["object_name"] = object_name_override
            elif not any(inputs.get(key) for key in object_keys):
                scene = metadata.get("scene", {})
                input_object_map = {}
                if isinstance(scene, dict):
                    input_object_map = scene.get("input_object_map", {}) or {}
                if isinstance(input_object_map, dict):
                    obj = input_object_map.get(step_target)
                    if obj:
                        inputs["object_name"] = obj
            metadata["inputs"] = inputs
            return metadata

        with step_timer(self.name, "analyze_supply") as _t:
            # Step 1: move to source
            steps_total += 1
            r1 = self._move.run(ExecutionContext(
                task=f"move to {source}",
                metadata=child_metadata(source),
            ))
            if r1.success:
                steps_ok += 1
            else:
                logger.warning("move_to_source failed (continuing): %s", r1.message)
            had_collision = had_collision or bool((r1.payload or {}).get("had_collision"))

            # Short-circuit on a hard move-to-source failure (opt-in or truly
            # unreached) — without reaching the source we cannot grasp.
            r1_reached = bool((r1.payload or {}).get("reached"))
            if not r1.success and (self._abort_on_move_fail or not r1_reached):
                log_step(self.name, "analyze_supply", ok=False, source=source, target=target,
                         failed_step="move_to_source", had_collision=had_collision, elapsed=round(_t.elapsed, 3))
                return SkillResult(
                    skill_name=self.name,
                    success=False,
                    message=f"Transport aborted: move-to-source failed at {source}",
                    payload={
                        "action": "analyze_supply", "source": source, "target": target,
                        "object_name": None, "steps_completed": steps_ok, "steps_total": steps_total,
                        "failed_step": "move_to_source", "move_message": r1.message,
                        "had_collision": had_collision, "fully_succeeded": False,
                    },
                )

            # Step 2: pick
            steps_total += 1
            source_grasp_initial_pose = (r1.payload or {}).get("final_base_pose") if r1.success else None
            r2 = self._pick.run(ExecutionContext(
                task=f"pick at {source}",
                metadata=child_metadata(source, source_grasp_initial_pose),
            ))
            if r2.success:
                steps_ok += 1
                resolved_object_name = (r2.payload or {}).get("object_name")
            else:
                logger.warning("pick failed: %s", r2.message)
                resolved_object_name = (r2.payload or {}).get("object_name")

            # Short-circuit: grasp failed → grasp_success gate cannot pass, so
            # the transport scores 0 regardless. Skip move+place to save time
            # and avoid a needless collision risk.
            if not r2.success:
                log_step(self.name, "analyze_supply", ok=False, source=source, target=target,
                         failed_step="pick", had_collision=had_collision, elapsed=round(_t.elapsed, 3))
                return SkillResult(
                    skill_name=self.name,
                    success=False,
                    message=f"Transport aborted: grasp failed at {source} — score gate (grasp_success) cannot pass",
                    payload={
                        "action": "analyze_supply", "source": source, "target": target,
                        "object_name": resolved_object_name, "steps_completed": steps_ok,
                        "steps_total": steps_total, "failed_step": "pick",
                        "pick_message": r2.message, "had_collision": had_collision,
                        "fully_succeeded": False,
                    },
                )

            # Step 3: move to target
            steps_total += 1
            r3 = self._move.run(ExecutionContext(
                task=f"move to {target}",
                metadata=child_metadata(target),
            ))
            if r3.success:
                steps_ok += 1
            else:
                logger.warning("move_to_target failed (continuing): %s", r3.message)
            had_collision = had_collision or bool((r3.payload or {}).get("had_collision"))

            # Step 4: place — carry the resolved object identity through.
            steps_total += 1
            r4 = self._place.run(ExecutionContext(
                task=f"place at {target}",
                metadata=child_metadata(target, None, resolved_object_name),
            ))
            if r4.success:
                steps_ok += 1
            else:
                logger.warning("place failed (continuing): %s", r4.message)

        # Honest success: the transport only counts if we grasped AND placed.
        transport_ok = bool(r2.success and r4.success)
        log_step(self.name, "analyze_supply", ok=transport_ok, source=source, target=target,
                 steps=f"{steps_ok}/{steps_total}", had_collision=had_collision,
                 elapsed=round(_t.elapsed, 3))
        return SkillResult(
            skill_name=self.name,
            success=transport_ok,
            message=f"Completed: {source} -> {target} ({steps_ok}/{steps_total} steps OK)",
            payload={
                "action": "analyze_supply",
                "source": source,
                "target": target,
                "object_name": resolved_object_name,
                "steps_completed": steps_ok,
                "steps_total": steps_total,
                "soft_failure": steps_ok < steps_total,
                "fully_succeeded": steps_ok == steps_total,
                "had_collision": had_collision,
            },
        )

    def _pick_best_source(self, available: dict[str, str]) -> str:
        """Select the best source from available crates.

        Strategy: among sources with a non-empty object, pick the input
        station closest to the robot's current position. Falls back to the
        first available source if none can be scored.
        """
        if not available:
            return ""
        fallback = list(available.keys())[0]
        try:
            base_xy, _ = self._backend.get_base_pose()
            best = None
            best_dist = float("inf")
            for port_name, obj_name in available.items():
                # Skip sources that report no actual object.
                if not obj_name:
                    continue
                try:
                    info = self._backend.env.input_ports.get(port_name)
                    if info is None:
                        continue
                    center = np.asarray(info["center"][:2])
                    dist = float(np.linalg.norm(center - base_xy))
                except Exception:
                    continue
                if dist < best_dist:
                    best_dist = dist
                    best = port_name
            return best or fallback
        except Exception:
            return fallback
