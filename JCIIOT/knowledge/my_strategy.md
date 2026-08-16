# SOP-Runner Final Execution Strategy — JCIIOT 2026

## Purpose and authority

This document is planner-facing knowledge for the **final submitted controller**. It supersedes all experimental strategies. Task identities and station mappings come from `task_config.json`; scene-specific SOP constraints come from `sop_gen_case_*.md`.

Never invent a station, object name or shortcut. Never request teleportation, direct final-pose placement, collision-flag modification, skipped skills, or a score-only action.

## Required skill program

For every object, emit exactly this ordered program:

1. `move(target=<source station>)`
2. `pick_up(target=<source station>, object_name=<exact object>)`
3. `move(target=<destination station>)`
4. `place_down(target=<destination station>)`

Use no retries unless the runtime explicitly reports a recoverable failure. Do not combine movement and manipulation into an unregistered skill.

L5 requires three complete four-step cycles, one for each white tote. Preferred planning order is center, front, back; the runtime verifies live positions and may conservatively substitute a same-family tote that is still at the source if the planner repeats a stale name.

## Final task map

| Level | Source | Destination | Exact planned object(s) |
|---|---|---|---|
| L1 | `input_5` | `output_4` | `line_5_container_h01_near` |
| L2 | `input_6` | `output_4` | `green_tote_b01_upper` |
| L3 | `aux_input_1` | `output_5` | `blue_tote_b01_far_right` |
| L4 | `input_2` | `output_5` | `blue_container_h01_back_upper` |
| L5 | `input_1` | `aux_output_1` | `white_tote_b01_left_center`, `white_tote_b01_left_front`, `white_tote_b01_left_back` |

If the natural-language task conflicts with this map, prefer the current task record in `task_config.json` and the generated SOP for that level. Do not fall back to legacy competition SOP files whose station wording may be outdated.

## Execution model

### Navigation

`MoveSkill` plans on the semantic occupancy grid with clearance-cost A*. It preserves the baseline passable-cell set, penalizes cells within 0.30 m of obstacles, forbids diagonal corner cutting, and incorporates the rendered-machine shell layer. The backend follows the resulting path with bounded speeds (`max_linear=0.45 m/s`, `max_angular=0.90 rad/s`).

The planner should provide only the semantic station name. It must not prescribe coordinates or bypass the path follower.

### Grasp

`PickUpSkill` resolves the station and calls the deterministic six-phase operational-space servo: clearance lift, XY approach, descent, settle, close, hold/lift verification.

Grasp-wall selection is automatic:

- containers whose nominal sites face the aisle retain those sites;
- regular input-line totes use the `+x` wall (`inset=0.30 m`, `span=±0.12 m`);
- auxiliary-input totes use the `−y` wall with a wall-normal centered stance.

Close-range stance correction uses bounded 0.02 m generalized-coordinate increments, a simulation step and a recorded frame per increment, plus contact/rendered-shell trial-and-revert guards. The planner must not add a separate stance skill.

### Transport and state consistency

The environment's `transport_attachment` keeps the grasped object synchronized during carrying. Grasp control uses an isolated evaluation environment; after success, only the active object's state is reconciled and the navigation base is moved incrementally to the sandbox's world pose. Other objects retain their live navigation-world state.

### Placement

`PlaceDownSkill` selects a clear landing slot, aligns the carried object's lever arm to the target ray, turns outside crowded tables, approaches radially, lowers to a table-aware release height, clears the transport attachment, and opens the grippers. The planner should provide only the destination station; it must not invent offsets or object poses.

## L5 multi-object safeguards

- Plan three distinct object names and three independent cycles.
- Never regard the first successful placement as completion of the entire level.
- Runtime reselection is allowed only when the requested object is more than 1.5 m from the source and a same-family candidate remains near the source.
- Placement candidates are evaluated against live object positions; a swing guard aborts before a new closest approach violates the configured separation.
- A successful level must contain three independent `grasp_end(success=true)` events.

## Scoring-aware checks

For each object, success requires both conditions after a verified grasp:

1. it leaves the source by more than 1 m along x or y;
2. its final planar distance to the destination center is below 0.8 m.

Any collision can deduct five points. Safety guards therefore take precedence over a shorter route. Do not modify or clear collision state.

## Final validated outcome

The 2026-08-16 submitted runs produced L1–L5 scores of 10, 15, 20, 25 and 30, totaling **100/100**. Final target errors were 0.17 m, 0.14 m, 0.11 m, 0.12 m, and 0.09/0.56/0.55 m for the three L5 totes. These are self-evaluation results from the repository scoring path; exact evidence is stored under `trajectories/`.

## Model note

`models/model_epoch_150.pth` is the archived robomimic behavior-cloning checkpoint from the explored learning path. The final scored grasp actions are generated by the deterministic servo, not by the checkpoint. Retain the model for provenance and ablation, but do not describe it as the final policy.
