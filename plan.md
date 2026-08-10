# JCIIOT 2026 — Final Status (Session #4, 2026-08-10)

> Score: **75/100** (L1=10, L2=15, L3=20, L4=25, L5=5)

## Key Breakthrough: Scripted Grasp Policy

Replaced the BC (Behavior Cloning) model with a **deterministic OSC waypoint-servo grasp policy** that directly executes the same motion plan used during demonstration collection. This eliminated the RGB rendering mismatch and multi-modal action distribution problems that made the BC model fail.

### Architecture
1. **Scripted Grasp** (`load_factory_sorting_evalization.py`): 6-phase motion plan (lift→approach→descend→settle→close→hold) using proportional OSC delta control
2. **Stance Correction** (`robosuite_backend.py`): Reads object world XY from MuJoCo free-joint, repositions robot to (obj_x + 0.85, obj_y) before creating eval env
3. **Adaptive xwall-grasp**: Detects whether nominal grasp sites are both on +x wall (containers: L1/L4) or need relocation (totes: L2/L3/L5)
4. **Semantic Map Place Fallback**: Falls back to semantic map output_ports when robosuite env doesn't have the target station

### Scores
| Level | Score | Grasp | Place |
|-------|-------|-------|-------|
| L1 | 10/10 | ✅ | ✅ |
| L2 | 15/15 | ✅ | ✅ |
| L3 | 20/20 | ✅ | ✅ |
| L4 | 25/25 | ✅ | ✅ |
| L5 | 5/30 | ✅ (1/3 totes) | ❌ (A* nav failed) |

### L5 Issues (for future work)
- A* path planning fails for long-distance transport (start→output_6)
- Robot orientation not reset between totes (yaw=-0.48 instead of -π after first place)
- Need to fix navigation or implement direct teleport for L5

### Key Parameters
- `standoff_x`: 0.85
- `coll_clearance`: 0.25
- `coll_arrival_tol`: 0.08
- `coll_settle_steps`: 150

### Modified Files
- `load_factory_sorting_evalization.py` — scripted grasp policy (replaced BC loop)
- `robosuite_backend.py` — stance correction + semantic map place fallback
- `robot_params.json` — standoff_x tuning
- `pick_up.py` — L5 multi-tote loop (pre-existing)
- `TECHNICAL_REPORT.md` — full technical report
- `team_submission/` — submission package (model, skills, knowledge)
