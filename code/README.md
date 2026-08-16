# 精简代码包

此目录只包含本队实现、配置、知识文件与模型，便于评委快速审查。仓库中的 `JCIIOT/` 是完整可运行工程；两者的映射如下。

| 精简包 | 完整工程目标位置 | 说明 |
|---|---|---|
| `skills/move.py` | `JCIIOT/src/robot_agent/skills/move.py` | 清障代价 A* 与路径执行 |
| `skills/library.py` | `JCIIOT/src/robot_agent/skills/library.py` | 技能装配、视觉外壳栅格注入 |
| `skills/pick_up.py` | `JCIIOT/src/robot_agent/skills/pick_up.py` | 工位解析、多物体重选、抓取入口 |
| `skills/place_down.py` | `JCIIOT/src/robot_agent/skills/place_down.py` | 杠杆臂对齐、落点选择、径向放置 |
| `skills/_factory_physics_patch.py` | `JCIIOT/src/robot_agent/skills/_factory_physics_patch.py` | 抓取/放置/记录后端的运行时实现 |
| `skills/_log.py` | `JCIIOT/src/robot_agent/skills/_log.py` | 结构化步骤日志 |
| `workflows/*.py` | `JCIIOT/src/robot_agent/workflows/` | 任务流与 SOP 知识生成 |
| `knowledge/*` | `JCIIOT/knowledge/` | 最终知识、参数和任务映射 |
| `models/model_epoch_150.pth` | `JCIIOT/models/model_epoch_150.pth` | BC 训练产物（LFS；最终伺服的溯源/消融资产） |

`skills/my_pick_up.py` 是赛事 `team_submission` 约定的兼容入口，并与 `pick_up.py` 保持逐字节一致；完整工程安装时使用后者，提交包检查器使用前者。

## 审计入口

- 导航：`skills/move.py::_plan_clearance_aware`
- 可见外壳危险层：`skills/_factory_physics_patch.py::_visual_shell_grid`
- 有界增量站位与接触回滚：`skills/_factory_physics_patch.py::_drive_base_to`
- 六阶段抓取：`skills/_factory_physics_patch.py::run_factory_sorting_grasp_in_wrapped_env`
- 最小世界同步：`skills/_factory_physics_patch.py::grasp_object_physics`
- 连续轨迹记录：`skills/_factory_physics_patch.py::_record_trajectory_frame`
- 放置后端：`skills/_factory_physics_patch.py::place_object_physics`
- 多物体重选：`skills/pick_up.py::_reselect_if_already_moved`
- 杠杆臂/径向放置：`skills/place_down.py::_prepare_radial_place`

## 语法检查

```bash
python -m compileall -q code
```

完整安装与运行命令见仓库根目录 [`README.md`](../README.md#可复现性)。
