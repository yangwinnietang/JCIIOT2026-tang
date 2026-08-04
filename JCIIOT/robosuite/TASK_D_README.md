# Task D — 自定义 BC 抓取策略训练流水线

本目录交付一套**可运行的采集→训练→部署**流水线，用于训练自定义 BC（行为克隆）抓取策略模型替换占位 checkpoint，从而真正打通评分闸门 `grasp_success`（它同时门控"成功离开"与"成功到达"两半分）。

> **当前状态（重要）**：`robosuite/robosuite/model_epoch_150.pth` 仅 134 字节，是占位 stub，不是真实训练模型。本沙箱 `import mujoco` 失败，无法在此采集/训练。**必须在装有 MuJoCo 的环境按本文档运行**，产出真模型后部署。

## 修改权限
本流水线文件均位于 `robosuite/` 下，符合 CLAUDE.md「训练自定义模型」许可；未修改受保护的 `core/`、`environments/`、`app.py`、`task_config.json`。

## 文件清单
| 文件 | 作用 |
|---|---|
| `robosuite/robosuite/environments/factory_sorting/load_factory_sorting_collect.py` | 泛化采集脚本：按 `task_config.json` 派生 5 关的 env/object/base pose，复用 scene-1 采集器的 OSC 双臂抓取流水线 |
| `robosuite/scripts/train_grasp_bc.py` | robomimic BC 训练入口（仿 `train.py::main`，加 `--output-dir/--epochs` 与时间戳日志） |
| `robosuite/scripts/bc_grasp_config.json` | BC 训练覆盖配置（obs_modality 已按实测 hdf5 填写） |
| `robosuite/TASK_D_README.md` | 本文档 |

## 前置条件
1. MuJoCo 可用：`python -c "import mujoco; import robosuite"` 成功。
2. 无头渲染后端：`export MUJOCO_GL=osmesa`（或 `egl`）。采集脚本默认 `--no-render`，但相机观测仍需 offscreen 渲染（`has_offscreen_renderer=True`），故必须设置 `MUJOCO_GL`。
3. GPU 可选但建议（`torch.cuda.is_available()`）。

## Step 1 — 采集演示数据（每关）
```
export MUJOCO_GL=osmesa
# 单关
python -m robosuite.environments.factory_sorting.load_factory_sorting_collect \
    --level 1 --num-rollouts 50 --no-render --output-name grasp_l1
# 批量 5 关（--level 支持多值）
python -m robosuite.environments.factory_sorting.load_factory_sorting_collect \
    --level 1 3 5 7 9 --num-rollouts 50 --no-render
```
- 产物：`robosuite/robosuite/models/assets/demonstrations_private/<时间戳>/grasp_l<level>_<时间戳>.hdf5`
- 5 关的 env 类已在 `factory_sorting/__init__.py` 注册，`suite.make(env_name=...)` 可解析任意关卡。
- L1 还原 scene-1 采集器（object `line_5_container_h01_near`, base `[8.0,4.6,0.0]`, yaw≈π），用于校验泛化正确性。

## Step 1.5 — 核对数据 obs 键
```
python robomimic/scripts/get_dataset_info.py --dataset <上一步 hdf5>
```
确认 obs 键与 `bc_grasp_config.json` 的 `algo.obs_modality` 一致。当前配置按 `grasp_l1_test.hdf5` 实测填写：
- low_dim：`robot0_left_eef_pos/quat/gripper_qpos`、`robot0_right_eef_pos/quat/gripper_qpos`
- rgb：`robot0_robotview_image`
- actions：20 维（Tiago 双臂）

若采集脚本改了相机/观测，需同步修改配置。

## Step 2 — 训练 BC
```
python robosuite/scripts/train_grasp_bc.py \
    --dataset <采集的 hdf5> \
    --config robosuite/scripts/bc_grasp_config.json \
    --output-dir robosuite/runs --epochs 150
# 快速冒烟（2 epoch）
python robosuite/scripts/train_grasp_bc.py --dataset <hdf5> --debug
```
- 产物：`robosuite/runs/grasp_bc/models/` 下的 epoch checkpoint。
- 配置默认 `experiment.rollout.enabled=false`（无 mujoco 不做 rollout 评估；有环境时可改 true）。
- `experiment.save.epochs=[150]` 确保保存第 150 epoch。

## Step 3 — 部署
1. 把训练好的 checkpoint 拷贝/重命名为 `model_epoch_150.pth`：
   ```
   cp robosuite/runs/grasp_bc/models/<对应 epoch 文件> robosuite/robosuite/model_epoch_150.pth
   ```
   或在 `knowledge/robot_params.json` 把 `grasp_policy.checkpoint_path` 改为新路径。
2. 受保护的 `robosuite_backend.py` 读取 `grasp_policy.checkpoint_path`（失败回退 `checkpoint_fallback_path`），重启 `app.py` 生效。
3. **obs_modality 必须与后端推理时喂入的观测键一致**——两者均来自同一 FactorySorting env 观测规格，故与采集 hdf5 一致即可。若后端报观测维度不匹配，回 Step 1.5 核对。

## Step 4 — 评估
```
python robomimic/scripts/run_trained_agent.py --agent <ckpt> --horizon 400
```
或直接在 `app.py` 跑分（抓取成功 → 解锁"离开/到达"两半分）。

## 复现说明
- `--num-rollouts 50`：每关 50 次尝试，保留成功 demo；数据越多越稳，建议 ≥30 成功 demo/关。
- `--epochs 150`：与现有 checkpoint 命名一致，便于 drop-in 部署；可按收敛情况调整。
- `--seed`：可固定以保证可复现。

## 已知限制
- 本沙箱无 mujoco，上述命令未在此运行验证；语法已通过 `py_compile`/`ast.parse` 校验。团队须在 mujoco 环境首跑确认（尤其 obs 键与后端推理一致性）。
- stub 模型是当前最大得分风险；本流水线是通向可用抓取的唯一路径，强烈建议赛前完成训练部署。
