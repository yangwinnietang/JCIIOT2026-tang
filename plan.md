# JCIIOT 2026 满分冲刺 — 交接计划(plan.md)

> 最后更新:2026-08-05 03:20(会话交接)。目标:客观分 100/100(60%权重)+ 创新性评审满分(40%权重)。
> 下一个会话请先读完本文件,再继续执行。

---

## 1. 比赛与评分机制(已逐行核实)

- 平台:MuJoCo + robosuite + robomimic,Tiago 双臂移动机器人,5 个 FactorySorting 场景。
- 任务:把指定工位的物料抓到并运到目标工位。五关共 100 分:L1=10、L2=15、L3=20、L4=25、L5=30。
- 官方评分在 `JCIIOT/app.py`(禁改):`_score_steps()`(约 L1870)与 `_score_l5_multi_object()`(约 L1750),规则版本 `grasp_success_gate_l5_multi_v2`。
- **评分逻辑(关键)**:
  1. 轨迹 JSON 的 `events[]` 里必须有 `grasp_end` 事件且 `success` 为真、source/object 匹配 —— 这是闸门,没有则整关 0 分。
  2. 离开源站(半分):抓取后物体相对源站中心 |dx|>1m 或 |dy|>1m。
  3. 到达目标(半分):最后一帧物体 XY 距目标站中心 <0.80m。
  4. 任一帧 `has_collision=true` → 总分 -5(碰撞判定=机器人 geom 与 `scene_aabb_proxy_*` 障碍代理盒接触,一旦触发整局锁存)。
  5. 用时只作平局决胜,不计分。
  6. L5 特殊:3 个白色料箱 `white_tote_b01_left_{center,front,back}` 各 10 分(离开5+放置5),独立计分。
- 修改权限:✅ `src/robot_agent/skills/`、`src/robot_agent/workflows/`、`knowledge/robot_params.json`;❌ `src/robot_agent/core/`、`src/robot_agent/environments/`、`app.py`、`knowledge/task_config.json`。
- **API 密钥严禁写入仓库文件**(决赛代码要公开)。智谱 key 只走环境变量:`OPENAI_API_KEY`/`OPENAI_BASE_URL=https://open.bigmodel.cn/api/paas/v4`/`OPENAI_MODEL=glm-5.2`;VLM 用 `VLM_API_KEY`/`VLM_MODEL=glm-5v-turbo`(key 值见会话记录,用户已提供;本文件不保存)。

## 2. 已核实的现状结论

| 项 | 状态 |
|---|---|
| L1 | 历史有 1 次 10/10(录于用户 Windows 机 `E:\BaiduNetdiskDownload\JCIIOT 2026\Final\`),其后 2 次 grasp=no 得 0 分 |
| L2–L5 | 无任何运行记录 |
| BC 抓取模型 | 本地 `JCIIOT/robosuite/robosuite/model_epoch_150.pth` 是 134B LFS 指针;官方仓库 LFS 有真身(139,543,773 B,sha256 `ef5910f6a9f6309b...3252169f`) |
| 其余 10 个 LFS 文件 | **均非必需**(五关场景 .obj 网格是 git 真文件;USD zip/hdf5/lowered_table 与跑分无关),不用下载 |
| 交付物 | `team_submission/` 近空(缺 skills/models/config.yaml/zip)、无技术报告、sop_gen_case_*.md 有 19 处 "VLM 429" 占位符、全部工作未提交 |
| L5 规划缺口 | planner(`core/planner.py`,禁改)强制单循环 4 步计划 → 已用 skills 层循环解决(见 §3.2) |

## 3. 本会话已完成的工作

### 3.1 环境搭建(Phase 0 完成 ~90%)
- 系统库:OSMesa(`libosmesa6-dev` 等)已装,`MUJOCO_GL=osmesa` 无头渲染可用。
- Python venv:`/mnt/workspace/JCIIOT2026/.venv`(Python 3.11.11),`requirements.txt` + `pip install -e ./robosuite` + `pip install -e .` **全部装完(INSTALL_ALL_OK)**。
- 冒烟测试:`test_scene_load.py` **ALL CHECKS PASSED**(robosuite 1.5.2、FactorySorting headless 构建/reset/10步成功)。
- torch 2.7.0+cu126,**当前无 GPU**(cuda:False)。用户说"需要 GPU 时重启电脑就有"——训练 BC 前需要用户重启提供 GPU。
- 开发评分器:`JCIIOT/score_dev.py`(从 app.py AST 提取官方评分函数,零漂移)。已用历史轨迹验证:满分轨迹复现 10/10、失败轨迹复现 0/10。用法:
  ```bash
  cd /mnt/workspace/JCIIOT2026/JCIIOT
  python3 score_dev.py recordings/<env>/trajectory_xxx.json [--save]
  ```

### 3.2 L5 三料箱循环(Phase 2 代码已写完,待实测)
- `skills/pick_up.py`:新增 L5 检测与 `_run_l5_multi_transport()` —— 在 FactorySorting9 场景、target=input_1 时,按 center→front→back 顺序循环:读 `env.material_metadata` 实测箱体位置 → MoveSkill 到每箱抓取站位(箱体 x + 接近点偏移 1.44m,y 同箱体)→ `grasp_object_physics(object_name=该箱)` → 运至 output_6 → 放置。单箱失败不中断,其余箱仍可得分。
- `skills/place_down.py`:多箱循环后空夹爪的尾随 place_down 步返回成功 no-op,不再报错。
- `skills/library.py`:给 PickUpSkill 注入 move/place 子技能。
- 语法已校验;运行时尚未实测(需要模型文件)。

### 3.3 SOP 生成修复(Phase 3 代码已写完,待重跑)
- `workflows/generate_sop_knowledge.py`:新增 `_retry_call()` 指数退避重试(4 次,3s→24s),VLM 并发从 5 降到 2,LLM 结构化同样重试。修掉 19 处 "HTTP 429" 占位符的重跑命令(需要 API 环境变量):
  ```bash
  cd /mnt/workspace/JCIIOT2026/JCIIOT
  export OPENAI_API_KEY=<智谱key> OPENAI_BASE_URL=https://open.bigmodel.cn/api/paas/v4 OPENAI_MODEL=glm-5.2
  export VLM_API_KEY=<智谱key> VLM_BASE_URL=https://open.bigmodel.cn/api/paas/v4 VLM_MODEL=glm-5v-turbo
  .venv/bin/python -m robot_agent.workflows.generate_sop_knowledge
  ```

### 3.4 模型下载(进行中)
- GitHub 直连限流严重(~17KB/s,16 路并发触发封禁)。已部署 **独立会话守护进程** `.dl_supervisor.sh`(setsid 启动,会话结束后继续运行):循环执行 `.dl_model.sh`(16 块 × 8.7MB,4 路并发,断点续传),校验 sha256 后自动部署到 `JCIIOT/robosuite/robosuite/model_epoch_150.pth`。
- 监控命令:
  ```bash
  tail -20 /mnt/workspace/JCIIOT2026/model_download_supervisor.log
  du -sh /mnt/workspace/JCIIOT2026/.model_chunks
  ls -la /mnt/workspace/JCIIOT2026/JCIIOT/robosuite/robosuite/model_epoch_150.pth
  ```
- 下载完成的标志:日志出现 `DOWNLOAD_VERIFIED_AND_DEPLOYED`,且模型文件 139543773 字节。
- **更快的备选**:用户 Windows 机器上有真模型(百度网盘下载的完整包)。若用户能上传,直接覆盖 `JCIIOT/robosuite/robosuite/model_epoch_150.pth` 即可。

## 4. 下一步(按顺序执行)

### Phase 0 收尾
1. 确认模型下载完成(§3.4 监控命令)。下载期间可先做 Phase 3 重跑。
2. 验证模型可读:
   ```bash
   cd /mnt/workspace/JCIIOT2026/JCIIOT
   MUJOCO_GL=osmesa .venv/bin/python -c "import torch; p=torch.load('robosuite/robosuite/model_epoch_150.pth', map_location='cpu', weights_only=False); print(type(p))"
   ```

### Phase 1 — L1–L4 逐关跑分(核心)
3. 设置环境变量(智谱 key,只进环境不入库)。
4. 逐关运行(与 app.py Execute 完全同路径;task 文本必须与 app.py TASKS 原文一致):
   ```bash
   cd /mnt/workspace/JCIIOT2026/JCIIOT
   MUJOCO_GL=osmesa .venv/bin/python src/robot_agent/task_subprocess_runner.py \
     --task "For this task, you need to transport a blue, hollow plastic box. Please move it from the starting point \"Pick Station 2\" to the destination \"Place Station 3\". Please follow the Standard Operating Procedure (SOP)." \
     --task-index 0 --timestamp $(date +%Y%m%d_%H%M%S) \
     --result-json recordings/FactorySorting1_3FO3ERFHISEM/result_dev.json \
     --app-dir /mnt/workspace/JCIIOT2026/JCIIOT --knowledge-enabled true
   ```
   L2–L5 的 task 文本与 task-index 见 `app.py` 的 `TASKS`(约 L2163)。
5. 评分:`python3 score_dev.py recordings/<env>/trajectory_<ts>_OK.json --save`
6. 调优(只改许可区):`knowledge/robot_params.json`、`skills/move.py|pick_up.py|place_down.py`。
7. **验收:每关连续 3 次满分且零碰撞帧**(历史有 grasp=no 方差,稳定性是硬指标)。
8. 后备:若官方模型某关抓不起 → TASK_D 重训(见 §6 风险)。

### Phase 2 实测 — L5
9. 用 task-index 4 跑 L5,确认三箱各自 grasp_end + 放置;score_dev 验收 30/30。

### Phase 3 — SOP 重跑
10. 按 §3.3 命令重跑;验收:sop_gen_case_{1,3,5,7,9}.md 无 "(VLM error" 字样,`_sop_gen_log.json` 全 ok。

### Phase 4 — 提交包(创新性 40% 依赖)
11. `team_submission/skills/my_pick_up.py`(按 `src/competition_platform/interface/skill_contract.py` 契约写,可直接包装现有 PickUpSkill)、`team_submission/models/model_epoch_150.pth`、`team_submission/config.yaml`、完善 `team_submission/knowledge/my_strategy.md`(当前是占位 stub;写入 L5 多箱策略与调优心得)。
12. 技术报告(新建 `TECHNICAL_REPORT.md` 或扩充 README):方案框架(LLM 规划 + SOP 知识库 + clearance-aware A* + BC 抓取 + L5 多箱循环调度)、**新颖性声明**(对照 SOP-MapGuard 100 分榜首)、五关得分/用时表、局限性。
13. 可选视频:`RobosuiteBackend.replay_trajectory()` 渲染。
14. `team_submission/` 打 zip。
15. 排行榜 GitHub issue 由用户本人提交。

### Phase 5 — 清理与提交
16. 删 `robosuite/=3.3.0` 垃圾文件;`new/__init__.py` 坏导入(引用不存在的 .world)修复或标注。
17. git 提交全部工作(本会话的代码改动在提交前不要丢)。

## 5. 关键文件速查

| 用途 | 路径 |
|---|---|
| 官方评分 | `JCIIOT/app.py`(L1680–2070 评分区;L2163 TASKS 原文) |
| 开发评分器 | `JCIIOT/score_dev.py` |
| 运行入口 | `JCIIOT/src/robot_agent/task_subprocess_runner.py` |
| 抓取后端 | `JCIIOT/src/robot_agent/environments/robosuite_backend.py`(grasp L935、place L1206、follow_path L1827+) |
| 任务/站位配置 | `JCIIOT/knowledge/task_config.json`(锁)、各场景 `generated_maps/*_semantic_map.json` |
| 训练流水线 | `JCIIOT/robosuite/TASK_D_README.md`、`robosuite/scripts/train_grasp_bc.py`、`load_factory_sorting_collect.py` |
| L5 箱体坐标 | `factory_sorting_9_3fo3ert2c5fp.py` WHITE_TOTE_REPLACEMENTS:x=-14.674,y=4.415/4.955/5.521,z=1.329 |
| 下载守护 | `.dl_supervisor.sh` / `.dl_model.sh` / `model_download_supervisor.log` |

## 6. 风险与对策

| 风险 | 对策 |
|---|---|
| GitHub 限流下载失败 | 守护进程自动重试;用户上传 Windows 机器上的模型最快 |
| 官方模型只对部分关卡有效 | TASK_D 流水线重训:`load_factory_sorting_collect.py --level N` 脚本化采集 → `train_grasp_bc.py`(需 GPU) |
| L5 前/后箱抓取位姿偏移 | 已用 material_metadata 实测坐标驱动站位;不行再微调偏移量 |
| LLM 规划抖动 | team knowledge 强约束;planner 提示词已注入当关 SOP 行与物体映射 |
| 无 GPU 训练慢 | 用户重启电脑提供 GPU;CPU 仅用于推理跑分 |
| 评分漂移 | 始终用 score_dev.py(提取 app.py 原函数),最终以 app.py 手动 Execute 复核 |
