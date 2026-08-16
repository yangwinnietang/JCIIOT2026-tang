# 最终轨迹与评分证据

每个关卡目录都包含以下四类同时间戳文件：

- `trajectory_*_OK.json`：机器人底盘、关节、可移动物体与抓取事件的逐帧轨迹；
- `score_*_OK.json`：赛事评分逻辑输出；
- `result_*.json`：LLM 计划、技能逐步结果、耗时与错误状态；
- `scene_ready_*.json`：场景名称、任务索引与启动时间。

| 关卡 | 场景 | 轨迹帧 | 成功抓取事件 | 最终误差 | 得分 |
|---|---|---:|---:|---:|---:|
| [L1](L1/) | FactorySorting1_3FO3ERFHISEM | 2,051 | 1 | 0.17 m | 10/10 |
| [L2](L2/) | FactorySorting3_3FO3ERRPH7X9 | 1,800 | 1 | 0.14 m | 15/15 |
| [L3](L3/) | FactorySorting5_3FO3ERTPXEUT | 1,969 | 1 | 0.11 m | 20/20 |
| [L4](L4/) | FactorySorting7_3FO3ERFKY9RN | 2,734 | 1 | 0.12 m | 25/25 |
| [L5](L5/) | FactorySorting9_3FO3ERT2C5FP | 6,060 | 3 | 0.09 / 0.56 / 0.55 m | 30/30 |

这些结果是仓库内评分器生成的自测证据，并非主办方赛后认证。报告中的数值只引用这里的 JSON，不依赖手工转录。
