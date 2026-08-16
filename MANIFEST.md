# JCIIOT 2026 提交包清单

生成日期：2026-08-16<br>
队伍：OeacnYang<br>
方案名称：SOP-Runner

## 完成情况

| 提交要求 | 位置 | 状态 |
|---|---|:---:|
| 路径生成与机器人技能代码 | [`code/`](code/)；完整工程 [`JCIIOT/`](JCIIOT/) | ✓ |
| 五关最终轨迹 | [`trajectories/L1..L5`](trajectories/) | ✓ |
| 评分与运行结果 JSON | 各关轨迹目录内 `score/result/scene_ready` | ✓ |
| 中文技术报告 | [`README.md`](README.md) | ✓ |
| 英文技术报告 | [`docs/TECHNICAL_REPORT_EN.md`](docs/TECHNICAL_REPORT_EN.md) | ✓ |
| 新颖性说明 | 中文报告“新颖性说明”；英文报告 §5 | ✓ |
| 结果与局限分析 | 中文报告“定量结果/局限性”；英文报告 §4/§8 | ✓ |
| 五关三视角整合视频 | [`videos/composed/`](videos/composed/) | ✓ |
| 15 个独立视角视频 | [`videos/individual/`](videos/individual/) | ✓ |
| 设计过程与失败实验 | [`docs/DEVELOPMENT_LOG_ZH.md`](docs/DEVELOPMENT_LOG_ZH.md) | ✓ |
| BC checkpoint | `code/models/` 与 `JCIIOT/models/`（Git LFS） | ✓ |

## 最终评分

L1–L5 分别为 **10 / 15 / 20 / 25 / 30**，总计 **100/100**。详情与直接 JSON 链接见 [`README.md`](README.md#定量结果)。成绩为仓库内赛事评分逻辑的可审计自测输出。

## 安全与敏感信息

- 仓库不包含 GitHub token、模型 API key 或其他凭据；
- 所有 API 凭据均通过环境变量传入；
- Python 缓存、临时模型分片、运行中轨迹与日志由 `.gitignore` 排除；
- MP4 使用 ASCII 文件名，避免跨平台路径编码问题。
