# UE5 等价性报告（当前状态）

> **状态：历史快照已归档，当前版本待重跑。**
>
> 本文件为 `docs/ue_equivalence_report.md` 的当前版本占位说明。完整的
> pre-20260905 UE 验证报告（旧架构、旧场景）已移至
> `archive/ue_equivalence_report.pre20260905.md`，其中保留了旧碰撞量与旧房间数的
> 历史数值，**不作为当前项目结论**。

## 当前权威结论来源（jihua20260905.md §9.2 / §12）

- 场景单一真源：`config/scene_home.json` **v6.0**，当前为 **5 个房间**
  （living_room / kitchen / bedroom / bathroom / hallway），初始位姿
  `(0.5, -2.2, 0.0)`，规划半径 `0.35 m`。
- 契约单一真源：`config/simulation_contract.yaml` 与 `config/geometry_spec.yaml`
  （robot `radius=0.35`、footprint `0.70×0.50`、Nav2 `inflation_radius=0.43`）。
- 精度基准：**C++ nav_core 固定种子长稳回归**（非 UE）。
- UE bridge 定位等价性、AMCL 覆盖率、坐标 RMS、断链停车等指标，以
  `artifacts/<run_id>/` 下验收 JSON 与 `scripts/run_regression.sh` 的门禁为准。

## 何时更新本报告

当 UE5 + bridge 真实联调环境就绪（当前 M5 受环境阻塞）后，按 jihua20260905.md
第 5 节清单重新取得 1800 / 9000 / 36000 帧证据，再用 `scripts/gen_release_report.py`
生成本文件的当前数据段。在重跑完成前，任何文档不得再把旧房间数、旧 Nav2 半径、
旧 dock 坐标当作当前项目事实。
