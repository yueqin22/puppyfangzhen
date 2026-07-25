# 一键复现实验汇总报告

**生成时间**: 2026-07-13 06:19:26
**运行模式**: demo
**总耗时**: 21.0 秒
**输出目录**: `E:\puppyfangzhen\experiment_results`

## 1. 实验状态总览

| 实验 | 状态 | 说明 |
|------|------|------|
| 基线对比 | ✅ 完成 | 基线对比 |
| 7维消融 | ✅ 完成 | 7维消融 |
| 协同效应 | ✅ 完成 | 协同效应 |
| 参数敏感性 | ✅ 完成 | 参数敏感性 |
| 失败案例 | ✅ 完成 | 失败案例 |
| 理论分析 | ✅ 完成 | 理论分析 |

## 2. 基线对比实验

| 策略 | 最终覆盖率(%) | T80(s) |
|------|-------------|--------|
| random_walk | 52.14 | 149.2 |
| nearest_frontier | 77.45 | 149.2 |
| greedy_info | 72.83 | 149.2 |
| oracle | 96.01 | 18.0 |

![基线对比](baselines/baselines_coverage.png)

## 3. 7维消融实验

### 覆盖率指标（mean ± 95% CI）

| 配置 | 均值 | CI下限 | CI上限 | n |
|------|------|--------|--------|---|
| proposed_full | 93.78 | 91.76 | 95.81 | 5 |
| wo_aufe | 90.92 | 89.68 | 92.17 | 5 |
| wo_risk | 90.62 | 89.28 | 91.95 | 5 |
| wo_cbf | 91.93 | 90.40 | 93.46 | 5 |
| wo_gnn | 92.28 | 91.36 | 93.20 | 5 |
| wo_dynobs | 94.59 | 93.21 | 95.96 | 5 |
| wo_amcl | 96.46 | 95.84 | 97.09 | 5 |
| wo_teb | 89.55 | 86.63 | 92.47 | 5 |

### 配对 t 检验（proposed_full vs 消融配置）

| 指标 | 对比 | t | p值 | Cohen d | 显著 |
|------|------|---|-----|---------|------|
| 覆盖率(%) | wo_aufe | 4.930 | 0.000 | 2.116 | *** |
| 覆盖率(%) | wo_risk | 3.793 | 0.000 | 2.295 | *** |
| 覆盖率(%) | wo_cbf | 1.610 | 0.142 | 1.282 | ns |
| 覆盖率(%) | wo_gnn | 2.205 | 0.044 | 1.190 | ns |
| 覆盖率(%) | wo_dynobs | -2.265 | 0.039 | -0.575 | ns |
| 覆盖率(%) | wo_amcl | -4.153 | 0.000 | -2.221 | *** |
| 覆盖率(%) | wo_teb | 4.268 | 0.000 | 2.091 | *** |
| T80%(s) | wo_aufe | -7.227 | 0.000 | -4.556 | *** |
| T80%(s) | wo_risk | -11.395 | 0.000 | -7.996 | *** |
| T80%(s) | wo_cbf | -9.344 | 0.000 | -3.341 | *** |
| T80%(s) | wo_gnn | -4.489 | 0.000 | -3.898 | *** |
| T80%(s) | wo_dynobs | 0.787 | 0.465 | 0.410 | ns |
| T80%(s) | wo_amcl | 1.950 | 0.075 | 0.705 | ns |
| T80%(s) | wo_teb | -7.659 | 0.000 | -3.594 | *** |
| 定位误差(m) | wo_aufe | -9.366 | 0.000 | -4.385 | *** |

## 4. 协同效应验证

- 协同效应值: **-15.2650**
- 结论: **负协同（1+1<2，模块相互抑制）**
- 线性叠加预期覆盖率: 109.05%
- 完整系统实际覆盖率: 93.78%
- 模块数: 7

![协同效应](synergy/synergy_bar.png)

## 5. 参数敏感性分析

### 敏感性排序（降序）

| 排名 | 参数 | 归一化敏感度 | 鲁棒性 | 判定 |
|------|------|------------|--------|------|
| 1 | risk_lambda | 0.1995 | 0.3712 | 中敏感 |
| 2 | dyn_emergency_dist | 0.0523 | 0.0521 | 鲁棒 |
| 3 | amcl_n_particles | 0.0447 | 0.0809 | 鲁棒 |
| 4 | aufe_boost | 0.0132 | 0.0211 | 鲁棒 |

![敏感性曲线](sensitivity/sensitivity_curves.png)

## 6. 失败案例与局限性

- 收集失败案例数: 3
- 假设条目数: 15
- 边界条件数: 18
- 失败分析报告: `E:\puppyfangzhen\experiment_results\failure_analysis\failure_analysis_report.md`
- 局限性讨论: `E:\puppyfangzhen\experiment_results\failure_analysis\limitations_discussion.md`

### 失败案例分类统计

| 失败类型 | 案例数 |
|----------|--------|
| amcl_kidnapped | 1 |
| narrow_passage_stuck | 1 |
| dynamic_collision | 1 |
| dead_zone | 0 |

## 7. 理论分析

### 7.1 AUFE 水填充最优性证明

- 平均近似误差: 0.0000
- 最大近似误差: 0.0000

### 7.2 KLD 收敛性分析

### 7.3 Regret 分析

## 8. 结果文件索引

```
experiment_results/
├── SUMMARY.md                  (本汇总报告)
├── baselines/
│   ├── baselines_results.json
│   ├── baselines_coverage.png
│   └── baselines_comparison.txt
├── ablation/
│   ├── ablation_summary.json
│   └── ablation_report.md
├── synergy/
│   ├── synergy_result.json
│   └── synergy_bar.png
├── sensitivity/
│   └── sensitivity_results.json
├── failure_analysis/
│   ├── failure_cases.json
│   ├── failure_analysis_report.md
│   ├── limitations_analysis.json
│   └── limitations_discussion.md
└── theory/
    └── theory_analysis_summary.json
```

## 9. 复现说明

### 快速演示（模拟数据）
```bash
python run_all_experiments.py --demo
```

### 完整复现（需 CoppeliaSim）
```bash
python run_all_experiments.py
```

### 仅理论分析与统计（跳过仿真）
```bash
python run_all_experiments.py --skip-sim
```

---
*本报告由 run_all_experiments.py 自动生成*
