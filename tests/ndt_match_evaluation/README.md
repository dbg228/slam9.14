# NDT 评估合成测试

从工程根目录运行：

```bash
MPLCONFIGDIR=/tmp/test_nav_mpl \
python3 -m unittest -v tests.ndt_match_evaluation.test_core
```

测试覆盖固定 SE(2) 坐标变换、Tracker 到 `base_link` 航向安装偏置、`±π`
航向跨界、位置/四元数时间插值、长时间动捕丢帧拒绝、被拒绝的原始
NDT 结果保留，以及 CSV/SVG/PNG 报告输出。

初始化后静止阶段的统计逻辑单独运行：

```bash
MPLCONFIGDIR=/tmp/test_nav_mpl \
python3 -m unittest -v tests.ndt_match_evaluation.test_post_init_analysis
```

这一组测试检查首次成功且接受的匹配仅作初始化、后续真实匹配尝试按固定次数排除、
被拒绝或失败的尝试不被悄悄跳过、正式段的大更新仍保留，以及候选修正和实际应用更新
分开统计。对齐只使用排除的稳定期；三段位姿共用同一个固定参考，不用正式段重新拟合。
净效果使用同一参考下的 `e_ndt − e_init`，未知数据不能冒充零误差。

实测包分析入口为根目录 `ndt_post_init_analysis.py`，例如：

```bash
python3 ndt_post_init_analysis.py \
  --bag robot_data/experiments/ndt_cold_start/<RUN_ID>/bag \
  --output robot_data/experiments/ndt_cold_start/<RUN_ID>/report_post_init \
  --settling-matches 3 \
  --max-interpolation-gap-seconds 0.10 \
  --tracker-to-base-config robot_data/config/mocap_tracker0.yaml
```

完整定义、报告文件和录包要求见 [使用说明](../../docs/ndt_match_evaluation.md)。
