# 蓝色钢珠卡扣粗笔

![preview](three_quarter.jpg)

蓝色连续笔杆带浅色标签和半透明笔帽，约 179 × 20 × 20 mm。笔身、帽壳和三颗内部钢球，共 **5 个物理刚体**。帽沿 −X 拔出；三球通过径向滑动关节连接帽壳，行程 0–0.8 mm。球仅在碰撞预览显示。

原生钢珠弹簧卡扣；参数为未标定工程代理。

```text
body [rigid]
  barrel
  detent_ridge
  label
  neck
  tip
cap [rigid]
  end
  lip
  shell
cap_ball1..3 [rigid]
  ball
```

## 物理实测

Isaac Sim 5.1.0 / Isaac Lab 2.3.2，4 ms 步长。未标定的仿真值。

| 物理量 | CPU | GPU | 测试与测量方式 |
| --- | --- | --- | --- |
| 拔帽力（紧帽） | 2.36 N | 2.73 N | `pull_comparison`：夹持笔帽以 5 mm/s 沿 −X 拉出，取拉力峰值 |
| 拔帽力（松帽，初始开口 3 mm） | 0.007 N | ≈ 0 N | 同上 |
| 合帽力 | 4.79 N | 2.26 N | `close`：0.3 N 限力推到卡扣前停住，然后以 0.6 N/s 提高上限，取盖紧（开口 < 0.2 mm）时的推力 |
| 重力翻转 180°（30°/s）后的状态 | 紧帽保持，松帽滑落 | 紧帽保持，松帽滑落 | `gravity_rotation` |

CPU 与 GPU 的合帽力相差约 2 倍，见 [#15](https://github.com/xucj98/table-1000/issues/15)。
