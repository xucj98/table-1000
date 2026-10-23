# 红色三层工具柜

![preview](three_quarter.jpg)

红色喷塑钢柜体，三个深灰钢抽屉，镀铬横拉手，四个黑色橡胶脚垫；360 × 240 × 232 mm，三个抽屉开口各高 68 mm。包括四个刚体：柜体及三个抽屉。三个 SLIDER 关节 `tool_chest.drawer1..3.slide` 限位均为 [0, 170 mm]，正值沿 −Y 拉出，零位闭合；原点为脚垫落地面中心，+X 宽度、+Y 进深、+Z 向上。

```text
tool_chest
  drawer1..3 [rigid]
    bottom
    front
    handle
    rear
    side1..2
  housing [rigid]
    back
    bottom
    foot1..4
    shelf1..2
    side1..2
    top
```

## 物理实测

Isaac Sim 5.1.0，CPU PhysX，4 ms 步长，单环境；未标定的仿真值。测试见 `physics_test.py`（通用实现 `table_1000/physics/drawer_tests.py`）。

| 测试 | 结果（CPU） |
| --- | --- |
| `drawer_slide`：固定柜体，0.8 N 拉第一个抽屉后松手，再以 2 N 推拉 | 0.8 N 拉不动（关节静摩擦 1 N），2 N 拉到 170 mm 限位、推回 0 mm |
| `pull_unit`：柜体放在地面上，拉力逐渐加到 8 N 拉底层抽屉 | 抽屉拉到底，柜体只移动 0.3 mm |
| `gravity_tilt`：夹住柜体从水平前倾到 45° | 三个抽屉起滑角约 20.1° |
