# 三抽屉塑料盒（橡胶脚垫）

![preview](three_quarter.jpg)

乳白色柜壳带三个半透明抽屉和四个黑色橡胶脚垫，约 286 × 285 × 290 mm。包括四个刚体：外壳及三个抽屉。三个 SLIDER 关节限位均为 [0, 0.173] m，正值沿 −Y 拉出，零位闭合；原点为脚垫落地面中心。

```text
cabinet
  drawer1..3 [rigid]
    bottom
    front
    handle
    rear
    side1..2
  housing [rigid]
    back
    rubber_pad1..4
    shelf1..2
    shell
```

## 物理实测

Isaac Sim 5.1.0 / Isaac Lab 2.3.2，4 ms 步长。未标定的仿真值。

| 物理量 | CPU | GPU | 测试与测量方式 |
| --- | --- | --- | --- |
| 抽屉起滑角（drawer1 / 2 / 3） | 18.94° / 18.94° / 18.92° | 16.64° / 16.10° / 15.82° | `gravity_tilt`：夹持外壳，在 15 s 内从水平连续下倾到 45°，取各抽屉相对位移达到 0.5 mm 时的倾角 |

关节轴静摩擦 0.50 N 对应的解析起滑角约 18.6°。GPU 比 CPU 早约 2.5°，原因未定。
