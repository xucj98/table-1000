# 白蓝迷你订书机

![preview](three_quarter.jpg)

白色浅底座、浅蓝上盖和金属钉槽，约 70 × 30 × 43 mm（初始打开姿态）。

包括两个刚体组件：基座；上盖、金属钉槽和压针件构成的活动上部。铰链 `upper_press` 沿 +Y 轴，限位约 [−0.299254, 0] rad。零位为最大打开姿态，负值下压至金属钉槽接触底座。

```text
stapler
  base [rigid]
    anvil
    axle
    clinch_groove1..2
    hinge_cheek1..2
    plate
    shell
  upper [rigid]
    bracket1..2
    channel
    cover
    driver
    rear_bridge
```
