# 尖嘴胶水

![preview](three_quarter.jpg)

白色小瓶、黄色标签和尖嘴瓶盖，约 25 × 25 × 95 mm。

整体为一个刚体。

```text
glue
  bottle [rigid]
    base_rim
    container
    label_band
    label_block
    label_print1..3
    label_stripe
    neck
    nozzle_cap
    screw_collar
```

## 场景加载的物理配置

`physics.py` 使用原生 USD/PhysX 为原有几何配置质量、接触材料和求解属性。整件刚体质量 20 g，静/动摩擦 0.45/0.35、恢复系数 0。保留整体刚体结构，不新增开盖、挤胶或涂抹能力。

以上质量、摩擦与阻力均为未实物标定的工程估计；质心和惯量由引擎根据碰撞几何估算。通过统一物理构建入口生成 `object.usdz`，不修改几何或 `metadata.json`。本次只核对构建、刚体、关节和材料属性，**未运行第二章 CPU/GPU 单资产物理测试**；场景静置检查不能替代抓取、滑动或完整物理验收。
