# 粉色玻璃花瓶

![四分之三视图：左视觉，右碰撞](three_quarter.jpg)

粉色方形刻面玻璃花瓶 75 × 75 × 105 mm，插一小束粉白花，总高约 230 mm。尺寸为工程估计。碰撞外形中的花束体积远大于实物质量占比，physics.py 把质心显式设在玻璃瓶底部附近。一个完整刚体，无关节；原点为底面中心，+X 最长边、+Z 向上。

```text
flower_vase
  body [rigid]
    bouquet
    vase
```
