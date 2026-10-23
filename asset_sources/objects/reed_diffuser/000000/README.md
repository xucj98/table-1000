# 藤条香薰

![四分之三视图：左视觉，右碰撞](three_quarter.jpg)

透明多棱面玻璃瓶配金色瓶口，直径约 70 mm、瓶高 85 mm，六根黑色藤条散开到总高 240 mm。尺寸为工程估计。碰撞外形中的藤条体积远大于实物质量占比，physics.py 把质心显式设在玻璃瓶底部附近。一个完整刚体，无关节；原点为底面中心，+X 最长边、+Z 向上。

```text
reed_diffuser
  body [rigid]
    bottle
    collar
    reeds
```
