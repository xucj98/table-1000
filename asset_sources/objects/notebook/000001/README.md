# 深蓝色闭合小笔记本

![深蓝色笔记本：左视觉、右碰撞](three_quarter.jpg)

深蓝色封面、浅色页块和深蓝束带；约 103 × 143 × 21.4 mm，尺寸为照片引导的工程估计。
封面、页块、书脊和完整束带环组成 1 个刚体，无关节；这是闭合收纳模型，不支持翻页、开合或束带变形。
原点在底面中心，+X 指向书口、+Y 沿书脊、+Z 朝上封面，书脊在 −X。

实际模型层级（省略网格层，部件均随整本固定）：

```text
notebook
  closed [rigid]
    cover_bottom
    cover_top
    page_edge_detail
    pages
    spine
    strap
```
