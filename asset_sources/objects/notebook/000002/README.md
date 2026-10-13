# 深灰黑大幅闭合笔记本

![深灰黑大幅闭合笔记本：左视觉、右碰撞](three_quarter.jpg)

斑纹封面、浅色内收页块和固定书签。闭合本体约 210 × 297 × 12 mm，整体最大长度约 332 mm。尺寸为工程估计。
封面、页块、书脊和书签组成 1 个完整刚体，0 关节；保持闭合，不支持翻页、开合或附件变形。
原点在本体底面中心，+X 指向书口、+Y 沿书脊、+Z 朝上封面，书脊在 −X。

实际模型层级（省略网格层，部件均随整本固定）：

```text
notebook
  closed [rigid]
    bookmark
    cover_bottom
    cover_top
    page_edge_detail
    pages
    spine
```
