# 洞洞板双抽屉收纳架

![四分之三视图：左视觉，右碰撞](three_quarter.jpg)

白色洞洞板与右侧三层斜格柜、下方平台和抽屉外壳组成一个主体，配透明收纳盒、白色挂钩及两个烟灰透明抽屉。主体约 **320 × 220 × 380 mm**；闭合装配含把手、安装销约 **320 × 233 × 380 mm**。尺寸为工程估计，非实测。

共五个完整刚体组件：`pegboard_cabinet.housing`、`tray`、`hook`、`drawer_left`、`drawer_right`（后四者使用相同前缀）。左右抽屉各通过一个 `SLIDER` 连接主体，行程 **0–170 mm**，正值沿 **−Y** 拉出。收纳盒和钩子以独立直插销组件装配到洞洞板，沿 −Y 拔出；背面安装结构为推定结构。每个抽屉的盒底、四壁、前板和把手属于同一刚体。

原点为主体底面中心，+X 为宽度，+Y 向后，+Z 向上。

实际模型资产树（省略视觉/碰撞网格；部件均随所属刚体运动）：

```text
pegboard_cabinet
  drawer_left [rigid]
    bin
    handle
  drawer_right [rigid]
    bin
    handle
  hook [rigid]
    crook
    mounting
  housing [rigid]
    back_wall
    bottom
    cubby_base_trim
    cubby_left_wall
    cubby_right_wall
    cubby_shelf1..3
    cubby_top
    divider
    left_wall
    pegboard
    platform
    right_wall
  tray [rigid]
    bin
    mounting
```

洞洞板保留左侧 4 × 5 个原八边形通孔，孔口对边距 10 mm、板厚 8 mm。碰撞以孔间连续实体分块，不封闭闲置孔；三层斜格、两个抽屉及收纳盒保持空腔，把手和安装销随各自刚体移动。

本资产提供第一章几何、结构和运动空间预览。附件受力挂接、抽屉限位带动柜体及机器人操作尚未进行动力学验证。源码快照依赖同版本 `table_1000` 包。
