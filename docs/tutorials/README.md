# Table-1000 操作手册

手册介绍从单张图片生成可供仿真使用的资产，以三抽屉盒子、按动笔和带盖笔为例。每章按输入、命令、输出和验收组织。命令在 `table-1000` 仓库根目录执行，资产及其预览、物理测试结果写入已被 Git 忽略的 `assets/`，临时实验可使用 `outputs/`。

| 章节 | 当前内容 |
| --- | --- |
| [01 单图资产建模](01-blender-modeling.md) | 参考照片与资产输出展示、AI 编写 Python、视觉与碰撞建模、关节及独立刚体姿态预览、运动粗测、复杂度及源码留存 |
| [02 物理属性、USD 导出与运动测试](02-physics-assets.md) | 原生 USDA 物理层、官方行为脚本、带盖笔测试、视频与曲线验收、RTF |
| 03 设计场景 | 待发布 |
| 04 编写专家轨迹脚本 | 待发布 |
| 05 渲染专家轨迹视频 | 待发布 |

标准批量构建入口是 [`scripts/assets/build_assets.py`](../../scripts/assets/build_assets.py)，资产验收与预览入口是 [`scripts/assets/validate_and_preview.py`](../../scripts/assets/validate_and_preview.py)，实现在 `table_1000/modeling/`，回归测试在对应的 `tests/modeling/`。单资产定义位于 `asset_sources/objects/`；批量构建默认发现全部资产，也可用 `--assets` 指定相对资产目录。资产按[存储布局](../designs/asset-pipeline/README.md#资产布局)分别保存源码、配置和生成产物。第一章实测环境为 Blender 4.5.14 LTS，后续章节逐块编写与复现。
