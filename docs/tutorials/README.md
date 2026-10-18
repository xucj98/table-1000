# Table-1000 操作手册

手册介绍从单张图片生成可供仿真使用的资产，以三抽屉盒子、按动笔和带盖笔为例。每章按输入、命令、输出和验收组织。命令在 `table-1000` 仓库根目录执行，资产及其预览、物理测试结果写入已被 Git 忽略的 `assets/`，临时实验可使用 `outputs/`。

| 章节 | 当前内容 |
| --- | --- |
| [01 单图资产建模](01-blender-modeling.md) | 参考照片与资产输出展示、AI 编写 Python、视觉与碰撞建模、关节及独立刚体姿态预览、运动粗测、复杂度及源码留存 |
| [02 物理属性、USD 导出与运动测试](02-physics-assets.md) | Python 生成原生 USD 物理层、官方行为脚本、笔与抽屉柜 CPU/GPU 测试、视频与曲线验收、RTF |
| [03 机器人导入与验收](03-robot-import.md) | 厂商 URDF 包的数据集存档、按误差简化外观、凸块碰撞与覆盖检查、固定关节合并、物理层、机器人描述与控制接口（25 Hz 环境）、CPU/GPU 物理常识用例、单卡多环境检查与对比指标、键盘遥控 |
| [04 场景、专家轨迹与渲染](04-scene.md) | 场景源码与随机初始化、初态的无穿透与静置检查、专家程序（运动原语、几何查询、抓取几何、英文子任务）、批量生成与 Isaac Lab 轨迹文件、成功率与失败分布、按状态重放渲染与评审图、LeRobot v3.0 数据集 |

标准批量构建入口是 [`scripts/assets/build_assets.py`](../../scripts/assets/build_assets.py)，资产批量验收与预览入口是 [`scripts/assets/validate_and_preview.py`](../../scripts/assets/validate_and_preview.py)，实现在 `table_1000/modeling/`，回归测试在对应的 `tests/modeling/`。单资产定义位于 `asset_sources/objects/`；批量构建默认发现全部资产，也可用 `--assets` 指定相对资产目录。资产按[存储布局](../designs/asset-pipeline/README.md#资产布局)分别保存源码、配置和生成产物。第一章实测环境为 Blender 4.5.14 LTS，后续章节逐块编写与复现。
