# Table-1000 操作手册

手册面向使用者：每章说明交付物、要运行的命令和验收方法。资产文件的详细规范在 [docs/designs/asset-pipeline](../designs/asset-pipeline/README.md)，供编写资产的 AI 阅读。

命令都在仓库根目录执行。源码在 `asset_sources/`，构建结果、预览和测试结果写到不进 Git 的 `assets/`，专家轨迹和数据集写到 `data/`，日志和临时实验放在 `outputs/`。

| 章节 | 内容 |
| --- | --- |
| [00 环境准备](00-environment.md) | uv、FFmpeg、Blender 与 Python 环境 |
| [01 单图资产建模](01-blender-modeling.md) | 从照片生成资产的外观、碰撞、刚体和关节，自动检查与预览 |
| [02 物理资产与物理测试](02-physics-assets.md) | 质量、摩擦与驱动，USD 导出，CPU/GPU 物理测试与关键指标 |
| [03 机器人导入与验收](03-robot-import.md) | 从厂商 URDF 包生成机器人，构建、物理用例、并行检查与键盘遥控 |
| [04 场景与专家轨迹](04-scene.md) | 场景与专家程序，三步生产：构建场景（初态）、生成轨迹、渲染录像 |
