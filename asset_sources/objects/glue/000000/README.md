# 尖嘴胶水

![左视觉、右碰撞](three_quarter.jpg)

白色小瓶、黄色标签和尖嘴瓶盖，约 95 × 25 × 25 mm。

一个 Compound 刚体。瓶身、底边、螺口圈和锥形尖嘴各为低分段闭合凸碰撞块；标签为视觉细节。当前整体拾放，瓶盖不拆卸。

原点在瓶身参考中点下方，中心 Z=0.0124 m；+X 指向黄色尖嘴，+Y 为宽度，+Z 向上。

无关节；`preview.json` 提供六视图和四分之三视图。

在仓库根目录生成并验收：

```bash
blender -b --python-exit-code 1 -P asset_sources/objects/glue/000000/object.py -- --output assets/objects/glue/000000
uv run python scripts/assets/validate_and_preview.py assets/objects/glue/000000/object.blend --views asset_sources/objects/glue/000000/preview.json
```

`three_quarter.jpg` 为 600 × 300 的源目录缩略图，左视觉、右碰撞，随 Git 提交。完整模型、六视图、动画和验收报告保存在生成目录，不提交。资产身份见 `metadata.json`；修改既有资产时保留 UUID。

生成目录还保存 `asset_builders.py`，与 `object.py`、`preview.json` 和 `metadata.json` 构成可独立重建的源码快照。共用函数在 `table_1000/modeling/asset_builders.py` 维护。
