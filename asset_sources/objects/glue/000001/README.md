# 黄色固体胶

![左视觉、右碰撞](three_quarter.jpg)

黄色圆筒胶棒、白色盖和简化纸质标签，约 107 × 28 × 28 mm。

一个 Compound 刚体。圆筒、肩环、白盖和盖沿各为 16 分段凸碰撞形状；标签与条码只提供视觉线索。当前整体拾放，盖和底旋钮不独立运动。

原点在筒体参考中点下方，中心 Z=0.014 m；+X 指向白盖，+Y 为宽度，+Z 向上。

无关节；`preview.json` 提供六视图和四分之三视图。

在仓库根目录生成并验收：

```bash
blender -b --python-exit-code 1 -P asset_sources/objects/glue/000001/object.py -- --output assets/objects/glue/000001
uv run python scripts/assets/validate_and_preview.py assets/objects/glue/000001/object.blend --views asset_sources/objects/glue/000001/preview.json
```

`three_quarter.jpg` 为 600 × 300 的源目录缩略图，左视觉、右碰撞，随 Git 提交。完整模型、六视图、动画和验收报告保存在生成目录，不提交。资产身份见 `metadata.json`；修改既有资产时保留 UUID。

生成目录还保存 `asset_builders.py`，与 `object.py`、`preview.json` 和 `metadata.json` 构成可独立重建的源码快照。共用函数在 `table_1000/modeling/asset_builders.py` 维护。
