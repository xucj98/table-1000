# 三抽屉塑料盒

![左视觉、右碰撞](three_quarter.jpg)

白色外壳和烟灰半透明抽屉，约 286 × 276 × 284 mm。

外壳与三个空心抽屉共四个 Compound 刚体。凸壳体分段和盒形板件保留抽屉空腔、拉出通道及间隙。

原点为外壳底面中心；+X 为宽度，+Y 指向背面，+Z 向上，抽屉沿 −Y 拉出。

三个原生 `SLIDER`：`drawer1_open`、`drawer2_open`、`drawer3_open`，限位 [0, 0.173] m；`open.mp4` 以 24 FPS 依次拉出并关闭。当前为几何行程预览，质量、摩擦和受力运动在物理阶段设置。

在仓库根目录生成并验收：

```bash
blender -b --python-exit-code 1 -P asset_sources/objects/cabinet/000000/object.py -- --output assets/objects/cabinet/000000
uv run python scripts/assets/validate_and_preview.py assets/objects/cabinet/000000/object.blend --views asset_sources/objects/cabinet/000000/preview.json
```

`three_quarter.jpg` 为 600 × 300 的源目录缩略图，左视觉、右碰撞，随 Git 提交。完整模型、六视图、动画和验收报告保存在生成目录，不提交。资产身份见 `metadata.json`；修改既有资产时保留 UUID。
