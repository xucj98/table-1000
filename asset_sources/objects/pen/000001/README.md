# 白黑色按动圆珠笔

![左视觉、右碰撞](three_quarter.jpg)

白色笔杆、黑色后壳与长鼻锥，按照片与尺度线索建模，约 162 × 11 × 11 mm。

笔壳、按钮、笔芯共三个 Compound 刚体。黑色后壳属于固定笔壳，端部小按钮单独运动；16 段空心凸碰撞块保留芯通道。

原点在笔杆参考中点下方，杆中心 Z=0.005 m；+X 沿长度指向按钮，+Y 为宽度，+Z 向上，笔尖朝 −X。

两个原生 `SLIDER`：`button_press` 与 `refill_extend`，各限位 [−0.003, 0] m。`press_and_extend.mp4` 以 24 FPS 演示独立指定的按压与伸缩行程；自锁、内部凸轮和弹簧动力学留待下一阶段。

在仓库根目录生成并验收：

```bash
blender -b --python-exit-code 1 -P asset_sources/objects/pen/000001/object.py -- --output assets/objects/pen/000001
uv run python scripts/assets/validate_and_preview.py assets/objects/pen/000001/object.blend --views asset_sources/objects/pen/000001/preview.json
```

`three_quarter.jpg` 为 600 × 300 的源目录缩略图，左视觉、右碰撞，随 Git 提交。完整模型、六视图、动画和验收报告保存在生成目录，不提交。资产身份见 `metadata.json`；修改既有资产时保留 UUID。

生成目录还保存 `asset_builders.py`，与 `object.py`、`preview.json` 和 `metadata.json` 构成可独立重建的源码快照。共用函数在 `table_1000/modeling/asset_builders.py` 维护。
