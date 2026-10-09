# 资产存储布局

Git 保存资产定义，生成或下载的资产保存在已忽略的 `assets/`：

```text
asset_sources/
  objects/<category>/000000/
    object.py
    metadata.json
    preview.json
    README.md
    three_quarter.jpg
    physics.json
  scenes/scene-000000/
assets/
  objects/<category>/000000/
    object.blend
    object.usdz
    preview/
  scenes/scene-000000/
```

对象采用类别加六位数字编号的目录形式，编号从 `000000` 开始；场景编号同样使用六位数字。对象通过 `category/id` 引用。源目录和产物目录使用相同的相对路径。

## 资产定义与产物

`object.py` 只生成自己的资产；统一执行入口调用其 `main(argv)`，通过 `--output` 指定产物目录。`preview.json` 定义预览，`physics.json` 定义物理属性；尚未做到相应阶段时不要求提供对应配置。

每个源目录的简洁 `README.md` 只引用一张 `three_quarter.jpg`，并简述外观、尺寸和刚体数；多组件资产列出组件名称以及关节类型、限位和运动方向。复现命令、碰撞实现和阶段说明放在教程或设计文档。`three_quarter.jpg` 是约 480 × 240 的四分之三缩略图，左视觉、右碰撞；两者随 Git 保存。缩略图优先保持辨识度，当前 8 张缩略图大小约 4–12 KB（中位数约 5 KB）。完整六视图、动画和验收报告仍属于生成物，保存在 `assets/`，不提交。

一个包含多个可动部件的逻辑对象作为一个对象资产保存。生成的 `.blend` 保存建模结果，`.usd` 或 `.usdz` 保存仿真导出结果，预览图片和视频保存在 `preview/`。这些文件不提交到 Git；未完成导出的阶段不要求存在 `.usdz`。

生成目录保存 `object.py`、预览配置和元数据副本；资产使用共用建模代码时，也保存重建所需的共用模块副本。源码修改在 `asset_sources/` 和 `table_1000/` 进行，再重新生成这些快照。

脚本中定义的材质参数随源码提交。外部纹理、参考图片等资源使用独立的资产存储，源目录保存资源引用及必要的版本、来源和许可信息，不引用私有绝对路径。具体资源存储服务与清单格式在引入外部资源时确定。

日志、测试结果与调试输出写入已忽略的 `outputs/`。外部仿真框架下载的资产与权重遵循其自身目录约定，不与本项目资产混放。

## 资产元数据

每个对象源目录包含最小的 `metadata.json`：

```json
{
  "uuid": "<object-uuid>"
}
```

当前只保留 `uuid`。其他字段仅在出现明确使用方和需求时增加，并同步更新本文与读取方。

`metadata.json` 表示对象身份，不是仿真器格式的加载依赖。几何、层级、关节、接触、材质和物理行为由对应资产文件表示。
