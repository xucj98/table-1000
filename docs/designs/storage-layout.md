# 资产存储布局

Table-1000 自己维护的可复用资产放在 `assets/`：

```text
assets/
  objects/<category>/000000/
    metadata.json
    object.blend
    object.usdz
    preview/
  scenes/scene-000000/
```

对象采用类别加六位数字编号的目录形式，编号从 `000000` 开始；场景编号同样使用六位数字。对象通过 `category/id` 引用，UUID 保存在 [metadata.json](asset-metadata.md) 中。

一个包含多个可动部件的逻辑对象作为一个对象资产保存，部件关系和物理行为由资产文件表示。`.blend` 是创作源，`.usdz` 是后续仿真导出产物，未完成导出的阶段不要求存在 `.usdz`。

外部仿真框架下载的资产与权重遵循其自身目录约定，不与本项目资产混放。可重建中间结果、调试输出和临时视频写入 `outputs/`，不提交；通过验收的可复用对象与预览保存到对应对象目录。
