# 资产存储布局

Git 保存资产定义，生成或下载的资产保存在已忽略的 `assets/`：

```text
asset_sources/
  objects/<category>/000000/
    object.py
    metadata.json
    preview.json
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

一个包含多个可动部件的逻辑对象作为一个对象资产保存。生成的 `.blend` 保存建模结果，`.usd` 或 `.usdz` 保存仿真导出结果，预览图片和视频保存在 `preview/`。这些文件不提交到 Git；未完成导出的阶段不要求存在 `.usdz`。

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
