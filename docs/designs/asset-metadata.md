# 资产元数据

## 对象目录

可复用对象放在 `assets/objects/<category>/000000/`。`category` 按对象类型分组，六位数字编号区分该类别下的对象实例。对象通过 `category/id` 引用，UUID 保存在 metadata 中。

## metadata.json

每个对象目录包含最小的 `metadata.json`：

```json
{
  "uuid": "<object-uuid>"
}
```

当前只保留 `uuid`。其他字段仅在出现明确使用方和需求时增加，并同步更新本设计文档与读取方。

## 资产文件关系

`metadata.json` 是对象身份的伴随元数据，不是仿真器格式的加载依赖。几何、层级、关节、接触、材质和物理行为由对应资产文件表示；`.blend` 保存创作源，`.usd` 或 `.usdz` 保存仿真器使用的表示。
