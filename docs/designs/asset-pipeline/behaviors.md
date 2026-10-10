# behavior.py

状态：草案。定义资产的附加力与状态转换，由统一加载器按实例运行。

行为声明、绑定、参数和装配接口统一见 [physics.json：行为与装配接口](physics.md#行为与装配接口)。

## 生命周期

```python
class Behavior:
    def __init__(self, context, bindings, parameters): ...
    def reset(self): ...
    def before_step(self, dt): ...
    def after_step(self, dt): ...
    def close(self): ...
```

每个实例独立保存状态。复位后调用 `reset`；每个物理步前后分别调用 `before_step`、`after_step`；移除实例时调用 `close`。

`context` 提供状态读取、施力/力矩、关节驱动及兼容接口查询；具体方法签名待实现确定。状态与施力采用世界坐标和 SI 单位，每次施力只在当前物理步有效。

内部作用成对施力并计入作用点力矩，脱离后保持力归零。插件不修改刚体姿态、不增加隐藏子步；测试外力与夹具由 `physics_test.json` 定义。

## 分发与自动加载

构建产物包含 `object.usdz` 和可选 `runtime/`，后者保存行为代码、本地依赖与 `manifest.json`：

```json
{
  "physics": "../physics.json",
  "modules": {"behavior.py": "behavior.py"},
  "supported_backends": ["isaac"]
}
```

`physics` 和 `modules` 的路径相对 manifest；`supported_backends` 列出支持的后端。

USD 资产根的 `customData.table1000.runtime` 保存字符串 `runtime/manifest.json`，相对来源 USDZ 所在目录解析。仿真程序创建一次 `table_1000.physics.runtime.Runtime(world)`，通过 `Runtime.load()` 加载资产；运行时读取该入口、实例化行为并注册 PhysX 物理步回调，场景无需单独导入资产脚本。声明的行为无法加载时应报错。

[标准 USDZ](https://openusd.org/release/spec_usdz.html)不包含 Python；交付单位为整个资产目录。当前已实现 Python 运行时，尚未提供 Isaac/Kit Extension；直接在 Isaac 编辑器打开 USDZ 不会执行行为。
