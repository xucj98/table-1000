# behavior.py

状态：草案。定义资产的附加力与状态转换，由统一加载器按实例运行。

## 声明

在 `physics.json.behaviors` 中按名称声明：

```json
{
  "behaviors": {
    "click_mechanism": {
      "entry": "behavior.py:ClickMechanism",
      "bindings": {"joint": "button_press"},
      "parameters": {
        "press_threshold_m": -0.0028,
        "release_threshold_m": -0.0005
      }
    }
  }
}
```

| 字段 | 含义 |
| --- | --- |
| `entry` | 资产源目录内的 `文件.py:类名` |
| `bindings` | 插件逻辑名称到本资产刚体、关节或接口名的映射 |
| `parameters` | 插件参数，字段由对应插件定义 |

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

## 装配接口

在 `physics.json.interfaces` 中声明，配合律参数由插件定义：

```json
{
  "interfaces": {
    "tip": {
      "body": "body",
      "role": "male",
      "compatible": "marker_18mm",
      "pose": [-0.032, 0, 0, 1, 0, 0, 0]
    },
    "cap_mouth": {
      "body": "cap",
      "role": "female",
      "compatible": "marker_18mm",
      "pose": [0, 0, 0, 1, 0, 0, 0]
    }
  },
  "behaviors": {
    "cap_fit": {
      "entry": "behavior.py:CapFit",
      "bindings": {"interface": "cap_mouth"},
      "parameters": {"static_retention_N": 2, "dynamic_retention_N": 1.4}
    }
  }
}
```

| 字段 | 含义 |
| --- | --- |
| `body` | 所属刚体根名称 |
| `role` | `male` 或 `female` |
| `compatible` | 兼容类别 |
| `pose` | 刚体根局部位姿 `[x, y, z, w, qx, qy, qz]`；+X 为插合轴，对齐时两接口同向 |

配对条件为兼容类别及相对姿态。注册器保证同一仿真环境内一对一配对，脱离后释放；每对只计算一次内力，不限于原资产内的部件。

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

USD 资产根的 `customData.table1000.runtime` 保存字符串 `runtime/manifest.json`，相对来源 USDZ 所在目录解析。加载器自动发现 runtime 并注册物理步回调，场景无需单独绑定脚本。声明的行为无法加载时应报错。

[标准 USDZ](https://openusd.org/release/spec_usdz.html)不包含 Python；交付单位为整个资产目录。Isaac 编辑器需要启用相应的统一加载扩展。
