# 随资产分发的行为插件

状态：第二阶段设计稿。目标是搭场景时只引用资产，不为每支笔手动插入专用脚本。插件由统一加载器执行，标准 USDZ 自身不执行 Python。

## 资产声明

`physics.json` 的可选 `behaviors` 按行为名称声明入口、绑定和参数：

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

`entry` 是源目录内的 Python 文件和类名；`bindings` 把插件的逻辑名称绑定到本资产的刚体、关节或交互接口，具体键由该插件说明。示例阈值由资产行程决定，不能照搬到其他笔。

资产行为描述附加保持力、卡扣状态或按动自锁；不能暗中包含固定笔身、夹持、主动对准或测试拉力。这些外部操作由测试配置明确表达。

## 最小运行接口

```python
class ClickMechanism:
    def __init__(self, context, bindings, parameters): ...
    def reset(self): ...
    def before_step(self, dt): ...
    def after_step(self, dt): ...
    def close(self): ...
```

加载时按资产实例创建独立对象；仿真复位后调用 `reset`，每个物理步前调用 `before_step(dt)`，引擎推进一次后调用 `after_step(dt)`。停止或移除资产时调用 `close`。不按视频帧调用，不偷偷增加子步，也不直接写刚体姿态来伪造受力效果。

`context` 由后端适配器提供，负责读取刚体姿态/速度和关节状态、向刚体施加力/力矩、设置关节驱动，以及查询兼容交互接口。运动状态、力、力矩以仿真世界坐标与 SI 单位交换；局部施力点由适配器变换。每步施力只对当前步有效，持续力必须逐步提交；力施加到指定点时包含相应力矩。

插件拥有自己的状态，不使用模块全局变量共享多实例状态。两部件间的内部作用必须成对施加等大反向力，并按作用点计算力矩。脱离后保持力归零，不能留下隐形约束。跨后端插件只依赖通用 context；必须使用引擎专有能力的插件明确声明后端限制，不宣称自动跨引擎。

首版只实现实际用到的 context 操作。具体 Python 方法签名随实现确认，不提前建设完整控制框架。

## 不同笔与盖子的配对

不能把“原装 body 与 cap”写死为唯一配对。需要互换时，在 `physics.json` 声明装配接口，例如：

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

接口 `pose` 是相对所属刚体根的局部位姿，格式为 `[x, y, z, w, qx, qy, qz]`；与预览中的“相对保存姿态的增量变换”含义不同。接口 +X 定义插合轴，男女接口对齐时轴向同向；实际插入距离、径向偏差、朝向容差和力曲线由插件参数定义，上例仅展示声明结构，不是完整配合律参数集。

加载器注册同一物理场景内的接口，插件根据兼容类别和实际相对姿态请求配对。配对由统一注册器保证一对一，部件脱离后释放；只由一个配对执行一次内力，不能两边重复施力。多个隔离仿真环境之间不能互相配对。三个兼容盖子可以分别套任一支兼容笔，不需要场景逐对指定。

`compatible` 是候选筛选，不证明尺寸一定合适。仍需使用实际几何与标定实验确认间隙、保持力和释放条件。

## 分发与自动加载

```text
assets/objects/pen/000003/
  object.usdz
  physics.json
  physics_test.json
  runtime/
    manifest.json
    behavior.py
```

运行时 manifest 由构建器生成，只保存加载所需的信息；配置和入口路径相对运行时目录解析：

```json
{
  "physics": "../physics.json",
  "modules": {"behavior.py": "behavior.py"},
  "supported_backends": ["isaac"]
}
```

`modules` 把源入口中的文件名映射到实际随包文件；本地依赖一并打包。`supported_backends` 列出实现并验证过的后端，普通无插件资产不需要 runtime 目录。导出的 USD 资产根在 `customData` 的 `table1000` 字典中保存字符串 `runtime: "runtime/manifest.json"`。这不是 USD 包内的资源引用；加载器根据该资产实际来源 USDZ 的所在目录解析这个字符串，不能按宿主场景路径或 USDZ 包内层路径解析。该解析规则须通过搬移加载测试，不依赖构建机器绝对路径。

Table-1000 的统一加载入口引用 USDZ，再发现并加载 runtime，为所有资产实例注册物理步回调。场景只需资产目录/路径及位姿。Isaac 原生编辑器可用统一扩展提供同样的发现与注册机制：扩展安装一次，资产随场景加载；普通 USD 查看器或未启用扩展的仿真器只能看到原生物理属性，不能自动获得 Python 行为。有行为声明而加载器不支持时，应明确拒绝把它作为完整物理资产运行。

标准 [USDZ 规范](https://openusd.org/release/spec_usdz.html)列出的文件类型不含 Python，所以交付单位是整个资产目录，不能只拷走 `object.usdz` 并期待附加行为仍然生效。Isaac 的 [Behavior Scripts](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_modular_scripting.html)是可评估的原生接入方式，不改变“需要运行时支持”的前提。

加载验收应覆盖多实例状态独立、reset 清空状态、资产目录搬移，以及多支兼容笔之间换帽；测试场景不能手工调用某支笔的特殊初始化函数来掩盖自动加载缺失。
