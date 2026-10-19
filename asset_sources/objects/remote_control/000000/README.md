# 白色遥控器

![四分之三视图：左视觉，右碰撞](three_quarter.jpg)

白色圆角壳、深色面区、两个圆按钮、指示点和固定金属挂环；遮挡处为素面。

整体95 × 35 × 15 mm，包含末端挂环。 尺寸为工程估计。一个完整刚体，视觉与碰撞随整体移动；无关节。

```text
remote_control
  body [rigid]
    button1..2
    end_loop
    face_panel
    indicator
    shell
```

物理资产为同一刚体，质量60g（壳18g、PCB/按键12g、电池25g、其余5g的未标定工程估计）。质心和惯量由碰撞体积均匀估算，不代表电池真实分布。塑料静/动摩擦0.40/0.30、按钮0.65/0.50、金属挂环0.25/0.18，恢复系数0、平均组合；接触裕量0.05mm、rest offset 0。CPU TGS 16/4、CCD及speculative CCD开启，不加人工阻尼。

`physics.py`通过统一构建入口生成`object.usdz`，可直接原生加载，无behavior依赖。`physics_test.py`覆盖背/按钮面/侧面推移撤力、30mm落放、整件力矩翻转、固定挂环孔与理想静态销接触、质心/挂环点等冲量对照。默认步长4ms，落放为2ms：4ms诊断冲击瞬时穿地2.96mm，超过2mm实验限值，2ms诊断降至0.61mm。测试外力是施加载荷，不是测得接触反力。

按钮无行程，挂环不相对壳体活动；不模拟电子功能、弹性、损坏、电池活动或真实挂钩防脱。挂环孔试验为零重力理想直销夹具，不代表其他资产组合或额定吊挂能力。参数未经实物标定，实验完成须结合视频、曲线与实际运行条件审查。

在仓库根目录、已配置Isaac与Blender环境后，仅构建和测试本件：

```bash
"$ISAAC_PYTHON" scripts/assets/build_physics.py --assets remote_control/000000 --gpu 0
"$ISAAC_PYTHON" scripts/assets/physics_test.py assets/objects/remote_control/000000 --gpu 0
```
