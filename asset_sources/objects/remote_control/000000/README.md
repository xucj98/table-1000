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

在仓库根目录、按项目教程配置现有环境后，仅构建和测试本件。构建按现行规则协调；功能和性能使用统一资源入口。下列为已分配资源环境的底层CLI复现方式，现有队列任务勿重复启动：

```bash
uv run python scripts/assets/build_physics.py --assets remote_control/000000 --gpu "$ASSIGNED_GPU"
uv run python scripts/assets/physics_test.py --assets remote_control/000000 --scripts asset_sources/objects --physics isaac-cpu isaac-gpu --gpus "$ASSIGNED_GPU"
# 功能验证后，在协调好的性能窗口测量：
uv run python scripts/assets/physics_test.py --assets remote_control/000000 --scripts asset_sources/objects --physics isaac-cpu isaac-gpu --gpus "$ASSIGNED_GPU" --benchmark
```

## 物理实测

以下为2026-10-20新版执行器的未标定仿真值，来源为`physics_test/summary.json`。CPU为1环境，GPU为4环境；多环境断言逐步采样全部副本，而非只检查视频所在的环境0。

|物理量|CPU|GPU|来源与测量方式|
|---|---|---|---|
|背面起滑载荷|0.233 N|0.2278 N|slide_faces：位移首次超过0.5mm时施加力，环境0|
|按钮面起滑载荷|0.2586 N|0.2611 N|slide_faces：同上|
|侧面起滑载荷|0.2176 N|0.2176 N|slide_faces：同上|
|背面撤力停止距离|32.1 mm|33.95 mm|slide_faces：1.12s撤完至终态X位移，环境0|
|按钮面撤力停止距离|5.503 mm|5.807 mm|slide_faces：同上|
|侧面撤力停止距离|33.81 mm|34.85 mm|slide_faces：同上|
|落放最大几何穿地|0.6072 mm|1.404 mm|drop_orientations：全部实例、全部环境、每物理步碰撞顶点|
|翻转最大几何穿地|0.7948 mm|0.8139 mm|flip_release：同上|
|翻转面法线Z最小值|-0.999 1|-1.0 1|flip_release：全部环境，要求<-0.5|
|最大孔心偏移|1.126 mm|1.075 mm|ring_contact：全部环境，孔心至静态销轴XY距离|
|自由对照件最小位移|241.6 mm|241.2 mm|ring_contact：全部环境，1.6s末态X位移|
|最大线动量误差|6.087e-05 %|0.001666 %|free_eccentric：全部环境，m·vx对施加冲量相对误差|

软件：Isaac Sim5.1.0rc.19，Isaac Lab源码VERSION2.3.2（扩展发行元数据0.54.3；本机可运行但未证明与锁定2.3.2.post1 wheel完全等价），Python3.11.16、Torch2.7.0+cu128。TGS；CPU刚体16/4、CCD开启；GPU场景最小迭代64/16、逐次施加外力开启、CCD关闭，不能当同求解设置比较。落放CPU/GPU均2ms；翻转CPU4ms/GPU2ms，其余4ms。GPU翻转4ms曾瞬时穿地2.423mm，超过原2mm限值；仅细化步长后全部四环境低于2mm，未改变载荷、材料或验收阈值。

GPU接触响应并非副本完全相同：滑移终态最大坐标差约13.95mm，翻转约5.31mm；四环境均分别满足原行程、翻转、稳定和穿地条件。孔约束偏移和自由冲量也逐环境验证。起滑和停止距离受支撑几何、接触求解和步长影响，不是实物摩擦标定。挂环销为初始化前声明、随环境复制的静态接触夹具；没有运行时USD物理写入或增删约束。

## 正式性能测量（2026-10-20）

统一队列01fb321f8e6cd496于21:31:58–21:53:06（UTC+8）完成官方--benchmark，CPU1/GPU4各五项completed。本组实际审查本次10段新视频全部630帧、52曲线面板及挂环原分辨率末帧；以CSV复核168个独立无渲染重复终态，共同记录列最大标量差均为0。每项3轮，每轮slide/drop/flip12.8s、ring11.2s、free10.8s；准备/复位/首次渲染成本单列。

原assets功能证据保留；本次性能和新媒体位于仓库根目录 outputs/asset-physics/coordinator-benchmarks/01fb321f8e6cd496/snapshot/assets/objects/remote_control/000000/physics_test/ 。原始result/trace/plots和MP4另存，不覆盖原功能summary。本组审查位于 outputs/asset-physics/remote_control/benchmark-review-01fb321f8e6cd496/ 。本地结果不随源码自动分发，异机按同版本教程复现。

|后端|测试|dt ms|环境数|三轮无渲染RTF范围|中位数|视频RTF|无渲染总吞吐env·s/s中位数|
|---|---|---|---|---|---|---|---|
|isaac-cpu|drop_orientations|2|1|0.5446–0.5749|0.5686|0.1999|0.5686|
|isaac-cpu|flip_release|4|1|1.3482–1.4682|1.4549|0.2802|1.4549|
|isaac-cpu|free_eccentric|4|1|1.1440–1.2352|1.1545|0.2725|1.1545|
|isaac-cpu|ring_contact|4|1|1.3128–1.3538|1.3129|0.2642|1.3129|
|isaac-cpu|slide_faces|4|1|0.9820–1.0226|0.9822|0.2533|0.9822|
|isaac-gpu|drop_orientations|2|4|0.2318–0.2366|0.2335|0.1389|0.9339|
|isaac-gpu|flip_release|2|4|0.2897–0.3039|0.2934|0.1573|1.1738|
|isaac-gpu|free_eccentric|4|4|0.4613–0.4859|0.4657|0.1906|1.8630|
|isaac-gpu|ring_contact|4|4|0.5536–0.5861|0.5582|0.2152|2.2327|
|isaac-gpu|slide_faces|4|4|0.4258–0.4450|0.4377|0.1920|1.7507|

|后端|测试|进程CPU平均核数（无渲染/视频）|整卡GPU平均%（无渲染/视频）|整机CPU平均%（无渲染/视频）|
|---|---|---|---|---|
|isaac-cpu|drop_orientations|2.99/1.54|5.0/3.3|24.4/29.1|
|isaac-cpu|flip_release|3.38/1.30|2.4/7.2|26.0/21.6|
|isaac-cpu|free_eccentric|2.63/1.25|2.4/4.2|24.8/19.6|
|isaac-cpu|ring_contact|2.95/1.37|4.1/3.0|27.0/32.0|
|isaac-cpu|slide_faces|2.75/1.30|5.1/3.7|24.0/31.7|
|isaac-gpu|drop_orientations|1.37/1.33|22.6/16.8|20.6/21.1|
|isaac-gpu|flip_release|1.46/1.34|26.0/18.6|20.8/21.4|
|isaac-gpu|free_eccentric|1.35/1.42|13.3/16.4|22.7/22.4|
|isaac-gpu|ring_contact|1.43/1.32|21.1/13.6|24.9/20.5|
|isaac-gpu|slide_faces|1.34/1.35|23.0/22.2|20.4/20.5|

EPYC7542、物理GPU2 RTX4090、驱动580.82.07；CLI及采样子进程亲和性CPU32–38/96–102（7物理核及SMT同胞），CPU并非独占。125资源快照load1范围11.88–34.71、子进程最高269线程。报告整卡计算进程数2（含本进程），未证明另一PID身份，不宣称无竞争；整卡GPU/显存含其他进程。双后端串行，按共享主机分配资源条件解释，不标独占或严格空闲性能。

Kit/TBB/PhysX实际32/32/4；OMP/MKL/NUMEXPR环境1，OPENBLAS环境32，但原生BLAS/OpenMP线程池读回均1。RT2、960×480/25FPS、motion blur关、反弹12、directLighting采样2、dome2、TAA8；不是严格1spp/Kit4参考设置。首次额外render成本保留。CPU16/4 CCD开，GPU场景64/16逐次外力且CCD关；落放和GPU翻转2ms不外推4ms同条件性能。软件及未证明锁定wheel等价的限制同前。

审查前当前/冻结108输入哈希全部一致，本轮仅README及交付记录变化，物理测试未改。有效新版生命周期继续复用，无新增GUI或组合验证。本组新媒体及共享条件性能自审通过；队列measured_pending_review，总控独立验收待办。

完整视频、曲线、CSV与报告在`physics_test/isaac-cpu/`、`physics_test/isaac-gpu/`，汇总在`physics_test/summary.json`。本轮已审查完整帧序列及关注曲线。渲染RT2、960×480/25FPS，常规每帧两次不推进物理的渲染；部分CPU首次预热需三次，实际次数见result。新版CPU1/GPU4的停止/复位与卸载重载均通过（最大状态复位误差约6.98e-10）；GPU四份销路径均已核对。无behavior，未进行GUI交互或机器人组合任务验收。
