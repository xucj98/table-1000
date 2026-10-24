# 资产抓取与晃动测试

入口为 scripts/experts/eval_asset_grasp.py（下文命令），复用场景的真实物理资产、专家的抓取候选与运动规划、机器人验收的桌子和工作点。每个环境只加载一个待测资产，不加载原场景的障碍物。

## 范围与协议

配置 configs/asset_grasp_000103.py 引用场景 000103 的资产清单，按资产路径去重：55 个实例、49 个资产，其中 39 个桌面可移动资产参与本轮。10 个背景设备或容器逐实例列出排除理由，不能当作通过。其他资产可用同样结构的配置接入；不增加 metadata 字段。

全部直接放在桌面，不使用垫台；薄物体实际抓取失败照常记录。

本轮 39 个目标均为单刚体。带活动部件的资产需要另外定义要抓取的部件及其他刚体的保持条件，不能把本轮检查一个刚体相对 TCP 的结果推广为整套多刚体机构的通过。

- 默认测试中心为世界坐标 (0.50, -0.15) m（X1Pro RM001 验收工作点前方 5 cm），机器人朝 +X。默认每资产 20 个独立 seed，资产原点在测试中心周围 10 × 10 cm 方形内均匀采样，偏航在 [-π, π) 均匀采样；竖直方向选碰撞几何的稳定支撑姿态。这里随机的是桌面朝向，不是任意翻滚。
- 静置 0.5 s，目标抬升 10 cm、实测至少上升 5 cm 才进入晃动；优先垂直向下接近，允许选左右手、调整躯干，不移动底盘。保存采样位姿、静置后位姿、最终握法、候选拒绝原因和规划日志。
- 抓起后参考机器人 grasp_and_shake：末端手腕关节相对抓起位置依次 +0.4、-0.4、+0.4、-0.4、0 rad，每段约 0.5 s，最后保持 0.5 s；若抓起时手腕靠近限位，先把振荡中心移到可执行范围，此过程也计入滑移。不施加外力。
- 在每个控制步读取物体相对实测 TCP 的变换。用所有碰撞顶点在 TCP 坐标中的最大位移检查稳定性，阈值 5 mm；这同时覆盖平移与转动，记录全过程峰值，不能只比较最终位置。另保存物体原点漂移。晃动实际关节行程须达到标称峰峰值的 90%，关节限位或未执行完整晃动都不能计作成功。
- 物理步长沿用公共 0.004 s，控制频率 25 Hz；报告写入实际生效步长。穿透作为诊断记录，不引入高速瞬时穿透的额外验收门槛。

所有资产使用同一组候选参数：指腹采样间距 0.5 mm，最小接触面积和最小有效夹持面积均为 1 mm²，指尖夹持余量为目标厚度的 1/4、最多 2 mm，开口余量为 2 mm，闭合间距筛选容差为 25 mm（包围盒会高估曲面物体的真实接触夹距）；规划允许手指与支撑面最多重叠 2 mm，物理桌面保持原样。近似凸的物体先搜索整物；有明显空洞的组合物体先搜索实体部件与已有碰撞凸块（按包围盒体积从大到小），用定向包围盒保留细杆的真实宽度。候选成本统一增加夹持宽度 × 10 rad/m，优先较窄的夹持方向，减少宽开口边缘握持。不按资产名称指定部件或阈值。部件握住后，抬升和晃动检查整个刚体；未实际抬起时释放并继续搜索下一几何部位，保留各次闭合与抬升记录。候选阈值只用于筛选握法，是否抓稳由真实抬升和晃动决定。

成功率以全部请求的 seed 为分母；抓取失败、晃动失败、设置/执行错误、进程未产出报告分别记录。抓取技能会自行搜索候选和有限重抓，因此这个比率衡量当前技能在随机摆放上的成功率，不是固定候选的一次成功率。by_grasp 按手臂、资产局部接近轴和闭合轴分组，只统计成功抬起后的晃动通过率；具体位姿仍保留在每次 trial 中。保存具体握法用于定位问题，不把规划不可达自动归因于资产几何，也不据一个失败种子直接修改摩擦或质量。

## 运行与结果

仓库根目录：

~~~bash
uv run python scripts/experts/eval_asset_grasp.py   --config configs/asset_grasp_000103.py --trials 20 --jobs 2 --gpus 0 1

# 单资产指定 seed，录像复核；每次必须使用新输出目录
uv run python scripts/experts/eval_asset_grasp.py   --asset objects/soldering_iron/000000 --seeds 0 1 --video --gpus 0

# GPU PhysX 对照
uv run python scripts/experts/eval_asset_grasp.py   --physics isaac-gpu --trials 20 --jobs 2 --gpus 0 1
~~~

进程分发沿用 --jobs、--gpus / --launcher、--retries；只有未写出结果的任务重试。默认输出到 runs/grasp/<scene>/<日期时间>/，每次独立目录；--output 也必须指向不存在的目录。每个资产的 trials/<seed>.json、summary.json 和可选 videos/<seed>.mp4 复制到 assets/<asset>/grasp_test/<robot>/<backend>/<运行>/。完整运行的 conditions.json、summary.json、summary.md、评估器与入口源码快照和工作进程日志保留在 runs/ 中；conditions.json 记录软件版本、实际工作点和协议参数；trial 保存主机、CPU 负载采样、CPU 时间和墙钟时间。失败返回非零退出码，但其结果仍完整保留。

人工检查 `summary.md` 的逐资产抓起/抓稳次数；对失败 seed 查看 `trials/<seed>.json` 的 `target_attempts`、`closure`、`lift` 和 `hold.trace`，对应录像 `videos/<seed>.mp4` 检查是否真正离桌、是否在晃动中滑落。`initial_unreachable` 单列初始未能执行抓取的数量；闭合失败后物体失去可达性的案例归入 `object not held`，原始规划信息仍保留。

## 复用依据

查看了 RoboTwin 的 [Actor 接触点接口](https://github.com/RoboTwin-Platform/RoboTwin/blob/main/envs/utils/actor_utils.py)（contact_points_pose 与 get_contact_point）以及 ManiSkill 的 [PickCube 专家](https://github.com/haosulab/ManiSkill/blob/main/mani_skill/examples/motionplanning/panda/solutions/pick_cube.py)（OBB 推导接近方向、夹持方向、预抓取、闭合和抬升）。本仓库已有同源的几何候选和阻塞式技能，直接复用；本阶段不引入逐物体手工抓点、预烘焙候选排名或仿真固定关节吸附。
