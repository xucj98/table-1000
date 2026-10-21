# X1Pro 双臂移动机器人（RM001 转动夹爪）

![preview](three_quarter.jpg)

与 [x1pro](../x1pro/) 是同一台机器人，换装真机当前使用的 rm001_g1_dvt1 夹爪：两指绕夹爪 z 轴转动，而不是平动。底盘、抬升柱、头部、手臂、相机与 x1pro 相同，`robot.urdf` 来自同一个原始包 `ex001_render` 中的 `urdf/ex001_robot_rm001_g1_dvt1.urdf`，关节、限位、质量与惯量保持真机值，对厂商数据的改动与 x1pro 相同（见其 README）。

本目录只保存与 x1pro 不同的部分：

- `meshes/`：夹爪座、两指和腕部相机座 4 个网格及其凸块。`robot.urdf` 中其余网格直接引用 `../x1pro/meshes/`，生成时不重写（`prepare.py` 的 `BASE = 'x1pro'`）。
- `prepare.py`、`physics.py`、`robot.py`、`robot_test.py` 读取 x1pro 的对应文件，只写出不同的值。

## 夹爪

| 关节 | 类型 | 限位 | 说明 |
| --- | --- | --- | --- |
| `{left,right}_arm_gripper_left_joint` | 转动 | [−0.073, 0.872] rad | 正值张开 |
| `{left,right}_arm_gripper_right_joint` | 转动 | [−0.872, 0.073] rad | 负值张开 |

两指转轴在 `*_arm_link6` 前方 0.077 m、夹爪中线两侧各 0.025 m。手指长 0.14 m，指垫点取指尖坐标系 `*_arm_gripper_{left,right}_tip_link`（在手指轴线内侧 0.015 m）。

真机由一个电机角 `*_arm_gripper`（0–1.89 rad）通过 mimic 带动两指，每指角度为 0.5 × 电机角 − 0.073。这里与 x1pro 一样：电机关节设为固定，左指按角度驱动，右指通过 PhysX mimic 跟随（右指 = −左指）。

动作和观测仍然用两指尖的间距（米）：0 对应 −0.072 rad（两指尖相碰，限位 −0.073 rad），上限 0.245 m 对应 0.872 rad。换算见 `robot.py` 的 `FINGER_PIVOT` 和 `ArmSpec.finger_pivot`。两指张开时呈 V 形，夹宽于约 18 mm（0 rad 时两指内侧平行）的物体时，接触在物体顶棱上，而不是像平动夹爪那样贴住两侧。

TCP 在 `*_arm_link6` 前方 0.198 m、两指中间（与手指同在轴线下 0.01 m）。间距不超过 0.1 m 时，指尖在 TCP 前方 0.018–0.020 m，与 x1pro（0.0185 m）相当。

`ready` 姿态与 x1pro 相同，只是躯干升到 0.45 m（x1pro 为 0.40 m）。TCP 比 x1pro 远 0.055 m，躯干在 0.40 m 时，主手臂够不到验收中自上而下抓取的接近位姿（桌面以上约 0.15 m）。

## 物理

与 x1pro 相同，只有手指驱动不同：刚度 100 N·m/rad、阻尼 10 N·m·s/rad，力矩上限取 URDF 的 20 N·m。

- 刚度按 x1pro 的手指增益折算（1500 N/m × 0.14² ≈ 30 N·m/rad）时，夹持晃动中方块和圆柱漂移 6 mm，超过 5 mm 的门槛。
- 阻尼小时，两指以 URDF 速度上限 11.5 rad/s 合拢（指尖 1.6 m/s），撞入细棒 4 mm。阻尼 10 使合拢的时间常数约 0.1 s。
- 阻尼 20 时细棒在晃动中滑脱，阻尼 6、刚度 60 时方块漂移 6.7 mm：夹持用例对这两个值敏感，下面的结果裕量不大。

以上都是仿真取值，未对照真机控制器标定。

## 使用与修改

使用只需本目录和 `../x1pro/`：

```bash
uv run python scripts/robots/build_robot.py x1pro_rm001 --gpu 0
uv run python scripts/robots/check_robot.py x1pro_rm001 --output outputs/robots/x1pro_rm001-check --gpu 0
```

修改时与 x1pro 一样先下载原始包，再重新生成；x1pro 的网格改动后，本目录需要重新生成：

```bash
uv run python scripts/assets/remote.py pull robots/x1pro/raw/ex001_render
uv run python scripts/robots/prepare_robot.py x1pro_rm001
uv run python -m table_1000.robots.preview asset_sources/robots/x1pro_rm001 --joints '{"lift_joint": 0.4, "head_pitch_joint": 0.3, "right_arm_joint2": 0.4, "right_arm_joint3": -1.0, "left_arm_joint2": 0.4, "left_arm_joint3": -1.0, "right_arm_gripper_left_joint": 0.2, "right_arm_gripper_right_joint": -0.2, "left_arm_gripper_left_joint": 0.2, "left_arm_gripper_right_joint": -0.2}'
```

缩略图左侧为外观网格，右侧为碰撞凸块，每个刚体一种颜色。

## 验收结果

- **源码**：目录 0.9 MB（x1pro 8.5 MB）。夹爪 4 个网格的简化误差最大 0.72 mm，其凸块共 6 块（夹爪座 3 块，其余各 1 块），外观越出凸块最多 0.42 mm（左指）。刚体数 22，与 x1pro 相同。
- **构建**：约 15 s；`robot.urdf` 共 80 个碰撞凸块（x1pro 86 个）。
- **CPU 物理用例**：12 个全部通过，主要结果如下。

| 用例 | 结果要点 |
| --- | --- |
| 手臂撞固定块 | 被挡住，最大穿透 0.3 mm |
| 物体落到机身上 | 最大水平速度 1.83 m/s（方块从上臂弹开），撞击穿透 4.7 mm |
| 自碰撞 | 右臂摆向底盘立柱时停下，能退回 |
| 夹持与晃动 | 方块、圆柱、细棒的指尖间距 75、78、16 mm，漂移 2.7、0.4、0.6 mm，穿透不超过 0.9 mm |
| 持物行驶 | 方块漂移 0.1 mm |

对比指标：IK 成功率 100%；自碰撞规划成功率 95–100%，中位耗时约 5.3 s；关节跟踪最大误差 0.4 mrad；每物理步耗时静止 3.9 ms、手臂运动 5.9 ms。

- **GPU 物理用例**：10 个通过，2 个未通过（x1pro 在 GPU 上 12 个全部通过）：
  - 夹持与晃动：圆柱漂移 9.4 mm、细棒漂移 5.1 mm（门槛 5 mm）；
  - 躯干升入隔板：头顶撞到隔板时，头部俯仰关节速度达到 URDF 上限的 2.4 倍。

  GPU 不在当前的验收范围内（见 issue #11）。
