# 01 单图资产建模与形状验收

本章以三抽屉塑料盒子和黑色圆珠笔为例：**参考图 → AI 编写 Python → Blender 同时生成视觉与碰撞模型 → 静态视图及运动粗测 → 修改源码或交付**。本阶段验证形状、关节范围和复杂度；摩擦、质量分布与受力运动在后续仿真阶段验收。

## 1. 输入图片与要求

**输入：** 下面的单张照片。目标是左上方的三抽屉盒子，以及桌面中下方带银色环的黑色圆珠笔。尺寸和不可见结构由 AI 根据图片与常识推断。

![参考照片](images/reference.jpg)

这张压缩参考图随教程保存，便于直接阅读和复现；生成的资产与预览不进入 Git。

示范 prompt：

```text
根据图片，用 Blender Python 重建三抽屉盒子和黑色圆珠笔，自行推断尺寸。
同时完成视觉与碰撞模型，按刚体划分：外壳、三个空心抽屉各为一个刚体；笔为一个刚体。
碰撞使用基础形状或明确制作的凸块，保留空腔和活动间隙；视觉与碰撞可以不同。
根据常识定义局部坐标系，为抽屉定义滑动关节、零位和限位。
优先使用低面数模型，省略不影响操作的装饰细节；每件示例的视觉三角数不超过 600。
遵循本章入口与验收约定，输出单资产源码、模型、六视图配置和开合动画配置。
```

**输出：** [盒子 object.py](../../asset_sources/objects/cabinet/000000/object.py) 和 [笔 object.py](../../asset_sources/objects/pen/000000/object.py)，以及各自的 `preview.json`。每份脚本只生成一个资产，统一实现 `main(argv=None)` 和 `--output` 参数。

## 2. 资产结构与约束

每个刚体对应一棵明确标识的完整子树，视觉和碰撞几何都有唯一的刚体归属。整体允许树或森林，不要求统一总根或固定层数。

Object 是父子树的节点。Mesh Object 引用一个 Mesh 数据块；Empty 没有网格数据。刚体是附加在 Mesh Object 上的物理设置。本例使用 Blender 的 **Compound**：根是零顶点 Mesh Object，直接子级的碰撞形状共同构成一个刚体，根本身没有额外的实心碰撞几何。

```text
盒子资产坐标（Empty）
├─ 外壳刚体根（Compound）
│  ├─ 视觉 Mesh Object：壳体、背板、隔板等
│  └─ 碰撞 Mesh Object：凸壳体分段、盒形板件等
├─ 上抽屉刚体根（Compound）
│  ├─ 视觉 Mesh Object：前板、侧壁、底板、背板、拉手等
│  └─ 碰撞 Mesh Object：对应板件与拉手
├─ 中抽屉刚体根（Compound）
└─ 下抽屉刚体根（Compound）
三个独立关节对象：分别引用外壳根和对应抽屉根
```

视觉对象不设刚体；碰撞子对象设置 `BOX` 或 `CONVEX_HULL`，关闭渲染显示。Compound 的这些子对象提供形状，不是独立运动的刚体。需要保留空腔时，不能把整只抽屉合并后求一个凸包。[Blender Compound 定义](https://docs.blender.org/api/4.3/bpy_types_enum_items/rigidbody_object_shape_items.html)。

本章统一脚本使用这些约定：

| 信息 | 存放位置 |
| --- | --- |
| 刚体身份 | 根对象 `rigid_body_root = True`，碰撞形状 `COMPOUND` |
| 几何用途 | 子对象 `geometry_role = "visual"` 或 `"collision"` |
| 关节 | 原生 Rigid Body Constraint，`preview_joint = True`；约束对象名作为关节名 |
| 关节零位 | 保存 `.blend` 时的装配姿态；滑动值用米，转动值用弧度 |
| 禁碰关系 | 关节 `disable_collisions`；报告列出被排除的刚体对。本例三个关节均不禁碰 |
| 复杂度预算 | Scene 的 `complexity_budget` JSON，总量上限 |
| 穿透容差 | Scene 的 `penetration_tolerance_m`，本例 0.0002 m |

当前粗测代码支持 **BOX 和显式凸网格**，不自动分解凹网格；其他基础形状会明确报错，需扩展检测与显示后再使用。 **TODO：评估并接入现成碰撞引擎的查询接口，替换自写碰撞检测，统一支持其他基础形状的检测与预览；上述类型限制只是当前实现限制，不是长期资产规范。**碰撞修改器和缩放须应用。几何粗测采用零碰撞裕量；后续目标仿真器的接触参数另行验证。

| 资产 | 原点 | 局部坐标 |
| --- | --- | --- |
| 盒子 | 外壳底面中心 | +X 宽度，+Y 指向背面，+Z 向上；正面和拉出方向为 −Y |
| 笔 | 笔杆中点下方，笔杆中心 Z=0.006 m | +X 沿长度指向按钮，+Y 横向，+Z 向上；笔尖 −X |

预览以带 `asset_id` 的顶层对象定义资产参考系；没有该对象时使用场景坐标系，因此不要求森林增加共同父节点。多棵树须在同一个明确的资产参考系内。

## 3. 执行建模

**输入：**两个生成脚本及旁边的配置。需安装 uv、Blender 与 FFmpeg，并让终端能找到 `uv`、`blender`、`ffmpeg`。本章实测 Blender 4.5.14 LTS。外层 uv 环境没有运行时第三方依赖，负责启动 Blender；建模、检测和渲染在 Blender 自带的 Python 中执行，使用其自带的 bpy、bmesh、mathutils 和 NumPy。Blender 通过源码路径导入本项目，不使用宿主 `.venv`，无需额外 pip 安装；两个 Python 环境独立，不要求同版本。

在 `table-1000` 仓库根目录执行：

```bash
OUT=outputs/blender-modeling
uv sync --locked
blender -b --python-exit-code 1 -P asset_sources/objects/cabinet/000000/object.py -- --output assets/objects/cabinet/000000
blender -b --python-exit-code 1 -P asset_sources/objects/pen/000000/object.py -- --output assets/objects/pen/000000
```

**输出：**每个产物目录包含 `object.blend`，以及源码和配置的副本 `object.py`、`preview.json`、`metadata.json`；日志输出 `MODEL_BUILT`。源码中的材质由 Blender 内置节点和脚本参数构成，不依赖外部纹理。`assets/` 与 `outputs/` 均被 Git 忽略。

统一生成入口是 `main(argv=None)`，`--output` 指定直接存放上述文件的目录。脚本读取并复制同目录的配置，不在代码内重复定义预览。也可用[批量入口](../../scripts/tutorials/build_modeling_demo.py)：

```bash
blender -b --python-exit-code 1 -P scripts/tutorials/build_modeling_demo.py -- --output assets
```

## 4. 检查并渲染

**输入：**生成的模型和 `preview.json`。

使用标准资产入口，一次完成验收和预览：

```bash
for asset_name in cabinet pen; do
  uv run python scripts/assets/validate_and_preview.py "assets/objects/$asset_name/000000/object.blend" \
    --views "asset_sources/objects/$asset_name/000000/preview.json"
done
```

**输出：**每件资产旁边的 `preview/` 中保存 `acceptance.json` 和七张左右对照 JPG。盒子另有展开图 `open_three_quarter.jpg` 和开合视频 `open.mp4`。**左视觉、右碰撞**，同一相机、姿态和画幅；右侧按刚体着色，显示凸块边界。

`validate_and_preview.py` 先按视频 FPS 对每帧摆姿并检查不同刚体间的凸体穿透，再进行渲染；使用分离轴检测，正常接触允许，超过容差即失败。失败报告列出输出名称、帧号、刚体对和碰撞块；失败时退出非零。不运行 Bullet 动力学，也不检查帧间轨迹。

- `--views` 可选：省略时只输出前、右、顶和四分之三视图，使用零位；不会自动读取旁边的配置。
- `--output` 可选：省略时使用模型旁的 `preview/`。
- `--device auto|cpu|gpu`：默认 `auto`，优先使用一个可见 GPU，无可用后端时使用 CPU；`gpu` 要求 GPU 可用。NVIDIA 机器可用 `CUDA_VISIBLE_DEVICES` 指定可见卡。
- `--check-only`：只执行同一套验收并输出 JSON，不渲染。
- 渲染只修改内存场景，不回写 `.blend`；MP4 使用 H.264，临时帧编码后清理。保持 480×480 单侧画幅、16 samples 与完整 FPS 采样，复用渲染缓存；报告包含实际设备与耗时。

性能参考：同一台机器、同一盒子配置（8 张 JPG＋73 帧 MP4），旧 CPU 流程约 265 秒，新流程使用单张 RTX 4090 / OptiX 约 59 秒，约快 4.5 倍；采用 GPU 渲染与降噪、渲染缓存和无压缩临时图片。验收结果、画幅、采样数和帧数一致；不同降噪后端的像素不保证完全相同，耗时随硬件变化。

### preview.json

键为输出文件名；`.jpg` 指定单帧，`.mp4` 指定关键帧和 FPS：

```json
{
  "open_three_quarter.jpg": {
    "camera": [0.82, -0.82, 0.72, 0, 0, 1],
    "joints": {"drawer1_open": 0.173}
  },
  "open.mp4": {
    "fps": 24,
    "frames": [
      {"frame": 0, "camera": [0.82, -0.82, 0.72, 0, 0, 1]},
      {"frame": 24, "joints": {"drawer1_open": 0.173}},
      {"frame": 48, "joints": {"drawer1_open": 0}}
    ]
  }
}
```

每项从关节零位开始。关键帧依次合并：省略的相机沿用上一帧，`joints` 按关节名合并，空 `{}` 不表示复位；复位须显式写 `0`。展开关键帧后线性插值，包含首尾帧；0–48 共 49 帧。未知关节和越界值报错。

`camera` 前三个数为局部系中**从物体指向相机**的方向，后三个数为画面向上的参考方向；两者不能平行。相机正交投影、自动居中。动画使用整段共同画幅，避免开合时缩放跳动。相机六元组也线性插值；路径若出现零方向或方向与向上平行，则报错。

七视图方向为前 −Y、后 +Y、左 −X、右 +X、顶 +Z、底 −Z、四分之三 (+0.82,−0.82,+0.72)。通常画面向上取 +Z；顶视图取 +Y，底视图取 −Y。笔的前视图因此是侧面，左视图看向笔尖。

## 5. 验收与迭代

| 验收项 | 通过要求 |
| --- | --- |
| 刚体组织 | 每个刚体对应一棵明确标识的完整子树，全部几何归属唯一。 |
| 坐标尺度 | 米制，缩放已应用；尺寸合理，局部坐标、零位和装配姿态明确。 |
| 视觉与碰撞 | 外观接近参考图；碰撞覆盖接触表面，保留空腔和活动间隙；凸块闭合且凸。 |
| 运动粗测 | 关节引用、轴与限位正确；按视频 FPS 采样，无非预期穿透；禁碰关系明确。 |
| 复杂度 | 视觉三角数、基础形状数、凸体数，以及凸网格顶点数、面数均报告总和，并满足预算。 |
| 可复现交付 | 源码和配置提交到 Git，模型与验收结果保存到产物目录，统一命令能重建与验收。 |

几何检测不会自动判断单图重建是否准确，也不能证明视觉与碰撞的每个细节匹配；必须对照原图和左右预览人工看图。

复杂度参考 RoboDojo 的 `Assets/Object/RoboDojo/Rigid/box/00000/object.usdz`：文件内所有 Mesh 共 286 个三角形（含碰撞网格），是简单板件模型。本例同样采用低面数板件；盒子保留三套抽屉空腔，外壳每角只用一个斜面；笔圆周使用 16 段，装饰环仅用于视觉，碰撞由笔杆、笔尖、按钮和笔夹组成。RoboDojo 的笔资产复杂度差异很大，不作为面数上限。标记为 `convexDecomposition` 的 USD 网格，其实际凸体数量需要物理引擎处理后统计，不能把一个网格当成一个凸体。

本例实测：

| 总量 | 盒子 | 笔 |
| --- | ---: | ---: |
| 刚体 / 关节 | 4 / 3 | 1 / 0 |
| 视觉三角数 | 412 | 508 |
| 基础碰撞形状数（BOX） | 21 | 2 |
| 凸体数量 | 8 | 3 |
| 凸网格顶点总数 | 64 | 96 |
| 凸网格面总数（未三角化多边形） | 48 | 54 |
| 验收采样数 | 81（8 静态＋73 动画帧） | 7 静态 |

示例预算：盒子视觉三角数 ≤600、碰撞体总数 ≤32、凸顶点/面总数 ≤80/60；笔分别 ≤600、≤5、≤100/60。这是本例预算，其他资产按用途设定。三只抽屉依次打开至 0.173 m、依次关闭，24 FPS；采样中无超过 0.2 mm 容差的穿透。笔单刚体，没有内部独立刚体对需要碰撞检测，主要验收形状与复杂度。

人工查看本地的 `assets/objects/cabinet/000000/preview/open_three_quarter.jpg`、`open.mp4` 和 `assets/objects/pen/000000/preview/three_quarter.jpg`。这些生成预览不随 Git 提交。碰撞与可见外壳使用匹配的圆角分段；必要的内部几何同属一个刚体，允许自身板件相接或重叠。盒子约 0.286×0.285×0.284 m，笔约 0.1485×0.0120×0.0123 m，均为推断尺寸。

不通过时，把具体帧号、视图及报告反馈给 AI，修改源码后重复建模与验收。以下回归测试包含依赖 Blender 和生成资产的集成检查：

```bash
blender -b --python-exit-code 1 -P tests/modeling/test_geometry_checks.py -- \
  --assets-root assets/objects --output "$OUT/tests"
```

## 6. 保存成果

按[资产布局](../designs/storage-layout.md)留存成果，目录下不放 README。生成源码、预览配置和已有 UUID 保存在 `asset_sources/`，随 Git 提交；生成的模型、源码快照和预览在 `assets/`，日志与测试结果在 `outputs/`，不提交。

```bash
git add asset_sources/objects/cabinet/000000 asset_sources/objects/pen/000000
```

修改资产时编辑 `asset_sources/` 下的源码和配置，再重新执行本章命令。也可以从产物目录中的源码快照和同目录配置独立重建到新的本地目录：

```bash
blender -b --python-exit-code 1 -P assets/objects/cabinet/000000/object.py -- --output "$OUT/from-source/objects/cabinet/000000"
blender -b --python-exit-code 1 -P assets/objects/pen/000000/object.py -- --output "$OUT/from-source/objects/pen/000000"
```

本章不验收摩擦、质量分布或受力运动；USD 导出的形状、关节和过滤映射也需在后续章节独立验证。
