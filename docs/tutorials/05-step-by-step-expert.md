# 05 逐步编写专家

多步任务的专家（如场景 000002：开抽屉、取笔、关抽屉……）一次跑完整个流程时，后面步骤的失败要等前面都成功才看得到，每轮迭代都很慢。逐步编写的做法：先把第 1 步迭代可靠，用它成功的结束状态作为第 2 步的起始状态，再迭代第 2 步，依此类推。每一步的起始状态都来自真实执行，底盘位置、物体被放下的位置都和全流程一致，不需要手写。

前提：专家由步骤组成（`Step` 与 `run_steps`，见[场景与专家轨迹](../designs/asset-pipeline/scenes.md#步骤)）。步骤之间手里不拿东西，每步的 `check` 检查结束状态是否满足下一步的起始条件。

## 1. 迭代第 1 步

```bash
uv run python scripts/scenes/run_expert.py 000002 --physics isaac-cpu --steps 1 --jobs 20 --launcher '<调度命令>'
```

- 先用约 20 个 seed 的初态（`--seeds 0:20`），改专家、重跑，直到成功率约 95%。要保留上一轮的结果作对比时，给 `--output` 另一个根目录。
- 失败按 `summary.json` 的 `failures` 和 `episodes/<seed>.json` 分析；结果在 `data/scenes/000002/isaac-cpu/steps1-1/`（场景批次目录下的 `steps<k>-<m>/`，每次运行替换同名目录；跑完整流程替换 `isaac-cpu/` 时保留这些子目录），录像用 `render.py 000002 --physics isaac-cpu/steps1-1`。
- 达到目标后用 100 个 seed 跑一遍。这一批的 `checkpoints/step1.hdf5` 就是第 2 步的起始状态，约 95 个。

## 2. 下一步从检查点开始

```bash
uv run python scripts/scenes/run_expert.py 000002 --physics isaac-cpu --steps 2 \
    --from data/scenes/000002/isaac-cpu/steps1-1 --jobs 20 --launcher '<调度命令>'
```

- 每个有第 1 步检查点的 seed 都从它开始：恢复状态、静置 0.2 s，只运行第 2 步。
- 迭代第 2 步时只改第 2 步用到的代码，同一份检查点可以反复使用。
- `summary.json` 的 `steps` 给出每步的成功率和逐步完成率（前面各步都成功才计入，分母是第 1 步的 seed 数），可以直接看出整条流程走到哪一步损失最多。

## 3. 什么时候合并步骤

逐步运行时，每一步只看到前一步成功的那部分状态。几步都可靠后（单步约 95% 以上），把它们合起来跑（`--steps 1-3`，或不带 `--steps` 跑完整流程），确认合起来的成功率约等于各步之积。明显偏低时，说明前一步虽然通过了 `check`，留下的状态对下一步不利（如底盘停的位置、物体放下的朝向），应该收紧前一步的 `check` 或修改前一步。

## 4. 改了共享技能后

`table_1000/experts/skills/` 的改动会影响所有步骤和场景。改动后：

1. 已经迭代完的步骤，用原来的检查点各跑一遍，确认成功率没有下降；
2. 场景 000000 跑完整流程（100 个 seed），和 main 比较。

检查点记录了生成它的提交（文件属性 `commit`）。上一步的代码改了以后，它的检查点仍可用于迭代下一步，但最终数据要重新逐步生成或跑完整流程。

## 5. 接成完整轨迹

```bash
uv run python scripts/scenes/join_steps.py data/scenes/000002/isaac-cpu/steps1-1 data/scenes/000002/isaac-cpu/steps2-2
```

同一 seed 在各批的轨迹按顺序接起来，每段截到下一批开始用的检查点；每条轨迹的 `parts` 属性记录各段的来源批次与提交。接好的轨迹默认就是这个场景的批次 `data/scenes/000002/isaac-cpu/`（替换其中原有的轨迹，保留 `steps*/`），和一次跑完整流程得到的批次格式相同、只是生产方式不同，下游不需要区分；用 `render.py 000002 --physics isaac-cpu` 渲染。

## 看失败的案例

```bash
uv run python scripts/experts/failure_cases.py data/scenes/000002/isaac-cpu/steps2-2 outputs/cases/000002 [--seeds 4 44]
```

在 GPU 上运行（例如用 `gq submit`）。对批次里每个失败的 seed，渲染失败子任务开始时和轨迹结束时的场景，视角包括机器人头部相机、场景自带的视角，以及俯视、左、右、前四个视角，结果写到 `outputs/cases/000002/<seed>/`。`case.json` 记录提交、失败原因、该子任务里规划放弃各个选项的原因，以及碰到的物体。

想确认同样的代码重跑是否还会这样失败，可以从前一步的检查点只重跑这些 seed：

```bash
uv run python scripts/scenes/run_expert.py 000002 --physics isaac-cpu --steps 2 \
    --from data/scenes/000002/isaac-cpu/steps1-1 --seeds 4 44 --output outputs/rerun --jobs 2 --launcher '<调度命令>'
```

## 同一检查点重复运行

同一检查点、同一提交重复运行，随机数相同（`ctx.rng` 和 MPlib 的种子由 seed 和起始步骤决定），但 RRTConnect 有规划时限，机器负载不同时路径可能不同，个别 seed 的结果可能不一致。
