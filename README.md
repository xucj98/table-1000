# Table-1000

Table-1000 是面向机器人桌面整理任务的 benchmark 与场景资产项目。

## 文档

- [一期实施计划](docs/core/plan.md)
- [研究方案](docs/core/research-proposal.md)
- [技术参考](docs/reference/reference-assessment.md)
- [开发约定](docs/repository-conventions.md)
- [资产生产规范](docs/designs/asset-pipeline/README.md)
- [可拆装笔帽方案选型](docs/designs/assembly-method-selection.md)
- [操作手册](docs/tutorials/README.md)

## 开发环境

在仓库根目录执行：

```bash
uv sync --locked
uv run python -c "import table_1000; print(table_1000.__file__)"
```

项目环境使用 Python 3.11（Isaac Sim 5.1 的要求）。核心包没有运行时第三方依赖；按需要安装的部分在 `pyproject.toml` 的可选依赖组中，版本由 `uv.lock` 锁定：

| 组 | 用途 | 主要内容 |
|---|---|---|
| `sim` | 仿真与资产物理测试 | Isaac Sim 5.1.0、Isaac Lab 2.3.2.post1、PyTorch 2.7.0（CUDA 12.8） |
| `robots` | 机器人运动学与规划 | MPlib、toppra、trimesh、rtree |
| `dev` | 资产与机器人开发 | Open3D、CoACD、huggingface_hub、pytest |
| `data` | 导出 LeRobot 数据集 | LeRobot 0.4.4；与前三组冲突，单独装在 `.venv-data` |

```bash
uv sync --locked --extra sim --extra robots --extra dev     # 或只装需要的组
UV_PROJECT_ENVIRONMENT=.venv-data uv sync --locked --extra data
```

LeRobot 要求的包版本（如 `packaging>=24.2`）与 Isaac Lab 冲突，`pyproject.toml` 把 `data` 声明为与其他组互斥，所以不能用 `--all-extras`。

`sim` 组约 19 GB，安装后 `.venv/bin/python` 即仿真解释器。Blender 使用其自带的 Python；策略使用各自的环境。

共享 uv 缓存时可在上述安装命令中添加 `--link-mode symlink`，避免复制环境中的大型依赖。物体构建、物理测试和机器人入口会自动从当前 Python 环境定位 Isaac Sim 资源，无需手动设置资源路径；显式设置的 `ISAAC_PATH`、`EXP_PATH`、`CARB_APP_PATH` 仍优先使用。

需要安装到已有的兼容 Python 环境时，可使用 `uv pip install -e .`。发行包名为 `table-1000`，导入名为 `table_1000`。

## 目录

```text
docs/                         # 计划、研究方案与设计文档
configs/                      # pipeline、场景与测试配置
asset_sources/                # 资产生成源码、配置与资源引用
assets/                       # 本地生成或下载的资产，不提交
table_1000/                   # 可复用代码
scripts/                      # 薄入口与教程示例
tests/                        # 以单元测试为主，目录对应 table_1000/
outputs/                      # 本地生成结果，不提交
pyproject.toml                 # 包定义与依赖
uv.lock                       # 核心环境锁文件
```
