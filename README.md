# Table-1000

Table-1000 是面向机器人桌面整理任务的 benchmark 与场景资产项目。

## 文档

- [一期实施计划](docs/core/plan.md)
- [研究方案](docs/core/research-proposal.md)
- [技术参考](docs/reference/reference-assessment.md)
- [开发约定](docs/repository-conventions.md)
- [资产存储布局](docs/designs/storage-layout.md)
- [操作手册](docs/tutorials/README.md)

## 开发环境

在仓库根目录执行：

```bash
uv sync --locked
uv run python -c "import table_1000; print(table_1000.__file__)"
```

默认开发环境使用 Python 3.12，包支持 Python 3.11 及以上版本。目前没有运行时第三方依赖；Blender、仿真器和策略使用各自的环境。

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
