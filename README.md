# Table-1000

Table-1000 是面向机器人桌面整理任务的 benchmark 与场景资产项目。

## 文档

- [一期实施计划](docs/core/plan.md)
- [研究方案](docs/core/research-proposal.md)
- [技术参考](docs/reference/reference-assessment.md)
- [开发约定](docs/repository-conventions.md)
- [资产存储布局](docs/designs/storage-layout.md)
- [资产元数据](docs/designs/asset-metadata.md)

## 目录

```text
docs/                         # 计划、研究方案与设计文档
assets/
  objects/<category>/000000/  # 可复用对象资产
  scenes/scene-000000/        # 场景资产
scripts/                      # 可复用工具与示范
tests/                        # 自动化测试
outputs/                      # 本地生成结果，不提交
```
