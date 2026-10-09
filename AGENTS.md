# Table-1000 agent 入口

开始工作前阅读 [README.md](README.md)、[开发约定](docs/repository-conventions.md)、[资产存储布局](docs/designs/storage-layout.md) 和任务涉及的设计文档。

- 改动通过 PR 提交到 `main`，合并前完成审查和相应验证。
- 本地环境、凭据、机器配置和调试产物不进入公共仓库；生成结果放在已忽略的 `outputs/`。
- 几何、层级、关节和物理属性由资产文件表示；仅在出现明确消费者需求时扩展 `metadata.json`，并同步设计文档和读取方。
