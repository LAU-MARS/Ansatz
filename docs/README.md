# Ansatz 工程文档

本文档集解释当前系统的边界、关键决策及后续演进。先读项目
[README](../README.md) 与 [AGENTS.md](../AGENTS.md)，再按问题进入下表。
文档以中文为主，接口、模块和算法名称保留英文。

## 阅读入口

| 想了解的问题 | 文档 |
|---|---|
| 组件如何分工，模型如何流入求解与诊断，裁判如何验证结果 | [架构与架构图](architecture.md) |
| 为什么采用这些技术和语义，替代方案与代价是什么 | [架构决策记录 ADR](adr/README.md) |
| 接下来先做什么，依赖和验收门是什么 | [演进 roadmap](roadmap.md) |
| 如何运行产品与绑定 | [英文 README](../README.md) / [中文 README](../README.zh-CN.md) |
| 评测器、提案投递箱和谱系目前实际能做什么 | [evolve 实验区](../evolve/README.zh-CN.md) |
| 输入输出字段和允许的数据形状 | [模型 Schema](../schema/model.schema.json) / [报告 Schema](../schema/solve-report.schema.json) |

## 文档基线与状态

本轮文档的代码基线为 `51d18d4`（Ansatz `0.4.0`，evolve `0.2.0`），
记录日期为 **2026-10-13**。后续代码变更应同步更新相关说明。

| 范围 | 基线状态 |
|---|---|
| 产品求解 | 一个 Rust 核心、四个薄壳；3D 装配及基础关节；2D 仅点到原点距离 |
| 数值与诊断 | 局部 SO(3) 增量、尺度均衡的密集 SVD、有限分解预算、直接矛盾证明 |
| 回归与裁判 | 数值反例、位级 parity、完整报告检查及独立几何可行性检查 |
| 演进实验区 | Stage A：评测当前二进制、归档基线、扫描并列出提案 |
| 一般 2D、连续拖拽、稀疏后端、自动提案应用和 CAD 适配 | 计划，详见 roadmap |

“已采纳”描述决策状态；具体能力是否已实现，应查看 ADR 的实施状态和
roadmap 的证据。特别是 Onshape 优先方向已采纳，但适配层尚未实现。
代码基线和现有回归集也不代表任意模型上的正确性证明或全部平台的 CI 结果。

## 目录

```text
docs/
├── README.md           文档导航、基线及维护约定
├── architecture.md     当前组件、求解流程、评测流程与未来扩展边界
├── roadmap.md          能力分期、依赖、指标、验收证据与优先任务
└── adr/
    ├── README.md       决策索引与状态约定
    ├── template.md     新决策记录模板
    └── 000*.md         具体决策、替代方案、后果及实现证据
```

## 事实从哪里来

| 事实 | 首要依据 |
|---|---|
| 支持的模型和字段 | [model.rs](../crates/ansatz-core/src/model.rs) 及导出的 Schema |
| 求解、容差与失败行为 | [solver.rs](../crates/ansatz-core/src/solver.rs)、[assembly.rs](../crates/ansatz-core/src/assembly.rs) 与回归测试 |
| 报告及工具错误 | [report.rs](../crates/ansatz-core/src/report.rs) 与各壳的转换代码 |
| 位级要求 | [构建配置](../Cargo.toml)、[parity 测试](../crates/ansatz-core/tests/parity_test.rs) 与各壳冒烟 |
| 独立几何裁判 | [geometry_check.rs](../evolve/crates/evaluator/src/geometry_check.rs) 与[裁判测试](../evolve/crates/evaluator/tests/geometry_oracle.rs) |
| CI 实际执行的门禁 | [ci.yml](../.github/workflows/ci.yml) |
| 决策理由与允许的取舍 | [ADR](adr/README.md)；已有产品方向的原始依据保留在 AGENTS.md |

若描述与代码不一致，先确认是实现回归还是文档过期，依据测试和 ADR 修正；
不要通过改期望值把失败结果直接变成正确答案。Schema 从 Rust 类型导出，
不能以手工修改 Schema 替代接口变更。

## 修改时同步哪些文档

| 变更 | 需要同步 |
|---|---|
| 新增或改变约束语义、容差、DOF 或诊断结论 | 模型注释、Schema、行为与独立裁判测试、架构；有实质取舍时新增 ADR |
| 改迭代、旋转更新、分解或确定性策略 | 数值回归、跨目标 parity、相关 ADR；先验证几何正确性再刷新位模式 |
| 新增壳或改变错误映射 | 架构图、对应 README、接口类型及跨壳冒烟 |
| 引入状态、图分解或稀疏求解 | 状态/依赖图、相关 ADR、确定性与交互验证；更新 roadmap 的实施状态 |
| 完成 roadmap 里程碑 | 附可复查的测试、语料或基准证据，再更新状态；保留未覆盖的边界 |

ADR 按编号追加。已有决策被替代时保留背景和原决策，用新 ADR 说明替代关系。
历史决策补写应标明“回溯记录”，不要把补写日期当成原始采纳日期。

Mermaid 图与 Markdown 一起存入版本库，可在 GitHub 渲染。修改图时需检查
图语法与节点含义，当前实现与拟议组件应分别标明。

## 开发验证入口

在仓库根目录运行：

```bash
cargo build --workspace
cargo test --workspace
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --manifest-path evolve/Cargo.toml --workspace
```

完整的跨目标构建、Schema 漂移、WASM 冒烟和黄金语料命令以
[CI 配置](../.github/workflows/ci.yml) 为准。HarmonyOS 当前是允许失败的
工具链探测，不等于已有可部署的 NAPI 交付物。

文档变更至少检查相对链接、Mermaid 语法以及 `git diff --check`。
纯文档修改无需重复数值测试；涉及接口或行为时按上表运行相关验证。
