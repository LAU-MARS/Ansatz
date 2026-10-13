# ADR-0003：JSON 契约与错误边界

- 状态：已采纳，回溯记录
- 记录日期：2026-10-13
- 决策范围：模型、报告、Schema 与宿主 API

## 背景

同一求解器面向人、应用程序和 agent。调用方必须区分「模型未满足约束」与「输入无效或当前能力不支持」，同时保留失败位形的诊断。若只返回成功布尔值或异常，欠约束、冗余和工具损坏容易混为一谈。

Rust、JSON Schema、TypeScript 和多个壳还需要共享稳定的数据形状。

## 决策

- 核心入口返回 `Result<SolveReport, SolveError>`。
- `Ok(report)` 代表得到求解层结论，`outcome` 可以是 `converged`、`inconsistent` 或 `not_converged`；后两者仍返回实体和结构化诊断。
- `Err(error)` 代表工具级错误，包括 `unknown_entity`、`unsupported_constraint` 和 `invalid_model`。JSON 无法解析等宿主错误由壳映射。
- `converged` 只回答返回配置是否满足全部约束；欠约束与冗余分别记录在 `diagnostics`，不改变该结局。
- FFI、WASM 和 Node 使用核心定义的统一信封：`{ok:true,result}` 或 `{ok:false,error:{kind,message}}`。
- 信封的 `ok:true` 表示有有效求解报告，不等于 `outcome=converged`；调用方必须检查 `result.outcome`。
- CLI 直接输出报告：收敛为 stdout + exit 0；求解层未成功为 stdout + exit 1；工具错误为 stderr + exit 2。
- `model.rs` 和 `report.rs` 是结构类型来源；Schema 通过 `schemars` 导出并提交，CI 再生检查漂移。
- TypeScript 类型目前是手写镜像，需要随契约变更同步；不能把 Schema 漂移检查当作 TypeScript 自动同步。

契约完整性与模型语义合法性分层检查。例如 JSON 可以表达某个几何类型，不表示该类型上的全部约束已经实现。`invalid_model` 也用于当前有限性检查及数值分解失败，细节保留在消息中。

## 考虑的替代方案

| 方案 | 未采用或延后的原因 |
|---|---|
| 全部失败都抛异常或退出 2 | 丢失返回位形，把非收敛误当作工具故障 |
| 只提供成功标志 | 不能表达欠约束、冗余、证实矛盾与迭代失败的区别 |
| 每个壳单独定义返回形状 | 容易漂移，提高宿主切换和调用方处理成本 |
| 手写 Schema 作为另一套类型来源 | 与 Rust 类型产生双重事实来源，维护困难 |

## 影响与代价

- 调用方需要同时处理工具错误和求解层结局，而不能只检查信封 `ok` 或进程是否启动。
- 每条残差、分组和建议保留 `human_message`；机器逻辑应使用 id、枚举与数值字段，不解析自然语言。
- 契约扩展必须同步 Rust、Schema、类型声明、壳和评测器，修改期望值要有行为理由。
- 部分类型有 `deny_unknown_fields`；`Suggestion` 因 `flatten` 有例外，评测器对此额外检查。不能声称所有核心反序列化都统一拒绝未知字段。
- 对能力边界的稳定错误比静默忽略未支持约束更利于调用方可靠恢复。

## 实现与验证

- 类型与错误：[model.rs](../../crates/ansatz-core/src/model.rs)、[report.rs](../../crates/ansatz-core/src/report.rs)。
- 契约：[model.schema.json](../../schema/model.schema.json)、[solve-report.schema.json](../../schema/solve-report.schema.json)、[TypeScript 类型](../../crates/ansatz-wasm/types/ansatz.d.ts)。
- CLI 映射与回归：[main.rs](../../crates/ansatz-cli/src/main.rs)、[cli.rs](../../crates/ansatz-cli/tests/cli.rs)。
- 完整报告检查：[geometry_check.rs](../../evolve/crates/evaluator/src/geometry_check.rs)、[geometry_oracle.rs](../../evolve/crates/evaluator/tests/geometry_oracle.rs)。
- Schema 再生门：[CI](../../.github/workflows/ci.yml)；参见 [诊断证据 ADR](0006-evidence-limited-conflict-diagnostics.md)。

## 重新评估条件

增加状态化会话、异步求解、数值失败专用错误或契约版本字段时，必须复查错误边界和兼容性。不得通过改变壳的局部返回形状绕开核心契约。
