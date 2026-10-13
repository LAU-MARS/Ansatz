# ADR-0001：单一 Rust 核心、薄壳与 MIT 实现边界

- 状态：已采纳，回溯记录
- 记录日期：2026-10-13
- 决策范围：产品模块边界、可移植性、实现来源

## 背景

Ansatz 要在原生应用、浏览器、Node 和命令行中提供同一组求解行为与诊断。各宿主独立实现数学会扩大数值差异，也使修复、契约维护和回归验证成倍增长。

产品还要求跨平台位级一致，并允许调用方嵌入核心。宿主 I/O、异常机制与链接方式因此不能进入数值算法的分支。

## 决策

- 求解、模型语义、数值运算和诊断统一放在 `ansatz-core`。
- 核心默认以 `#![no_std]` 配合 `alloc` 工作，禁止 `unsafe`；不依赖宿主 I/O 或按平台分叉的求解逻辑。
- `serde` 是可选功能；用于导出 Schema 的 `schema` 功能可引入 `std`，只在宿主构建启用。默认核心的 `no_std` 路径仍须单独检查。
- CLI、C ABI、WASM 和 napi-rs 壳负责类型转换、输入输出、错误映射与宿主边界保护；壳中不实现约束数学。
- `evolve/` 是独立 Cargo workspace，评测和实验依赖不进入产品发布依赖图。
- Ansatz 的产品代码是独立的 MIT 实现。README 中的 SolveSpace、FreeCAD 和 LEDAS 是设计参考，不是代码依赖；新的代码复用和依赖应核对其许可，不能从「参考」推导出复制实现的许可。

当前平台接口已经存在；HarmonyOS 的完整 NAPI 交付仍属于后续工作。CI 的 OHOS `cargo check` 探索任务不等同于完整可用宿主集成。

## 考虑的替代方案

| 方案 | 未采用或延后的原因 |
|---|---|
| 每个宿主维护自己的求解实现 | 修复和诊断难以同步，跨平台确定性难以约束 |
| 核心直接依赖宿主 SDK、线程与 I/O | 损害 `no_std` 和可嵌入性，并把部署行为混入数学行为 |
| 将现有参考求解器作为产品后端 | 当前选择是自有语义与 MIT 独立实现；后端接入需要另行评估接口及许可 |
| 产品与实验区共用 workspace | 会扩大产品构建依赖与发布范围，削弱黑盒评测边界 |

## 影响与代价

- 一次核心修复可传递到全部壳；核心测试、契约和 parity 是共同的验证基础。
- 核心可以分配内存；`no_std` 不等于零分配，也不意味着硬实时或嵌入式内存预算已经满足。
- FFI 的指针操作和 panic 保护留在壳；核心的 `forbid(unsafe_code)` 不约束整个仓库所有宿主代码。
- 新平台要适配构建、链接、调用约定和错误通道，不能借此改写求解规则。
- 共享核心仍需要分别验证壳的 JSON、内存所有权和调用行为。

## 实现与验证

- 核心边界：[lib.rs](../../crates/ansatz-core/src/lib.rs)、[核心 Cargo.toml](../../crates/ansatz-core/Cargo.toml)。
- workspace 与许可：[根 Cargo.toml](../../Cargo.toml)、[MIT LICENSE](../../LICENSE)、[README 参考说明](../../README.md)。
- 外壳：[CLI](../../crates/ansatz-cli/src/main.rs)、[C ABI](../../crates/ansatz-ffi/src/lib.rs)、[WASM](../../crates/ansatz-wasm/src/lib.rs)、[Node](../../crates/ansatz-node/src/lib.rs)。
- 验证：[CI](../../.github/workflows/ci.yml) 包含裸核心 WASM 检查、原生矩阵、WASM 构建和壳冒烟。
- 总览：[架构](../architecture.md)；后续平台交付见 [演进路线](../roadmap.md)。

## 重新评估条件

新增宿主必须要求核心 I/O、线程状态或非 MIT 后端时，先评估独立适配层能否满足需求。只有附带依赖范围、许可、确定性与可移植性证据的新 ADR，才能改变这一边界。
