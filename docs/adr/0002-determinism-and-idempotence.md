# ADR-0002：`f64`、跨平台确定性与回灌幂等

- 状态：已采纳，回溯记录
- 记录日期：2026-10-13
- 决策范围：浮点行为、JSON 往返、数值回归

## 背景

调用方需要在原生和 WASM 中复现求解结果，评测也需要把行为变化与宿主差异分开。只有容差检查无法发现平台间 1 ULP 的漂移。

另一方面，交互系统会反复回灌已经满足约束的实体。即使每次结果都在容差内，重复投影仍可能改变坐标位模式。跨平台确定性与这种幂等性是两个不同要求。

## 决策

- 核心浮点统一使用 `f64`，保持严格 IEEE 754 运算语义，不采用 fast-math 或未经评估的 FMA 融合。
- 核心的超越函数和相关数学函数统一通过 `libm`；线性代数使用纯 Rust `nalgebra` 的 `libm` 路径，无 BLAS 分发。
- 固定模型组装和数值更新顺序。依赖、编译策略或算法变化均应纳入数值评审。
- 同一求解器版本、同一输入在目标平台应产生相同的数值位模式。parity 文件用十六进制字符串保存期望 `f64` bits。
- 已满足约束的输入保留实体坐标；2D 闭式路径不重复投影，SO(3) 零增量直接保留原 rotation vector。
- JSON 使用 `serde_json` 的 `float_roundtrip`，避免解析往返本身改变有效 `f64` bits。
- 合法的算法改进可以更新 parity 期望；更新必须同时保留容差行为回归与几何正确性证据。

幂等承诺针对回灌实体坐标，不要求首次迭代报告与回灌报告的 `iterations` 等诊断字段相同。几何等价但数值表示不同的输入也不必输出相同 bits。

## 考虑的替代方案

| 方案 | 未采用或延后的原因 |
|---|---|
| 仅用 epsilon 断言 | 无法发现逐位漂移，不能支撑位级承诺 |
| 使用各平台数学库或 BLAS | 引入平台实现和运算顺序差异 |
| 永远冻结已提交的数值 bits | 会阻碍合理算法改进；承诺限定同版本跨平台 |
| 已收敛结果仍重新投影 | 会使重复调用产生微小漂移，破坏回灌行为 |

## 影响与代价

- 需要维护原生架构和 WASM 的同一组 parity 数据，而不能仅在一台机器生成、检查结果。
- 禁止任意改变归约顺序；稀疏化、并行化和分解库升级都必须检查确定性影响。
- 确定性不证明解正确；几何检查和容差测试分别承担正确性验证。
- 当前固定 parity 算例给出具体覆盖，不能视为所有输入和未来平台的普遍证明。
- 极端有限输入可能在运算中溢出；输入与中间值的有限性校验仍是必要边界。

## 实现与验证

- 运算与构建策略：[根 Cargo.toml](../../Cargo.toml)、[so3.rs](../../crates/ansatz-core/src/so3.rs)、[linalg.rs](../../crates/ansatz-core/src/linalg.rs)。
- 幂等路径：[solver.rs](../../crates/ansatz-core/src/solver.rs)。
- 数值位模式：[parity_test.rs](../../crates/ansatz-core/tests/parity_test.rs)、[parity 算例](../../crates/ansatz-core/tests/parity/)。
- 坐标回灌及 JSON 往返：[input_validation_test.rs](../../crates/ansatz-core/tests/input_validation_test.rs)、[regression_test.rs](../../crates/ansatz-core/tests/regression_test.rs)。
- 宿主检查：[WASM 冒烟](../../crates/ansatz-wasm/smoke/node.mjs)、[Node 冒烟](../../crates/ansatz-node/smoke/native.mjs)、[CI 矩阵](../../.github/workflows/ci.yml)。

## 重新评估条件

更换数学库、引入并行或稀疏后端、修改 JSON 编解码、扩展目标架构时重新评估。变更须给出跨平台 parity、回灌幂等和几何行为三类证据，不能以其中一类代替另外两类。
