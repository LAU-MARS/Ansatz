# ADR-0006：冲突诊断限定于可证明的矛盾

- 状态：已采纳，回溯记录
- 记录日期：2026-10-13
- 决策范围：`inconsistent`、`not_converged` 与修复建议

## 背景

非线性残差可能在有解模型的退化初值处停滞。两锚点初始重合、目标距离非零就是实例：当前一阶信息不足以给出移动方向，但把刚体移开即可满足约束。

把零雅可比、左零空间残差或迭代失败当作无解证明，会错误建议用户调整合法约束，损害 diagnostics-first 的可信度。

## 决策

- `converged` 表示返回配置满足全部约束的几何容差。
- `inconsistent` 只在当前实现具有直接矛盾证据时返回，`diagnostics.conflicts` 给出相应约束组。
- 当前直接证据覆盖：负距离目标；所有参与刚体都已固定而约束不满足；同一锚点对的距离目标容差区间不相交。
- 同锚点对同时识别交换 a/b 的写法；目标差必须大于两侧容差之和，区间重叠不能认定冲突。
- 全固定证明依赖输入 `fixed` 约束所锁定的位姿。当前组可能只列未满足约束，固定前提需结合模型读取；它不是脱离模型前提的完整无解证书。
- 未满足而没有上述证明，包括非线性驻点、无法接受步和迭代预算耗尽，返回 `not_converged`，保留返回位形与残差。
- 左零空间投影仅说明当前线性化无法通过一阶步消除的分量，不作为非线性全局冲突证明。
- 退化距离初值和方向反向驻点采用确定性有限处理；逃逸失败也不能升级成无解证明。
- 约束调整建议应随证据边界解释，不承诺当前冲突组是最小无解子集，也不承诺单次删除即可修复全局模型。

2D 闭式路径也保留直接目标矛盾与实际返回几何的诊断。负距离属于可证明的求解层矛盾；NaN、无穷、非法容差等属于工具级 `invalid_model`。

## 考虑的替代方案

| 方案 | 未采用或延后的原因 |
|---|---|
| 驻点残差非零即 `inconsistent` | 局部停滞不能证明非线性模型无解 |
| 多初值全部失败后宣布无解 | 失败样本有限，不构成普遍证明 |
| 所有失败统一 `not_converged` | 会丢失已经可证实的矛盾，削弱可解释性 |
| 当前返回最小冲突集合 | 尚未实现通用最小无解子集搜索和证据验证 |

## 影响与代价

- 某些实际无解的模型仍返回 `not_converged`；这是证据不足的保守结论。
- 可证明矛盾与数值无法完成分开处理，调用方可以选择检查约束或调整初值。
- `conflicts` 的含义更严格；新增证明规则必须具有几何理由和反例覆盖。
- DOF 和冗余仍有局部一阶边界，不能作为通用矛盾证明的替代来源。
- 现有建议提供可操作方向；经过复求解验证的修复建议是后续能力。

## 实现与验证

- 3D 直接证明与分流：[assembly.rs](../../crates/ansatz-core/src/assembly.rs)。
- 2D 目标区间及闭式结果：[solver.rs](../../crates/ansatz-core/src/solver.rs)。
- 局部投影说明：[linalg.rs](../../crates/ansatz-core/src/linalg.rs)、[outcome 类型](../../crates/ansatz-core/src/model.rs)。
- 非线性停滞、全固定反向轴和同锚点矛盾：[regression_test.rs](../../crates/ansatz-core/tests/regression_test.rs)。
- 容差区间重叠：[input_validation_test.rs](../../crates/ansatz-core/tests/input_validation_test.rs)。
- 黑盒回归：[非线性驻点案例](../../evolve/corpus/cases/case-3d-coaxial-vs-spherical-conflict/)、[同锚点矛盾案例](../../evolve/corpus/cases/case-3d-conflict-distances/)；参见 [局部秩 ADR](0005-bounded-dense-svd-and-local-rank.md)。

## 重新评估条件

新增冲突证明、最小冲突组或自动修复时，必须同时给出已知可行反例与已知矛盾案例，明确证明适用条件。任何数值搜索失败都不能在没有独立证据的情况下改名为 `inconsistent`。
