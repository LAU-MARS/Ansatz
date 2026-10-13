# ADR-0007：独立几何裁判与 Stage A 黑盒评测

- 状态：已采纳，回溯记录
- 记录日期：2026-10-13
- 决策范围：`evolve/`、回归可信度、自改进实验边界

## 背景

只比较候选报告中的 outcome、DOF 或声称的残差，会让未满足约束的伪报告通过；只冻结某个参考解，又会错误拒绝欠约束模型的其他有效特解。

在让程序自动改进求解器之前，必须先让评测器独立判断返回几何是否成立，并把已实现的手动评测与未来自动闭环分开。

## 决策

- `evolve/` 使用独立 workspace，以 CLI 子进程的 stdin/stdout JSON 调用候选；候选数学代码不加载到评测进程。
- 子进程有超时，崩溃、超时和不符合输出协议都记录为失败。此边界是进程隔离，不是操作系统权限或完整资源沙箱。
- 每个案例重复执行，要求退出码、stdout 和 stderr 可重复；时间记录只供观察，不参与确定性或选优门槛。
- 评测器共享固定仓库的 `Model`、`SolveReport` 等数据类型；几何运算独立实现，不调用候选或核心的 solver、残差、SO(3)、线性代数。
- 所有结局检查完整报告结构、模型成员、id 集合、有限数、固定体、关节条目和诊断自洽性。
- 对 `converged` 独立重建旋转与锚点，逐条验证当前受支持的几何约束并复算关节变量；候选声称零残差或 `expected.json` 未写姿态不能绕过检查。
- 书面期望额外约束 outcome、DOF、冲突和冗余组、建议以及连续值。欠约束模型可以返回任意满足几何和期望契约的特解。
- 空语料为失败；公开黄金语料的变更必须有明确理由，并通过语料 hash 进入谱系。
- 当前仅实现 Stage A：评测已构建求解器、追加 baseline 谱系、扫描并列出手动 patch 投递箱。自动 apply/build/eval/select/land 与 LLM proposer 均未实现。

`ThresholdSelector` trait 实现当前只比较通过率不低于基线。Stage A 命令给出「全语料通过可入库」提示，并不实际评测每个 patch 或执行落地。

## 考虑的替代方案

| 方案 | 未采用或延后的原因 |
|---|---|
| 直接相信候选 residual 和 outcome | 被伪造报告绕过，不能证明几何满足 |
| 用同一核心残差作裁判 | 候选与裁判可能共享同一数学错误 |
| 所有模型必须匹配冻结姿态 | 误拒绝欠约束模型的其他有效解 |
| 立即自动化修改与合并 | 独立裁判、隔离构建和真实模型证据仍需扩充 |

## 影响与代价

- 第二套数学实现增加维护成本；每新增约束必须同步独立裁判与正反案例。
- 类型共享保证契约对齐，但仍是共同依赖。评测候选时必须保持裁判基线固定，不能让候选同时改写验收标准。
- 当前裁判不独立证明任意报告的 DOF、冗余或全局无解；这些结论主要由书面期望和专门测试验证。
- append-only JSONL 当前记录版本、语料 hash、计数与说明；不含候选 commit/patch hash，也无并发写入事务，不构成完整供应链证明。
- 公开语料是回归集，不能证明泛化。生成压力集、冻结留出集和真实客户模型仍是未来工作。
- 将候选输出反喂生成器时，必须有独立 oracle，不能自动把输出视作黄金真值。

## 实现与验证

- 黑盒运行、超时及双跑：[evaluator/lib.rs](../../evolve/crates/evaluator/src/lib.rs)。
- 独立数学与完整契约：[geometry_check.rs](../../evolve/crates/evaluator/src/geometry_check.rs)。
- 伪收敛、漏字段、方向语义及有效替代解：[geometry_oracle.rs](../../evolve/crates/evaluator/tests/geometry_oracle.rs)。
- Stage A 与选择器：[harness/lib.rs](../../evolve/crates/harness/src/lib.rs)、[harness/main.rs](../../evolve/crates/harness/src/main.rs)。
- 语料和谱系：[corpus README](../../evolve/corpus/README.md)、[archive README](../../evolve/archive/README.md)。
- 产品验收：[CI evolve-eval](../../.github/workflows/ci.yml) 要求所有案例通过；后续 Stage B—D 见 [evolve README](../../evolve/README.md) 与 [演进路线](../roadmap.md)。

## 重新评估条件

进入自动 apply/build 或引入 LLM proposer 前，补齐隔离构建、候选身份、逐项非回退门槛与留出验证。新增裁判规则、语料生成或 oracle 替换须独立审查，并以已知错误报告验证裁判不会再次被绕过。
