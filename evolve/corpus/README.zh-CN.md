[English](README.md) | 简体中文

# 黄金语料（golden corpus）

自进化闭环的**判定地面真值**。每个 case 是一个目录：

```
cases/<case-id>/
├── model.json     # 求解器输入（符合 schema/model.schema.json）
└── expected.json  # 期望（见下；缺省字段 = 通配）
```

## expected.json 字段契约

| 字段 | 类型 | 语义 |
|---|---|---|
| `description` | string | 人类说明，不参与判定 |
| `outcome` | string | 期望的求解结局（`converged` / `inconsistent` / `not_converged`）；同时隐含退出码语义（converged ⇔ exit 0） |
| `dof_total` / `dof_remaining` | u32 | 期望的自由度计数 |
| `entities` | array | 期望实体终态（point2 的 `x`/`y`，rigid3 的 `pose{tx,ty,tz,rx,ry,rz}`），按 `value_tolerance` 比较。只对解唯一的模型写出——欠约束模型的特解随迭代策略合法变化 |
| `joints` | array | 期望关节变量 `{constraint_id, value}`，按 `value_tolerance` 比较 |
| `value_tolerance` | f64 | 实体与关节的绝对容差，缺省 `1e-7` |
| `conflict_constraint_ids` | `number[][]` | 期望的冲突组，**集合相等**；写 `[]` 即断言没有冲突 |
| `redundant_constraint_ids` | `number[][]` | 期望的冗余组，**集合相等**；写 `[]` 即断言没有冗余 |
| `suggestion_actions` | string[] | 期望出现的建议动作，**子集匹配**（报出更多/更聪明的建议不算失败）。格式 `remove_constraint:2` / `relax_constraint:7` / `add_constraint:1` |
| `tool_error_kind` | string | 期望的工具错误（exit 2 + stderr JSON 的 `kind`）；与 `outcome` 互斥 |

## 设计纪律

1. **断言行为，不冻结噪声**：离散事实（结局、自由度、约束组、建议）精确比较，
   连续量按容差比较。更快或更准的迭代策略不会被判为回退；跨平台逐位一致由
   产品仓库的 parity 测试负责。
2. **确定性**：评测器把每个 case 运行两次，stdout / stderr / 退出码必须逐字节相同。
3. **期望缺省即通配**：`expected.json` 中不写的字段不与冻结值比较。但每份求解
   报告仍必须完整满足契约：实体/约束集合准确、数值有限、固定几何不变、结局与
   退出码一致。每份 `converged` 报告都通过独立几何检查，不采信候选自报残差；
   关节变量也由返回几何重新计算。已写下的断言只允许「有意地」修改，并说明理由。
4. **判定与耗时无关**：评测器记录 `duration_ms` 但不参与 pass/fail——
   计时天然非确定。引入速度进 fitness 是阶段 B 之后的事，届时用多次中位数。
5. 语料本身也要进化（阶段 D：语料共演化），但**只能增改、不能静默漂移**：
   语料目录有内容哈希进谱系记录，变更可追溯。

## 当前 case 一览

| case | 覆盖路径 |
|---|---|
| `case-exact-345` | 闭式解精确有理数路径；converged + 剩余 1 DOF + add_constraint 建议 |
| `case-irrational-normalize` | sqrt/除法非精确路径 |
| `case-conflict-distances` | inconsistent + 冲突组 + remove 建议 + 冲突点特解 |
| `case-redundant-duplicate` | converged + 冗余组（冗余与结局正交） |
| `case-negative-distance` | inconsistent（负距离）+ relax_constraint 建议 |
| `case-empty-model` | converged（exit 0）空模型 |
| `case-unsupported-coincident` | 工具错误路径（exit 2 + unsupported_constraint） |
| `case-3d-shaft-converged` | 3D 全约束装配（fixed + coaxial + distance + angle）；终态位姿 |
| `case-3d-mate-underconstrained` | 单 mate 消 3 DOF |
| `case-3d-conflict-distances` | 同锚点距离目标不相容：由目标值直接证明冲突 |
| `case-3d-coaxial-vs-spherical-conflict` | coaxial 与轴外 spherical 求解失败：局部驻点不是无解证明，保守报 `not_converged` |
| `case-3d-anchored-distances` | 同一对刚体、不同锚点的两条距离不构成冲突 |
| `case-3d-same-body-mate` | 两体约束指向同一刚体：工具错误 invalid_model |
| `case-3d-driven-arm` | 三连杆臂：fixed + 3×revolute（双驱动）+ 传动耦合；绝对关节角与终态位姿 |
| `case-3d-huge-drive` | 有限 `drive=1e20`，有界角度折回；子进程超时守住原来的挂死反例 |
| `case-3d-coincident-distance` | 重合初值下雅可比为零，但正距离目标有解 |
| `case-3d-parallel-angle` | 初始平行的参考方向可以求到直角 |
| `case-3d-nonunit-angle` | 方向向量的长度不改变夹角语义 |
| `case-3d-antiparallel-fixed` | 双方固定的反向轴不能满足同轴同向语义 |
| `case-3d-zero-roll-prismatic` | 零滚转、零位移驱动完全锁定移动副 |
| `case-3d-equivalent-rotation` | 旋转向量 `2π` 与恒等姿态应有相同物理自由度 |
| `case-3d-unit-anchor-lock` | 三个非共线球副锚点完全锁定刚体 |
| `case-3d-large-anchor-lock` | 锚点尺度放大到 `1e12`，不能丢失平移秩 |
| `case-2d-already-normalized` | 返回的非精确归一化点回灌后仍满足约束 |
| `case-3d-negative-distance` | 3D 负距离仍属于直接可证明的矛盾 |

数值审查回归有意不冻结欠约束位姿或迭代数。核心回归测试还断言 2D 回灌后的
坐标位模式不变，并比较等价姿态、不同锚点尺度的物理自由度。上述非线性 case
从 `inconsistent` 改为 `not_converged` 是有意的语义修正：雅可比左零空间残差
只说明当前线性化的冲突，不能证明原非线性模型无解。
