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
| `outcome` | string | 期望的求解结局；同时隐含退出码语义（converged ⇔ exit 0） |
| `dof_total` / `dof_remaining` | u32 | 期望的自由度计数 |
| `entities` | array | 期望实体终态；`x`/`y` 为 **IEEE 754 位模式的十六进制**（"0x…"），位级比较、非 epsilon——与产品 `tests/parity` 同一约定 |
| `redundant_constraint_ids` | `number[][]` | 期望的冗余/冲突约束组，**集合相等**（多报、漏报都算行为退化） |
| `suggestion_actions` | string[] | 期望出现的建议动作，**子集匹配**（报出更多/更聪明的建议不算失败）。格式 `remove_constraint:2` / `relax_constraint:7` / `add_constraint:1` |
| `tool_error_kind` | string | 期望的工具错误（exit 2 + stderr JSON 的 `kind`）；与 `outcome` 互斥 |

## 设计纪律

1. **位级严格**：数值断言一律用位模式。求解器数值行为的任何改动都会被语料暴露，
   这与产品「跨平台位级一致」的承诺同源。
2. **缺省即通配**：不写的字段不判定。新增诊断能力不会误伤存量 case；
   但已写下的断言只允许「有意地」修改（修 case = 修改地面真值，需在提案中说明理由）。
3. **判定与耗时无关**：评测器记录 `duration_ms` 但不参与 pass/fail——
   计时天然非确定。引入速度进 fitness 是阶段 B 之后的事，届时用多次中位数。
4. 语料本身也要进化（阶段 D：语料共演化），但**只能增改、不能静默漂移**：
   语料目录有内容哈希进谱系记录，变更可追溯。

## 当前 case 一览

| case | 覆盖路径 |
|---|---|
| `case-exact-345` | 闭式解精确有理数路径；欠约束 + 特解 + add_constraint 建议 |
| `case-irrational-normalize` | sqrt/除法非精确路径（位级一致性真检验） |
| `case-conflict-distances` | Inconsistent + 冲突组 + remove/relax 建议 + 冲突点特解 |
| `case-redundant-duplicate` | Overconstrained（冗余）+ 满足解照常返回 |
| `case-negative-distance` | Inconsistent（负距离）+ relax_constraint 建议 |
| `case-empty-model` | Converged（exit 0）空模型 |
| `case-unsupported-coincident` | 工具错误路径（exit 2 + unsupported_constraint） |
