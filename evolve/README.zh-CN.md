[English](README.md) | 简体中文

# evolve/ —— 自进化实验区（RSI 跑道）

> RSI = Recursive Self-Improvement（递归自我改进）。本目录的使命：
> 让「改进 Ansatz 求解器」这件事本身，成为一个可被自动化迭代的闭环。

## 与产品代码的关系

**完全隔离。** evolve/ 是独立 cargo workspace（根 workspace 的 members 只有
`crates/*`），产品的构建、CI、发布永远不带上这里的依赖（未来会有 LLM 客户端）。
反过来，本区对求解器**只做黑盒访问**：

```
候选求解器 ──ansatz-cli 子进程 + 超时 + stdin/stdout JSON──> evaluator
```

这是安全护栏的第一条：**演化产物绝不在评测进程内执行**。超时、崩溃、
资源滥用都只是一个失败的 case 记录，伤不到闭环本身。

## 闭环设计

```
                ┌─────────────────────────────────────────────────┐
                │                  evolve/ 循环                    │
                │                                                 │
   proposals/   │  ┌──────────┐   ┌──────────┐   ┌─────────────┐  │
  （投递箱）────┼─▶│ Proposer │──▶│  apply +  │──▶│  evaluator  │  │
   或 LLM      │  │ (trait)  │   │  build    │   │ （黑盒+超时）│  │
              │  └──────────┘   └──────────┘   └──────┬──────┘  │
                │                                      │         │
                │                       ┌──────────────▼──────┐  │
                │                       │ Selector (threshold)│  │
                │                       └──────────┬──────────┘  │
                │                       通过 ┌─────┴─────┐ 失败  │
                │                           ▼           ▼       │
                │                    archive/       archive/    │
                │                  （入库+落地）  （记录后丢弃）  │
                └─────────────────────────────────────────────────┘
                                     │
                     改进后的求解器反哺语料生成（阶段 D）＝ 递归
```

- **fitness 来源**：黄金语料通过率（`ansatz-eval`）。判定只依赖确定性事实
  （退出码 / outcome / DOF / f64 位模式 / 冗余组 / 建议动作），耗时只记录不判定。
- **单调选择**：候选不得劣于基线——任何「变得更聪明」都不许破坏已有黄金行为。
- **开放归档（DGM 式）**：失败者也进谱系（`archive/lineage.jsonl`，JSONL 追加，
  锚点 `solver_version + corpus_hash`，无时间戳等非确定字段）。

## 目录

```
evolve/
├── Cargo.toml            # 独立 workspace
├── corpus/               # 黄金语料（判定地面真值；格式契约见其 README）
│   └── cases/            #   7 个 case：converged/under/over/inconsistent/
│                         #   负距离/位级非精确路径/工具错误，全覆盖
├── crates/
│   ├── evaluator/        # ansatz-eval：确定性评测器（lib + bin）
│   └── harness/          # ansatz-evolve：Proposer/Selector/谱系 + 手动模式
├── proposals/            # 提案投递箱（*.patch + 可选 *.json 元数据）
└── archive/              # lineage.jsonl 谱系（运行时产物，不进 git）
```

## 使用（阶段 A：手动提案模式）

```bash
# 1. 评测当前求解器 + 记录基线谱系（在 evolve/ 下）
cargo run -p ansatz-evolve                          # 自动探测 ../target 里的 ansatz-cli
cargo run -p ansatz-eval -- --solver ../target/release/ansatz-cli   # 只评测不记谱系

# 2. 提交一个改进：把 unified diff 放进投递箱
#    （人工审阅后 git apply 到产品 workspace）
cp my-improvement.patch proposals/

# 3. 应用后在根 workspace 构建新求解器，再跑 ansatz-evolve：
#    全语料通过 = 提案可入库（谱系追加一条 kind=proposal）
```

## 安全护栏（不可退化的不变量）

1. **黑盒执行**：候选只以子进程 + 超时运行；评测进程不加载任何演化代码。
2. **黄金行为单调性**：语料通过率不得下降；冗余组为集合相等断言（多报/漏报
   都算退化），数值为位模式断言（与产品位级一致性承诺同源）。
3. **产品不变量优先**：根 workspace 的测试全绿（含 parity 位级测试）与
   schema 无漂移，优先级高于任何 fitness 提升——由产品 CI 强制。
4. **地面真值的变更是提案**：修改 expected.json 必须走投递箱并说明理由，
   禁止静默漂移（语料哈希进谱系，可追溯）。

## 路线图

| 阶段 | 内容 | 状态 |
|---|---|---|
| **A** | 手动提案模式：投递箱 + 确定性评测 + 单调选择 + 谱系归档 | ✅ 本骨架 |
| B | 提案自动应用 → 隔离 worktree 构建 → 评测 → 选择 → 入库的全自动循环 | 计划 |
| C | `LlmProposer`：接 LLM 自动生成提案（vendor 无关的 trait 已留好） | 计划 |
| D | 语料共演化：用改进后的求解器生成更难的黄金 case —— **改进者改进自己的考卷**，闭环回流到求解器进化本身 | 计划 |

阶段 D 是本区从「自动化改进」跨入 RSI（递归自我改进）的那一步：
评测基准本身成为被改进对象，且其改进由被它改进出来的求解器驱动。

## 已验证的事实

- 7/7 case 通过（当前 ansatz-cli 0.1.0，语料哈希见谱系）；
- 评测器首跑即捕获 `0.0 × (−3.0) = −0.0` 的位模式差异（IEEE 负零），
  证明位级判定有真实分辨力（已作为黄金行为记入 case-negative-distance）；
- 谱系追加与投递箱扫描实测可用。
