# Ansatz

[English](README.md) | 简体中文

[![CI](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml/badge.svg)](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/ansatz-wasm?logo=npm&label=ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![npm downloads](https://img.shields.io/npm/dm/ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> *Ansatz*，数学名词：先设一个带待定参数的试探解，再由约束条件把它定下来——这正是几何约束求解器做的事。

一个可移植的几何约束求解器，面向 2D 草图与 3D 装配。单一 Rust 核心，原生运行于 macOS / Linux / Windows / HarmonyOS，通过 WASM 支持浏览器与 Node，以 JSON 输入输出，对 Agent 友好。

**结构化诊断是一等公民**：不只回答「解出来了没有」，还要回答「为什么没解出来」——自由度计数、冗余/冲突约束组、每条约束的残差、机器可读的修复建议，且每条诊断都带同时面向人类与 LLM 的 `human_message`。

服务两类问题：

- **2D 草图约束**：重合、共线、平行、垂直、相切、距离、角度、对称、固定
- **3D 装配约束**：6-DOF 刚体配合——贴合、同轴、距离、角度

## 当前阶段：骨架（端到端管线已通，算法留白）

⚠️ 本仓库处于**阶段 0：骨架与平台可行性验证**。`ansatz-core::solve` 只实现一个平凡算例（**2D 点 + 到原点的距离约束**，闭式解 `p' = p·(d/‖p‖)`），其余约束类型返回 `unsupported_constraint` 工具错误。数据模型、诊断结构、四个壳、schema 契约、CI（含位级一致性测试框架）均已就位并验证；通用数值求解（Newton/LM、稀疏雅可比、秩分析）属于后续阶段，届时只替换 `ansatz-core/src/solver.rs`。

## 架构

```
                ┌──────────────────────────────────────────┐
                │              ansatz-core                 │
                │   纯算法 · no_std · f64 · 诊断一等公民     │
                │   （唯一允许出现数学的地方）                │
                └───────┬──────────┬──────────┬────────────┘
                        │          │          │
        ┌───────────────┤          │          ├───────────────┐
        ▼               ▼          ▼          ▼               ▼
   ansatz-ffi      ansatz-wasm           ansatz-node     ansatz-cli
   C ABI           wasm-bindgen          napi-rs         stdin→stdout
   cdylib+staticlib 浏览器/Node           Node 原生        诊断 JSON
   (HarmonyOS      （单线程，无 SAB/      （与 wasm 壳     + 退出码语义
    NAPI 及任意     COOP-COEP 依赖）       同名同签名）
    C 宿主)
                        │
                        ▼
                schema/ JSON Schema —— Rust 类型导出的对外契约（CI 防漂移）
```

所有外壳**只做类型转换与错误映射**，不含任何数学逻辑；ffi / wasm / node 共享 core 里定义的统一信封 `{ok:true,result} | {ok:false,error:{kind,message}}`。

## Workspace 布局

```
ansatz/
├── Cargo.toml            # workspace 根 + [profile.release]（位级可复现性，见注释）
├── crates/
│   ├── ansatz-core/      # 纯算法，no_std 友好，不依赖任何宿主相关 crate
│   ├── ansatz-ffi/       # C ABI（cdylib + staticlib），cbindgen 生成 include/ansatz.h
│   ├── ansatz-wasm/      # wasm-bindgen，产出带 .d.ts 的 npm 包；smoke/ 含 Node 脚本与单文件 HTML
│   ├── ansatz-node/      # napi-rs，Node 原生路径
│   └── ansatz-cli/       # stdin JSON → stdout JSON
├── schema/               # model / solve-report 的 JSON Schema 契约（CI 校验漂移）
└── evolve/               # 自进化实验区（RSI 跑道）：独立 workspace，与产品隔离
```

## 快速开始

```bash
cargo build --workspace
cargo test --workspace        # 21 个测试，含位级一致性断言

# CLI：3-4-5 三角形缩放，(3,4) 施加到原点距离 10 → (6,8)
echo '{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
       "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}' \
  | cargo run -p ansatz-cli -- solve
```

输出（欠约束但给出特解 + 完整诊断）：

```json
{
  "outcome": "underconstrained",
  "entities": [{ "id": 1, "geometry": { "type": "point2", "x": 6.0, "y": 8.0 } }],
  "diagnostics": {
    "dof_total": 2,
    "dof_remaining": 1,
    "redundant_constraints": [],
    "residuals": [{ "constraint_id": 1, "residual": 0.0, "human_message": "约束 1 在返回的特解处残差为 0e0（即 ‖p‖ − 1e1）。" }],
    "max_residual": { "constraint_id": 1, "residual": 0.0, "human_message": "…" },
    "suggestions": [{
      "action": "add_constraint",
      "dof_remaining": 1,
      "human_message": "模型还有 1 个自由度未被约束。以「点到原点的距离」为例：它只固定了模长，点仍可沿圆周滑动，需要再施加角度或坐标类约束。"
    }],
    "jacobian_condition_estimate": 1.0
  }
}
```

### CLI 退出码（「工具错误」与「求解失败」严格分离）

| 退出码 | 含义 | 输出 |
|---|---|---|
| 0 | `converged` | 报告 JSON → stdout |
| 1 | 求解层失败（inconsistent / overconstrained / underconstrained / max_iterations），完整诊断照常 | 报告 JSON → stdout |
| 2 | 工具错误（非法 JSON / 模型无效 / 能力未实现） | 错误 JSON `{kind, message}` → stderr |

## npm 安装（ansatz-wasm）

wasm 壳发布为 npm 包（wasm-pack `--target nodejs`，单线程、无 SAB/COOP-COEP 依赖）：

```bash
npm install ansatz-wasm
```

```js
const { solve, solveJson, version } = require('ansatz-wasm')
// ESM 同样可用：import { solve } from 'ansatz-wasm'

const env = solve(model)   // { ok, result | error }，永不 throw
```

包内含手写模型类型 `types/ansatz.d.ts`（`AnsatzModel` / `SolveReport` / `Diagnostics` 等，与 schema 契约一一对应）。发布流程：推送 `v*` 标签触发 [npm-publish.yml](.github/workflows/npm-publish.yml)——CI 构建并整理包（`scripts/npm-prepare.mjs`，tag 必须与版本号一致），**wasm 位级一致性冒烟通过后才发布**；本地预演用 `node scripts/npm-prepare.mjs && npm pack`。

## 各壳 API

ffi（`#include "crates/ansatz-ffi/include/ansatz.h"`，随仓库提交）、wasm（`import { solve, solveJson, version }`）、node（同名同签名，可无痛切换）三者的字符串协议完全一致：

```c
// C：返回 JSON 信封字符串（归宿主所有），永远不返回非法 JSON
char* ansatz_solve_json(const char* model_json);
void  ansatz_string_free(char* s);
const char* ansatz_version(void);   // 静态字符串，禁止 free
```

```js
// wasm / node（同名同签名）
const env = solve(modelObject);          // 结构化：{ ok, result | error }，永不 throw
const s   = solveJson(modelJsonString);  // 字符串：与 FFI 完全同形
const v   = version();
```

- 所有 `extern "C"` 函数内部 `catch_unwind` 包裹，panic 绝不穿越 FFI 边界。
- wasm 壳**单线程**，不依赖 SharedArrayBuffer / COOP-COEP，`file://` 页面可直接加载（见 `crates/ansatz-wasm/smoke/web/index.html` 单文件冒烟页）。
- 冒烟测试：`node crates/ansatz-wasm/smoke/node.mjs`（wasm）、`node crates/ansatz-node/smoke/native.mjs`（node 原生）、`python crates/ansatz-ffi/smoke/ffi_smoke.py`（ctypes，本地用）。

## 数据契约（schema/）

`schema/model.schema.json`（输入）与 `schema/solve-report.schema.json`（输出，含诊断）由 Rust 类型经 schemars 导出，**手改无效**——CI 重新导出并与已提交文件 diff，漂移即失败：

```bash
cargo run -p ansatz-cli -- export-schema model
cargo run -p ansatz-cli -- export-schema solve-report
```

## 结构化诊断

求解失败不是异常，而是携带完整证据的报告：

- `dof_total / dof_remaining`：自由度计数（骨架阶段为通用计数近似）
- `redundant_constraints`：线性相关的约束组（互相重复或互相冲突，id 集合）
- `residuals` / `max_residual`：每条约束在返回解处的残差
- `suggestions`：机器可读动作（`remove_constraint{id}` / `relax_constraint{id}` / `add_constraint{dof_remaining}`）
- `jacobian_condition_estimate`：雅可比条件数估计（骨架阶段单条归一化约束行精确为 1.0；退化/秩亏为 null）

## 数值可复现性（跨平台位级一致）

同一输入在 x86-64 / aarch64 / wasm32 上**逐位相同**，这是硬承诺：

1. 严格 IEEE 754 语义：不启用任何等价 `-ffast-math` 的选项，不手写 FMA 融合（依据与锁死方式见根 `Cargo.toml` 的 `[profile.release]` 注释）；
2. 超越函数只经 `libm`（纯 Rust，所有目标编译同一份代码，正确舍入是构造性的）；
3. **位模式测试框架**：`crates/ansatz-core/tests/parity/` 存放固定算例与期望值（十六进制位模式，非 epsilon 比较）。同一份期望文件被三处断言：
   - Rust 测试（CI 在 linux-x86_64 / macos-aarch64 / windows-msvc 矩阵上原位运行）
   - wasm Node 冒烟（`crates/ansatz-wasm/smoke/node.mjs`）
   - node 原生冒烟（`crates/ansatz-node/smoke/native.mjs`）

   `case2`（点 (1,1) 归一化）走真实 sqrt/除法舍入路径，是跨架构一致性的真金火检验；当前已在 x86-64（Rust 原生 + Python 参照 + wasm + node）上逐位一致。

## 3D 刚体位姿参数化

采用**指数映射（旋转向量）** `v = θ·û` 而非四元数：3 参数 ↔ 3 自由度的最小参数化，求解时无需额外的单位范数约束，雅可比无冗余列（四元数的 `JᵀJ` 必然奇异）；`θ→0` 连续、无双重覆盖、无万向锁。`Rotation3` 是参数化的唯一入口，未来切换表示只动它一处——理由与切换抽象点详见 `ansatz-core/src/model.rs`。

## CI

`.github/workflows/ci.yml`：fmt / clippy `-D warnings` / test；构建矩阵 x86_64-linux、aarch64-darwin、x86_64-windows-msvc、wasm32（web + nodejs 双 target）；schema 漂移检查；evolve 黄金语料回归；以及 aarch64-unknown-linux-ohos（HarmonyOS，tier 3）的 `continue-on-error` 探路 job——tier 3 无预编译 std，需 nightly `-Z build-std`，链接需 OHOS NDK，因此只做 check 且不阻塞流水线。

## 自进化实验区（evolve/）

[`evolve/`](evolve/README.zh-CN.md) 是独立的 cargo workspace，使命是让「改进求解器」本身成为可自动化迭代的闭环（RSI：递归自我改进）。当前为**阶段 A（手动提案模式）**，已就位并实测：

- **黄金语料**（`evolve/corpus/`）：7 个 case 覆盖全部结局路径，数值断言用位模式——与产品位级一致性承诺同源；
- **确定性评测器**（`ansatz-eval`）：候选求解器只以子进程 + 超时黑盒运行，fitness 只依赖确定性事实（首跑即捕获 `0.0×(−3.0)=−0.0` 的负零位差异）；
- **循环骨架**（`ansatz-evolve`）：`proposals/` 投递箱 + `Proposer`/`Selector` trait（vendor 无关，为 LLM 提案者预留）+ JSONL 谱系归档；
- 产品 CI 的 `evolve-eval` job 强制求解器对黄金语料 7/7 全过——求解器行为变化必须「有意地」同步语料。

路线：A 手动提案 → B 自动 apply/build/eval 闭环 → C LLM 提案者 → D 语料共演化（改进者改进自己的考卷，闭环回流——真正跨入 RSI）。详见 `evolve/README.md`。

## 路线图

1. **阶段 0（本仓库）**：骨架——管线、诊断框架、契约、CI、位级一致性设施全通
2. 阶段 1：通用数值核心（阻尼 Newton / Levenberg–Marquardt + 稀疏雅可比）
3. 阶段 2：真实秩分析（DOF / 冗余 / 冲突的判定不再靠计数近似）
4. 阶段 3：2D 草图约束全集 → 3D 装配约束（贴合/同轴/距离/角度）
5. 阶段 4：HarmonyOS NAPI 交付物（基于 ansatz-ffi + OHOS NDK）

并行轨道（evolve/）：A 手动提案（已就位）→ B 自动闭环 → C LLM 提案者 → D 语料共演化（RSI）。

## 致谢

Ansatz 站在以下项目的肩膀上，它们的设计塑造了我们的设计，我们谨致谢忱：

- **[SolveSpace](https://github.com/solvespace/solvespace)**（C++，GPLv3）—— 开源几何约束求解器事实上的标杆。它证明了：单一求解器既能服务 2D 草图，也能支持原生 3D 装配约束；对非线性最小二乘残差采用 Dogleg 信任域法求解，速度足以支撑交互式使用；欠约束的草图也能通过拖拽保持自然可操作。它以可复用库的形式发布（`libslvs`），并有 WASM 移植（Blender 的 *Geometry Sketcher* 插件内嵌的正是它）—— 这直接启发了我们「单一核心、随处嵌入」的架构。

- **[FreeCAD](https://github.com/FreeCAD/FreeCAD) 的 planegcs / Sketcher 求解器**（C++，LGPL）—— FreeCAD Sketcher 工作台背后的引擎。我们从它学到了可切换求解策略（DogLeg / Levenberg–Marquardt / BFGS）的价值、自由度（DOF）分析的价值，以及将约束图分解为可独立求解子块的价值 —— 最重要的是：冗余与冲突约束必须被**诊断并解释**，而不是仅仅报一个求解失败。这正是我们对自己坚持的体验标准。

- **[LEDAS LGS 2D/3D](https://ledas.com/)**（商业软件）—— 开源求解器衡量自身时所对标的品质天花板。研读其公开资料让我们看清了研究原型与生产级求解器之间的差距：病态系统上的鲁棒性，以及完整的 3D 装配覆盖。我们的目标，就是缩小这一差距。

它们是灵感来源，而非依赖：Ansatz 是独立的、采用 MIT 许可证的自研 Rust 实现。
