# Ansatz

[English](README.md) | 简体中文

[![CI](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml/badge.svg)](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/ansatz-wasm?logo=npm&label=ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![npm downloads](https://img.shields.io/npm/dm/ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> *Ansatz*，数学名词：先设一个带待定参数的试探解，再由约束条件把它定下来——这正是几何约束求解器做的事。

一个可移植的几何约束求解器，面向 2D 草图与 3D 装配。单一 Rust 核心，原生运行于 macOS / Linux / Windows / HarmonyOS，通过 WASM 支持浏览器与 Node，以 JSON 输入输出，对 Agent 友好。

**结构化诊断是一等公民**：不只回答「解出来了没有」，还要回答「为什么没解出来」——自由度计数、冲突约束组（与「一致但冗余」的约束组分开报告）、每条约束的残差、机器可读的修复建议，且每条诊断都带同时面向人类与 LLM 的 `human_message`。

**求解是位姿的纯函数**：把求解结果原样再解一次，得到逐位相同的结果——关节零位由几何定义，不依赖「本次输入时的位形」。

服务两类问题：

- **2D 草图约束**：当前实现点到原点距离；重合、共线、平行、垂直、相切等仍待实现
- **3D 装配约束**：6-DOF 刚体配合——贴合、同轴、距离、角度

## 当前阶段：3D 装配 + 机器人学原语已实现

✅ **3D 装配约束（6-DOF 刚体）**：mate（面贴合）、coaxial（同轴）、
distance（**锚点**距离）、angle（方向夹角）、fixed（固定）。

✅ **机器人学原语**：
- **关节族**（first-class，诊断按关节报告）：revolute（消 5 留 1）、
  cylindrical（消 4 留 2）、prismatic（消 5 留 1）、spherical（球副，消 3 留 3）。
  移动副的 `a_ref`、`b_ref`、`roll` 必须一起提供；全部缺省时按轴选取确定性参考
  方向并锁定零滚转，仍是移动副
- **关节变量是绝对量**：revolute 的 θ 是绕轴从 `R_a·a_ref` 转到 `R_b·b_ref`
  的有向角（`a_ref`/`b_ref` 为各自体坐标系内的零位参考，可选，缺省按轴方向
  确定性选取）；prismatic 的 δ 是两轴原点沿轴的有向距离。`drive` 直接给出
  这些绝对目标（弧度 / 长度），消除该关节剩余的运动。转动目标通过有界余数运算
  按 `2π` 折回，有限巨大角度也能返回；满足后的位姿再求解不会移动
- **关节状态诊断**：`diagnostics.joints` 逐关节报告当前变量
- **传动耦合**：`transmission` 约束在两个不同的 revolute 关节间施加
  `θ_b = ratio · θ_a`（齿轮/带轮，外啮合为负速比），差值按 `2π` 折回；
  尚不支持连续展开的传动相位追踪

数值内核：Levenberg–Marquardt 在 SO(3) 局部切空间使用解析雅可比与 SVD 阻尼步。
平移和旋转分别以三列为一组均衡，再用于步长、秩和条件数计算。线性代数使用
**nalgebra**（纯 Rust、无 BLAS、no_std），核心负责更新顺序与收敛语义。
回归测试覆盖退化初值、等价旋转表示和大尺度锚点；parity 同时覆盖迭代位姿与数值诊断。

夹角方向先归一化，按真实弧度与 `tolerance` 比较；精确的 `0`、`π` 目标各消除
两个旋转自由度。同轴要求轴方向相同。3D 零距离表示锚点重合，消除三个位置自由度；
2D 点到原点零距离把点固定在原点，剩余自由度为零。所有数值输入必须有限，
`tolerance` 必须为正；无效输入或初始约束求值产生非有限量时返回 `invalid_model`。

⚠️ 尚未实现：通用 2D 草图约束（coincident/parallel/tangent 等，对 rigid3
以外实体仍返回 `unsupported_constraint`）。2D 模型目前只支持「点到原点距离」
的闭式解。已满足约束的点直接保留坐标，回灌时不重新投影，保证逐位幂等。

## 架构

工程文档入口：[docs 导航](docs/README.md)、[架构与 Mermaid 架构图](docs/architecture.md)、
[架构决策记录 ADR](docs/adr/README.md)、[演进 roadmap](docs/roadmap.md)。
文档明确区分当前实现、已采纳方向和拟议能力。

```
                ┌──────────────────────────────────────────┐
                │              ansatz-core                 │
                │   纯算法 · no_std · f64 · 诊断一等公民     │
                │   （产品数值算法所在）                      │
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
cargo test --workspace        # 含数值回归与位级 parity 断言

# CLI：3-4-5 三角形缩放，(3,4) 施加到原点距离 10 → (6,8)
echo '{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
       "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}' \
  | cargo run -p ansatz-cli -- solve
```

输出（约束满足即 converged，退出码 0；剩余 1 个自由度在诊断里）：

```json
{
  "outcome": "converged",
  "entities": [{ "id": 1, "geometry": { "type": "point2", "x": 6.0, "y": 8.0 } }],
  "diagnostics": {
    "dof_total": 2,
    "dof_remaining": 1,
    "iterations": 0,
    "conflicts": [],
    "redundant_constraints": [],
    "residuals": [{ "constraint_id": 1, "residual": 0.0, "human_message": "约束 1 在返回的特解处残差为 0e0（即 ‖p‖ − 1e1）。" }],
    "max_residual": { "constraint_id": 1, "residual": 0.0, "human_message": "…" },
    "suggestions": [{
      "action": "add_constraint",
      "dof_remaining": 1,
      "human_message": "模型还有 1 个自由度未被约束。以「点到原点的距离」为例：它只固定了模长，点仍可沿圆周滑动，需要再施加角度或坐标类约束。"
    }],
    "joints": [],
    "jacobian_condition_estimate": 1.0
  }
}
```

### CLI 退出码（「工具错误」与「求解失败」严格分离）

| 退出码 | 含义 | 输出 |
|---|---|---|
| 0 | `converged`：返回位形满足全部约束（可能仍有剩余自由度或冗余，见诊断） | 报告 JSON → stdout |
| 1 | 求解层失败（`inconsistent` / `not_converged`），完整诊断照常 | 报告 JSON → stdout |
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

求解失败不是异常，而是携带完整证据的报告。`outcome` 只回答一个问题——返回的位形是否满足全部约束：

| `outcome` | 含义 |
|---|---|
| `converged` | 全部约束在几何容差内满足 |
| `inconsistent` | 已直接证明矛盾，`diagnostics.conflicts` 给出冲突组 |
| `not_converged` | 未满足，且未能证明冲突，包括非线性驻点处的求解失败 |

欠约束与冗余是与结局正交的事实，放在 `diagnostics` 里：

- `dof_total / dof_remaining`：自由度计数，`dof_remaining = 自由参数数 − rank(J)`，
  J 为返回位姿处经列均衡的局部雅可比。这是一阶局部信息；一般奇异机构仍需额外分析
- `iterations`：实际迭代数（2D 闭式路径为 0）
- `conflicts`：当前直接证明覆盖负距离、只涉及固定刚体但不满足的约束，以及同一锚点
  对上容差区间不相交的距离目标。驻点残差或雅可比左零空间投影不能单独证明
  非线性模型无解；这类失败报告 `not_converged`
- `redundant_constraints`：满足位姿处的局部一阶冗余，**与书写顺序无关**——去掉某条
  约束后局部 rank(J) 不变，贡献相互重叠的约束归为一组。这不证明删除后全局非线性
  解集不变；同一刚体上的重复 `fixed` 直接报告
- `residuals` / `max_residual`：每条约束在返回解处的残差
- `joints`：逐关节的绝对关节变量
- `suggestions`：机器可读动作（`remove_constraint{id}` / `relax_constraint{id}` / `add_constraint{dof_remaining}`）
- `jacobian_condition_estimate`：列均衡后局部雅可比的条件数估计 `σ_max/σ_min(非零)`；
  无约束或退化时为 null

## 数值可复现性（跨平台位级一致）

同一版本、同一输入，在 x86-64 / aarch64 / wasm32 上**逐位相同**，这是硬承诺：

1. 严格 IEEE 754 语义：不启用任何等价 `-ffast-math` 的选项，不手写 FMA 融合（依据与锁死方式见根 `Cargo.toml` 的 `[profile.release]` 注释）；
2. 超越函数只经 `libm`（纯 Rust，所有目标编译同一份代码，正确舍入是构造性的）；
3. **位模式测试框架**：`crates/ansatz-core/tests/parity/` 存放固定算例与期望值（十六进制位模式，非 epsilon 比较）。同一份期望文件被三处断言：
   - Rust 测试（CI 在 linux-x86_64 / macos-aarch64 / windows-msvc 矩阵上原位运行）
   - wasm Node 冒烟（`crates/ansatz-wasm/smoke/node.mjs`）
   - node 原生冒烟（`crates/ansatz-node/smoke/native.mjs`）

   `case3`（3D 全约束装配）与 `case4`（驱动 + 传动的三连杆臂）覆盖 LM 迭代；
   `case5` 覆盖非精确 2D 归一化，`case6` 覆盖等价整周姿态下的零滚转关节。
   位姿坐标、每条残差和关节变量及适用的条件数均按位断言，回灌与 JSON 往返另验幂等。
JSON 往返同样在承诺内——serde_json 全线启用
`float_roundtrip`（默认快速解析对某些 17 位小数差 1 ULP）。

承诺的是跨平台确定性，而不是冻结数值：有意修改迭代策略或升级分解库都可能
合法地改变这些位模式，届时重新生成 parity 文件即可。行为正确性由常规测试与
evolve 语料按容差断言。

## 3D 刚体位姿参数化

JSON 位姿保持**旋转向量** `v = θ·û`。求解器在 SO(3) 局部切空间对三个旋转增量
求导并组合更新，而非直接向全局旋转向量加增量，避免 `2π` 坐标奇异破坏物理 DOF。
旋转向量并非全局唯一表示。未来即便采用四元数存储，也可以仍用三个局部优化变量，
无需额外加入单位范数求解方程。

## CI

`.github/workflows/ci.yml`：fmt / clippy `-D warnings` / test；构建矩阵 x86_64-linux、aarch64-darwin、x86_64-windows-msvc、wasm32（web + nodejs 双 target）；schema 漂移检查；evolve 黄金语料回归；以及 aarch64-unknown-linux-ohos（HarmonyOS，tier 3）的 `continue-on-error` 探路 job——tier 3 无预编译 std，需 nightly `-Z build-std`，链接需 OHOS NDK，因此只做 check 且不阻塞流水线。

## 自进化实验区（evolve/）

[`evolve/`](evolve/README.zh-CN.md) 是独立的 cargo workspace，使命是让「改进求解器」本身成为可自动化迭代的闭环（RSI：递归自我改进）。当前为**阶段 A（手动提案模式）**，已就位并实测：

- **黄金语料**（`evolve/corpus/`）：25 个 case 覆盖结局与数值审查反例。写出的期望
  比较结局、DOF、约束组与建议，位姿与关节变量按容差比较；
- **独立评测器**（`ansatz-eval`）：候选只以子进程 + 超时运行，每个 case 两次输出
  必须逐字节相同。每份报告都校验完整 JSON 契约及模型成员；对 `converged` 结果
  独立重建几何、复算所有已支持约束，并检查固定体与关节变量。候选自报残差或期望
  字段缺省不能绕过这些检查。共享的仅是固定仓库数据类型，不采信候选数学实现；
- **循环骨架**（`ansatz-evolve`）：`proposals/` 投递箱 + `Proposer`/`Selector` trait（vendor 无关，为 LLM 提案者预留）+ JSONL 谱系归档；
- 产品 CI 的 `evolve-eval` job 要求黄金语料全部通过——求解器行为变化必须「有意地」同步语料。

路线：A 手动提案 → B 自动 apply/build/eval 闭环 → C LLM 提案者 → D 语料共演化（改进者改进自己的考卷，闭环回流——真正跨入 RSI）。详见 `evolve/README.md`。

## 路线图

完整的依赖、验收门和证据要求见 [演进 roadmap](docs/roadmap.md)，当前顺序为：

1. M0：可靠性基础已完成首轮——有界数值计算、语义与诊断修正、独立几何检查及反例回归。
2. M1：扩展生成测试、真实模型留出集与可复查的质量指标。
3. M2：加强闭链、退化、尺度与装配分支下的稳健 3D 求解。
4. M3：先定义子元素引用与工作平面，再实现一般 2D 草图。
5. M4：约束图分解、可编译模型、增量编辑与拖拽、缓存及确定性稀疏后端。
6. M5：真实产品流程验证、版本兼容，优先评估 Onshape 集成，并按已验证需求扩展几何与机构。

evolve 并行轨道仍处于 A：基线评测、谱系记录和手动提案扫描。自动应用/构建、LLM
提案和语料共演化属于后续阶段，需要补齐裁判与执行隔离条件。HarmonyOS 交付、关节
限位、螺旋副、样条和 URDF 导入仍是计划，不代表当前已具备这些能力。

## 致谢

Ansatz 站在以下项目的肩膀上，它们的设计塑造了我们的设计，我们谨致谢忱：

- **[SolveSpace](https://github.com/solvespace/solvespace)**（C++，GPLv3）—— 开源几何约束求解器事实上的标杆。它证明了：单一求解器既能服务 2D 草图，也能支持原生 3D 装配约束；对非线性最小二乘残差采用 Dogleg 信任域法求解，速度足以支撑交互式使用；欠约束的草图也能通过拖拽保持自然可操作。它以可复用库的形式发布（`libslvs`），并有 WASM 移植（Blender 的 *Geometry Sketcher* 插件内嵌的正是它）—— 这直接启发了我们「单一核心、随处嵌入」的架构。

- **[FreeCAD](https://github.com/FreeCAD/FreeCAD) 的 planegcs / Sketcher 求解器**（C++，LGPL）—— FreeCAD Sketcher 工作台背后的引擎。我们从它学到了可切换求解策略（DogLeg / Levenberg–Marquardt / BFGS）的价值、自由度（DOF）分析的价值，以及将约束图分解为可独立求解子块的价值 —— 最重要的是：冗余与冲突约束必须被**诊断并解释**，而不是仅仅报一个求解失败。这正是我们对自己坚持的体验标准。

- **[LEDAS LGS 2D/3D](https://ledas.com/)**（商业软件）—— 开源求解器衡量自身时所对标的品质天花板。研读其公开资料让我们看清了研究原型与生产级求解器之间的差距：病态系统上的鲁棒性，以及完整的 3D 装配覆盖。我们的目标，就是缩小这一差距。

它们是灵感来源，而非依赖：Ansatz 是独立的、采用 MIT 许可证的自研 Rust 实现。
