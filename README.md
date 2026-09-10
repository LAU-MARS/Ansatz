# Ansatz

English | [简体中文](README.zh-CN.md)

A portable geometric constraint solver for 2D sketches and 3D assemblies. One Rust core, native on macOS/Linux/Windows/HarmonyOS, WASM for browser and Node, JSON in/out for agents.

**Structured diagnostics are a first-class citizen**: the solver never just says "failed" — it reports degrees of freedom, redundant/conflicting constraint groups, per-constraint residuals, machine-readable suggestions, and a `human_message` on every diagnostic entry, written for humans *and* LLMs.

## Current stage: skeleton (full pipeline, placeholder algorithm)

⚠️ This repository is at **stage 0: skeleton & platform feasibility**. `ansatz-core::solve` implements exactly one trivial case (**a 2D point + a distance-to-origin constraint**, closed form `p' = p·(d/‖p‖)`); every other constraint kind returns an `unsupported_constraint` tool error. The data model, diagnostics, all four shells, schema contracts and CI (including the bit-level parity test framework) are real and verified. The general numerical solver (Newton/LM, sparse Jacobian, rank analysis) comes in later stages and will only replace `ansatz-core/src/solver.rs`.

## Architecture

```
                ┌──────────────────────────────────────────┐
                │              ansatz-core                 │
                │  pure algorithms · no_std · f64 ·        │
                │  diagnostics-first (the only place       │
                │  math is allowed)                        │
                └───────┬──────────┬──────────┬────────────┘
                        │          │          │
        ┌───────────────┤          │          ├───────────────┐
        ▼               ▼          ▼          ▼               ▼
   ansatz-ffi      ansatz-wasm           ansatz-node     ansatz-cli
   C ABI           wasm-bindgen          napi-rs         stdin→stdout
   cdylib+staticlib browser/Node         Node native      diagnostics
   (HarmonyOS      (single-threaded,     (same API as    JSON + exit
    NAPI & any      no SAB/COOP-COEP)    wasm shell)      code contract
    C host)
                        │
                        ▼  (black-box, subprocess + timeout)
                evolve/ —— self-improvement lab (RSI runway):
                golden corpus · deterministic evaluator ·
                proposer/selector/lineage skeleton
```

Every shell does **type conversion and error mapping only — no math**. ffi / wasm / node share one envelope defined in core: `{ok:true,result} | {ok:false,error:{kind,message}}`.

## Quick start

```bash
cargo build --workspace
cargo test --workspace        # 21 tests, including bit-level parity assertions

echo '{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
       "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}' \
  | cargo run -p ansatz-cli -- solve
# → (3,4) scaled to distance 10 = (6,8); outcome "underconstrained" + full diagnostics; exit 1
```

CLI exit codes: `0` converged · `1` solve-level failure (full diagnostics still on **stdout**) · `2` tool error (JSON error on **stderr**). "Solver says no" and "tool is broken" are never conflated.

## Shells

- **ffi** — stable C ABI: `ansatz_solve_json(const char*) -> char*` + `ansatz_string_free` + `ansatz_version`. Header `crates/ansatz-ffi/include/ansatz.h` is cbindgen-generated and committed. Every `extern "C"` function is wrapped in `catch_unwind`; panics never cross the FFI boundary.
- **wasm** — `wasm-pack build --target web` and `--target nodejs` both pass, `.d.ts` generated. Single-threaded; no SharedArrayBuffer / COOP-COEP. Smoke tests: a Node script (`smoke/node.mjs`) and a fully self-contained single-file HTML (`smoke/web/index.html`, wasm inlined as base64, `initSync`, works over `file://`).
- **node** — napi-rs; exports `solve` / `solveJson` / `version` with the same names and signatures as the wasm shell, so hosts can switch between them freely.
- **cli** — see quick start above; `export-schema model|solve-report` regenerates the JSON contracts.

## Self-improvement lab (evolve/)

[`evolve/`](evolve/README.md) is a **separate cargo workspace** whose mission is to make "improving the solver" itself an automatable iterative loop (RSI: recursive self-improvement). Currently at **stage A (manual-proposal mode)**, working and verified:

- **Golden corpus** (`evolve/corpus/`): 7 cases covering every outcome path; numeric assertions are bit patterns, same discipline as the product parity suite.
- **Deterministic evaluator** (`ansatz-eval`): candidates run strictly as black-box subprocesses with timeouts; fitness depends only on deterministic facts (its very first run caught a `-0.0` bit-pattern divergence).
- **Loop skeleton** (`ansatz-evolve`): a `proposals/` inbox, vendor-neutral `Proposer`/`Selector` traits (reserved for an LLM proposer), and an append-only JSONL lineage archive.
- The product CI `evolve-eval` job forces 7/7 on the corpus — solver behavior changes must update the corpus deliberately.

Path: A manual proposals → B automated apply/build/eval loop → C LLM proposer → D corpus co-evolution (the improver improves its own test set — the step that actually crosses into RSI). Details in `evolve/README.md`.

## Contracts

`schema/model.schema.json` and `schema/solve-report.schema.json` are exported from the Rust types via schemars; CI regenerates and diffs them, so drift fails the build:

```bash
cargo run -p ansatz-cli -- export-schema model
cargo run -p ansatz-cli -- export-schema solve-report
```

## Bit-level numerical reproducibility

The same input must produce bit-identical f64 output on x86-64 / aarch64 / wasm32:

1. Strict IEEE 754 semantics — no `-ffast-math` equivalents, no hand-rolled FMA (rationale and lock-down: `[profile.release]` comments in the root `Cargo.toml`).
2. Transcendentals go through `libm` only — pure Rust, one codebase on every target.
3. A **parity framework** at `crates/ansatz-core/tests/parity/`: fixed cases with expected outputs committed as **hex bit patterns** (not epsilon), asserted identically by Rust tests on the CI arch matrix (linux-x86_64, macos-aarch64, windows-msvc) and by the wasm / node smoke scripts.

## 3D rigid pose parameterization

Rotation uses the **exponential map (rotation vector)**, not quaternions: a minimal 3-params ↔ 3-DOF parameterization — no unit-norm constraint to add, no redundant Jacobian columns (`JᵀJ` of a quaternion parameterization is necessarily singular). `Rotation3` is the single seam for switching parameterizations later; rationale in `ansatz-core/src/model.rs`.

## CI

`.github/workflows/ci.yml`: fmt / clippy `-D warnings` / test; build matrix x86_64-linux, aarch64-darwin, x86_64-windows-msvc, wasm32 (web + nodejs); schema drift check; plus a `continue-on-error` scouting job for `aarch64-unknown-linux-ohos` (HarmonyOS) — a tier 3 target that needs nightly `-Z build-std` and the OHOS NDK for linking, so it must never block the pipeline.

## Roadmap

1. **Stage 0 (this repo)** — skeleton: pipeline, diagnostics framework, contracts, CI, parity infrastructure
2. Stage 1 — general numerical core (damped Newton / Levenberg–Marquardt + sparse Jacobian)
3. Stage 2 — real rank analysis (DOF / redundancy / conflicts, beyond counting heuristics)
4. Stage 3 — full 2D sketch constraint set → 3D assembly (mate / coaxial / distance / angle)
5. Stage 4 — HarmonyOS NAPI deliverable (ansatz-ffi + OHOS NDK)

Parallel track (evolve/): A manual proposals (done) → B automated loop → C LLM proposer → D corpus co-evolution (RSI).

## Acknowledgements

Ansatz stands on the shoulders of the following projects. Their designs shaped ours, and we owe them real gratitude:

- **[SolveSpace](https://github.com/solvespace/solvespace)** (C++, GPLv3) — the de facto gold standard among open-source geometric constraint solvers. It proved that a single solver can serve both 2D sketches *and* native 3D assembly constraints; that a dogleg trust-region method over nonlinear least-squares residuals can be fast enough for interactive use; and that under-constrained sketches can stay naturally manipulable via dragging. Its reusable library form (`libslvs`) and WASM port (as embedded in Blender's *Geometry Sketcher* add-on) were direct inspiration for our embed-anywhere, one-core architecture.

- **[FreeCAD](https://github.com/FreeCAD/FreeCAD) — the planegcs / Sketcher solver** (C++, LGPL) — the engine behind FreeCAD's Sketcher workbench. From it we learned the value of switchable solving strategies (DogLeg / Levenberg-Marquardt / BFGS), of degree-of-freedom analysis, and of decomposing the constraint graph into independently solvable blocks — above all, that redundant and conflicting constraints must be *diagnosed and explained*, not merely failed. That is the UX bar we hold ourselves to.

- **[LEDAS LGS 2D/3D](https://ledas.com/)** (commercial) — the quality ceiling open-source solvers measure themselves against. Studying its published materials made clear what separates research prototypes from production-grade solvers: robustness on ill-conditioned systems and full 3D assembly coverage. We aim to close that gap.

These are inspirations, not dependencies: Ansatz is an independent, MIT-licensed implementation with its own Rust core.
