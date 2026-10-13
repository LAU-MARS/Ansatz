# Ansatz

English | [简体中文](README.zh-CN.md)

[![CI](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml/badge.svg)](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/ansatz-wasm?logo=npm&label=ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![npm downloads](https://img.shields.io/npm/dm/ansatz-wasm)](https://www.npmjs.com/package/ansatz-wasm)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A portable geometric constraint solver for 2D sketches and 3D assemblies. One Rust core, native on macOS/Linux/Windows/HarmonyOS, WASM for browser and Node, JSON in/out for agents.

**Structured diagnostics are a first-class citizen**: the solver never just says "failed" — it reports degrees of freedom, conflicting constraint groups (separately from consistent-but-redundant ones), per-constraint residuals, machine-readable suggestions, and a `human_message` on every diagnostic entry, written for humans *and* LLMs.

**Solving is a pure function of the poses**: feeding a solution back in returns it unchanged, bit for bit — joint zeros are defined by geometry, never by "whatever the input pose was".

## Current stage: 3D assembly + robotics primitives implemented

✅ **3D assembly constraints (6-DOF rigid bodies)**: mate (planar contact),
coaxial, distance (**anchored**), angle, and fixed.

✅ **Robotics primitives**:
- **Joint family** (first-class, diagnosed per joint): revolute (removes 5,
  leaves 1), cylindrical (4/2), prismatic (5/1), spherical (3/3). Prismatic
  `a_ref`, `b_ref`, and `roll` must be supplied together; omitting all three
  selects deterministic references and zero roll, keeping the roll lock.
- **Joint variables are absolute**: a revolute angle θ is the signed angle about
  the axis from `R_a·a_ref` to `R_b·b_ref` (zero references in each body frame,
  optional; a deterministic default is derived from the axis). A prismatic δ is
  the signed axial distance between the axis origins. `drive` targets these
  absolute values (radians / length), removing the joint's remaining motion.
  Revolute targets are reduced modulo `2π` with bounded remainder arithmetic,
  including large finite values; re-solving a satisfied pose moves nothing.
- **Joint-state diagnostics**: `diagnostics.joints` reports each joint variable
- **Transmission coupling**: the `transmission` constraint enforces
  `θ_b = ratio · θ_a` between two different revolute joints (gears/belts;
  external meshing is a negative ratio). The difference is reduced modulo
  `2π`; continuous, unwrapped phase tracking is not implemented.

Numerical core: Levenberg–Marquardt with analytic Jacobians in the local SO(3)
tangent space and SVD-damped steps. Translation and rotation column triplets
are equilibrated before stepping, rank analysis, and condition estimation.
Linear algebra uses **nalgebra** (pure Rust, no BLAS, no_std); the core controls
update order and convergence semantics. Regression tests cover degenerate
initial poses, equivalent rotation representations, and large anchor scales;
parity cases cover iterated poses and numerical diagnostics.

Direction vectors are normalized before angle evaluation. Angles are checked
in radians against `tolerance`; exact `0` and `π` targets each remove two
rotational DOF. Coaxial axes must be codirected. A zero 3D distance is anchor
coincidence (three positional constraints); a zero 2D point-to-origin distance
fixes the point at the origin (zero remaining DOF). All numeric inputs must be
finite, and `tolerance` must be positive; invalid inputs or a nonfinite initial
constraint evaluation produce `invalid_model`.

⚠️ Not yet implemented: general 2D sketch constraints (still
`unsupported_constraint` for non-rigid3 entities). 2D models currently only
support point-to-origin distance via a closed form. An already satisfied point
is returned without reprojection, preserving coordinate bits on re-input.

## Engineering documentation

The [documentation index](docs/README.md) links the current
[architecture and Mermaid diagrams](docs/architecture.md),
[architecture decision records](docs/adr/README.md), and the
[evolution roadmap](docs/roadmap.md). These engineering documents are in Chinese;
implemented capabilities and proposed work are explicitly distinguished.

## Architecture

```
                ┌──────────────────────────────────────────┐
                │              ansatz-core                 │
                │  pure algorithms · no_std · f64 ·        │
                │  diagnostics-first · product numerical   │
                │  core                                    │
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
cargo test --workspace        # includes numerical regressions and bit-level parity

echo '{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
       "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}' \
  | cargo run -p ansatz-cli -- solve
# → (3,4) scaled to distance 10 = (6,8); outcome "converged", dof_remaining 1; exit 0
```

CLI exit codes: `0` converged · `1` solve-level failure — `inconsistent` or `not_converged` (full diagnostics still on **stdout**) · `2` tool error (JSON error on **stderr**). "Solver says no" and "tool is broken" are never conflated.

## Solve report

`outcome` answers exactly one question — does the returned configuration satisfy every constraint?

| `outcome` | Meaning |
|---|---|
| `converged` | Every constraint is satisfied within its geometric tolerance |
| `inconsistent` | A direct contradiction was proved; `diagnostics.conflicts` names the groups |
| `not_converged` | Not satisfied, and no conflict was proved, including nonlinear stationary failures |

Under-constraint (`dof_remaining > 0`) and redundancy (`redundant_constraints`) are orthogonal facts reported in `diagnostics`; a satisfied sketch that can still be dragged is `converged`.

- **DOF**: the free parameter count minus the rank of the equilibrated local
  Jacobian at the returned pose. This is local first-order information;
  arbitrary singular mechanisms still need separate analysis.
- **Conflicts**: the current proofs cover negative distances, unsatisfied
  constraints involving only fixed bodies, and incompatible distance targets
  on the same anchors whose tolerance intervals do not intersect. A stationary
  residual or Jacobian left-null-space projection alone is not proof of a
  nonlinear conflict; such failures return `not_converged`.
- **Redundancy** (order-independent): at a satisfied pose, removing the
  constraint leaves the local Jacobian rank unchanged; overlapping contributions
  are grouped. This is first-order redundancy at that pose, not proof that
  removing the constraint preserves the global nonlinear solution set.
  Duplicate `fixed` on one body is reported directly.

## npm package (ansatz-wasm)

The wasm shell ships as an npm package (wasm-pack `--target nodejs`; single-threaded, no SAB/COOP-COEP):

```bash
npm install ansatz-wasm
```

```js
const { solve, solveJson, version } = require('ansatz-wasm')
// ESM works too: import { solve } from 'ansatz-wasm'

const env = solve(model)   // { ok, result | error } — never throws
```

The package bundles the handwritten model types at `types/ansatz.d.ts` (`AnsatzModel` / `SolveReport` / `Diagnostics`, mirroring the schema contracts). Publishing: push a `v*` tag to trigger [npm-publish.yml](.github/workflows/npm-publish.yml) — CI builds and polishes the package (`scripts/npm-prepare.mjs`; the tag must equal the version), and **publishes only after the wasm bit-parity smoke passes**. Local rehearsal: `node scripts/npm-prepare.mjs && npm pack`.

## Shells

- **ffi** — stable C ABI: `ansatz_solve_json(const char*) -> char*` + `ansatz_string_free` + `ansatz_version`. Header `crates/ansatz-ffi/include/ansatz.h` is cbindgen-generated and committed. Every `extern "C"` function is wrapped in `catch_unwind`; panics never cross the FFI boundary.
- **wasm** — `wasm-pack build --target web` and `--target nodejs` both pass, `.d.ts` generated. Single-threaded; no SharedArrayBuffer / COOP-COEP. Smoke tests: a Node script (`smoke/node.mjs`) and a fully self-contained single-file HTML (`smoke/web/index.html`, wasm inlined as base64, `initSync`, works over `file://`).
- **node** — napi-rs; exports `solve` / `solveJson` / `version` with the same names and signatures as the wasm shell, so hosts can switch between them freely.
- **cli** — see quick start above; `export-schema model|solve-report` regenerates the JSON contracts.

## Self-improvement lab (evolve/)

[`evolve/`](evolve/README.md) is a **separate cargo workspace** whose mission is to make "improving the solver" itself an automatable iterative loop (RSI: recursive self-improvement). Currently at **stage A (manual-proposal mode)**, working and verified:

- **Golden corpus** (`evolve/corpus/`): 25 cases covering outcomes and numerical
  audit regressions. Written expectations assert outcomes, DOF, groups, and
  suggestions, with poses and joint values compared within a tolerance.
- **Independent evaluator** (`ansatz-eval`): candidates run as black-box
  subprocesses with timeouts and byte-identical repeated output. Every report
  must satisfy the complete JSON contract and model membership. The evaluator
  reconstructs geometry independently to check every supported constraint on a
  `converged` result, fixed bodies, and reported joint variables; claimed
  residuals and omitted expectations cannot bypass these checks. It shares
  fixed repository data types, not the candidate's mathematical implementation.
- **Loop skeleton** (`ansatz-evolve`): a `proposals/` inbox, vendor-neutral `Proposer`/`Selector` traits (reserved for an LLM proposer), and an append-only JSONL lineage archive.
- The product CI `evolve-eval` job requires every corpus case to pass — solver behavior changes must update the corpus deliberately.

Path: A manual proposals → B automated apply/build/eval loop → C LLM proposer → D corpus co-evolution (the improver improves its own test set — the step that actually crosses into RSI). Details in `evolve/README.md`.

## Contracts

`schema/model.schema.json` and `schema/solve-report.schema.json` are exported from the Rust types via schemars; CI regenerates and diffs them, so drift fails the build:

```bash
cargo run -p ansatz-cli -- export-schema model
cargo run -p ansatz-cli -- export-schema solve-report
```

## Bit-level numerical reproducibility

With the same solver version, the same input must produce bit-identical f64 output on x86-64 / aarch64 / wasm32:

1. Strict IEEE 754 semantics — no `-ffast-math` equivalents, no hand-rolled FMA (rationale and lock-down: `[profile.release]` comments in the root `Cargo.toml`).
2. Transcendentals go through `libm` only — pure Rust, one codebase on every target.
3. A **parity framework** at `crates/ansatz-core/tests/parity/`: fixed cases with expected outputs committed as **hex bit patterns** (not epsilon), asserted identically by Rust tests on the CI arch matrix (linux-x86_64, macos-aarch64, windows-msvc) and by the wasm / node smoke scripts. Iterated assembly cases 3/4 and regression cases 5/6 check pose coordinates, every reported residual and joint value, and applicable condition estimates. Cases 5/6 cover irrational 2D normalization and a zero-roll joint at an equivalent full-turn pose; re-input and JSON-roundtrip checks protect idempotence.

The promise is cross-platform determinism, not frozen numerics: a deliberate change to the iteration strategy or the decomposition library may change the bits, and the parity files are then regenerated. Behavioral correctness is asserted with tolerances by the regular tests and the evolve corpus.

## 3D rigid pose parameterization

JSON poses retain the **rotation vector** `v = θ·û`. The solver differentiates
and composes three-component rotational increments in the local SO(3) tangent
space instead of adding increments to the global vector. DOF therefore does
not inherit the exponential-map coordinate singularity at `2π`. Rotation
vectors are not globally unique. A future quaternion storage representation
can likewise use three local optimization variables, without adding a
unit-norm equation to the solve.

## CI

`.github/workflows/ci.yml`: fmt / clippy `-D warnings` / test; build matrix x86_64-linux, aarch64-darwin, x86_64-windows-msvc, wasm32 (web + nodejs); schema drift check; plus a `continue-on-error` scouting job for `aarch64-unknown-linux-ohos` (HarmonyOS) — a tier 3 target that needs nightly `-Z build-std` and the OHOS NDK for linking, so it must never block the pipeline.

## Roadmap

The [detailed roadmap](docs/roadmap.md) defines dependencies, evidence, and exit
criteria. Its current sequence is:

1. M0 — reliability foundation implemented: bounded numerics, corrected
   semantics and diagnostics, independent geometry checks, and regression cases.
2. M1 — expand generated tests, held-out real models, and measured quality gates.
3. M2 — strengthen 3D solving for closed loops, degeneracies, scale, and branches.
4. M3 — implement general 2D sketches after stable sub-element references and
   workplane semantics are defined.
5. M4 — graph decomposition, compiled models, incremental edits and dragging,
   caches, and a deterministic sparse backend.
6. M5 — validate real workflows and version compatibility, prioritize Onshape
   integration, and extend geometry or mechanisms according to demonstrated needs.

The separate evolve track remains at Stage A: baseline evaluation, lineage
recording, and listing manual proposals. Automatic apply/build, LLM proposals,
and corpus co-evolution are planned stages with additional evaluator and
execution requirements. HarmonyOS delivery, joint limits, screw joints,
splines, and URDF import are future work, not evidence of current capability.

## Acknowledgements

Ansatz stands on the shoulders of the following projects. Their designs shaped ours, and we owe them real gratitude:

- **[SolveSpace](https://github.com/solvespace/solvespace)** (C++, GPLv3) — the de facto gold standard among open-source geometric constraint solvers. It proved that a single solver can serve both 2D sketches *and* native 3D assembly constraints; that a dogleg trust-region method over nonlinear least-squares residuals can be fast enough for interactive use; and that under-constrained sketches can stay naturally manipulable via dragging. Its reusable library form (`libslvs`) and WASM port (as embedded in Blender's *Geometry Sketcher* add-on) were direct inspiration for our embed-anywhere, one-core architecture.

- **[FreeCAD](https://github.com/FreeCAD/FreeCAD) — the planegcs / Sketcher solver** (C++, LGPL) — the engine behind FreeCAD's Sketcher workbench. From it we learned the value of switchable solving strategies (DogLeg / Levenberg-Marquardt / BFGS), of degree-of-freedom analysis, and of decomposing the constraint graph into independently solvable blocks — above all, that redundant and conflicting constraints must be *diagnosed and explained*, not merely failed. That is the UX bar we hold ourselves to.

- **[LEDAS LGS 2D/3D](https://ledas.com/)** (commercial) — the quality ceiling open-source solvers measure themselves against. Studying its published materials made clear what separates research prototypes from production-grade solvers: robustness on ill-conditioned systems and full 3D assembly coverage. We aim to close that gap.

These are inspirations, not dependencies: Ansatz is an independent, MIT-licensed implementation with its own Rust core.
