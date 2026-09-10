# ansatz-wasm

[![CI](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml/badge.svg)](https://github.com/LAU-MARS/Ansatz/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/LAU-MARS/Ansatz/blob/main/LICENSE)

> *Ansatz* — math term for a trial solution with undetermined parameters that
> constraints then pin down. Exactly what a geometric constraint solver does.

The [Ansatz](https://github.com/LAU-MARS/Ansatz) geometric constraint solver
core, compiled to WebAssembly for Node (wasm-pack `--target nodejs`).
Single-threaded — no SharedArrayBuffer / COOP-COEP required. Works the same in
Electron, plain Node, and bundlers.

**Structured diagnostics are a first-class citizen**: the solver never just
says "failed" — it reports degrees of freedom, redundant/conflicting constraint
groups, per-constraint residuals, machine-readable suggestions, and a
`human_message` on every diagnostic entry, written for humans *and* LLMs.

## Install

```bash
npm install ansatz-wasm
```

## Usage

```js
const { solve, solveJson, version } = require('ansatz-wasm')
// ESM: import { solve, solveJson, version } from 'ansatz-wasm'

// 1) structured: JS object in, envelope out — never throws
const env = solve({
  entities: [{ id: 1, geometry: { type: 'point2', x: 3, y: 4 } }],
  constraints: [{ id: 1, kind: { type: 'distance', a: 1, b: null, value: 10 } }],
})
// env = { ok: true, result: { outcome: 'underconstrained', entities: [...],
//         diagnostics: { dof_total: 2, dof_remaining: 1, suggestions: [...], ... } } }

// 2) string: same JSON envelope as the C ABI (ansatz_solve_json)
const s = solveJson(JSON.stringify(model))

// 3) solver version
version() // '0.1.0'
```

Envelope: `{ok:true, result: SolveReport} | {ok:false, error:{kind, message}}`.
A solve-level failure (`inconsistent`, `overconstrained`, ...) is still
`ok:true` with full diagnostics in `result` — only tool errors (bad JSON,
unknown entity, unsupported constraint) are `ok:false`.

Model types (`AnsatzModel`, `SolveReport`, `Diagnostics`, `Suggestion`, ...)
are included at `ansatz-wasm/types/ansatz.d.ts`.

## Current stage

Stage-0 skeleton: the solver implements one trivial case (a 2D point + a
distance-to-origin constraint, closed form) and rejects other constraint kinds
with a precise `unsupported_constraint` tool error. The contract (JSON schema,
diagnostics structure, envelope) is finished — see
[`schema/`](https://github.com/LAU-MARS/Ansatz/tree/main/schema) — and the
general numerical core arrives in later stages without API changes.

## Numerical reproducibility

Same input → bit-identical f64 output across x86-64 / aarch64 / wasm32 (hex
bit-pattern test framework in the repo). No `-ffast-math` equivalents, no FMA
contraction; transcendentals go through `libm`.

License: MIT.
