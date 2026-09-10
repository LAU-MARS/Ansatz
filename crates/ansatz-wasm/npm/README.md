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

3D assembly constraints are implemented and solvable: `mate` (planar contact),
`coaxial`, `distance`, `angle`, and `fixed` on 6-DOF rigid bodies
(`rigid3` entities), solved by Levenberg–Marquardt with analytic SO(3)
Jacobians and full diagnostics (DOF / redundancy groups / residuals /
suggestions). General 2D sketch constraints (coincident/parallel/tangent...)
are not yet implemented and return a precise `unsupported_constraint` tool
error. The JSON contract is frozen in
[`schema/`](https://github.com/LAU-MARS/Ansatz/tree/main/schema).

Example — fully constrain a body onto a base (converges, 0 DOF remaining):

```js
solve({
  entities: [
    { id: 1, geometry: { type: 'rigid3', pose: { translation: { x: 0, y: 0, z: 0 }, rotation: { vector: [0, 0, 0] } } } },
    { id: 2, geometry: { type: 'rigid3', pose: { translation: { x: 0.5, y: 0.3, z: 1.2 }, rotation: { vector: [0.1, -0.2, 0.15] } } } },
  ],
  constraints: [
    { id: 90, kind: { type: 'fixed', a: 1 } },
    { id: 1, kind: { type: 'coaxial', a: 1, b: 2,
        a_axis: { origin: { x: 0, y: 0, z: 0 }, direction: { x: 0, y: 0, z: 1 } },
        b_axis: { origin: { x: 0.1, y: 0.2, z: 0.3 }, direction: { x: 0.1, y: 0.2, z: 1 } } } },
    { id: 2, kind: { type: 'distance', a: 2, b: 1, value: 2.0 } },
    { id: 3, kind: { type: 'angle', a: 1, b: 2, value: 1.5707963267948966,
        a_dir: { x: 1, y: 0, z: 0 }, b_dir: { x: 0, y: 1, z: 0 } } },
  ],
})
// -> { ok: true, result: { outcome: 'converged', entities: [...],
//      diagnostics: { dof_remaining: 0, ... } } }
```

## Numerical reproducibility

Same input → bit-identical f64 output across x86-64 / aarch64 / wasm32 (hex
bit-pattern test framework in the repo). No `-ffast-math` equivalents, no FMA
contraction; transcendentals go through `libm`.

License: MIT.
