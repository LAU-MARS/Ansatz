English | [简体中文](README.zh-CN.md)

# Golden corpus

The **ground truth for verdicts** in the self-improvement loop. Each case is one
directory:

```
cases/<case-id>/
├── model.json     # solver input (conforms to schema/model.schema.json)
└── expected.json  # expectations (below; omitted fields = wildcard)
```

## expected.json field contract

| Field | Type | Semantics |
|---|---|---|
| `description` | string | human note; not judged |
| `outcome` | string | expected solve outcome (`converged` / `inconsistent` / `not_converged`); also implies exit-code semantics (converged ⇔ exit 0) |
| `dof_total` / `dof_remaining` | u32 | expected degree-of-freedom counts |
| `entities` | array | expected final entity states (point2 `x`/`y`, rigid3 `pose{tx,ty,tz,rx,ry,rz}`), compared within `value_tolerance`. Only write these for models with a unique solution — the particular solution of an under-constrained model legitimately changes with the iteration strategy |
| `joints` | array | expected joint variables `{constraint_id, value}`, compared within `value_tolerance` |
| `value_tolerance` | f64 | absolute tolerance for entities and joints; default `1e-7` |
| `conflict_constraint_ids` | `number[][]` | expected conflict groups; **set equality**; `[]` asserts there are none |
| `redundant_constraint_ids` | `number[][]` | expected redundancy groups; **set equality**; `[]` asserts there are none |
| `suggestion_actions` | string[] | expected suggestion actions; **subset matching** (more or smarter suggestions do not fail a case). Format: `remove_constraint:2` / `relax_constraint:7` / `add_constraint:1` |
| `tool_error_kind` | string | expected tool error (exit 2 + the stderr JSON's `kind`); mutually exclusive with `outcome` |

## Design discipline

1. **Assert behavior, not noise**: discrete facts (outcome, DOF, groups,
   suggestions) are compared exactly; continuous values within a tolerance. A
   faster or more accurate iteration strategy is never scored as a regression;
   cross-platform bit-identity is the product parity suite's job.
2. **Determinism**: the evaluator runs every case twice; stdout, stderr and the
   exit code must be byte-identical.
3. **Omitted expectations = wildcard**: fields omitted from `expected.json` are
   not compared with a frozen value. Every solve report must still satisfy the
   full report contract: exact entity/constraint membership, finite numeric
   values, unchanged fixed geometry, and outcome/exit consistency. Every
   `converged` report is checked against geometry independently of its claimed
   residuals; reported joint variables are also recomputed. Written expectations
   may only change *deliberately*, with a rationale in the proposal.
4. **Verdicts ignore timing**: the evaluator records `duration_ms` but it never
   gates pass/fail — timing is inherently nondeterministic. Making speed part
   of fitness is a post-stage-B concern, using medians-of-many then.
5. The corpus itself must evolve too (stage D: corpus co-evolution), but
   **additions/edits only, never silent drift**: the corpus directory has a
   content hash recorded in the lineage, so changes are traceable.

## Current cases

| case | Covered path |
|---|---|
| `case-exact-345` | closed form on exact rationals; converged + 1 DOF remaining + add_constraint suggestion |
| `case-irrational-normalize` | sqrt/division inexact path |
| `case-conflict-distances` | inconsistent + conflict group + remove suggestion + particular solution at the conflict |
| `case-redundant-duplicate` | converged + redundancy group (redundancy is orthogonal to the outcome) |
| `case-negative-distance` | inconsistent (negative distance) + relax_constraint suggestion |
| `case-empty-model` | converged (exit 0) empty model |
| `case-unsupported-coincident` | tool-error path (exit 2 + unsupported_constraint) |
| `case-3d-shaft-converged` | 3D fully-constrained assembly (fixed + coaxial + distance + angle); final pose |
| `case-3d-mate-underconstrained` | single mate removes 3 of 6 DOF |
| `case-3d-conflict-distances` | same-anchor distance conflict, proved directly from incompatible targets |
| `case-3d-coaxial-vs-spherical-conflict` | nonlinear failure (coaxial vs off-axis spherical): local stationarity is not proof; conservative `not_converged` |
| `case-3d-anchored-distances` | two distances on one body pair with different anchors are *not* a conflict |
| `case-3d-same-body-mate` | two-body constraint pointing at one body: tool error invalid_model |
| `case-3d-driven-arm` | three-link arm: fixed + 3 revolutes (2 driven) + transmission; absolute joint angles and final poses |
| `case-3d-huge-drive` | finite `drive=1e20`, bounded angle reduction; subprocess timeout guards the former hang |
| `case-3d-coincident-distance` | feasible distance from a coincident initialization with zero Jacobian |
| `case-3d-parallel-angle` | feasible right angle from initially parallel directions |
| `case-3d-nonunit-angle` | direction magnitude does not change angle semantics |
| `case-3d-antiparallel-fixed` | fixed opposite axes cannot satisfy codirected coaxial semantics |
| `case-3d-zero-roll-prismatic` | zero roll and zero displacement drive fully lock a prismatic joint |
| `case-3d-equivalent-rotation` | a `2π` rotation vector must have the same physical DOF as identity |
| `case-3d-unit-anchor-lock` | three noncollinear spherical anchors fully lock a rigid body |
| `case-3d-large-anchor-lock` | the same lock at anchor scale `1e12`: scale must not erase translation rank |
| `case-2d-already-normalized` | a returned irrational point-to-origin solution remains satisfied on re-input |
| `case-3d-negative-distance` | a negative 3D distance remains a directly proved contradiction |

The numerical audit regressions intentionally omit under-constrained poses and
iteration counts. The core regression suite additionally checks exact coordinate
bits on 2D re-input and compares the physical DOF at equivalent poses and anchor
scales. The change from `inconsistent` to `not_converged` for the nonlinear case
is deliberate: a Jacobian left-null-space residual proves a conflict of the
linearization, not unsatisfiability of the nonlinear model.
