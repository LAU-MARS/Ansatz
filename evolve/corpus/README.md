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
| `outcome` | string | expected solve outcome; also implies exit-code semantics (converged ⇔ exit 0) |
| `dof_total` / `dof_remaining` | u32 | expected degree-of-freedom counts |
| `entities` | array | expected final entity states; `x`/`y` are **hex IEEE 754 bit patterns** ("0x…") compared bit-exactly, not by epsilon — same convention as the product's `tests/parity` |
| `redundant_constraint_ids` | `number[][]` | expected redundancy/conflict groups; **set equality** (extra or missing groups are both behavior regressions) |
| `suggestion_actions` | string[] | expected suggestion actions; **subset matching** (more or smarter suggestions do not fail a case). Format: `remove_constraint:2` / `relax_constraint:7` / `add_constraint:1` |
| `tool_error_kind` | string | expected tool error (exit 2 + the stderr JSON's `kind`); mutually exclusive with `outcome` |

## Design discipline

1. **Bit-level strictness**: numeric assertions are always bit patterns. Any
   change to the solver's numeric behavior is exposed by the corpus — same
   origin as the product's "bit-identical across platforms" promise.
2. **Omitted = wildcard**: fields that are not written are not judged. New
   diagnostic capabilities do not break existing cases; but every assertion that
   *is* written may only change *deliberately* (editing a case means changing
   the ground truth and requires a rationale in the proposal).
3. **Verdicts ignore timing**: the evaluator records `duration_ms` but it never
   gates pass/fail — timing is inherently nondeterministic. Making speed part
   of fitness is a post-stage-B concern, using medians-of-many then.
4. The corpus itself must evolve too (stage D: corpus co-evolution), but
   **additions/edits only, never silent drift**: the corpus directory has a
   content hash recorded in the lineage, so changes are traceable.

## Current cases

| case | Covered path |
|---|---|
| `case-exact-345` | closed form on exact rationals; underconstrained + particular solution + add_constraint suggestion |
| `case-irrational-normalize` | sqrt/division inexact path (the real bit-parity canary) |
| `case-conflict-distances` | Inconsistent + conflict group + remove/relax suggestions + particular solution at the conflict |
| `case-redundant-duplicate` | Overconstrained (redundant) + satisfying solution still returned |
| `case-negative-distance` | Inconsistent (negative distance) + relax_constraint suggestion |
| `case-empty-model` | Converged (exit 0) empty model |
| `case-unsupported-coincident` | tool-error path (exit 2 + unsupported_constraint) |
| `case-3d-shaft-converged` | 3D fully-constrained assembly (fixed + coaxial + distance + angle); **LM solution bit patterns** |
| `case-3d-mate-underconstrained` | single mate removes 3 of 6 DOF |
| `case-3d-conflict-distances` | provably inconsistent distance pair on rigid bodies |
| `case-3d-driven-arm` | three-link arm: fixed + 3 revolutes (2 driven) + transmission coupling; **joint-space drive bit patterns** |
