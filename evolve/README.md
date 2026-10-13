English | [简体中文](README.zh-CN.md)

# evolve/ —— Self-improvement lab (RSI runway)

> RSI = Recursive Self-Improvement. This directory's mission: make "improving the
> Ansatz solver" itself an automatable, iterable closed loop.

## Relationship to the product code

evolve/ is a separate cargo workspace (the root workspace's members are only
`crates/*`), so product builds and releases do not acquire lab dependencies.
Candidate solvers are accessed **strictly as a black box**:

```
candidate solver ──ansatz-cli subprocess + timeout + stdin/stdout JSON──> evaluator
```

That is safety rail #1: **evolved artifacts never execute inside the evaluating
process.** Timeouts, crashes, and resource abuse are just a failed case record —
they cannot harm the loop itself.

The evaluator shares the fixed repository's `ansatz-core` JSON data types to
parse complete models and reports. Its geometric checker reconstructs rotations
with its own quaternion implementation and recomputes relationships directly;
it does not call the core solver, residual evaluator, SO(3), or linear algebra.

## Loop design

```
                ┌─────────────────────────────────────────────────┐
                │                  evolve/ loop                   │
                │                                                 │
   proposals/   │  ┌──────────┐   ┌──────────┐   ┌─────────────┐  │
  (inbox) ──────┼─▶│ Proposer │──▶│  apply +  │──▶│  evaluator  │  │
   or LLM       │  │ (trait)  │   │  build    │   │ (black box  │  │
                │  └──────────┘   └──────────┘   │  + timeout) │  │
                │                                └──────┬──────┘  │
                │                       ┌───────────────▼──────┐  │
                │                       │ Selector (threshold) │  │
                │                       └──────────┬───────────┘  │
                │                  accept ┌────────┴─────┐ reject │
                │                        ▼              ▼        │
                │                   archive/         archive/    │
                │               (commit + land)  (record, drop)  │
                └─────────────────────────────────────────────────┘
                                     │
        the improved solver feeds corpus generation (stage D) = recursion
```

- **Fitness source**: golden-corpus pass rate (`ansatz-eval`). Every report is
  validated against the full JSON contract, model membership, finite numeric
  values, fixed bodies, and reported joint variables. For `converged`, every
  supported constraint is checked against independently reconstructed geometry,
  even when expected poses are omitted or claimed residuals are zero. Written
  expectations additionally check outcomes, DOF, groups, required suggestion
  actions, and continuous values within a tolerance. Every case runs twice
  with byte-identical output required; timings are recorded but never gate.
- **Monotonic selection**: a candidate may not be worse than the baseline — no
  "getting smarter" is allowed to break existing golden behavior.
- **Open archive (DGM-style)**: failures enter the lineage too
  (`archive/lineage.jsonl`, append-only JSONL anchored by
  `solver_version + corpus_hash`, no nondeterministic fields such as timestamps).

## Layout

```
evolve/
├── Cargo.toml            # separate workspace
├── corpus/               # golden corpus (ground truth; format contract in its README)
│   └── cases/            #   25 cases: outcomes, joints, direct contradictions,
│                         #   conservative nonlinear failures, numerical audit
│                         #   regressions, and tool errors
├── crates/
│   ├── evaluator/        # ansatz-eval: deterministic evaluator (lib + bin)
│   └── harness/          # ansatz-evolve: Proposer/Selector/lineage + manual mode
├── proposals/            # proposal inbox (*.patch + optional *.json metadata)
└── archive/              # lineage.jsonl (runtime artifact, not in git)
```

## Usage (stage A: manual-proposal mode)

```bash
# 1. Evaluate the current solver + record a baseline lineage entry (under evolve/)
cargo run -p ansatz-evolve                          # auto-detects ansatz-cli under ../target
cargo run -p ansatz-eval -- --solver ../target/release/ansatz-cli   # evaluate only, no lineage

# 2. Submit an improvement: drop a unified diff into the inbox
#    (human review, then git apply onto the product workspace)
cp my-improvement.patch proposals/

# 3. After applying, build the new solver in the root workspace and run
#    ansatz-evolve again: full-corpus pass = the proposal may enter the
#    lineage (a kind=proposal record is appended)
```

## Safety rails (non-regressable invariants)

1. **Black-box execution**: candidates only ever run as subprocesses with
   timeouts; the evaluating process loads no evolved code.
2. **Golden-behavior monotonicity**: corpus pass rate may not decrease;
   conflict and redundancy groups are set-equality assertions (extra or missing
   groups both count as regression), continuous values are tolerance
   assertions, and output must be repeatable (two runs, byte-identical).
   Independent geometry checks apply regardless of omitted expectation fields;
   an empty corpus is an evaluation failure, not a perfect score.
3. **Product invariants first**: the root workspace's tests green (including
   parity bit tests) and schema non-drift take priority over any fitness gain —
   enforced by the product CI.
4. **Changing the ground truth is a proposal**: modifying expected.json must go
   through the inbox with a stated rationale; silent drift is forbidden (the
   corpus hash enters the lineage and is traceable).

## Roadmap

| Stage | Content | Status |
|---|---|---|
| **A** | Manual-proposal mode: inbox + deterministic evaluation + monotonic selection + lineage archive | ✅ this skeleton |
| B | Fully automated loop: apply proposal → isolated worktree build → evaluate → select → land | planned |
| C | `LlmProposer`: an LLM generates proposals automatically (the vendor-neutral trait is already in place) | planned |
| D | Corpus co-evolution: use the improved solver to generate harder golden cases — **the improver improves its own test set**, and the loop feeds back into solver evolution itself | planned |

Stage D is the step where this lab crosses from "automated improvement" into RSI
proper: the evaluation benchmark itself becomes an improvement target, driven by
the very solver it improved.

Generated stress tests and held-out real models remain future work. The current
public corpus and checker tests establish concrete regression checks; they do
not constitute a private generalization benchmark. Candidate-generated cases
will also need an independent oracle rather than treating a solver's output as
ground truth.

## Validation

- `cargo test --workspace` from this directory exercises report validation,
  independently recomputed geometry, fabricated convergence, missing fields,
  empty-corpus rejection, subprocess timeout, and the loop skeleton.
- Run `ansatz-eval` against a built candidate to check all 25 public cases. Its
  JSON report records pass/fail details and the corpus hash; acceptance requires
  zero failures.
- The nonlinear coaxial/off-axis-spherical case deliberately expects
  `not_converged`: a stationary residual is not a global contradiction proof.
  Negative distances, incompatible same-anchor targets, and unsatisfied
  constraints on fixed bodies remain directly provable conflicts.
