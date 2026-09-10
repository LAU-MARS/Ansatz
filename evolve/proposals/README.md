English | [简体中文](README.zh-CN.md)

# Proposal inbox

The entry point of the self-improvement loop (stage A: manual / external delivery).

## Usage

Drop a unified diff named `<proposal-id>.patch` into this directory; an optional
sibling `<proposal-id>.json` carries metadata:

```json
{ "note": "fix redundant-group reporting for case-x: switch to Jacobian rank analysis" }
```

Running `cargo run -p ansatz-evolve` lists pending proposals in filename order.

- Stage A (now): proposals are human-reviewed and applied manually — `git apply`
  onto the product workspace, build there, then run `ansatz-evolve` here; a full
  corpus pass admits it into the lineage.
- Stage B (planned): the loop automates apply → isolated worktree build →
  evaluate → select → archive; this directory becomes a pure delivery interface.
- Stage C (planned): an `LlmProposer` generates proposals directly, bypassing
  this directory (but keeping the same format).

## Proposal discipline

1. Changing the golden corpus (expected.json) is also a proposal, and its note
   must state *why the ground truth itself* must change;
2. proposals must not touch the safety rails (see ../README.md);
3. patches must apply cleanly onto current main (conflicts are discarded; no
   three-way merges).
