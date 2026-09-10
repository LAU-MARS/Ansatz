English | [简体中文](README.zh-CN.md)

# Lineage archive

`lineage.jsonl`: one [`LineageRecord`] per line (see
`crates/harness/src/lib.rs`), appended by `ansatz-evolve`, **never hand-edited**.

- Each record's determinism anchor is `(solver_version, corpus_hash)` —
  deliberately no timestamps; the same state can be reproduced and aligned.
- DGM-style open archive: failed and rejected candidates are recorded too
  (from stage B on, `kind=proposal` records carry their accept/reject verdict).
- This file and the runtime contents of `lineage.jsonl` stay out of git (see the
  root .gitignore); the trusted long-term carrier of the lineage is git history
  itself.
