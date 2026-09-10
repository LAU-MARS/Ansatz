[English](README.md) | 简体中文

# 谱系归档（lineage archive）

`lineage.jsonl`：每行一条 [`LineageRecord`]（见 `crates/harness/src/lib.rs`），
由 `ansatz-evolve` 追加，**不手工编辑**。

- 每条记录的确定性锚点是 `(solver_version, corpus_hash)`——刻意不含时间戳，
  同一状态可复现对齐；
- DGM 式开放归档：失败与被拒绝的候选同样入库（阶段 B 起 `kind=proposal`
  的记录会携带 accept/reject 结论）；
- 本文件与 `lineage.jsonl` 的运行时内容不进 git（见根 .gitignore），
  长期谱系的可信载体是 git 历史本身。
