[English](README.md) | 简体中文

# 提案投递箱（proposal inbox）

自进化闭环的入口（阶段 A：手动/外部投递）。

## 用法

把一个 unified diff 命名为 `<提案id>.patch` 放在本目录；可选同名
`<提案id>.json` 携带元数据：

```json
{ "note": "修复 case-x 的冗余组报告：改用雅可比秩分析" }
```

运行 `cargo run -p ansatz-evolve` 会按文件名顺序列出待处理提案。

- 阶段 A（现在）：提案由人审阅、手动应用——`git apply` 后在根 workspace
  构建、回到本目录跑 `ansatz-evolve`，全语料通过即入谱系。
- 阶段 B（计划）：循环自动完成 apply → 隔离 worktree 构建 → 评测 →
  选择 → 归档，本目录成为纯投递接口。
- 阶段 C（计划）：`LlmProposer` 直接生成提案，不经本目录（但格式相同）。

## 提案纪律

1. 改黄金集（expected.json）也是提案，且必须在 note 里说明
   「为什么地面真值本身要改」；
2. 不得在提案中触碰安全护栏（见 ../README.zh-CN.md）；
3. patch 必须可干净应用于当前 main（冲突即废弃，不做三方合并）。
