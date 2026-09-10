//! 自进化循环骨架（RSI 跑道的「机器」部分）。
//!
//! 阶段定位：**A——手动提案模式**。闭环中「提案应用 → 构建 → 评测 → 选择 → 归档」
//! 的接口与数据结构在本骨架定型，其中评测与归档已可运行；提案来源为
//! `proposals/` 投递箱（人或外部 LLM 脚本把 unified diff 放进来），
//! patch 的自动应用与构建属于阶段 B（见 evolve/README.md 路线图）。
//!
//! 设计原则：
//! - **不绑定 LLM 供应商**：[`Proposer`] 是本地 trait，阶段 C 才出现
//!   `LlmProposer` 实现；在那之前 `ManualProposer` 足够支撑人机协作进化。
//! - **全量归档（DGM 式开放归档）**：失败者也被记录，谱系是 JSONL 追加文件，
//!   每条带语料哈希与求解器版本，可复现对齐。
//! - **单调选择**：[`ThresholdSelector`] 只接受不劣于基线的候选——
//!   任何 fitness 的改动都不许破坏已有黄金行为。

use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};

use serde::Serialize;

/// 一次改进提案（阶段 A：人/外部脚本投递的 unified diff）。
#[derive(Debug, Clone)]
pub struct Proposal {
    pub id: String,
    pub patch_path: PathBuf,
    /// 附带说明（同名 .json 的 note 字段，缺省为空）。
    pub note: String,
}

/// 提案者抽象。阶段 A = ManualProposer；阶段 C = LlmProposer（ vender 无关）。
pub trait Proposer {
    fn name(&self) -> &'static str;
    /// 依序取出下一个提案；None 表示本轮没有新提案。
    fn next_proposal(&mut self) -> Option<Proposal>;
}

/// 手动提案者：扫描投递箱目录中的 `*.patch`（同名 `.json` 可带 note 元数据）。
pub struct ManualProposer {
    dir: PathBuf,
    seen: Vec<String>,
}

impl ManualProposer {
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self {
            dir: dir.into(),
            seen: Vec::new(),
        }
    }
}

impl Proposer for ManualProposer {
    fn name(&self) -> &'static str {
        "manual"
    }

    fn next_proposal(&mut self) -> Option<Proposal> {
        let entries = std::fs::read_dir(&self.dir).ok()?;
        let mut patches: Vec<PathBuf> = entries
            .filter_map(|e| e.ok())
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|ext| ext == "patch"))
            .collect();
        patches.sort(); // 确定性：按文件名顺序消费
        for patch in patches {
            let id = patch.file_stem()?.to_string_lossy().to_string();
            if self.seen.contains(&id) {
                continue;
            }
            self.seen.push(id.clone());
            let note = std::fs::read_to_string(patch.with_extension("json"))
                .ok()
                .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
                .and_then(|v| v["note"].as_str().map(str::to_string))
                .unwrap_or_default();
            return Some(Proposal {
                id,
                patch_path: patch,
                note,
            });
        }
        None
    }
}

/// 谱系记录（JSONL 的一行）。刻意不含时间戳等非确定字段：
/// 谱系的确定性锚点是 (solver_version, corpus_hash)。
#[derive(Debug, Clone, Serialize)]
pub struct LineageRecord {
    /// 单调序号（= 文件中第几行）。
    pub seq: u64,
    /// baseline / proposal / corpus_update
    pub kind: &'static str,
    pub solver_version: String,
    pub corpus_hash: u64,
    pub passed: usize,
    pub total: usize,
    pub pass_rate: f64,
    pub note: String,
}

/// 选择器抽象：决定候选是否入库。
pub trait Selector {
    fn name(&self) -> &'static str;
    fn accept(&self, baseline_pass_rate: f64, candidate_pass_rate: f64) -> bool;
}

/// 单调阈值选择：候选通过率不得低于基线。
pub struct ThresholdSelector;

impl Selector for ThresholdSelector {
    fn name(&self) -> &'static str {
        "threshold"
    }

    fn accept(&self, baseline: f64, candidate: f64) -> bool {
        candidate >= baseline
    }
}

/// 追加一条谱系记录到 JSONL 文件（不存在则创建，含父目录）。
pub fn append_lineage(path: &Path, record: &LineageRecord) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut f = OpenOptions::new().create(true).append(true).open(path)?;
    let line = serde_json::to_string(record).expect("谱系记录可序列化");
    writeln!(f, "{line}")
}

/// 读取谱系当前长度，作为下一条记录的序号（空文件 → 1）。
pub fn next_seq(path: &Path) -> u64 {
    std::fs::read_to_string(path)
        .map(|s| s.lines().filter(|l| !l.trim().is_empty()).count() as u64)
        .unwrap_or(0)
        + 1
}
