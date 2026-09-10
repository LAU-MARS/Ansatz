//! ansatz-evolve：循环驱动入口（阶段 A = 手动提案模式）。
//!
//! 在 evolve/ 目录下运行：
//!   cargo run -p ansatz-evolve -- [--solver ../target/release/ansatz-cli]
//!
//! 本阶段行为：
//! 1. 以黑盒方式评测当前求解器（子进程 + 超时）；
//! 2. 把基线记录追加进 archive/lineage.jsonl；
//! 3. 扫描 proposals/ 投递箱，列出待处理提案（阶段 B 实现自动 apply→build→eval）；
//! 4. 退出码：0 = 全语料通过；1 = 存在失败。
//!
//! 求解器路径未指定时按 release → debug 顺序自动探测（带 Windows 的 .exe 后缀）。

use std::path::{Path, PathBuf};
use std::process::exit;
use std::time::Duration;

use ansatz_evolve::{
    append_lineage, next_seq, LineageRecord, ManualProposer, Proposer, Selector, ThresholdSelector,
};

fn main() {
    let mut solver_arg: Option<PathBuf> = None;
    let mut corpus = PathBuf::from("corpus/cases");
    let mut archive = PathBuf::from("archive/lineage.jsonl");
    let mut proposals_dir = PathBuf::from("proposals");
    let mut timeout_secs = 10u64;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--solver" => solver_arg = Some(PathBuf::from(args.next().unwrap_or_else(|| usage()))),
            "--corpus" => corpus = PathBuf::from(args.next().unwrap_or_else(|| usage())),
            "--archive" => archive = PathBuf::from(args.next().unwrap_or_else(|| usage())),
            "--proposals" => proposals_dir = PathBuf::from(args.next().unwrap_or_else(|| usage())),
            "--timeout-secs" => {
                timeout_secs = args
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or_else(|| usage());
            }
            "--help" | "-h" => usage(),
            other => {
                eprintln!("未知参数：{other}");
                usage();
            }
        }
    }
    let solver = solver_arg.unwrap_or_else(find_solver);

    // 1) 评测当前求解器（fitness 基线）
    let report = ansatz_eval::run_suite(&solver, &corpus, Duration::from_secs(timeout_secs));
    println!(
        "{}",
        serde_json::to_string_pretty(&report).expect("报告可序列化")
    );

    // 2) 归档基线谱系
    let record = LineageRecord {
        seq: next_seq(&archive),
        kind: "baseline",
        solver_version: report
            .solver
            .version
            .clone()
            .unwrap_or_else(|| "<unknown>".into()),
        corpus_hash: report.corpus.hash,
        passed: report.summary.passed,
        total: report.corpus.cases,
        pass_rate: report.summary.pass_rate,
        note: format!(
            "evaluator={} selector=threshold proposer=manual",
            ansatz_eval::VERSION_TAG
        ),
    };
    if let Err(e) = append_lineage(&archive, &record) {
        eprintln!("警告：谱系写入失败（{}）：{e}", archive.display());
    } else {
        eprintln!(
            "谱系已记录：seq={}, pass_rate={}",
            record.seq, record.pass_rate
        );
    }

    // 3) 扫描投递箱（阶段 A：列出；阶段 B：自动 apply → build → eval → select）
    let mut proposer = ManualProposer::new(&proposals_dir);
    let selector = ThresholdSelector;
    let mut listed = 0;
    while let Some(p) = proposer.next_proposal() {
        listed += 1;
        let accept_hint = if selector.accept(report.summary.pass_rate, 1.0) {
            "可入库（若全语料通过）"
        } else {
            "低于基线，将被拒绝"
        };
        eprintln!(
            "待处理提案 [{}]：{}{}（{}）",
            p.id,
            p.patch_path.display(),
            if p.note.is_empty() {
                String::new()
            } else {
                format!(" — {}", p.note)
            },
            accept_hint
        );
    }
    if listed == 0 {
        eprintln!(
            "投递箱 {} 暂无提案：把 unified diff 放进来（配套同名 .json 可带 note）即可参与下一轮",
            proposals_dir.display()
        );
    }

    if report.summary.failed == 0 {
        exit(0);
    }
    exit(1);
}

/// 依 release → debug 顺序探测根 workspace 构建出的 ansatz-cli。
fn find_solver() -> PathBuf {
    let exe = if cfg!(windows) {
        "ansatz-cli.exe"
    } else {
        "ansatz-cli"
    };
    for dir in [
        "../target/release",
        "../target/debug",
        "target/release",
        "target/debug",
    ] {
        let p = Path::new(dir).join(exe);
        if p.exists() {
            return p;
        }
    }
    eprintln!(
        "找不到 ansatz-cli（探测过 ../target 与 target 的 release/debug）。请用 --solver 指定，或先在根 workspace 执行 cargo build -p ansatz-cli"
    );
    usage();
}

fn usage() -> ! {
    eprintln!(
        "用法：ansatz-evolve [--solver <ansatz-cli>] [--corpus corpus/cases] [--archive archive/lineage.jsonl] [--proposals proposals] [--timeout-secs 10]"
    );
    std::process::exit(2);
}
