//! ansatz-eval：命令行入口。评测候选求解器并打印 JSON 报告。
//!
//! 用法（在 evolve/ 下）：
//!   cargo run -p ansatz-eval -- --solver ../target/release/ansatz-cli[.exe] \
//!                               --corpus corpus/cases [--timeout-secs 10]
//! 退出码：0 = 全部通过；1 = 存在失败/超时。

use std::path::PathBuf;
use std::process::exit;
use std::time::Duration;

use ansatz_eval::run_suite;

fn main() {
    let mut solver: Option<PathBuf> = None;
    let mut corpus = PathBuf::from("corpus/cases");
    let mut timeout_secs = 10u64;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--solver" => solver = Some(PathBuf::from(args.next().unwrap_or_else(|| usage()))),
            "--corpus" => corpus = PathBuf::from(args.next().unwrap_or_else(|| usage())),
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
    let Some(solver) = solver else {
        eprintln!("缺少 --solver（指向 ansatz-cli 可执行文件）");
        usage();
    };
    let report = run_suite(&solver, &corpus, Duration::from_secs(timeout_secs));
    println!(
        "{}",
        serde_json::to_string_pretty(&report).expect("报告可序列化")
    );
    if report.summary.failed == 0 {
        exit(0);
    }
    exit(1);
}

fn usage() -> ! {
    eprintln!(
        "用法：ansatz-eval --solver <ansatz-cli 路径> [--corpus corpus/cases] [--timeout-secs 10]"
    );
    std::process::exit(2);
}
