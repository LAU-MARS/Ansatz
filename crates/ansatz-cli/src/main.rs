//! Ansatz CLI 壳：stdin JSON → stdout JSON。
//!
//! 只做类型转换与错误映射，不含数学逻辑（求解全部在 ansatz-core）。
//!
//! ## 退出码契约
//!
//! | 退出码 | 含义 | 输出位置 |
//! |--------|------|----------|
//! | 0 | `Converged` | 报告 JSON → stdout |
//! | 1 | 求解层失败（`Inconsistent` / `Overconstrained` / `Underconstrained` / `MaxIterations`），完整诊断照常输出 | 报告 JSON → stdout |
//! | 2 | 工具错误（非法 JSON / 模型无效 / 能力未实现） | 错误 JSON → stderr |
//!
//! 「求解失败」与「工具错误」严格区分：前者是求解器对几何事实的陈述，
//! 消费方仍应读取 stdout 的诊断；后者是输入或环境问题。

use ansatz_core::{solve, Model, SolveError, SolveOutcome};
use clap::{Parser, Subcommand, ValueEnum};
use std::io::{Read, Write};

#[derive(Parser)]
#[command(
    name = "ansatz-cli",
    version,
    about = "Ansatz 几何约束求解器 CLI：stdin JSON -> stdout JSON"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// 从 stdin 读取模型 JSON，求解并把报告 JSON 打到 stdout
    Solve,
    /// 导出 JSON Schema（与 schema/ 目录下已提交契约比对，CI 防漂移）
    ExportSchema {
        /// 导出哪份契约
        #[arg(value_enum)]
        which: SchemaOutput,
    },
}

#[derive(Clone, Copy, ValueEnum)]
enum SchemaOutput {
    /// 输入模型契约（Model）
    Model,
    /// 输出报告契约（SolveReport，含 Diagnostics）
    SolveReport,
}

fn main() {
    let cli = Cli::parse();
    let code = match cli.command {
        Command::Solve => cmd_solve(),
        Command::ExportSchema { which } => cmd_export_schema(which),
    };
    std::process::exit(code);
}

fn cmd_solve() -> i32 {
    let mut input = String::new();
    if let Err(e) = std::io::stdin().read_to_string(&mut input) {
        return tool_error("io_error", &format!("读取 stdin 失败：{e}"));
    }
    let model: Model = match serde_json::from_str(&input) {
        Ok(model) => model,
        Err(e) => {
            return tool_error(
                "invalid_json",
                &format!("输入不是合法的模型 JSON（应符合 schema/model.schema.json）：{e}"),
            )
        }
    };
    match solve(&model) {
        Ok(report) => {
            match serde_json::to_string_pretty(&report) {
                Ok(json) => {
                    // stdout 可能被消费方管道化，写完后立刻冲刷
                    let _ = writeln!(std::io::stdout(), "{json}");
                }
                Err(e) => return tool_error("serialization", &format!("报告序列化失败：{e}")),
            }
            match report.outcome {
                SolveOutcome::Converged => 0,
                _ => 1,
            }
        }
        Err(e) => tool_error(solve_error_kind(&e), &e.human_message()),
    }
}

fn solve_error_kind(e: &SolveError) -> &'static str {
    e.kind_name()
}

fn tool_error(kind: &str, message: &str) -> i32 {
    // 工具错误写 stderr，与 stdout 的求解报告彻底分离
    let payload = serde_json::json!({ "kind": kind, "message": message });
    let _ = writeln!(std::io::stderr(), "{payload}");
    2
}

fn cmd_export_schema(which: SchemaOutput) -> i32 {
    let schema_json = match which {
        SchemaOutput::Model => serde_json::to_string_pretty(&schemars::schema_for!(Model)),
        SchemaOutput::SolveReport => {
            serde_json::to_string_pretty(&schemars::schema_for!(ansatz_core::SolveReport))
        }
    };
    match schema_json {
        Ok(json) => {
            println!("{json}");
            0
        }
        Err(e) => tool_error("serialization", &format!("Schema 序列化失败：{e}")),
    }
}
