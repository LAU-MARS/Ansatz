//! 确定性评测器：候选求解器 × 黄金语料 → 结构化指标。
//!
//! 这是自进化闭环的 **fitness 来源**，因此把确定性放在第一位：
//! - 求解器只以黑盒方式访问（`ansatz-cli solve` 子进程 + 超时 + stdin/stdout JSON），
//!   演化产物绝不在本进程内执行；
//! - 判定只依赖确定性事实（退出码 / outcome / DOF / 位模式 / 约束组 / 建议动作），
//!   耗时只记录、不参与判定（计时天然非确定，供人观察与阶段B之后加权使用）；
//! - 语料目录有内容哈希（FNV-64），谱系记录据此可复现对齐。
//!
//! 黄金 case 的期望值格式见 `evolve/corpus/README.md`；缺省字段即通配。

use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// 评测器版本标签，进谱系记录以便追溯「用哪一版判定规则得到的 fitness」。
pub const VERSION_TAG: &str = env!("CARGO_PKG_VERSION");

/// 单个 case 的期望（全部字段可选；未写出的字段视为通配）。
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Expected {
    #[serde(default)]
    pub description: Option<String>,
    /// 期望的求解结局（converged / underconstrained / ...）。
    #[serde(default)]
    pub outcome: Option<String>,
    #[serde(default)]
    pub dof_total: Option<u32>,
    #[serde(default)]
    pub dof_remaining: Option<u32>,
    /// 期望的实体终态。x/y 为 IEEE 754 位模式的十六进制字符串（"0x…"），
    /// 与产品 tests/parity 的约定一致——位级比较，不是 epsilon。
    #[serde(default)]
    pub entities: Vec<ExpectedEntity>,
    /// 期望的冗余/冲突约束组（集合相等：多报、漏报都算失败）。
    #[serde(default)]
    pub redundant_constraint_ids: Vec<Vec<u32>>,
    /// 期望出现的建议动作（子集匹配：报出更多、更聪明的建议不算失败）。
    /// 格式："remove_constraint:2" / "relax_constraint:7" / "add_constraint:1"。
    #[serde(default)]
    pub suggestion_actions: Vec<String>,
    /// 期望的工具错误 kind（与 outcome 互斥；出现即期望 exit 2 + stderr JSON）。
    #[serde(default)]
    pub tool_error_kind: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ExpectedEntity {
    pub id: u32,
    #[serde(default)]
    pub x: Option<String>,
    #[serde(default)]
    pub y: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SolverInfo {
    pub path: PathBuf,
    pub version: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct CorpusInfo {
    pub dir: PathBuf,
    /// 语料内容哈希（排序文件序列的 FNV-64），谱系可复现对齐的锚点。
    pub hash: u64,
    pub cases: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct Summary {
    pub passed: usize,
    pub failed: usize,
    pub pass_rate: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct CaseReport {
    pub id: String,
    /// pass / fail / timeout
    pub status: &'static str,
    pub exit_code: Option<i32>,
    pub duration_ms: u128,
    pub failures: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SuiteReport {
    pub solver: SolverInfo,
    pub corpus: CorpusInfo,
    pub summary: Summary,
    pub cases: Vec<CaseReport>,
}

/// 运行整套语料。返回完整报告；调用方根据 `summary.failed == 0` 决定退出码。
pub fn run_suite(solver: &Path, corpus: &Path, timeout: Duration) -> SuiteReport {
    let mut case_dirs: Vec<PathBuf> = std::fs::read_dir(corpus)
        .unwrap_or_else(|e| panic!("无法读取语料目录 {}: {e}", corpus.display()))
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    case_dirs.sort(); // 确定性顺序

    let version = probe_version(solver);
    let mut cases = Vec::with_capacity(case_dirs.len());
    for dir in &case_dirs {
        cases.push(run_case(solver, dir, timeout));
    }
    let passed = cases.iter().filter(|c| c.status == "pass").count();
    let failed = cases.len() - passed;
    SuiteReport {
        solver: SolverInfo {
            path: solver.to_path_buf(),
            version,
        },
        corpus: CorpusInfo {
            dir: corpus.to_path_buf(),
            hash: corpus_hash(corpus),
            cases: cases.len(),
        },
        summary: Summary {
            passed,
            failed,
            pass_rate: if cases.is_empty() {
                1.0
            } else {
                passed as f64 / cases.len() as f64
            },
        },
        cases,
    }
}

/// 语料目录的内容哈希：按排序后的相对路径 + 文件字节做 FNV-64。
/// （FNV 在此自己实现：不引入依赖，且跨 rustc 版本稳定——
/// std 的 DefaultHasher 不保证跨版本稳定，不能进谱系。）
pub fn corpus_hash(corpus: &Path) -> u64 {
    fn fnv64(state: &mut u64, bytes: &[u8]) {
        for &b in bytes {
            *state ^= u64::from(b);
            *state = state.wrapping_mul(0x100000001b3);
        }
    }
    fn walk(dir: &Path, base: &Path, state: &mut u64, files: &mut Vec<PathBuf>) {
        let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)
            .map(|rd| rd.filter_map(|e| e.ok()).map(|e| e.path()).collect())
            .unwrap_or_default();
        entries.sort();
        for p in entries {
            if p.is_dir() {
                walk(&p, base, state, files);
            } else {
                let rel = p.strip_prefix(base).unwrap_or(&p);
                fnv64(state, rel.to_string_lossy().as_bytes());
                fnv64(state, b"\0");
                files.push(p);
            }
        }
    }
    let mut state = 0xcbf29ce484222325u64;
    let mut files = Vec::new();
    walk(corpus, corpus, &mut state, &mut files);
    for p in files {
        if let Ok(bytes) = std::fs::read(&p) {
            fnv64(&mut state, &bytes);
        }
        fnv64(&mut state, b"\0");
    }
    state
}

fn probe_version(solver: &Path) -> Option<String> {
    let out = Command::new(solver).arg("--version").output().ok()?;
    let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
    if s.is_empty() {
        None
    } else {
        Some(s)
    }
}

fn run_case(solver: &Path, case_dir: &Path, timeout: Duration) -> CaseReport {
    let id = case_dir
        .file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "<unnamed>".into());
    let model = match std::fs::read_to_string(case_dir.join("model.json")) {
        Ok(m) => m,
        Err(e) => {
            return CaseReport {
                id,
                status: "fail",
                exit_code: None,
                duration_ms: 0,
                failures: vec![format!("读取 model.json 失败：{e}")],
            }
        }
    };
    let expected: Expected = match std::fs::read_to_string(case_dir.join("expected.json")) {
        Ok(s) => match serde_json::from_str(&s) {
            Ok(e) => e,
            Err(e) => {
                return CaseReport {
                    id,
                    status: "fail",
                    exit_code: None,
                    duration_ms: 0,
                    failures: vec![format!("解析 expected.json 失败：{e}")],
                }
            }
        },
        Err(e) => {
            return CaseReport {
                id,
                status: "fail",
                exit_code: None,
                duration_ms: 0,
                failures: vec![format!("读取 expected.json 失败：{e}")],
            }
        }
    };

    let start = Instant::now();
    let (exit_code, stdout, stderr, timed_out) = run_solver(solver, &model, timeout);
    let duration_ms = start.elapsed().as_millis();

    let mut failures = Vec::new();
    if timed_out {
        failures.push(format!("超时（>{timeout:?}）后被终止"));
    }
    if failures.is_empty() {
        check(&expected, exit_code, &stdout, &stderr, &mut failures);
    }
    let status = if failures.is_empty() { "pass" } else { "fail" };
    CaseReport {
        id,
        status,
        exit_code,
        duration_ms,
        failures,
    }
}

/// 以子进程 + 超时运行求解器（安全护栏：演化产物绝不在本进程内执行）。
/// 输出用独立线程读取，避免管道缓冲写满导致的死锁。
fn run_solver(
    solver: &Path,
    model: &str,
    timeout: Duration,
) -> (Option<i32>, String, String, bool) {
    let mut child = match Command::new(solver)
        .arg("solve")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
    {
        Ok(c) => c,
        Err(e) => return (None, String::new(), format!("启动求解器失败：{e}"), false),
    };
    if let Some(mut stdin) = child.stdin.take() {
        let _ = stdin.write_all(model.as_bytes());
    } // drop 关闭 stdin → 求解器读到 EOF

    let mut stdout_pipe = child.stdout.take().unwrap();
    let mut stderr_pipe = child.stderr.take().unwrap();
    let out_t = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stdout_pipe.read_to_string(&mut s);
        s
    });
    let err_t = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stderr_pipe.read_to_string(&mut s);
        s
    });

    let deadline = Instant::now() + timeout;
    let mut timed_out = false;
    let exit_code = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status.code(),
            Ok(None) => {
                if Instant::now() >= deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    timed_out = true;
                    break None;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(_) => break None,
        }
    };
    let stdout = out_t.join().unwrap_or_default();
    let stderr = err_t.join().unwrap_or_default();
    (exit_code, stdout, stderr, timed_out)
}

fn check(
    expected: &Expected,
    exit_code: Option<i32>,
    stdout: &str,
    stderr: &str,
    failures: &mut Vec<String>,
) {
    if let Some(kind) = &expected.tool_error_kind {
        // 工具错误路径：exit 2 + stderr 是 {"kind": ...}
        if exit_code != Some(2) {
            failures.push(format!("期望工具错误（exit 2），实际退出码 {exit_code:?}"));
        }
        match serde_json::from_str::<serde_json::Value>(stderr.trim()) {
            Ok(err) => {
                if err["kind"].as_str() != Some(kind.as_str()) {
                    failures.push(format!(
                        "期望错误 kind `{kind}`，实际 `{}`",
                        err["kind"].as_str().unwrap_or("<非字符串>")
                    ));
                }
            }
            Err(e) => failures.push(format!("stderr 不是合法 JSON：{e}")),
        }
        return;
    }

    // 求解层路径：exit 0 ⇔ converged；诊断照常输出到 stdout
    if !matches!(exit_code, Some(0) | Some(1)) {
        failures.push(format!(
            "期望求解层退出码 0/1，实际 {exit_code:?}（stderr: {}）",
            stderr.trim()
        ));
        return;
    }
    let report: serde_json::Value = match serde_json::from_str(stdout) {
        Ok(v) => v,
        Err(e) => {
            failures.push(format!("stdout 不是合法的报告 JSON：{e}"));
            return;
        }
    };
    let converged = report["outcome"] == "converged";
    if (exit_code == Some(0)) != converged {
        failures.push(format!(
            "退出码与 outcome 矛盾：exit={exit_code:?}, outcome={}",
            report["outcome"]
        ));
    }
    if let Some(want) = &expected.outcome {
        if report["outcome"].as_str() != Some(want.as_str()) {
            failures.push(format!(
                "期望 outcome `{want}`，实际 `{}`",
                report["outcome"].as_str().unwrap_or("<非字符串>")
            ));
        }
    }
    let diag = &report["diagnostics"];
    for (key, want) in [
        ("dof_total", expected.dof_total),
        ("dof_remaining", expected.dof_remaining),
    ] {
        if let Some(want) = want {
            let got = diag[key].as_u64();
            if got != Some(u64::from(want)) {
                failures.push(format!("期望 {key}={want}，实际 {got:?}"));
            }
        }
    }
    for ent in &expected.entities {
        let found = report["entities"].as_array().and_then(|arr| {
            arr.iter()
                .find(|e| e["id"].as_u64() == Some(u64::from(ent.id)))
        });
        let Some(found) = found else {
            failures.push(format!("报告中找不到实体 id={}", ent.id));
            continue;
        };
        for (axis, want_hex) in [("x", &ent.x), ("y", &ent.y)] {
            let Some(want_hex) = want_hex else { continue };
            let geometry = &found["geometry"];
            if geometry["type"].as_str() != Some("point2") {
                failures.push(format!(
                    "实体 {} 期望 point2，实际 `{}`",
                    ent.id,
                    geometry["type"].as_str().unwrap_or("<非字符串>")
                ));
                continue;
            }
            let Some(got) = geometry[axis].as_f64() else {
                failures.push(format!("实体 {} 的 {axis} 不是数值", ent.id));
                continue;
            };
            let want = hex_to_f64(want_hex);
            if got.to_bits() != want.to_bits() {
                failures.push(format!(
                    "实体 {} 的 {axis} 位模式不符：期望 {want_hex}（{want}），实际 {:#018x}（{got}）",
                    ent.id,
                    got.to_bits()
                ));
            }
        }
    }
    // 冗余/冲突组：集合相等（多报漏报都算行为变化）
    if !expected.redundant_constraint_ids.is_empty() {
        let mut got: Vec<Vec<u64>> = diag["redundant_constraints"]
            .as_array()
            .map(|arr| {
                arr.iter()
                    .filter_map(|g| {
                        g["constraint_ids"]
                            .as_array()
                            .map(|ids| ids.iter().filter_map(|v| v.as_u64()).collect())
                    })
                    .collect()
            })
            .unwrap_or_default();
        let mut want: Vec<Vec<u64>> = expected
            .redundant_constraint_ids
            .iter()
            .map(|g| g.iter().map(|&id| u64::from(id)).collect())
            .collect();
        for v in got.iter_mut() {
            v.sort_unstable();
        }
        for v in want.iter_mut() {
            v.sort_unstable();
        }
        got.sort_unstable();
        want.sort_unstable();
        if got != want {
            failures.push(format!("冗余/冲突约束组不符：期望 {want:?}，实际 {got:?}"));
        }
    }
    // 建议动作：子集匹配（允许未来报出更好的建议）
    if !expected.suggestion_actions.is_empty() {
        let actual: Vec<String> = diag["suggestions"]
            .as_array()
            .map(|arr| {
                arr.iter()
                    .map(|s| {
                        let tag = s["action"].as_str().unwrap_or("<?>").to_string();
                        if let Some(id) = s["id"].as_u64() {
                            format!("{tag}:{id}")
                        } else if let Some(dof) = s["dof_remaining"].as_u64() {
                            format!("{tag}:{dof}")
                        } else {
                            tag
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
        for want in &expected.suggestion_actions {
            if !actual.iter().any(|a| a == want) {
                failures.push(format!("期望出现建议动作 `{want}`，实际建议 {actual:?}"));
            }
        }
    }
}

fn hex_to_f64(s: &str) -> f64 {
    let bits = u64::from_str_radix(s.trim_start_matches("0x"), 16)
        .unwrap_or_else(|e| panic!("期望文件中的位模式非法 `{s}`：{e}"));
    f64::from_bits(bits)
}
