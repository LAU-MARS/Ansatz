//! 确定性评测器：候选求解器 × 黄金语料 → 结构化指标。
//!
//! 这是自进化闭环的 **fitness 来源**，判定分两层：
//! - **行为**：退出码 / outcome / DOF / 冲突组 / 冗余组 / 建议动作做精确断言，
//!   实体终态与关节变量按容差断言。语料冻结的是「求解器应当做什么」，
//!   不冻结数值噪声——更快、更准的迭代策略不会被判为回退。
//! - **确定性**：每个 case 运行两次，stdout / stderr / 退出码必须逐字节相同。
//!   跨平台逐位一致由产品仓库的 parity 测试在 CI 架构矩阵上负责。
//!
//! 其余约定：求解器只以黑盒方式访问（`ansatz-cli solve` 子进程 + 超时），
//! 演化产物绝不在本进程内执行；耗时只记录、不参与判定；语料目录有内容
//! 哈希（FNV-64），谱系记录据此可复现对齐。
//!
//! 黄金 case 的期望值格式见 `evolve/corpus/README.md`；缺省字段即通配。

use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

pub mod geometry_check;

/// 评测器版本标签，进谱系记录以便追溯「用哪一版判定规则得到的 fitness」。
pub const VERSION_TAG: &str = env!("CARGO_PKG_VERSION");

/// 实体终态与关节变量的缺省比较容差（绝对值）。
pub const DEFAULT_VALUE_TOLERANCE: f64 = 1e-7;

/// 单个 case 的期望（全部字段可选；未写出的字段视为通配）。
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Expected {
    #[serde(default)]
    pub description: Option<String>,
    /// 期望的求解结局（converged / inconsistent / not_converged）。
    #[serde(default)]
    pub outcome: Option<String>,
    #[serde(default)]
    pub dof_total: Option<u32>,
    #[serde(default)]
    pub dof_remaining: Option<u32>,
    /// 期望的实体终态（按 `value_tolerance` 比较）。只应对解唯一的模型写出：
    /// 欠约束模型的特解随迭代策略合法变化。
    #[serde(default)]
    pub entities: Vec<ExpectedEntity>,
    /// 期望的关节变量（按 `value_tolerance` 比较）。
    #[serde(default)]
    pub joints: Vec<ExpectedJoint>,
    /// 实体与关节的比较容差；缺省 [`DEFAULT_VALUE_TOLERANCE`]。
    #[serde(default)]
    pub value_tolerance: Option<f64>,
    /// 期望的冲突组（集合相等；`[]` 断言没有冲突）。
    #[serde(default)]
    pub conflict_constraint_ids: Option<Vec<Vec<u32>>>,
    /// 期望的冗余组（集合相等；`[]` 断言没有冗余）。
    #[serde(default)]
    pub redundant_constraint_ids: Option<Vec<Vec<u32>>>,
    /// 期望出现的建议动作（子集匹配：报出更多、更聪明的建议不算失败）。
    /// 格式："remove_constraint:2" / "relax_constraint:7" / "add_constraint:1"。
    #[serde(default)]
    pub suggestion_actions: Vec<String>,
    /// 期望的工具错误 kind（与 outcome 互斥；出现即期望 exit 2 + stderr JSON）。
    #[serde(default)]
    pub tool_error_kind: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedEntity {
    pub id: u32,
    /// point2 终态。
    #[serde(default)]
    pub x: Option<f64>,
    #[serde(default)]
    pub y: Option<f64>,
    /// rigid3 终态（平移 3 + 旋转向量 3）。
    #[serde(default)]
    pub pose: Option<ExpectedPose>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedPose {
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedJoint {
    pub constraint_id: u32,
    pub value: f64,
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

    let version = probe_version(solver, timeout);
    let mut cases = Vec::with_capacity(case_dirs.len());
    for dir in &case_dirs {
        cases.push(run_case(solver, dir, timeout));
    }
    if cases.is_empty() {
        cases.push(CaseReport {
            id: "<empty-corpus>".into(),
            status: "fail",
            exit_code: None,
            duration_ms: 0,
            failures: vec!["语料目录没有任何 case，不能评定候选求解器".into()],
        });
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
            cases: case_dirs.len(),
        },
        summary: Summary {
            passed,
            failed,
            pass_rate: passed as f64 / cases.len() as f64,
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

fn probe_version(solver: &Path, timeout: Duration) -> Option<String> {
    let (exit_code, stdout, _, timed_out) = run_process(solver, "--version", None, timeout);
    let s = stdout.trim().to_string();
    if timed_out || exit_code != Some(0) || s.is_empty() {
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
        let (exit2, stdout2, stderr2, timed_out2) = run_solver(solver, &model, timeout);
        if timed_out2 {
            failures.push(format!("第二次运行超时（>{timeout:?}）"));
        } else if (exit2, &stdout2, &stderr2) != (exit_code, &stdout, &stderr) {
            failures.push(String::from(
                "非确定性输出：同一输入运行两次，退出码或 stdout/stderr 不完全相同",
            ));
        }
    }
    if failures.is_empty() {
        check(
            &expected,
            &model,
            exit_code,
            &stdout,
            &stderr,
            &mut failures,
        );
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
    run_process(solver, "solve", Some(model), timeout)
}

fn run_process(
    solver: &Path,
    argument: &str,
    input: Option<&str>,
    timeout: Duration,
) -> (Option<i32>, String, String, bool) {
    let mut child = match Command::new(solver)
        .arg(argument)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
    {
        Ok(c) => c,
        Err(e) => return (None, String::new(), format!("启动求解器失败：{e}"), false),
    };
    let stdin_pipe = child.stdin.take();
    let mut stdout_pipe = child.stdout.take().unwrap();
    let mut stderr_pipe = child.stderr.take().unwrap();
    // A candidate may never read stdin. Writing a model larger than the pipe
    // buffer synchronously would prevent the timeout loop from ever starting.
    let input = input.map(str::to_owned);
    let in_t = std::thread::spawn(move || {
        if let Some(mut stdin) = stdin_pipe {
            if let Some(input) = input {
                let _ = stdin.write_all(input.as_bytes());
            }
        } // drop closes stdin so a well-behaved solver sees EOF.
    });
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
    let _ = in_t.join();
    let stdout = out_t.join().unwrap_or_default();
    let stderr = err_t.join().unwrap_or_default();
    (exit_code, stdout, stderr, timed_out)
}

fn check(
    expected: &Expected,
    model: &str,
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
        if !stdout.trim().is_empty() {
            failures.push("工具错误同时在 stdout 输出了报告".into());
        }
        match serde_json::from_str::<ansatz_core::ErrorPayload>(stderr.trim()) {
            Ok(err) => {
                if err.kind != *kind {
                    failures.push(format!("期望错误 kind `{kind}`，实际 `{}`", err.kind));
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
    if !stderr.trim().is_empty() {
        failures.push(format!("求解层报告同时写出 stderr：{}", stderr.trim()));
    }
    if let Err(errors) = geometry_check::parse_and_validate_report(model, stdout) {
        failures.extend(errors);
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
    let tol = expected.value_tolerance.unwrap_or(DEFAULT_VALUE_TOLERANCE);
    if !tol.is_finite() || tol < 0.0 {
        failures.push(format!("期望文件的 value_tolerance 无效：{tol}"));
        return;
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
        let geometry = &found["geometry"];
        match geometry["type"].as_str() {
            Some("point2") => {
                for (axis, want) in [("x", ent.x), ("y", ent.y)] {
                    let Some(want) = want else { continue };
                    let Some(got) = geometry[axis].as_f64() else {
                        failures.push(format!("实体 {} 的 {axis} 不是数值", ent.id));
                        continue;
                    };
                    check_close(
                        &format!("实体 {} 的 {axis}", ent.id),
                        got,
                        want,
                        tol,
                        failures,
                    );
                }
            }
            Some("rigid3") => {
                let Some(want_pose) = &ent.pose else {
                    failures.push(format!(
                        "实体 {} 是 rigid3 但期望文件缺 pose 六元组",
                        ent.id
                    ));
                    continue;
                };
                let pose = &geometry["pose"];
                let fields = [
                    ("tx", pose["translation"]["x"].as_f64(), want_pose.tx),
                    ("ty", pose["translation"]["y"].as_f64(), want_pose.ty),
                    ("tz", pose["translation"]["z"].as_f64(), want_pose.tz),
                    ("rx", pose["rotation"]["vector"][0].as_f64(), want_pose.rx),
                    ("ry", pose["rotation"]["vector"][1].as_f64(), want_pose.ry),
                    ("rz", pose["rotation"]["vector"][2].as_f64(), want_pose.rz),
                ];
                for (field, got, want) in fields {
                    let Some(got) = got else {
                        failures.push(format!("实体 {} 的 {field} 不是数值", ent.id));
                        continue;
                    };
                    check_close(
                        &format!("实体 {} 的 {field}", ent.id),
                        got,
                        want,
                        tol,
                        failures,
                    );
                }
            }
            other => failures.push(format!(
                "实体 {} 期望 point2/rigid3，实际 `{}`",
                ent.id,
                other.unwrap_or("<非字符串>")
            )),
        }
    }
    for want in &expected.joints {
        let got = diag["joints"].as_array().and_then(|arr| {
            arr.iter()
                .find(|j| j["constraint_id"].as_u64() == Some(u64::from(want.constraint_id)))
                .and_then(|j| j["value"].as_f64())
        });
        match got {
            Some(got) => check_close(
                &format!("关节 {} 的变量", want.constraint_id),
                got,
                want.value,
                tol,
                failures,
            ),
            None => failures.push(format!("报告中找不到关节 {} 的状态", want.constraint_id)),
        }
    }
    // 冲突组 / 冗余组：集合相等（多报漏报都算行为变化）
    for (key, label, want) in [
        ("conflicts", "冲突组", &expected.conflict_constraint_ids),
        (
            "redundant_constraints",
            "冗余组",
            &expected.redundant_constraint_ids,
        ),
    ] {
        let Some(want) = want else { continue };
        let got = normalized_groups(diag[key].as_array().map(|arr| {
            arr.iter()
                .filter_map(|g| {
                    g["constraint_ids"]
                        .as_array()
                        .map(|ids| ids.iter().filter_map(|v| v.as_u64()).collect())
                })
                .collect()
        }));
        let want = normalized_groups(Some(
            want.iter()
                .map(|g| g.iter().map(|&id| u64::from(id)).collect())
                .collect(),
        ));
        if got != want {
            failures.push(format!("{label}不符：期望 {want:?}，实际 {got:?}"));
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

/// 组内与组间都排序，使比较与报告顺序无关。
fn normalized_groups(groups: Option<Vec<Vec<u64>>>) -> Vec<Vec<u64>> {
    let mut groups = groups.unwrap_or_default();
    for g in groups.iter_mut() {
        g.sort_unstable();
    }
    groups.sort_unstable();
    groups
}

/// 容差断言：|got − want| ≤ tol。
fn check_close(what: &str, got: f64, want: f64, tol: f64, failures: &mut Vec<String>) {
    let diff = (got - want).abs();
    if diff.is_nan() || diff > tol {
        failures.push(format!(
            "{what} 超出容差：期望 {want}，实际 {got}（差 {diff:e} > {tol:e}）"
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: &str = r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],"constraints":[{"id":7,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}"#;

    fn report(x: f64, y: f64) -> String {
        serde_json::json!({
            "outcome":"converged",
            "entities":[{"id":1,"geometry":{"type":"point2","x":x,"y":y}}],
            "diagnostics":{
                "dof_total":2,"dof_remaining":1,"iterations":0,"conflicts":[],
                "redundant_constraints":[],
                "residuals":[{"constraint_id":7,"residual":0.0,"human_message":"forged?"}],
                "max_residual":{"constraint_id":7,"residual":0.0,"human_message":"forged?"},
                "suggestions":[],"joints":[],"jacobian_condition_estimate":1.0
            }
        })
        .to_string()
    }

    #[test]
    fn evaluator_entry_rejects_fake_convergence_without_expected_pose() {
        let mut failures = Vec::new();
        check(
            &Expected::default(),
            MODEL,
            Some(0),
            &report(3.0, 4.0),
            "",
            &mut failures,
        );
        assert!(
            failures.iter().any(|e| e.contains("独立几何检查")),
            "{failures:?}"
        );
        failures.clear();
        check(
            &Expected::default(),
            MODEL,
            Some(0),
            &report(0.0, -10.0),
            "",
            &mut failures,
        );
        assert!(failures.is_empty(), "{failures:?}");
    }

    #[test]
    fn evaluator_entry_rejects_incomplete_reports_and_incomplete_tool_errors() {
        let mut failures = Vec::new();
        check(
            &Expected::default(),
            MODEL,
            Some(0),
            r#"{"outcome":"converged","entities":[],"diagnostics":{"dof_total":2,"dof_remaining":1}}"#,
            "",
            &mut failures,
        );
        assert!(failures.iter().any(|e| e.contains("完整 JSON 契约")));
        failures.clear();
        let expected = Expected {
            tool_error_kind: Some("invalid_json".into()),
            ..Expected::default()
        };
        check(
            &expected,
            "{",
            Some(2),
            "",
            r#"{"kind":"invalid_json"}"#,
            &mut failures,
        );
        assert!(!failures.is_empty());
    }

    #[test]
    fn empty_corpus_is_an_explicit_failure_not_a_perfect_score() {
        let directory =
            std::env::temp_dir().join(format!("ansatz-empty-corpus-{}", std::process::id()));
        std::fs::create_dir(&directory).unwrap();
        let suite = run_suite(
            Path::new("nonexistent-candidate-solver"),
            &directory,
            Duration::from_millis(20),
        );
        std::fs::remove_dir(&directory).unwrap();
        assert_eq!(suite.corpus.cases, 0);
        assert_eq!(suite.summary.passed, 0);
        assert_eq!(suite.summary.failed, 1);
        assert_eq!(suite.summary.pass_rate, 0.0);
        assert!(suite.cases[0].failures[0].contains("没有任何 case"));
    }

    #[test]
    fn subprocess_timeout_terminates_a_hanging_candidate() {
        // Reuse this test executable as a dependency-free child fixture. Only
        // the filtered child invocation sleeps; normal cargo test runs do not.
        let executable = std::env::current_exe().unwrap();
        let input = "x".repeat(256 * 1024);
        let start = Instant::now();
        let (exit, _, _, timed_out) = run_process(
            &executable,
            "tests::slow_process_fixture",
            Some(&input),
            Duration::from_millis(30),
        );
        assert!(timed_out);
        assert_eq!(exit, None);
        assert!(start.elapsed() < Duration::from_secs(5));
    }

    #[test]
    fn slow_process_fixture() {
        if std::env::args().any(|arg| arg == "tests::slow_process_fixture") {
            std::thread::sleep(Duration::from_secs(10));
        }
    }
}
