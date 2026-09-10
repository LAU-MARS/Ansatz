//! CLI 端到端测试：stdin 模型 → stdout 报告 / stderr 错误 + 退出码契约。

use std::io::Write;
use std::process::{Command, Stdio};

fn run_cli(stdin_data: &str) -> (i32, String, String) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_ansatz-cli"))
        .arg("solve")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("启动 ansatz-cli");
    child
        .stdin
        .as_mut()
        .unwrap()
        .write_all(stdin_data.as_bytes())
        .unwrap();
    let out = child.wait_with_output().unwrap();
    (
        out.status.code().unwrap(),
        String::from_utf8(out.stdout).unwrap(),
        String::from_utf8(out.stderr).unwrap(),
    )
}

#[test]
fn converged_exits_zero_with_report_on_stdout() {
    let (code, stdout, stderr) = run_cli(r#"{"entities":[],"constraints":[]}"#);
    assert_eq!(code, 0);
    assert!(stderr.is_empty(), "成功时 stderr 必须为空");
    let report: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(report["outcome"], "converged");
    assert!(report["diagnostics"].is_object());
}

#[test]
fn inconsistent_still_prints_full_diagnostics_and_exits_nonzero() {
    let model = r#"{
        "entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
        "constraints":[
            {"id":1,"kind":{"type":"distance","a":1,"b":null,"value":3.0}},
            {"id":2,"kind":{"type":"distance","a":1,"b":null,"value":5.0}}]}"#;
    let (code, stdout, stderr) = run_cli(model);
    assert_eq!(code, 1); // 求解失败 ≠ 工具错误
    assert!(stderr.is_empty(), "求解层失败不写 stderr");
    let report: serde_json::Value = serde_json::from_str(&stdout).unwrap();
    assert_eq!(report["outcome"], "inconsistent");
    let diag = &report["diagnostics"];
    assert_eq!(
        diag["redundant_constraints"][0]["constraint_ids"],
        serde_json::json!([1, 2])
    );
    assert_eq!(diag["residuals"].as_array().unwrap().len(), 2);
    assert_eq!(diag["max_residual"]["residual"].as_f64().unwrap(), 2.0);
    assert!(!diag["suggestions"].as_array().unwrap().is_empty());
    assert!(diag["redundant_constraints"][0]["human_message"]
        .as_str()
        .unwrap()
        .contains("不一致"));
}

#[test]
fn malformed_json_is_tool_error_on_stderr() {
    let (code, stdout, stderr) = run_cli("{ not json");
    assert_eq!(code, 2);
    assert!(stdout.is_empty(), "工具错误不写 stdout");
    let err: serde_json::Value = serde_json::from_str(stderr.trim()).unwrap();
    assert_eq!(err["kind"], "invalid_json");
}

#[test]
fn unsupported_constraint_is_tool_error() {
    let model = r#"{
        "entities":[
            {"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}},
            {"id":2,"geometry":{"type":"point2","x":1.0,"y":1.0}}],
        "constraints":[{"id":1,"kind":{"type":"parallel","a":1,"b":2}}]}"#;
    let (code, stdout, stderr) = run_cli(model);
    assert_eq!(code, 2);
    assert!(stdout.is_empty());
    let err: serde_json::Value = serde_json::from_str(stderr.trim()).unwrap();
    assert_eq!(err["kind"], "unsupported_constraint");
    assert!(err["message"].as_str().unwrap().contains("parallel"));
}

#[test]
fn export_schema_outputs_valid_json_schema() {
    let out = Command::new(env!("CARGO_BIN_EXE_ansatz-cli"))
        .args(["export-schema", "model"])
        .output()
        .unwrap();
    assert!(out.status.success());
    let schema: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    assert!(schema["$schema"].is_string());
    assert!(schema["properties"]["entities"].is_object());
}
