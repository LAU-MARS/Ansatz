//! 位级一致性（bit-level parity）测试框架。
//!
//! 对固定的输入算例，断言输出的 f64 数值与 `tests/parity/*.expected.json`
//! 中提交的**十六进制位模式**完全相同（不是 epsilon 比较）。同一份测试在
//! CI 的 x86-64-linux / aarch64-darwin / x86-64-windows 上原位运行，
//! wasm32 上由 `ansatz-wasm/smoke/node.mjs` 用同一份期望文件做同样断言，
//! 从而覆盖「同一输入、逐位相同」的跨架构承诺。
//!
//! 期望文件中数值一律写成 "0x…" 字符串，避免 JSON 浮点序列化本身引入歧义。

use ansatz_core::{solve, Model};

fn hex_to_bits(s: &str) -> u64 {
    u64::from_str_radix(s.trim_start_matches("0x"), 16).expect("期望文件中必须是 0x… 十六进制")
}

fn assert_bits(case: &str, field: &str, got: f64, want_hex: &str) {
    assert_eq!(
        got.to_bits(),
        hex_to_bits(want_hex),
        "{case}/{field}: got {got} ({:#018x}), want 位模式 {want_hex}",
        got.to_bits()
    );
}

fn run_case(name: &str) -> serde_json::Value {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/parity");
    let model_json = std::fs::read_to_string(format!("{dir}/{name}.model.json")).unwrap();
    let expected: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(format!("{dir}/{name}.expected.json")).unwrap(),
    )
    .unwrap();

    let model: Model = serde_json::from_str(&model_json).unwrap();
    let report = solve(&model).expect("parity 用例必须是受支持的平凡算例");
    let got = serde_json::to_value(&report).expect("报告可序列化");

    assert_eq!(got["outcome"], expected["outcome"], "{name}: outcome");
    assert_eq!(
        got["diagnostics"]["dof_total"], expected["dof_total"],
        "{name}: dof_total"
    );
    assert_eq!(
        got["diagnostics"]["dof_remaining"], expected["dof_remaining"],
        "{name}: dof_remaining"
    );

    let want_entities = expected["entities"].as_array().unwrap();
    let got_entities = got["entities"].as_array().unwrap();
    assert_eq!(got_entities.len(), want_entities.len(), "{name}: 实体数");
    for (g, w) in got_entities.iter().zip(want_entities) {
        assert_eq!(g["id"], w["id"], "{name}: 实体顺序按模型保持");
        assert_bits(
            name,
            "x",
            g["geometry"]["x"].as_f64().unwrap(),
            w["x"].as_str().unwrap(),
        );
        assert_bits(
            name,
            "y",
            g["geometry"]["y"].as_f64().unwrap(),
            w["y"].as_str().unwrap(),
        );
    }

    // 最大残差（按绝对值）与条件数估计同为位模式断言
    let want_mr = hex_to_bits(expected["max_residual_abs"].as_str().unwrap());
    let got_mr = got["diagnostics"]["max_residual"]["residual"]
        .as_f64()
        .expect("parity 用例都有约束")
        .abs()
        .to_bits();
    assert_eq!(got_mr, want_mr, "{name}: max_residual_abs");

    let want_cond = expected["jacobian_condition_estimate"].as_str().unwrap();
    let got_cond = got["diagnostics"]["jacobian_condition_estimate"]
        .as_f64()
        .expect("parity 用例条件数非空");
    assert_bits(name, "cond", got_cond, want_cond);

    got
}

#[test]
fn parity_case1_exact_rational() {
    // 3-4-5 三角形缩放：闭式解精确可表示，残差恰为 0。
    let got = run_case("case1");
    assert_eq!(got["outcome"], "underconstrained");
}

#[test]
fn parity_case2_irrational_sqrt_path() {
    // (1,1) 归一化：走真实 sqrt/除法路径，是跨架构逐位一致性的真金火检验。
    let got = run_case("case2");
    assert_eq!(got["outcome"], "underconstrained");
}
