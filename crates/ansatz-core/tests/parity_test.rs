//! 跨平台位级一致性（bit-level parity）测试。
//!
//! 承诺的范围是：**同一版本、同一输入，在所有目标上逐位相同**。对固定的
//! 输入算例，断言输出的 f64 与 `tests/parity/*.expected.json` 中的十六进制
//! 位模式完全相同。同一份测试在 CI 的 x86-64-linux / aarch64-darwin /
//! x86-64-windows 上原位运行，wasm32 与 node 原生壳由冒烟脚本用同一份期望
//! 文件做同样断言。
//!
//! 这些位模式**不是**行为契约：求解器的数值改进（迭代策略、分解库升级）
//! 会合法地改变它们，届时重新生成即可。行为正确性由其余测试与 evolve
//! 语料按容差断言。
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
    let report = solve(&model).expect("parity 用例必须是受支持的模型");
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
        match g["geometry"]["type"].as_str() {
            Some("point2") => {
                assert_bits(
                    name,
                    "x",
                    g["geometry"]["x"].as_f64().unwrap(),
                    w["x"]
                        .as_str()
                        .unwrap_or_else(|| panic!("{name}: point2 期望 x")),
                );
                assert_bits(
                    name,
                    "y",
                    g["geometry"]["y"].as_f64().unwrap(),
                    w["y"]
                        .as_str()
                        .unwrap_or_else(|| panic!("{name}: point2 期望 y")),
                );
            }
            Some("rigid3") => {
                let pose = &g["geometry"]["pose"];
                let want = w["pose"]
                    .as_object()
                    .unwrap_or_else(|| panic!("{name}: rigid3 期望 pose"));
                for (field, path) in [
                    ("tx", &pose["translation"]["x"]),
                    ("ty", &pose["translation"]["y"]),
                    ("tz", &pose["translation"]["z"]),
                ] {
                    assert_bits(
                        name,
                        field,
                        path.as_f64().unwrap(),
                        want[field]
                            .as_str()
                            .unwrap_or_else(|| panic!("{name}: 期望 {field}")),
                    );
                }
                let rv = pose["rotation"]["vector"].as_array().unwrap();
                for (field, v) in [("rx", &rv[0]), ("ry", &rv[1]), ("rz", &rv[2])] {
                    assert_bits(
                        name,
                        field,
                        v.as_f64().unwrap(),
                        want[field]
                            .as_str()
                            .unwrap_or_else(|| panic!("{name}: 期望 {field}")),
                    );
                }
            }
            other => panic!("{name}: 未支持的 parity 实体类型 {other:?}"),
        }
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

    // Every scalar residual and joint value is part of the numerical output.
    for field in ["residuals", "joints"] {
        let actual = got["diagnostics"][field].as_array().unwrap();
        let want = expected[field].as_array().unwrap();
        assert_eq!(actual.len(), want.len(), "{name}: {field} length");
        let value = if field == "joints" {
            "value"
        } else {
            "residual"
        };
        for (g, w) in actual.iter().zip(want) {
            assert_eq!(g["constraint_id"], w["constraint_id"]);
            assert_bits(
                name,
                field,
                g[value].as_f64().unwrap(),
                w[value].as_str().unwrap(),
            );
        }
    }
    let mut replay = model;
    replay.entities = report.entities.clone();
    assert_eq!(
        serde_json::to_value(solve(&replay).unwrap()).unwrap()["entities"],
        got["entities"],
        "{name}: solving a solution preserves its serialized numeric bits"
    );

    got
}

#[test]
fn parity_case3_rigid3_lm_solution() {
    // 3D 全约束装配：LM 迭代解的逐位基准——迭代求解器跨平台一致性的真检验。
    let got = run_case("case3");
    assert_eq!(got["outcome"], "converged");
}

#[test]
fn parity_case4_driven_arm_with_transmission() {
    // 三连杆机械臂（双驱动 + 传动耦合）：关节空间驱动的逐位基准，
    // 覆盖 revolute/drive/transmission 的迭代路径。
    let got = run_case("case4");
    assert_eq!(got["outcome"], "converged");
    let j1 = &got["diagnostics"]["joints"];
    assert_eq!(
        j1.as_array().map(|a| a.len()),
        Some(3),
        "三个关节各报一条状态"
    );
}

#[test]
fn parity_case5_irrational_2d_projection() {
    run_case("case5");
}

#[test]
fn parity_case6_zero_roll_at_equivalent_full_turn_pose() {
    run_case("case6");
}
