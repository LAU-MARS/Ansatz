//! 求解器行为测试（同时充当 serde JSON 形状的契约测试：所有模型都用
//! JSON 构造，序列化路径与外壳完全一致）。

use ansatz_core::{solve, Action, Geometry, Model, SolveError, SolveOutcome};

fn model(json: &str) -> Model {
    serde_json::from_str(json).expect("测试模型 JSON 必须合法")
}

fn point(report: &ansatz_core::SolveReport) -> (f64, f64) {
    match &report.entities[0].geometry {
        Geometry::Point2 { x, y } => (*x, *y),
        other => panic!("期望 Point2，得到 {}", other.kind_name()),
    }
}

#[test]
fn closed_form_exact_3_4_5() {
    // 3-4-5 直角三角形：闭式解 (3,4)·(10/5) = (6,8)，全部为精确二进制有理数。
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]}"#,
    );
    let r = solve(&m).expect("平凡算例必须可解");
    assert_eq!(r.outcome, SolveOutcome::Underconstrained); // 2 DOF - 1 约束 = 1
    let (x, y) = point(&r);
    assert_eq!(
        (x.to_bits(), y.to_bits()),
        (6.0f64.to_bits(), 8.0f64.to_bits())
    );
    assert_eq!(r.diagnostics.dof_total, 2);
    assert_eq!(r.diagnostics.dof_remaining, 1);
    assert_eq!(r.diagnostics.jacobian_condition_estimate, Some(1.0));
    let mr = r.diagnostics.max_residual.as_ref().expect("有约束则有残差");
    assert_eq!(mr.residual.to_bits(), 0.0f64.to_bits());
    // 欠约束 → 必须给出 AddConstraint 建议
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::AddConstraint { dof_remaining: 1 })));
    // params 缺省也被接受（serde default 路径）
    assert_eq!(m.params.tolerance, 1e-9);
}

#[test]
fn conflicting_distances_are_inconsistent() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
            "constraints":[
              {"id":1,"kind":{"type":"distance","a":1,"b":null,"value":3.0}},
              {"id":2,"kind":{"type":"distance","a":1,"b":null,"value":5.0}}]}"#,
    );
    let r = solve(&m).expect("冲突属于求解层结论，不是工具错误");
    assert_eq!(r.outcome, SolveOutcome::Inconsistent);
    assert_eq!(
        r.diagnostics.redundant_constraints[0].constraint_ids,
        vec![1, 2]
    );
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::RemoveConstraint { id: 2 })));
    // 特解按 d_max=5 缩放 → 约束 1 的残差恰为 2.0（位精确）
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert_eq!(mr.constraint_id, 1);
    assert_eq!(mr.residual.to_bits(), 2.0f64.to_bits());
    assert!(mr.human_message.contains("残差"));
}

#[test]
fn negative_distance_is_inconsistent() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":1.0,"y":0.0}}],
            "constraints":[{"id":7,"kind":{"type":"distance","a":1,"b":null,"value":-3.0}}]}"#,
    );
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Inconsistent);
    assert_eq!(
        r.diagnostics.redundant_constraints[0].constraint_ids,
        vec![7]
    );
    assert!(r.diagnostics.redundant_constraints[0]
        .human_message
        .contains("负值"));
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::RelaxConstraint { id: 7 })));
}

#[test]
fn duplicate_distances_are_overconstrained_but_satisfiable() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":0.0,"y":2.0}}],
            "constraints":[
              {"id":1,"kind":{"type":"distance","a":1,"b":null,"value":2.0}},
              {"id":2,"kind":{"type":"distance","a":1,"b":null,"value":2.0},"label":"重复"}]}"#,
    );
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Overconstrained);
    assert_eq!(
        r.diagnostics.redundant_constraints[0].constraint_ids,
        vec![1, 2]
    );
    // 仍然给出满足全部约束的特解 (0,2)
    let (x, y) = point(&r);
    assert_eq!(
        (x.to_bits(), y.to_bits()),
        (0.0f64.to_bits(), 2.0f64.to_bits())
    );
    assert!(r
        .diagnostics
        .residuals
        .iter()
        .all(|res| res.residual == 0.0));
    assert_eq!(r.diagnostics.jacobian_condition_estimate, None); // 秩亏
}

#[test]
fn empty_model_converges() {
    let m = model(r#"{"entities":[],"constraints":[]}"#);
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Converged);
    assert_eq!(
        (r.diagnostics.dof_total, r.diagnostics.dof_remaining),
        (0, 0)
    );
    assert!(r.diagnostics.max_residual.is_none());
    assert_eq!(r.diagnostics.jacobian_condition_estimate, None);
}

#[test]
fn unsupported_constraint_kind_is_tool_error() {
    let m = model(
        r#"{"entities":[
             {"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}},
             {"id":2,"geometry":{"type":"point2","x":1.0,"y":1.0}}],
            "constraints":[{"id":1,"kind":{"type":"coincident","a":1,"b":2}}]}"#,
    );
    match solve(&m) {
        Err(SolveError::UnsupportedConstraint {
            constraint_kind, ..
        }) => assert_eq!(constraint_kind, "coincident"),
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn distance_to_entity_is_unsupported_yet() {
    let m = model(
        r#"{"entities":[
             {"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}},
             {"id":2,"geometry":{"type":"point2","x":3.0,"y":0.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":2,"value":3.0}}]}"#,
    );
    match solve(&m) {
        Err(SolveError::UnsupportedConstraint { reason, .. }) => {
            assert!(reason.contains("原点"))
        }
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn distance_to_non_point_is_unsupported() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"circle2","center":{"x":0.0,"y":0.0},"radius":1.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":3.0}}]}"#,
    );
    match solve(&m) {
        Err(SolveError::UnsupportedConstraint {
            constraint_kind,
            reason,
            ..
        }) => {
            assert_eq!(constraint_kind, "distance");
            assert!(reason.contains("Point2"));
        }
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn unknown_entity_is_tool_error() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":99,"b":null,"value":1.0}}]}"#,
    );
    match solve(&m) {
        Err(SolveError::UnknownEntity { entity_id, .. }) => assert_eq!(entity_id, 99),
        other => panic!("期望 UnknownEntity，得到 {other:?}"),
    }
}

#[test]
fn duplicate_ids_are_invalid_model() {
    let m = model(
        r#"{"entities":[
             {"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}},
             {"id":1,"geometry":{"type":"point2","x":1.0,"y":1.0}}],
            "constraints":[]}"#,
    );
    match solve(&m) {
        Err(SolveError::InvalidModel { reason }) => assert!(reason.contains("实体 id")),
        other => panic!("期望 InvalidModel，得到 {other:?}"),
    }
}

#[test]
fn underconstrained_rejected_returns_input_entities() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}],
            "params":{"tolerance":1e-9,"max_iterations":100,"allow_underconstrained":false}}"#,
    );
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    let (x, y) = point(&r); // 不接受特解：原样返回
    assert_eq!(
        (x.to_bits(), y.to_bits()),
        (3.0f64.to_bits(), 4.0f64.to_bits())
    );
}

#[test]
fn degenerate_point_at_origin_uses_deterministic_fallback() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":0.0,"y":0.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":5.0}}]}"#,
    );
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    let (x, y) = point(&r); // 方向退化 → 按 +x 轴取特解
    assert_eq!(
        (x.to_bits(), y.to_bits()),
        (5.0f64.to_bits(), 0.0f64.to_bits())
    );
    assert_eq!(r.diagnostics.jacobian_condition_estimate, None); // 原点处梯度为零
}

#[test]
fn unconstrained_rigid3_counts_six_dof() {
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"rigid3","pose":{
                "translation":{"x":0.0,"y":0.0,"z":0.0},
                "rotation":{"vector":[0.0,0.0,0.0]}}}}],
            "constraints":[]}"#,
    );
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(
        (r.diagnostics.dof_total, r.diagnostics.dof_remaining),
        (6, 6)
    );
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::AddConstraint { dof_remaining: 6 })));
}

#[test]
fn serde_shapes_roundtrip() {
    // 外壳依赖的 JSON 形状：kind/geometry 用内部标签，label 可省略。
    let m = model(
        r#"{"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
            "constraints":[{"id":1,"kind":{"type":"distance","a":1,"b":null,"value":10.0},"label":"L"}]}"#,
    );
    let r = solve(&m).unwrap();
    let json = serde_json::to_value(&r).unwrap();
    assert_eq!(json["outcome"], "underconstrained");
    assert_eq!(json["entities"][0]["geometry"]["type"], "point2");
    assert_eq!(json["diagnostics"]["dof_remaining"], 1);
    assert_eq!(
        json["diagnostics"]["suggestions"][0]["action"],
        "add_constraint"
    );
    assert_eq!(json["diagnostics"]["suggestions"][0]["dof_remaining"], 1);
}
