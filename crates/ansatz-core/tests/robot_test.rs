//! P0–P1.5 集成测试：锚点距离、关节族、驱动、传动（机器人学场景）。
//!
//! 数值断言 1e-6 级；位级断言在 tests/parity 的 case4。

use ansatz_core::{solve, Model, SolveOutcome};

fn model(json: &str) -> Model {
    serde_json::from_str(json).expect("测试模型 JSON 必须合法")
}

fn body(id: u32, t: [f64; 3], w: [f64; 3]) -> String {
    format!(
        r#"{{"id":{id},"geometry":{{"type":"rigid3","pose":{{"translation":{{"x":{},"y":{},"z":{}}},"rotation":{{"vector":[{}, {}, {}]}}}}}}}}"#,
        t[0], t[1], t[2], w[0], w[1], w[2]
    )
}

const BASE: &str = r#"{"id":1,"geometry":{"type":"rigid3","pose":{"translation":{"x":0.0,"y":0.0,"z":0.0},"rotation":{"vector":[0.0,0.0,0.0]}}}}"#;

fn fixed(id: u32) -> String {
    format!(r#"{{"id":90,"kind":{{"type":"fixed","a":{id}}}}}"#)
}

// ── P0：锚点距离 ────────────────────────────────────────────────

#[test]
fn anchored_distance_measures_between_anchors() {
    // body2 原点在 (0.3,0,0.8)；锚点 (0.1,0,0.2)（体坐标）→ 世界 (0.4,0,1.0)
    // 锚点距离 1.0 → 无需移动（初始即满足）；改为 2.0 需平移。
    let b2 = body(2, [0.3, 0.0, 0.8], [0.0, 0.0, 0.0]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":1,"kind":{{"type":"distance","a":2,"b":1,"value":1.0,
                "a_anchor":{{"x":0.1,"y":0.0,"z":0.2}}}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "初始位形即满足：{mr:?}");

    // 目标 2.0：球面半径差 1.0，LM 把锚点推到距离 2
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":1,"kind":{{"type":"distance","a":2,"b":1,"value":2.0,
                "a_anchor":{{"x":0.1,"y":0.0,"z":0.2}}}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "锚点距离应收敛：{mr:?}");
    let pose = match &r.entities[1].geometry {
        ansatz_core::Geometry::Rigid3 { pose } => pose,
        _ => panic!(),
    };
    let ax = pose.translation.x + 0.1; // 单位旋转下锚点 = t + anchor
    let az = pose.translation.z + 0.2;
    assert!(
        (ax * ax + az * az).sqrt() - 2.0 < 1e-8,
        "锚点应落在半径 2：({ax},{az})"
    );
}

// ── P0.5：关节族 ────────────────────────────────────────────────

#[test]
fn revolute_removes_five_dof_and_reports_joint_state() {
    let b2 = body(2, [0.1, -0.2, 0.3], [0.05, -0.1, 0.02]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":7,"kind":{{"type":"revolute","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.1,"y":0.2,"z":0.3}},"direction":{{"x":0.0,"y":0.1,"z":1.0}}}}}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 1, "revolute 消 5 留 1");
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "关节应收敛：{mr:?}");
    // 关节状态：以初始位形为零点，收敛位形与初始同侧时 θ ≈ 关节实际相对角
    let js = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 7)
        .unwrap();
    assert_eq!(js.joint_kind, "revolute");
    assert!(js.value.is_finite());
    assert!(js.human_message.contains("revolute"));
}

#[test]
fn cylindrical_removes_four_dof() {
    let b2 = body(2, [0.1, -0.2, 0.3], [0.05, -0.1, 0.02]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":8,"kind":{{"type":"cylindrical","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.1,"y":0.2,"z":0.3}},"direction":{{"x":0.0,"y":0.1,"z":1.0}}}}}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 2, "cylindrical 消 4 留 2");
}

#[test]
fn prismatic_with_roll_removes_five_dof() {
    let b2 = body(2, [0.1, -0.2, 0.3], [0.05, -0.1, 0.02]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":9,"kind":{{"type":"prismatic","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.1,"y":0.2,"z":0.3}},"direction":{{"x":0.0,"y":0.1,"z":1.0}}}},
                "a_ref":{{"x":1.0,"y":0.0,"z":0.0}},"b_ref":{{"x":0.0,"y":1.0,"z":0.0}},"roll":1.5707963267948966}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(
        r.diagnostics.dof_remaining, 1,
        "prismatic（含滚转锁）消 5 留 1"
    );
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "{mr:?}");
}

#[test]
fn spherical_removes_three_dof() {
    let b2 = body(2, [0.4, 0.2, 0.1], [0.1, 0.2, -0.1]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":10,"kind":{{"type":"spherical","a":1,"b":2,
                "a_point":{{"x":0.0,"y":0.0,"z":0.5}},"b_point":{{"x":0.1,"y":0.0,"z":0.0}}}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 3, "spherical 消 3 留 3");
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "{mr:?}");
}

// ── P1：驱动 ────────────────────────────────────────────────────

#[test]
fn revolute_drive_fully_constrains_and_reports_target() {
    let b2 = body(2, [0.1, -0.2, 0.3], [0.05, -0.1, 0.02]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":7,"kind":{{"type":"revolute","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.1,"y":0.2,"z":0.3}},"direction":{{"x":0.0,"y":0.1,"z":1.0}}}},
                "drive":0.5}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Converged, "驱动后关节完全确定");
    assert_eq!(r.diagnostics.dof_remaining, 0);
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "{mr:?}");
    let js = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 7)
        .unwrap();
    assert!(
        (js.value - 0.5).abs() < 1e-8,
        "关节状态应报告驱动目标 0.5，得到 {}",
        js.value
    );
}

#[test]
fn prismatic_drive_fully_constrains() {
    let b2 = body(2, [0.1, -0.2, 0.3], [0.05, -0.1, 0.02]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":9,"kind":{{"type":"prismatic","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.1,"y":0.2,"z":0.3}},"direction":{{"x":0.0,"y":0.1,"z":1.0}}}},
                "a_ref":{{"x":1.0,"y":0.0,"z":0.0}},"b_ref":{{"x":0.0,"y":1.0,"z":0.0}},"roll":1.5707963267948966,
                "drive":0.3}}}}]}}"#,
        fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Converged);
    assert_eq!(r.diagnostics.dof_remaining, 0);
    let js = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 9)
        .unwrap();
    assert!(
        (js.value - 0.3).abs() < 1e-8,
        "驱动目标 0.3，得到 {}",
        js.value
    );
}

// ── 三连杆机械臂：fixed + revolute×3 + 末端驱动（IK 缩影）────────

#[test]
fn three_link_arm_with_drives_converges() {
    // 基座 + 3 连杆，各关节绕 z；驱动关节 1 和 3，关节 2 由传动耦合（−0.5 速比）
    let l1 = body(2, [0.05, 0.1, 0.4], [0.02, -0.03, 0.05]);
    let l2 = body(3, [0.1, 0.35, 0.85], [-0.04, 0.06, -0.1]);
    let l3 = body(4, [0.5, 0.2, 1.3], [0.08, 0.05, 0.2]);
    let json = format!(
        r#"{{"entities":[{BASE},{l1},{l2},{l3}],"constraints":[
            {fixed1},
            {{"id":1,"kind":{{"type":"revolute","a":1,"b":2,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "drive":0.4}}}},
            {{"id":2,"kind":{{"type":"revolute","a":2,"b":3,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.4}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "drive":0.15}}}},
            {{"id":3,"kind":{{"type":"revolute","a":3,"b":4,
                "a_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.4}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}},
                "b_axis":{{"origin":{{"x":0.0,"y":0.0,"z":0.0}},"direction":{{"x":0.0,"y":0.0,"z":1.0}}}}}}}},
            {{"id":4,"kind":{{"type":"transmission","joint_a":1,"joint_b":3,"ratio":-0.5}}}}]}}"#,
        fixed1 = fixed(1)
    );
    let r = solve(&model(&json)).unwrap();
    // 驱动关节 1、2，传动把关节 3 耦合到 1（θ3 = −0.5·θ1 = −0.2）：
    // 三个关节变量全部确定 → 收敛，DOF 0，无冗余
    assert_eq!(
        r.outcome,
        SolveOutcome::Converged,
        "双驱动+传动的机械臂应完全确定"
    );
    assert_eq!(r.diagnostics.dof_remaining, 0);
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "{mr:?}");
    let j1 = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 1)
        .unwrap();
    let j2 = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 2)
        .unwrap();
    let j3 = r
        .diagnostics
        .joints
        .iter()
        .find(|j| j.constraint_id == 3)
        .unwrap();
    assert!((j1.value - 0.4).abs() < 1e-8);
    assert!((j2.value - 0.15).abs() < 1e-8);
    assert!(
        (j3.value + 0.2).abs() < 1e-8,
        "θ3 应由传动确定：{}",
        j3.value
    );
    assert!(
        (j3.value - (-0.5) * j1.value).abs() < 1e-8,
        "传动速比应精确保持"
    );
}

// ── 传动错误路径 ────────────────────────────────────────────────

#[test]
fn transmission_requires_revolute_references() {
    let b2 = body(2, [0.1, 0.0, 0.3], [0.0, 0.0, 0.0]);
    let json = format!(
        r#"{{"entities":[{BASE},{b2}],"constraints":[
            {},
            {{"id":1,"kind":{{"type":"transmission","joint_a":99,"joint_b":98,"ratio":1.0}}}}]}}"#,
        fixed(1)
    );
    match solve(&model(&json)) {
        Err(ansatz_core::SolveError::InvalidModel { reason }) => {
            assert!(reason.contains("joint_a"))
        }
        other => panic!("期望 InvalidModel，得到 {other:?}"),
    }
}
