//! 3D 装配求解器集成测试：收敛性、DOF 计数、结局判定、错误路径。
//!
//! 数值断言用 1e-6 级近似（收敛性验证）；位级断言留给 tests/parity 的
//! case3（冻结十六进制位模式）。雅可比的数学正确性由 assembly 模块内的
//! FD 交叉验证测试保证。

use ansatz_core::{solve, Action, Model, SolveError, SolveOutcome};

fn model(json: &str) -> Model {
    serde_json::from_str(json).expect("测试模型 JSON 必须合法")
}

fn rigid3(id: u32, t: [f64; 3], w: [f64; 3]) -> String {
    format!(
        r#"{{"id":{id},"geometry":{{"type":"rigid3","pose":{{"translation":{{"x":{},"y":{},"z":{}}},"rotation":{{"vector":[{}, {}, {}]}}}}}}}}"#,
        t[0], t[1], t[2], w[0], w[1], w[2]
    )
}

/// 两个刚体：1 = 基座（fixed），2 = 自由件。
const BASE: &str = r#"{"id":1,"geometry":{"type":"rigid3","pose":{"translation":{"x":0.0,"y":0.0,"z":0.0},"rotation":{"vector":[0.0,0.0,0.0]}}}}"#;
const FREE: &str = r#"{"id":2,"geometry":{"type":"rigid3","pose":{"translation":{"x":0.5,"y":0.3,"z":1.2},"rotation":{"vector":[0.1,-0.2,0.15]}}}}"#;

fn two_bodies(free: &str) -> String {
    format!(
        r#"{{"entities":[{BASE},{free}],"constraints":[{{"id":90,"kind":{{"type":"fixed","a":1}}}}]}}"#
    )
}

/// z 向轴（体坐标系）。
fn axis_json(origin: [f64; 3], dir: [f64; 3]) -> String {
    format!(
        r#"{{"origin":{{"x":{},"y":{},"z":{}}},"direction":{{"x":{},"y":{},"z":{}}}}}"#,
        origin[0], origin[1], origin[2], dir[0], dir[1], dir[2]
    )
}

fn plane_json(origin: [f64; 3], normal: [f64; 3]) -> String {
    format!(
        r#"{{"origin":{{"x":{},"y":{},"z":{}}},"normal":{{"x":{},"y":{},"z":{}}}}}"#,
        origin[0], origin[1], origin[2], normal[0], normal[1], normal[2]
    )
}

#[test]
fn unconstrained_rigid3_is_underconstrained() {
    let m = model(&two_bodies(FREE));
    let r = solve(&m).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_total, 12);
    assert_eq!(r.diagnostics.dof_remaining, 6);
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::AddConstraint { dof_remaining: 6 })));
}

#[test]
fn single_mate_leaves_three_dof() {
    // 基座顶面 z=1（法向 +z）与自由件底面（法向 −z 朝外）贴合。
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"mate","a":1,"b":2,
                "a_plane":{},"b_plane":{}}}}}]}}"#,
        plane_json([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
        plane_json([0.0, 0.0, -0.5], [0.0, 0.0, -1.0]),
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 3, "mate 应消去 3 个自由度");
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "残差应收敛：{mr:?}");
}

#[test]
fn single_coaxial_leaves_two_dof() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"coaxial","a":1,"b":2,
                "a_axis":{},"b_axis":{}}}}}]}}"#,
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.1, 0.2, 0.3], [0.1, 0.2, 1.0]),
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 2, "coaxial 应消去 4 个自由度");
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "残差应收敛：{mr:?}");
}

/// 经典全约束装配：轴完全定位（同轴 4 + 原点距 1 + 转角 1 = 6）。
#[test]
fn shaft_fully_converged() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"coaxial","a":1,"b":2,
                "a_axis":{},"b_axis":{}}}}},
            {{"id":2,"kind":{{"type":"distance","a":2,"b":1,"value":2.0}}}},
            {{"id":3,"kind":{{"type":"angle","a":1,"b":2,"value":1.5707963267948966,
                "a_dir":{{"x":1.0,"y":0.0,"z":0.0}},"b_dir":{{"x":0.0,"y":1.0,"z":0.0}}}}}}]}}"#,
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.1, 0.2, 0.3], [0.1, 0.2, 1.0]),
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Converged);
    assert_eq!(r.diagnostics.dof_remaining, 0, "6 个自由度应全部被约束");
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "残差应收敛：{mr:?}");
    assert!(r.diagnostics.jacobian_condition_estimate.is_some());
    // 几何不变量：B 的体坐标轴点 [0.1,0.2,0.3] 经位姿映射后落在世界 z 轴上
    // （coaxial 使 B 轴与基座 z 轴共线），原点间距 = 2，旋转绕 z。
    let pose = match &r.entities[1].geometry {
        ansatz_core::Geometry::Rigid3 { pose } => pose,
        _ => panic!("期望 rigid3"),
    };
    let rot = ansatz_core::so3::rotation(pose.rotation.vector);
    let axis_pt = [
        pose.translation.x + rot[0][0] * 0.1 + rot[0][1] * 0.2 + rot[0][2] * 0.3,
        pose.translation.y + rot[1][0] * 0.1 + rot[1][1] * 0.2 + rot[1][2] * 0.3,
    ];
    assert!(
        (axis_pt[0] * axis_pt[0] + axis_pt[1] * axis_pt[1]).sqrt() < 1e-8,
        "B 的轴点应落在 z 轴上：{axis_pt:?}"
    );
    let t = pose.translation;
    let dist = (t.x * t.x + t.y * t.y + t.z * t.z).sqrt();
    assert!((dist - 2.0).abs() < 1e-8, "原点间距应为 2：{dist}");
    // B 的体轴方向 [0.1,0.2,1] 映射后应平行于 z（x/y 分量为 0）
    let dir = [
        rot[0][0] * 0.1 + rot[0][1] * 0.2 + rot[0][2] * 1.0,
        rot[1][0] * 0.1 + rot[1][1] * 0.2 + rot[1][2] * 1.0,
    ];
    assert!(
        (dir[0] * dir[0] + dir[1] * dir[1]).sqrt() < 1e-8,
        "B 的体轴应平行 z：{dir:?}"
    );
}

/// mate + coaxial 同对施加：7 个方程、秩 6 → 1 个冗余。
#[test]
fn mate_plus_coaxial_is_overconstrained() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"coaxial","a":1,"b":2,
                "a_axis":{},"b_axis":{}}}}},
            {{"id":2,"kind":{{"type":"mate","a":1,"b":2,
                "a_plane":{},"b_plane":{}}}}}]}}"#,
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    );
    let r = solve(&model(&json)).unwrap();
    // 数学事实：coaxial 消 4（横向 2 + 姿态 2），mate 的法向沿轴时其姿态
    // 行与 coaxial 重叠、只剩轴向偏移 1 → 共消 5；绕轴转动仍自由 → 欠约束 1。
    // 块级贪心冗余判定不报「块内部分冗余」（mate 的 4 行仍带来秩 1）。
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 1);
    assert!(r.diagnostics.redundant_constraints.is_empty());
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "仍应收敛：{mr:?}");
}

#[test]
fn duplicate_distance_is_overconstrained_but_satisfiable() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"distance","a":2,"b":1,"value":2.0}}}},
            {{"id":2,"kind":{{"type":"distance","a":1,"b":2,"value":2.0}}}}]}}"#
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Overconstrained);
    assert_eq!(r.diagnostics.dof_remaining, 5, "2 个距离约束秩 1");
    assert!(r
        .diagnostics
        .redundant_constraints
        .iter()
        .any(|g| g.constraint_ids == vec![2]));
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::RemoveConstraint { id: 2 })));
}

#[test]
fn conflicting_distances_are_inconsistent() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"distance","a":2,"b":1,"value":2.0}}}},
            {{"id":2,"kind":{{"type":"distance","a":1,"b":2,"value":3.0}}}}]}}"#
    );
    let r = solve(&model(&json)).unwrap();
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
}

#[test]
fn negative_distance_is_inconsistent() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":7,"kind":{{"type":"distance","a":2,"b":1,"value":-3.0}}}}]}}"#
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Inconsistent);
    assert!(r
        .diagnostics
        .suggestions
        .iter()
        .any(|s| matches!(s.action, Action::RelaxConstraint { id: 7 })));
}

/// 双固定体 + 不可满足的 mate：零自由参数下残差仍超差 → 可证明 Inconsistent。
#[test]
fn all_fixed_unsatisfiable_mate_is_inconsistent() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":91,"kind":{{"type":"fixed","a":2}}}},
            {{"id":1,"kind":{{"type":"mate","a":1,"b":2,
                "a_plane":{},"b_plane":{}}}}}]}}"#,
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]), // 两面平行但相距 1.2：不可满足
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Inconsistent);
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(
        mr.residual > 1e-6,
        "零自由参数下残差应显著超差（面分离 + 姿态失配）：{mr:?}"
    );
}

/// 双固定体 + 已满足的 mate：Converged（fixed 消 12 DOF，mate 在零自由
/// 参数下残差为 0）。第二个体取满足位姿：单位旋转、平面过 z=0。
#[test]
fn all_fixed_satisfied_mate_converges() {
    let sat = rigid3(2, [1.0, 1.0, 0.0], [0.0, 0.0, 0.0]);
    let json = format!(
        r#"{{"entities":[{BASE},{sat}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":91,"kind":{{"type":"fixed","a":2}}}},
            {{"id":1,"kind":{{"type":"mate","a":1,"b":2,
                "a_plane":{},"b_plane":{}}}}}]}}"#,
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Converged);
}

#[test]
fn mixed_2d_entities_rejected() {
    let json = format!(
        r#"{{"entities":[{BASE},{{"id":9,"geometry":{{"type":"point2","x":1.0,"y":2.0}}}}],
        "constraints":[{{"id":90,"kind":{{"type":"fixed","a":1}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::InvalidModel { reason }) => assert!(reason.contains("rigid3")),
        other => panic!("期望 InvalidModel，得到 {other:?}"),
    }
}

#[test]
fn point2_constraint_on_rigid3_rejected() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"coincident","a":1,"b":2}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::UnsupportedConstraint {
            constraint_kind, ..
        }) => {
            assert_eq!(constraint_kind, "coincident")
        }
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn distance_to_origin_rejected_for_rigid3() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"distance","a":2,"b":null,"value":3.0}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::UnsupportedConstraint { reason, .. }) => {
            assert!(reason.contains("两个刚体"))
        }
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn angle_requires_dirs() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"angle","a":1,"b":2,"value":0.5}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::UnsupportedConstraint { reason, .. }) => assert!(reason.contains("a_dir")),
        other => panic!("期望 UnsupportedConstraint，得到 {other:?}"),
    }
}

#[test]
fn angle_out_of_range_rejected() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"angle","a":1,"b":2,"value":7.0,
                "a_dir":{{"x":1.0,"y":0.0,"z":0.0}},"b_dir":{{"x":0.0,"y":1.0,"z":0.0}}}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::InvalidModel { reason }) => assert!(reason.contains("[0, π]")),
        other => panic!("期望 InvalidModel，得到 {other:?}"),
    }
}

#[test]
fn zero_normal_rejected() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"mate","a":1,"b":2,
                "a_plane":{},"b_plane":{}}}}}]}}"#,
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        plane_json([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    );
    match solve(&model(&json)) {
        Err(SolveError::InvalidModel { reason }) => assert!(reason.contains("零向量")),
        other => panic!("期望 InvalidModel，得到 {other:?}"),
    }
}

#[test]
fn unknown_entity_rejected() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"distance","a":2,"b":99,"value":3.0}}}}]}}"#
    );
    match solve(&model(&json)) {
        Err(SolveError::UnknownEntity { entity_id, .. }) => assert_eq!(entity_id, 99),
        other => panic!("期望 UnknownEntity，得到 {other:?}"),
    }
}

#[test]
fn underconstrained_rejected_returns_input_poses() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"distance","a":2,"b":1,"value":2.0}}}}],
        "params":{{"allow_underconstrained":false}}}}"#
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    let pose = match &r.entities[1].geometry {
        ansatz_core::Geometry::Rigid3 { pose } => pose,
        _ => panic!(),
    };
    // 原样返回输入位姿
    assert_eq!(pose.translation.x, 0.5);
    assert_eq!(pose.rotation.vector[1], -0.2);
}

/// 三体链式装配：基座—连杆—连杆，coaxial 相连 → 各留 2 DOF，共 4。
#[test]
fn three_body_chain_counts_dof() {
    let link2 = rigid3(3, [0.8, -0.4, 0.9], [-0.1, 0.2, 0.05]);
    let json = format!(
        r#"{{"entities":[{BASE},{FREE},{link2}],"constraints":[
            {{"id":90,"kind":{{"type":"fixed","a":1}}}},
            {{"id":1,"kind":{{"type":"coaxial","a":1,"b":2,
                "a_axis":{},"b_axis":{}}}}},
            {{"id":2,"kind":{{"type":"coaxial","a":2,"b":3,
                "a_axis":{},"b_axis":{}}}}}]}}"#,
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        axis_json([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
    );
    let r = solve(&model(&json)).unwrap();
    assert_eq!(r.outcome, SolveOutcome::Underconstrained);
    assert_eq!(r.diagnostics.dof_total, 18);
    assert_eq!(
        r.diagnostics.dof_remaining, 4,
        "两个 coaxial 各消 4，基座 fixed 消 6"
    );
    let mr = r.diagnostics.max_residual.as_ref().unwrap();
    assert!(mr.residual.abs() <= 1e-9, "链式同轴应收敛：{mr:?}");
}

/// serde 形状：mate/coaxial 的几何字段、angle 的方向字段双向序列化。
#[test]
fn serde_shapes_roundtrip_3d() {
    let json = format!(
        r#"{{"entities":[{BASE},{FREE}],"constraints":[
            {{"id":1,"kind":{{"type":"coaxial","a":1,"b":2,
                "a_axis":{},"b_axis":{}}}}}]}}"#,
        axis_json([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis_json([0.1, 0.2, 0.3], [0.1, 0.2, 1.0]),
    );
    let m = model(&json);
    let back = serde_json::to_value(&m).unwrap();
    assert_eq!(
        back["constraints"][0]["kind"]["a_axis"]["direction"]["z"],
        1.0
    );
}
