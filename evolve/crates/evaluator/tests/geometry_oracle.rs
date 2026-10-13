//! These reports are hand-built, not produced by the solver under test. Both
//! valid alternative solutions and dishonest zero-residual reports are tested.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use ansatz_eval::geometry_check::{parse_and_validate_report, validate_report};
use serde_json::{json, Value};

fn rigid(id: u32, translation: [f64; 3], rotation: [f64; 3]) -> Value {
    json!({"id":id, "geometry":{"type":"rigid3", "pose":{
        "translation":{"x":translation[0],"y":translation[1],"z":translation[2]},
        "rotation":{"vector":rotation}
    }}})
}

fn vector(v: [f64; 3]) -> Value {
    json!({"x":v[0],"y":v[1],"z":v[2]})
}

fn axis() -> Value {
    json!({"origin":vector([0.0;3]),"direction":vector([0.0,0.0,1.0])})
}

fn model(kind: Value) -> Value {
    json!({"entities":[rigid(1,[0.0;3],[0.0;3]),rigid(2,[0.0;3],[0.0;3])],
        "constraints":[{"id":7,"kind":kind}],"params":{"tolerance":1e-9,"max_iterations":100}})
}

fn joint_kind(tag: &str) -> Value {
    json!({"type":tag,"a":1,"b":2,"a_axis":axis(),"b_axis":axis()})
}

fn complete_report(model: &Value) -> Value {
    let entities = model["entities"].as_array().unwrap();
    let dof_total: u32 = entities
        .iter()
        .map(|e| match e["geometry"]["type"].as_str().unwrap() {
            "point2" => 2,
            "rigid3" => 6,
            _ => unreachable!(),
        })
        .sum();
    let residuals: Vec<Value> = model["constraints"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|c| c["kind"]["type"] != "fixed")
        .map(|c| json!({"constraint_id":c["id"],"residual":0.0,"human_message":"fixture"}))
        .collect();
    let joints: Vec<Value> = model["constraints"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|c| matches!(c["kind"]["type"].as_str(),Some("revolute"|"cylindrical"|"prismatic")))
        .map(|c| json!({"constraint_id":c["id"],"joint_kind":c["kind"]["type"],"value":0.0,"human_message":"fixture"}))
        .collect();
    json!({"outcome":"converged","entities":entities,"diagnostics":{
        "dof_total":dof_total,"dof_remaining":dof_total,"iterations":0,
        "conflicts":[],"redundant_constraints":[],"residuals":residuals,
        "max_residual":residuals.first(),"suggestions":[],"joints":joints,
        "jacobian_condition_estimate":null
    }})
}

fn check(model: &Value, report: &Value) -> Result<ansatz_core::SolveReport, Vec<String>> {
    parse_and_validate_report(&model.to_string(), &report.to_string())
}

fn accepted(model: &Value, report: &Value) {
    if let Err(errors) = check(model, report) {
        panic!("expected a valid solution: {errors:?}");
    }
}

fn rejected(model: &Value, report: &Value, needle: &str) {
    let errors = check(model, report).expect_err("dishonest report passed the oracle");
    assert!(errors.iter().any(|e| e.contains(needle)), "{errors:?}");
}

#[test]
fn every_supported_3d_family_has_positive_and_negative_geometry_cases() {
    let mut cases = Vec::new();
    let mut distance = model(json!({"type":"distance","a":1,"b":2,"value":1.0}));
    distance["entities"][1] = rigid(2, [1.0, 0.0, 0.0], [0.0; 3]);
    cases.push((distance, "x", 2.0, None));

    let angle = model(json!({"type":"angle","a":1,"b":2,"value":FRAC_PI_2,
        "a_dir":vector([2.0,0.0,0.0]),"b_dir":vector([0.0,9.0,0.0])}));
    cases.push((angle, "rotation", 0.1, None));

    let mut mate = model(json!({"type":"mate","a":1,"b":2,
        "a_plane":{"origin":vector([0.0;3]),"normal":vector([0.0,0.0,2.0])},
        "b_plane":{"origin":vector([0.0;3]),"normal":vector([0.0,0.0,-3.0])}}));
    mate["entities"][1] = rigid(2, [0.6, 0.0, 0.0], [0.0; 3]);
    cases.push((mate, "z", 1.0, None));

    let mut coaxial = model(joint_kind("coaxial"));
    coaxial["entities"][1] = rigid(2, [0.0, 0.0, 1.0], [0.0; 3]);
    cases.push((coaxial, "x", 1.0, None));

    let mut revolute = model(joint_kind("revolute"));
    revolute["constraints"][0]["kind"]["drive"] = json!(0.3);
    revolute["constraints"][0]["kind"]["offset"] = json!(2.0);
    revolute["entities"][1] = rigid(2, [0.0, 0.0, -2.0], [0.0, 0.0, 0.3]);
    cases.push((revolute, "z", -1.0, Some(0.3)));

    let mut cylindrical = model(joint_kind("cylindrical"));
    cylindrical["entities"][1] = rigid(2, [0.0, 0.0, 1.0], [0.0, 0.0, 0.3]);
    cases.push((cylindrical, "x", 1.0, Some(0.3)));

    let mut prismatic = model(joint_kind("prismatic"));
    for name in ["a_ref", "b_ref"] {
        prismatic["constraints"][0]["kind"][name] = vector([1.0, 0.0, 0.0]);
    }
    prismatic["constraints"][0]["kind"]["roll"] = json!(0.4);
    prismatic["constraints"][0]["kind"]["drive"] = json!(2.0);
    prismatic["entities"][1] = rigid(2, [0.0, 0.0, 2.0], [0.0, 0.0, 0.4]);
    cases.push((prismatic, "z", 3.0, Some(2.0)));

    let mut spherical = model(json!({"type":"spherical","a":1,"b":2,
        "a_point":vector([1.0,0.0,0.0]),"b_point":vector([0.0;3])}));
    spherical["entities"][1] = rigid(2, [1.0, 0.0, 0.0], [0.3, -0.2, 0.1]);
    cases.push((spherical, "x", 2.0, None));

    let fixed = model(json!({"type":"fixed","a":1}));
    let report = complete_report(&fixed);
    accepted(&fixed, &report);
    let mut changed = report;
    changed["entities"][0]["geometry"]["pose"]["translation"]["x"] = json!(1.0);
    rejected(&fixed, &changed, "fixed");

    for (model, field, bad_value, joint_value) in cases {
        let mut report = complete_report(&model);
        if let Some(value) = joint_value {
            report["diagnostics"]["joints"][0]["value"] = json!(value);
        }
        accepted(&model, &report);
        if field == "rotation" {
            report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(bad_value);
        } else {
            report["entities"][1]["geometry"]["pose"]["translation"][field] = json!(bad_value);
        }
        rejected(&model, &report, "独立几何检查");
    }
}

#[test]
fn transmission_is_recomputed_from_poses_not_claimed_joint_values() {
    let mut model = model(joint_kind("revolute"));
    model["constraints"][0]["id"] = json!(1);
    let mut second = joint_kind("revolute");
    second["b"] = json!(3);
    model["constraints"]
        .as_array_mut()
        .unwrap()
        .push(json!({"id":2,"kind":second}));
    model["constraints"]
        .as_array_mut()
        .unwrap()
        .push(json!({"id":3,"kind":{
            "type":"transmission","joint_a":1,"joint_b":2,"ratio":-2.0
        }}));
    model["entities"][1] = rigid(2, [0.0; 3], [0.0, 0.0, 0.2]);
    model["entities"]
        .as_array_mut()
        .unwrap()
        .push(rigid(3, [0.0; 3], [0.0, 0.0, -0.4]));
    let mut report = complete_report(&model);
    report["diagnostics"]["joints"][0]["value"] = json!(0.2);
    report["diagnostics"]["joints"][1]["value"] = json!(-0.4);
    accepted(&model, &report);
    report["entities"][2]["geometry"]["pose"]["rotation"]["vector"][2] = json!(-0.5);
    // Also forge the diagnostic value to match the new pose. The transmission
    // still has to be rejected despite every reported residual remaining zero.
    report["diagnostics"]["joints"][1]["value"] = json!(-0.5);
    rejected(&model, &report, "传动角度");
}

#[test]
fn underconstrained_point2_may_choose_any_point_on_the_circle() {
    let model = json!({"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
        "constraints":[{"id":8,"kind":{"type":"distance","a":1,"b":null,"value":10.0}}]});
    let mut report = complete_report(&model);
    report["entities"][0]["geometry"] = json!({"type":"point2","x":0.0,"y":-10.0});
    accepted(&model, &report);
    report["entities"][0]["geometry"]["y"] = json!(-9.0);
    rejected(&model, &report, "独立几何检查");
}

#[test]
fn anchored_distance_applies_both_rotations_and_anchor_offsets() {
    let mut model = model(json!({"type":"distance","a":1,"b":2,"value":1.0,
        "a_anchor":vector([1.0,0.0,0.0]),"b_anchor":vector([1.0,0.0,0.0])}));
    model["entities"][0] = rigid(1, [0.0; 3], [0.0, 0.0, FRAC_PI_2]);
    model["entities"][1] = rigid(2, [1.0, 2.0, 0.0], [0.0, 0.0, -FRAC_PI_2]);
    let report = complete_report(&model);
    accepted(&model, &report);
    let mut changed = report;
    changed["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(FRAC_PI_2);
    rejected(&model, &changed, "锚点距离");
}

#[test]
fn same_direction_semantics_reject_antiparallel_axes_for_every_axis_family() {
    for tag in ["coaxial", "revolute", "cylindrical", "prismatic"] {
        let mut model = model(joint_kind(tag));
        model["constraints"][0]["kind"]["b_axis"]["direction"]["z"] = json!(-1.0);
        let report = complete_report(&model);
        rejected(&model, &report, "轴方向没有同向");
    }
}

#[test]
fn angle_zero_and_pi_use_actual_angle_error_not_cosine_error() {
    for target in [0.0, PI] {
        let mut model = model(json!({"type":"angle","a":1,"b":2,"value":target,
            "a_dir":vector([2.0,0.0,0.0]),"b_dir":vector([2.0,0.0,0.0])}));
        model["entities"][1] = rigid(2, [0.0; 3], [0.0, 0.0, target]);
        let mut report = complete_report(&model);
        accepted(&model, &report);
        // 1e-6 rad gives a cosine error far below the 1e-9 tolerance, but
        // violates the actual angular tolerance by three orders of magnitude.
        report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(target + 1e-6);
        rejected(&model, &report, "真实方向夹角");
    }
}

#[test]
fn prismatic_roll_is_signed_and_default_prismatic_locks_zero_roll() {
    let default = model(joint_kind("prismatic"));
    let mut report = complete_report(&default);
    accepted(&default, &report);
    report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(0.2);
    rejected(&default, &report, "缺省零滚转");

    let mut explicit = model(joint_kind("prismatic"));
    explicit["constraints"][0]["kind"]["a_ref"] = vector([2.0, 0.0, 1.0]);
    explicit["constraints"][0]["kind"]["b_ref"] = vector([3.0, 0.0, -1.0]);
    explicit["constraints"][0]["kind"]["roll"] = json!(0.4);
    explicit["entities"][1] = rigid(2, [0.0; 3], [0.0, 0.0, 0.4]);
    let mut report = complete_report(&explicit);
    accepted(&explicit, &report);
    report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(-0.4);
    rejected(&explicit, &report, "有向滚转");

    explicit["constraints"][0]["kind"]
        .as_object_mut()
        .unwrap()
        .remove("roll");
    rejected(&explicit, &complete_report(&explicit), "必须同时提供");
}

#[test]
fn rotation_vector_turns_are_geometrically_equivalent() {
    let mut model = model(joint_kind("revolute"));
    model["constraints"][0]["kind"]["drive"] = json!(0.0);
    let mut report = complete_report(&model);
    report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(TAU);
    accepted(&model, &report);
}

#[test]
fn enormous_drive_is_wrapped_before_subtracting_the_measured_angle() {
    let target = 1e20;
    let wrapped = target % TAU;
    let theta = if wrapped > PI { wrapped - TAU } else { wrapped };
    let mut model = model(joint_kind("revolute"));
    model["constraints"][0]["kind"]["drive"] = json!(target);
    model["entities"][1] = rigid(2, [0.0; 3], [0.0, 0.0, theta]);
    let mut report = complete_report(&model);
    report["diagnostics"]["joints"][0]["value"] = json!(theta);
    accepted(&model, &report);
    report["entities"][1]["geometry"]["pose"]["rotation"]["vector"][2] = json!(theta + 0.1);
    report["diagnostics"]["joints"][0]["value"] = json!(theta + 0.1);
    rejected(&model, &report, "驱动角");
}

#[test]
fn reports_must_follow_the_whole_contract_and_preserve_entity_identity() {
    let model = model(joint_kind("coaxial"));
    let report = complete_report(&model);
    accepted(&model, &report);
    let mut missing = report.clone();
    missing["diagnostics"]
        .as_object_mut()
        .unwrap()
        .remove("iterations");
    rejected(&model, &missing, "完整 JSON 契约");
    let mut invalid_outcome = report.clone();
    invalid_outcome["outcome"] = json!("almost_converged");
    rejected(&model, &invalid_outcome, "完整 JSON 契约");
    let mut missing_entity = report.clone();
    missing_entity["entities"] = json!([]);
    rejected(&model, &missing_entity, "ID 集合");
    let mut duplicate = report.clone();
    duplicate["entities"][1]["id"] = json!(1);
    rejected(&model, &duplicate, "重复");
    let mut different_type = report.clone();
    different_type["entities"][1]["geometry"] = json!({"type":"point2","x":0.0,"y":0.0});
    rejected(&model, &different_type, "几何类型");
    let mut diagnostic = report.clone();
    diagnostic["diagnostics"]["residuals"][0]["constraint_id"] = json!(999);
    rejected(&model, &diagnostic, "不存在的约束 ID");
    let mut unknown = report.clone();
    unknown["diagnostics"]["suggestions"] = json!([{"action":"remove_constraint","id":7,"human_message":"fixture","secret":"ignored?"}]);
    rejected(&model, &unknown, "完整 JSON 契约");
    let mut no_residual = report;
    no_residual["diagnostics"]["residuals"] = json!([]);
    no_residual["diagnostics"]["max_residual"] = Value::Null;
    rejected(&model, &no_residual, "缺少非 fixed");
}

#[test]
fn finite_values_and_honest_diagnostic_maxima_are_required() {
    let model = model(joint_kind("revolute"));
    let report = complete_report(&model);
    let typed_model: ansatz_core::Model = serde_json::from_value(model.clone()).unwrap();
    let mut typed_report: ansatz_core::SolveReport =
        serde_json::from_value(report.clone()).unwrap();
    typed_report.diagnostics.joints[0].value = f64::NAN;
    assert!(!validate_report(&typed_model, &typed_report).is_empty());
    let mut bad = report.clone();
    bad["diagnostics"]["max_residual"]["residual"] = json!(1.0);
    rejected(&model, &bad, "max_residual");
    let mut bad = report.clone();
    bad["diagnostics"]["joints"][0]["value"] = json!(1.0);
    rejected(&model, &bad, "关节变量与几何");
    let mut bad = report;
    bad["diagnostics"]["joints"] = json!([]);
    rejected(&model, &bad, "缺少关节状态");
}

#[test]
fn zero_directions_and_unsupported_combinations_cannot_fake_convergence() {
    let zero = model(json!({"type":"angle","a":1,"b":2,"value":0.0,
        "a_dir":vector([0.0;3]),"b_dir":vector([1.0,0.0,0.0])}));
    rejected(&zero, &complete_report(&zero), "非零向量");
    let unsupported = model(json!({"type":"coincident","a":1,"b":2}));
    rejected(&unsupported, &complete_report(&unsupported), "不支持");
}

#[test]
fn point2_distance_cannot_silently_ignore_3d_anchors() {
    for field in ["a_anchor", "b_anchor"] {
        let mut model = json!({"entities":[{"id":1,"geometry":{"type":"point2","x":3.0,"y":4.0}}],
            "constraints":[{"id":8,"kind":{"type":"distance","a":1,"b":null,"value":5.0}}]});
        model["constraints"][0]["kind"][field] = vector([0.0; 3]);
        // Even a zero anchor is unsupported, rather than an optional no-op.
        rejected(&model, &complete_report(&model), "2D distance 不支持");
    }
}

#[test]
fn same_body_3d_constraints_cannot_be_accepted_as_tautological_solutions() {
    let mut kinds = vec![
        json!({"type":"distance","a":1,"b":1,"value":0.0}),
        json!({"type":"angle","a":1,"b":1,"value":0.0,
            "a_dir":vector([1.0,0.0,0.0]),"b_dir":vector([1.0,0.0,0.0])}),
        json!({"type":"mate","a":1,"b":1,
            "a_plane":{"origin":vector([0.0;3]),"normal":vector([0.0,0.0,1.0])},
            "b_plane":{"origin":vector([0.0;3]),"normal":vector([0.0,0.0,-1.0])}}),
        json!({"type":"spherical","a":1,"b":1,
            "a_point":vector([0.0;3]),"b_point":vector([0.0;3])}),
    ];
    for tag in ["coaxial", "revolute", "cylindrical", "prismatic"] {
        let mut kind = joint_kind(tag);
        kind["b"] = json!(1);
        kinds.push(kind);
    }
    for kind in kinds {
        let model = model(kind);
        rejected(&model, &complete_report(&model), "两个不同");
    }
}

#[test]
fn mixed_dimension_models_cannot_fake_a_solve_level_report() {
    let mut model = model(joint_kind("coaxial"));
    model["entities"].as_array_mut().unwrap().push(json!({
        "id":3,"geometry":{"type":"point2","x":1.0,"y":2.0}
    }));
    rejected(&model, &complete_report(&model), "混合 2D");
}

#[test]
fn every_outcome_requires_complete_residual_and_joint_id_sets() {
    let model = model(joint_kind("revolute"));
    for outcome in ["converged", "inconsistent", "not_converged"] {
        let mut report = complete_report(&model);
        report["outcome"] = json!(outcome);
        if outcome == "inconsistent" {
            report["diagnostics"]["conflicts"] = json!([{
                "constraint_ids":[7],"human_message":"fixture"
            }]);
        }
        let mut missing_residual = report.clone();
        missing_residual["diagnostics"]["residuals"] = json!([]);
        missing_residual["diagnostics"]["max_residual"] = Value::Null;
        rejected(&model, &missing_residual, "缺少非 fixed");
        let mut missing_joint = report;
        missing_joint["diagnostics"]["joints"] = json!([]);
        rejected(&model, &missing_joint, "缺少关节状态");
    }
}
