//! Regression models from the numerical/diagnostic audit.
//!
//! These tests assert physical geometry and diagnostic facts, not a particular
//! under-constrained solution or a fixed iteration count. Bit-level platform
//! reproducibility remains the responsibility of the parity suite.

use ansatz_core::{solve, Action, Geometry, Model, Pose3, SolveError, SolveOutcome, SolveReport};
use serde_json::{json, Value};

fn body(id: u32, translation: [f64; 3], rotation: [f64; 3]) -> Value {
    json!({
        "id": id,
        "geometry": { "type": "rigid3", "pose": {
            "translation": vector(translation), "rotation": { "vector": rotation }
        } }
    })
}

fn vector(v: [f64; 3]) -> Value {
    json!({ "x": v[0], "y": v[1], "z": v[2] })
}

fn axis(direction: [f64; 3]) -> Value {
    json!({ "origin": vector([0.0; 3]), "direction": vector(direction) })
}

fn assembly(rotation: [f64; 3], constraints: Vec<Value>) -> Model {
    let mut all = vec![json!({ "id": 90, "kind": { "type": "fixed", "a": 1 } })];
    all.extend(constraints);
    serde_json::from_value(json!({
        "entities": [body(1, [0.0; 3], [0.0; 3]), body(2, [0.0; 3], rotation)],
        "constraints": all
    }))
    .unwrap()
}

fn pose(report: &SolveReport, id: u32) -> Pose3 {
    match report
        .entities
        .iter()
        .find(|e| e.id == id)
        .unwrap()
        .geometry
    {
        Geometry::Rigid3 { pose } => pose,
        _ => panic!("expected rigid body {id}"),
    }
}

fn direction(report: &SolveReport, id: u32, local: [f64; 3]) -> [f64; 3] {
    let r = ansatz_core::so3::rotation(pose(report, id).rotation.vector);
    ansatz_core::so3::mat3_mul_vec(&r, local)
}

fn finite_report(report: &SolveReport) {
    for entity in &report.entities {
        let Geometry::Rigid3 { pose } = entity.geometry else {
            continue;
        };
        assert!([
            pose.translation.x,
            pose.translation.y,
            pose.translation.z,
            pose.rotation.vector[0],
            pose.rotation.vector[1],
            pose.rotation.vector[2]
        ]
        .iter()
        .all(|v| v.is_finite()));
    }
    assert!(report
        .diagnostics
        .residuals
        .iter()
        .all(|r| r.residual.is_finite()));
    assert!(report
        .diagnostics
        .joints
        .iter()
        .all(|j| j.value.is_finite()));
}

#[test]
fn huge_finite_revolute_drive_returns_within_iteration_budget() {
    // The former repeated-subtraction wrap never advanced at this magnitude.
    // The matching corpus case runs out of process with the evaluator timeout.
    let mut m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "revolute", "a": 1, "b": 2,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0]),
            "a_ref": vector([1.0, 0.0, 0.0]), "b_ref": vector([1.0, 0.0, 0.0]),
            "drive": 1e20
        } })],
    );
    m.params.max_iterations = 24;
    let report = solve(&m).unwrap();
    assert!(report.diagnostics.iterations <= m.params.max_iterations);
    assert_ne!(report.outcome, SolveOutcome::Inconsistent);
    finite_report(&report);
}

#[test]
fn coincident_distance_initialization_escapes_zero_jacobian() {
    let m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "distance", "a": 1, "b": 2, "value": 1.0
        } })],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert!(report.diagnostics.conflicts.is_empty());
    let t = pose(&report, 2).translation;
    assert!((libm::sqrt(t.x * t.x + t.y * t.y + t.z * t.z) - 1.0).abs() < 1e-8);
}

#[test]
fn initially_parallel_angle_can_reach_right_angle() {
    let m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "angle", "a": 1, "b": 2, "value": core::f64::consts::FRAC_PI_2,
            "a_dir": vector([1.0, 0.0, 0.0]), "b_dir": vector([1.0, 0.0, 0.0])
        } })],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert!(report.diagnostics.conflicts.is_empty());
    assert!(direction(&report, 2, [1.0, 0.0, 0.0])[0].abs() < 1e-8);
}

#[test]
fn angle_direction_magnitude_does_not_change_the_constraint() {
    for (a, b) in [(2.0, 2.0), (1e12, 1e-12)] {
        let m = assembly(
            [0.0; 3],
            vec![json!({ "id": 1, "kind": {
                "type": "angle", "a": 1, "b": 2, "value": 0.0,
                "a_dir": vector([a, 0.0, 0.0]), "b_dir": vector([b, 0.0, 0.0])
            } })],
        );
        let report = solve(&m).unwrap();
        assert_eq!(report.outcome, SolveOutcome::Converged);
        assert!(report.diagnostics.conflicts.is_empty());
        assert!((direction(&report, 2, [1.0, 0.0, 0.0])[0] - 1.0).abs() < 1e-8);
    }
}

#[test]
fn antiparallel_coaxial_axes_cannot_be_false_converged() {
    let m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "coaxial", "a": 1, "b": 2,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, -1.0])
        } })],
    );
    let report = solve(&m).unwrap();
    assert_ne!(report.outcome, SolveOutcome::Inconsistent);
    if report.outcome == SolveOutcome::Converged {
        let world = direction(&report, 2, [0.0, 0.0, -1.0]);
        assert!(
            (world[2] - 1.0).abs() < 1e-8,
            "axes must be codirected: {world:?}"
        );
    }
}

#[test]
fn fixed_antiparallel_coaxial_axes_are_provably_inconsistent() {
    let m = assembly(
        [0.0; 3],
        vec![
            json!({ "id": 91, "kind": { "type": "fixed", "a": 2 } }),
            json!({ "id": 1, "kind": {
                "type": "coaxial", "a": 1, "b": 2,
                "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, -1.0])
            } }),
        ],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Inconsistent);
    assert_eq!(report.diagnostics.conflicts[0].constraint_ids, [1]);
}

#[test]
fn zero_roll_and_zero_drive_prismatic_has_zero_dof() {
    let m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "prismatic", "a": 1, "b": 2,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0]),
            "a_ref": vector([1.0, 0.0, 0.0]), "b_ref": vector([1.0, 0.0, 0.0]),
            "roll": 0.0, "drive": 0.0
        } })],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert_eq!(report.diagnostics.dof_remaining, 0);
    assert!(report.diagnostics.redundant_constraints.is_empty());
}

#[test]
fn geometrically_equivalent_rotation_vectors_have_equal_dof() {
    let mut dof = Vec::new();
    for rotation in [[0.0; 3], [core::f64::consts::TAU, 0.0, 0.0]] {
        let m = assembly(
            rotation,
            vec![json!({ "id": 1, "kind": {
                "type": "revolute", "a": 1, "b": 2,
                "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0]),
                "a_ref": vector([1.0, 0.0, 0.0]), "b_ref": vector([1.0, 0.0, 0.0]),
                "drive": 0.0
            } })],
        );
        let report = solve(&m).unwrap();
        assert_eq!(report.outcome, SolveOutcome::Converged);
        dof.push(report.diagnostics.dof_remaining);
    }
    assert_eq!(dof, [0, 0]);
}

#[test]
fn large_anchor_scale_preserves_rigid_lock_rank() {
    for scale in [1.0, 1e12] {
        let constraints = [[0.0; 3], [scale, 0.0, 0.0], [0.0, scale, 0.0]]
            .into_iter()
            .enumerate()
            .map(|(index, point)| {
                json!({ "id": index + 1, "kind": {
                "type": "spherical", "a": 1, "b": 2,
                "a_point": vector(point), "b_point": vector(point)
            } })
            })
            .collect();
        let report = solve(&assembly([0.0; 3], constraints)).unwrap();
        assert_eq!(report.outcome, SolveOutcome::Converged);
        assert_eq!(report.diagnostics.dof_remaining, 0, "anchor scale {scale}");
        assert!(
            report.diagnostics.redundant_constraints.is_empty(),
            "anchor scale {scale}"
        );
    }
}

#[test]
fn irrational_2d_solution_is_bitwise_idempotent() {
    let mut m: Model = serde_json::from_value(json!({
        "entities": [{ "id": 1, "geometry": { "type": "point2", "x": 1.0, "y": 1.0 } }],
        "constraints": [{ "id": 1, "kind": { "type": "distance", "a": 1, "b": null, "value": 1.0 } }]
    }))
    .unwrap();
    let first = solve(&m).unwrap();
    let Geometry::Point2 { x, y } = first.entities[0].geometry else {
        panic!()
    };
    assert_eq!(first.outcome, SolveOutcome::Converged);
    assert!((libm::sqrt(x * x + y * y) - 1.0).abs() < 1e-12);
    m.entities = first.entities;
    for _ in 0..5 {
        let next = solve(&m).unwrap();
        assert_eq!(next.outcome, SolveOutcome::Converged);
        let Geometry::Point2 { x: nx, y: ny } = next.entities[0].geometry else {
            panic!()
        };
        assert_eq!(nx.to_bits(), x.to_bits());
        assert_eq!(ny.to_bits(), y.to_bits());
        m.entities = next.entities;
    }
}

#[test]
fn subnormal_initial_distance_has_a_finite_escape_direction() {
    let mut m = assembly(
        [0.0; 3],
        vec![json!({"id": 1, "kind": {
            "type": "distance", "a": 1, "b": 2, "value": 1.0
        }})],
    );
    if let Geometry::Rigid3 { pose } = &mut m.entities[1].geometry {
        pose.translation.x = 1e-320;
    }
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    finite_report(&report);
}

#[test]
fn negative_distance_reports_real_geometry_and_other_constraints() {
    let mut m = assembly(
        [0.0; 3],
        vec![
            json!({"id": 1, "kind": {"type": "distance", "a": 1, "b": 2, "value": -1.0}}),
            json!({"id": 2, "kind": {"type": "angle", "a": 1, "b": 2, "value": 0.0,
            "a_dir": vector([1.0,0.0,0.0]), "b_dir": vector([1.0,0.0,0.0])}}),
        ],
    );
    if let Geometry::Rigid3 { pose } = &mut m.entities[1].geometry {
        pose.translation.x = 3.0;
    }
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Inconsistent);
    assert_eq!(report.diagnostics.residuals.len(), 2);
    assert_eq!(report.diagnostics.residuals[0].residual, 4.0);
    // fixed leaves six parameters; length removes one and direction lock two.
    assert_eq!(report.diagnostics.dof_remaining, 3);
    assert_eq!(report.entities, m.entities);
}

#[test]
fn huge_transmission_product_cannot_round_away_the_other_joint_angle() {
    let ratio = core::f64::consts::TAU * (1u64 << 54) as f64;
    let m: Model = serde_json::from_value(json!({
        "entities": [body(1,[0.0;3],[0.0;3]), body(2,[0.0;3],[0.0,0.0,1.0]), body(3,[0.0;3],[0.0,0.0,0.3])],
        "constraints": [
            {"id":90,"kind":{"type":"fixed","a":1}},
            {"id":91,"kind":{"type":"fixed","a":2}},
            {"id":92,"kind":{"type":"fixed","a":3}},
            {"id":1,"kind":{"type":"revolute","a":1,"b":2,"a_axis":axis([0.0,0.0,1.0]),"b_axis":axis([0.0,0.0,1.0])}},
            {"id":2,"kind":{"type":"revolute","a":1,"b":3,"a_axis":axis([0.0,0.0,1.0]),"b_axis":axis([0.0,0.0,1.0])}},
            {"id":3,"kind":{"type":"transmission","joint_a":1,"joint_b":2,"ratio":ratio}}
        ]
    })).unwrap();
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Inconsistent);
    assert!((report.diagnostics.max_residual.unwrap().residual - 0.3).abs() < 1e-10);
}

#[test]
fn nonlinear_stationarity_does_not_prove_conflict() {
    let mut m = assembly(
        [0.1, -0.2, 0.15],
        vec![
            json!({ "id": 1, "kind": {
                "type": "coaxial", "a": 1, "b": 2,
                "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0])
            } }),
            json!({ "id": 2, "kind": {
                "type": "spherical", "a": 1, "b": 2,
                "a_point": vector([1.0, 0.0, 0.0]), "b_point": vector([0.0; 3])
            } }),
        ],
    );
    if let Geometry::Rigid3 { pose } = &mut m.entities[1].geometry {
        pose.translation.x = 0.5;
        pose.translation.y = 0.3;
        pose.translation.z = 1.2;
    }
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::NotConverged);
    assert!(report.diagnostics.conflicts.is_empty());
    assert!(!report.diagnostics.suggestions.iter().any(|s| matches!(
        s.action,
        Action::RemoveConstraint { .. } | Action::RelaxConstraint { .. }
    )));
}

#[test]
fn same_anchor_contradiction_remains_provably_inconsistent() {
    let m = assembly(
        [0.0; 3],
        vec![
            json!({ "id": 1, "kind": { "type": "distance", "a": 1, "b": 2, "value": 1.0 } }),
            json!({ "id": 2, "kind": { "type": "distance", "a": 2, "b": 1, "value": 2.0 } }),
        ],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Inconsistent);
    assert_eq!(report.diagnostics.conflicts[0].constraint_ids, [1, 2]);
}

#[test]
fn default_prismatic_references_lock_roll_with_and_without_drive() {
    for drive in [None, Some(0.0)] {
        let mut kind = json!({
            "type": "prismatic", "a": 1, "b": 2,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0])
        });
        if let Some(value) = drive {
            kind["drive"] = json!(value);
        }
        let report = solve(&assembly([0.0; 3], vec![json!({ "id": 1, "kind": kind })])).unwrap();
        assert_eq!(report.outcome, SolveOutcome::Converged);
        assert_eq!(
            report.diagnostics.dof_remaining,
            if drive.is_some() { 0 } else { 1 }
        );
        assert!(report.diagnostics.redundant_constraints.is_empty());
    }
}

#[test]
fn partial_prismatic_reference_and_roll_combinations_are_invalid() {
    // Each optional field must either be absent together or present together.
    // Check all six incomplete combinations, rather than one arbitrary example.
    for mask in 1..7 {
        let mut kind = json!({
            "type": "prismatic", "a": 1, "b": 2,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0])
        });
        if mask & 1 != 0 {
            kind["a_ref"] = vector([1.0, 0.0, 0.0]);
        }
        if mask & 2 != 0 {
            kind["b_ref"] = vector([1.0, 0.0, 0.0]);
        }
        if mask & 4 != 0 {
            kind["roll"] = json!(0.0);
        }
        let m = assembly([0.0; 3], vec![json!({ "id": 1, "kind": kind })]);
        assert!(
            matches!(solve(&m), Err(SolveError::InvalidModel { .. })),
            "mask {mask}"
        );
    }
}

#[test]
fn exact_zero_and_pi_angles_remove_two_rotation_dof() {
    for (target, b_dir) in [
        (0.0, [1.0, 0.0, 0.0]),
        (core::f64::consts::PI, [-1.0, 0.0, 0.0]),
    ] {
        let m = assembly(
            [0.0; 3],
            vec![json!({ "id": 1, "kind": {
                "type": "angle", "a": 1, "b": 2, "value": target,
                "a_dir": vector([1.0, 0.0, 0.0]), "b_dir": vector(b_dir)
            } })],
        );
        let report = solve(&m).unwrap();
        assert_eq!(report.outcome, SolveOutcome::Converged);
        // Coincident/opposite directions constrain both transverse rotations;
        // the remaining motion is three translations and rotation about x.
        assert_eq!(report.diagnostics.dof_remaining, 4, "target {target}");
        assert!(report.diagnostics.redundant_constraints.is_empty());
    }
}

#[test]
fn small_nonzero_angle_is_checked_in_radians_not_cosine_error() {
    let target = 1e-5;
    let m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "angle", "a": 1, "b": 2, "value": target,
            "a_dir": vector([1.0, 0.0, 0.0]), "b_dir": vector([1.0, 0.0, 0.0])
        } })],
    );
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    // The initial cosine residual is <1e-9, but its angle error is 1e-5.
    assert!(report.diagnostics.iterations > 0);
    let d = direction(&report, 2, [1.0, 0.0, 0.0]);
    let angle = libm::atan2(libm::hypot(d[1], d[2]), d[0]);
    assert!(
        (angle - target).abs() <= m.params.tolerance,
        "actual angle {angle}"
    );
    assert_eq!(report.diagnostics.dof_remaining, 5);
}

#[test]
fn zero_distance_target_is_anchor_coincidence_and_removes_three_dof() {
    let mut m = assembly(
        [0.0; 3],
        vec![json!({ "id": 1, "kind": {
            "type": "distance", "a": 1, "b": 2, "value": 0.0
        } })],
    );
    if let Geometry::Rigid3 { pose } = &mut m.entities[1].geometry {
        pose.translation.x = 0.3;
        pose.translation.y = -0.2;
        pose.translation.z = 0.1;
    }
    let report = solve(&m).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert_eq!(report.diagnostics.dof_remaining, 3);
    let t = pose(&report, 2).translation;
    assert!(libm::hypot(libm::hypot(t.x, t.y), t.z) <= m.params.tolerance);
}

#[test]
fn finite_transmission_ratio_overflow_returns_error_instead_of_hanging() {
    for ratio in [f64::MAX, -f64::MAX] {
        // θ_a=3 is finite and valid, but multiplying it by this finite ratio
        // overflows. Nonfinite intermediate residuals must not enter wrapping
        // loops or decomposition routines.
        let revolute = |id, b| {
            json!({ "id": id, "kind": {
            "type": "revolute", "a": 1, "b": b,
            "a_axis": axis([0.0, 0.0, 1.0]), "b_axis": axis([0.0, 0.0, 1.0]),
            "a_ref": vector([1.0, 0.0, 0.0]), "b_ref": vector([1.0, 0.0, 0.0])
        } })
        };
        let mut m = assembly(
            [0.0, 0.0, 3.0],
            vec![
                revolute(1, 2),
                revolute(2, 3),
                json!({ "id": 3, "kind": {
                "type": "transmission", "joint_a": 1, "joint_b": 2, "ratio": ratio
            } }),
            ],
        );
        m.entities
            .push(serde_json::from_value(body(3, [0.0; 3], [0.0; 3])).unwrap());
        m.params.max_iterations = 1;
        assert!(matches!(solve(&m), Err(SolveError::InvalidModel { .. })));
    }
}
