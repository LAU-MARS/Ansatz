//! Public-API regressions for invalid numbers, stable projection, and solution replay.

use ansatz_core::model::{Axis3, Plane3};
use ansatz_core::{
    solve, Constraint, ConstraintKind, Entity, Envelope, Geometry, Model, Point2, Pose3, Rotation3,
    SolveError, SolveOutcome, Vec3,
};

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3 { x, y, z }
}

fn point_distance(x: f64, y: f64, distances: &[f64]) -> Model {
    Model {
        entities: vec![Entity {
            id: 1,
            geometry: Geometry::Point2 { x, y },
        }],
        constraints: distances
            .iter()
            .enumerate()
            .map(|(index, &value)| Constraint {
                id: index as u32 + 1,
                kind: ConstraintKind::Distance {
                    a: 1,
                    b: None,
                    value,
                    a_anchor: None,
                    b_anchor: None,
                },
                label: None,
            })
            .collect(),
        ..Model::default()
    }
}

fn assert_invalid(model: &Model) {
    let error = solve(model).expect_err("invalid numeric input must fail before solving");
    assert!(
        matches!(error, SolveError::InvalidModel { .. }),
        "{error:?}"
    );
    let envelope = serde_json::to_value(Envelope::from_solve_error(&error)).unwrap();
    assert_eq!(envelope["ok"], false);
    assert_eq!(envelope["error"]["kind"], "invalid_model");
    assert!(envelope.get("result").is_none());
}

fn vector_fields<'a>(fields: &mut Vec<&'a mut f64>, vector: &'a mut Vec3) {
    fields.extend([&mut vector.x, &mut vector.y, &mut vector.z]);
}

fn point_fields<'a>(fields: &mut Vec<&'a mut f64>, point: &'a mut Point2) {
    fields.extend([&mut point.x, &mut point.y]);
}

fn axis_fields<'a>(fields: &mut Vec<&'a mut f64>, axis: &'a mut Axis3) {
    vector_fields(fields, &mut axis.origin);
    vector_fields(fields, &mut axis.direction);
}

fn optional_vector_fields<'a>(fields: &mut Vec<&'a mut f64>, vector: &'a mut Option<Vec3>) {
    if let Some(vector) = vector {
        vector_fields(fields, vector);
    }
}

fn geometry_fields(geometry: &mut Geometry) -> Vec<&mut f64> {
    let mut fields = Vec::new();
    match geometry {
        Geometry::Point2 { x, y } => fields.extend([x, y]),
        Geometry::Line2 { start, end } => {
            point_fields(&mut fields, start);
            point_fields(&mut fields, end);
        }
        Geometry::Circle2 { center, radius } => {
            point_fields(&mut fields, center);
            fields.push(radius);
        }
        Geometry::Arc2 {
            center,
            radius,
            start_angle,
            end_angle,
        } => {
            point_fields(&mut fields, center);
            fields.extend([radius, start_angle, end_angle]);
        }
        Geometry::Rigid3 { pose } => {
            vector_fields(&mut fields, &mut pose.translation);
            fields.extend(pose.rotation.vector.iter_mut());
        }
    }
    fields
}

fn constraint_fields(kind: &mut ConstraintKind) -> Vec<&mut f64> {
    let mut fields = Vec::new();
    match kind {
        ConstraintKind::Distance {
            value,
            a_anchor,
            b_anchor,
            ..
        }
        | ConstraintKind::Angle {
            value,
            a_dir: a_anchor,
            b_dir: b_anchor,
            ..
        } => {
            fields.push(value);
            optional_vector_fields(&mut fields, a_anchor);
            optional_vector_fields(&mut fields, b_anchor);
        }
        ConstraintKind::Mate {
            a_plane, b_plane, ..
        } => {
            vector_fields(&mut fields, &mut a_plane.origin);
            vector_fields(&mut fields, &mut a_plane.normal);
            vector_fields(&mut fields, &mut b_plane.origin);
            vector_fields(&mut fields, &mut b_plane.normal);
        }
        ConstraintKind::Coaxial { a_axis, b_axis, .. } => {
            axis_fields(&mut fields, a_axis);
            axis_fields(&mut fields, b_axis);
        }
        ConstraintKind::Revolute {
            a_axis,
            b_axis,
            offset,
            a_ref,
            b_ref,
            drive,
            ..
        } => {
            axis_fields(&mut fields, a_axis);
            axis_fields(&mut fields, b_axis);
            optional_vector_fields(&mut fields, a_ref);
            optional_vector_fields(&mut fields, b_ref);
            fields.extend(offset.iter_mut());
            fields.extend(drive.iter_mut());
        }
        ConstraintKind::Cylindrical {
            a_axis,
            b_axis,
            a_ref,
            b_ref,
            ..
        } => {
            axis_fields(&mut fields, a_axis);
            axis_fields(&mut fields, b_axis);
            optional_vector_fields(&mut fields, a_ref);
            optional_vector_fields(&mut fields, b_ref);
        }
        ConstraintKind::Prismatic {
            a_axis,
            b_axis,
            a_ref,
            b_ref,
            roll,
            drive,
            ..
        } => {
            axis_fields(&mut fields, a_axis);
            axis_fields(&mut fields, b_axis);
            optional_vector_fields(&mut fields, a_ref);
            optional_vector_fields(&mut fields, b_ref);
            fields.extend(roll.iter_mut());
            fields.extend(drive.iter_mut());
        }
        ConstraintKind::Spherical {
            a_point, b_point, ..
        } => {
            vector_fields(&mut fields, a_point);
            vector_fields(&mut fields, b_point);
        }
        ConstraintKind::Transmission { ratio, .. } => fields.push(ratio),
        _ => {}
    }
    fields
}

#[test]
fn every_geometry_numeric_field_rejects_nonfinite_rust_api_input() {
    let point = Point2 { x: 1.0, y: 2.0 };
    let fixtures = [
        Geometry::Point2 { x: 1.0, y: 2.0 },
        Geometry::Line2 {
            start: point,
            end: point,
        },
        Geometry::Circle2 {
            center: point,
            radius: 1.0,
        },
        Geometry::Arc2 {
            center: point,
            radius: 1.0,
            start_angle: 0.0,
            end_angle: 1.0,
        },
        Geometry::Rigid3 {
            pose: Pose3 {
                translation: v(0.0, 0.0, 0.0),
                rotation: Rotation3 { vector: [0.0; 3] },
            },
        },
    ];
    for mut fixture in fixtures {
        let count = geometry_fields(&mut fixture).len();
        for field in 0..count {
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut geometry = fixture;
                *geometry_fields(&mut geometry)[field] = invalid;
                assert_invalid(&Model {
                    entities: vec![Entity { id: 1, geometry }],
                    ..Model::default()
                });
            }
        }
    }
}

#[test]
fn every_constraint_numeric_field_rejects_nonfinite_rust_api_input() {
    let anchor = v(0.0, 0.0, 0.0);
    let direction = v(0.0, 0.0, 1.0);
    let reference = Some(v(1.0, 0.0, 0.0));
    let axis = Axis3 {
        origin: anchor,
        direction,
    };
    let plane = Plane3 {
        origin: anchor,
        normal: direction,
    };
    let fixtures = [
        ConstraintKind::Distance {
            a: 1,
            b: Some(2),
            value: 1.0,
            a_anchor: Some(anchor),
            b_anchor: Some(anchor),
        },
        ConstraintKind::Angle {
            a: 1,
            b: Some(2),
            value: 1.0,
            a_dir: Some(direction),
            b_dir: Some(direction),
        },
        ConstraintKind::Mate {
            a: 1,
            b: 2,
            a_plane: plane,
            b_plane: plane,
        },
        ConstraintKind::Coaxial {
            a: 1,
            b: 2,
            a_axis: axis,
            b_axis: axis,
        },
        ConstraintKind::Revolute {
            a: 1,
            b: 2,
            a_axis: axis,
            b_axis: axis,
            offset: Some(0.0),
            a_ref: reference,
            b_ref: reference,
            drive: Some(0.0),
        },
        ConstraintKind::Cylindrical {
            a: 1,
            b: 2,
            a_axis: axis,
            b_axis: axis,
            a_ref: reference,
            b_ref: reference,
        },
        ConstraintKind::Prismatic {
            a: 1,
            b: 2,
            a_axis: axis,
            b_axis: axis,
            a_ref: reference,
            b_ref: reference,
            roll: Some(0.0),
            drive: Some(0.0),
        },
        ConstraintKind::Spherical {
            a: 1,
            b: 2,
            a_point: anchor,
            b_point: anchor,
        },
        ConstraintKind::Transmission {
            joint_a: 1,
            joint_b: 2,
            ratio: 1.0,
        },
    ];
    for mut fixture in fixtures {
        let count = constraint_fields(&mut fixture).len();
        for field in 0..count {
            for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut kind = fixture;
                *constraint_fields(&mut kind)[field] = invalid;
                assert_invalid(&Model {
                    constraints: vec![Constraint {
                        id: 1,
                        kind,
                        label: None,
                    }],
                    ..Model::default()
                });
            }
        }
    }
}

#[test]
fn tolerance_must_be_finite_and_strictly_positive() {
    for tolerance in [0.0, -0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut model = point_distance(1.0, 0.0, &[1.0]);
        model.params.tolerance = tolerance;
        assert_invalid(&model);
    }
    for tolerance in ["0.0", "-1.0"] {
        let json = format!(
            "{{\"entities\":[],\"constraints\":[],\"params\":{{\"tolerance\":{tolerance}}}}}"
        );
        assert_invalid(&serde_json::from_str(&json).unwrap());
    }
}

#[test]
fn negative_circle_and_arc_radii_are_invalid() {
    for geometry in [
        Geometry::Circle2 {
            center: Point2 { x: 0.0, y: 0.0 },
            radius: -1.0,
        },
        Geometry::Arc2 {
            center: Point2 { x: 0.0, y: 0.0 },
            radius: -1.0,
            start_angle: 0.0,
            end_angle: 1.0,
        },
    ] {
        assert_invalid(&Model {
            entities: vec![Entity { id: 1, geometry }],
            ..Model::default()
        });
    }
}

#[test]
fn replay_of_irrational_solution_preserves_every_coordinate_bit() {
    let mut model = point_distance(1.0, 1.0, &[1.0]);
    let first = solve(&model).unwrap();
    assert_eq!(first.outcome, SolveOutcome::Converged);
    for _ in 0..10 {
        model.entities = first.entities.clone();
        let replay = solve(&model).unwrap();
        assert_eq!(replay.outcome, SolveOutcome::Converged);
        let Geometry::Point2 { x, y } = replay.entities[0].geometry else {
            unreachable!()
        };
        let Geometry::Point2 { x: old_x, y: old_y } = first.entities[0].geometry else {
            unreachable!()
        };
        assert_eq!(
            (x.to_bits(), y.to_bits()),
            (old_x.to_bits(), old_y.to_bits())
        );
    }
}

#[test]
fn satisfied_input_is_preserved_even_with_nonzero_residual_and_negative_zero() {
    let model = point_distance(-0.0, 1.0 + 0.25e-9, &[1.0, 1.0 + 0.5e-9]);
    let report = solve(&model).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    let Geometry::Point2 { x, y } = report.entities[0].geometry else {
        unreachable!()
    };
    assert_eq!(x.to_bits(), (-0.0f64).to_bits());
    assert_eq!(y.to_bits(), (1.0f64 + 0.25e-9).to_bits());
}

#[test]
fn extreme_scale_projection_neither_overflows_nor_loses_direction() {
    for (x, y, target) in [
        (3e200, 4e200, 10.0),
        (3e-200, 4e-200, 10.0),
        (f64::MAX, f64::MAX, 1.0),
        (f64::from_bits(1), f64::from_bits(1), 1.0),
        (1e-300, 0.0, 1e300),
        (1e300, 0.0, 1e-300),
    ] {
        let mut model = point_distance(x, y, &[target]);
        if target < 1e-100 {
            model.params.tolerance = 1e-320;
        }
        let report = solve(&model).unwrap();
        assert_eq!(
            report.outcome,
            SolveOutcome::Converged,
            "{x}, {y}, {target}"
        );
        let Geometry::Point2 { x: nx, y: ny } = report.entities[0].geometry else {
            unreachable!()
        };
        assert!(nx.is_finite() && ny.is_finite());
        assert!(report.diagnostics.residuals[0].residual.is_finite());
        assert!((libm::hypot(nx, ny) - target).abs() <= model.params.tolerance);
        assert!((nx * (y / x) - ny).abs() < 1e-12);
    }
}

#[test]
fn overflowed_residual_cannot_be_serialized_as_a_success_with_null() {
    let model = point_distance(1.0, 0.0, &[f64::MAX, -f64::MAX]);
    assert_invalid(&model);
}

#[test]
fn zero_distance_leaves_no_continuous_degree_of_freedom() {
    let report = solve(&point_distance(3.0, 4.0, &[0.0, 0.0])).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert_eq!(report.diagnostics.dof_total, 2);
    assert_eq!(report.diagnostics.dof_remaining, 0);
    assert_eq!(
        report.entities[0].geometry,
        Geometry::Point2 { x: 0.0, y: 0.0 }
    );
    assert_eq!(report.diagnostics.jacobian_condition_estimate, None);
}

#[test]
fn overlapping_distance_tolerance_intervals_do_not_prove_a_conflict() {
    let model = point_distance(2.0, 0.0, &[1.0, 1.0 + 1.5e-9]);
    let report = solve(&model).unwrap();
    assert_eq!(report.outcome, SolveOutcome::Converged);
    assert!(report.diagnostics.conflicts.is_empty());
    assert!(report
        .diagnostics
        .residuals
        .iter()
        .all(|residual| residual.residual.abs() <= model.params.tolerance));
}
