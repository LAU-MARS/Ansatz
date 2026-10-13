//! Independent validation of candidate reports.
//!
//! `ansatz_core` supplies only the JSON data types. This module deliberately
//! never calls its solver, residual evaluation, SO(3), or linear algebra code.
//! Rotations are reconstructed with quaternions, and geometric relationships
//! are measured directly from the returned entities. A candidate's reported
//! residuals and frozen example poses are not evidence of feasibility.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use ansatz_core::model::Axis3;
use ansatz_core::{
    Action, ConstraintKind, Geometry, Model, Pose3, SolveOutcome, SolveReport, Vec3,
};

/// Parse the complete repository JSON contract before checking its semantics.
/// Missing required fields, unknown outcome/geometry tags, and invalid numeric
/// types are rejected even when an expected.json leaves those fields open.
pub fn parse_and_validate_report(
    model_json: &str,
    report_json: &str,
) -> Result<SolveReport, Vec<String>> {
    let model: Model = serde_json::from_str(model_json)
        .map_err(|e| vec![format!("语料模型不符合完整 JSON 契约：{e}")])?;
    let report: SolveReport = serde_json::from_str(report_json)
        .map_err(|e| vec![format!("报告不符合完整 JSON 契约：{e}")])?;
    // serde's flatten cannot deny unknown fields on Suggestion. Check this one
    // contract seam explicitly rather than silently accepting extra fields.
    let raw: serde_json::Value =
        serde_json::from_str(report_json).map_err(|e| vec![format!("报告 JSON 无效：{e}")])?;
    if let Some(suggestions) = raw["diagnostics"]["suggestions"].as_array() {
        for suggestion in suggestions {
            let Some(object) = suggestion.as_object() else {
                return Err(vec!["suggestion 必须是对象".into()]);
            };
            let parameter = match suggestion["action"].as_str() {
                Some("add_constraint") => "dof_remaining",
                _ => "id",
            };
            if object
                .keys()
                .any(|key| !["action", "human_message", parameter].contains(&key.as_str()))
            {
                return Err(vec!["suggestion 含有未知字段".into()]);
            }
        }
    }
    let failures = validate_report(&model, &report);
    if failures.is_empty() {
        Ok(report)
    } else {
        Err(failures)
    }
}

/// Check structure for all outcomes and independently check feasibility for
/// `converged`. Under-constrained models may return any satisfying solution.
pub fn validate_report(model: &Model, report: &SolveReport) -> Vec<String> {
    let mut failures = Vec::new();
    let tol = model.params.tolerance;
    if !tol.is_finite() || tol <= 0.0 {
        return vec!["语料模型 tolerance 必须为有限正数".into()];
    }
    let mut input_entities = BTreeMap::new();
    for entity in &model.entities {
        if input_entities.insert(entity.id, entity.geometry).is_some() {
            failures.push(format!("语料实体 id={} 重复", entity.id));
        }
    }
    let mut entities = BTreeMap::new();
    for entity in &report.entities {
        if entities.insert(entity.id, entity.geometry).is_some() {
            failures.push(format!("报告实体 id={} 重复", entity.id));
        }
        if !finite_geometry(entity.geometry) {
            failures.push(format!("报告实体 {} 含非有限数值", entity.id));
        }
        match input_entities.get(&entity.id) {
            Some(input) if geometry_tag(*input) != geometry_tag(entity.geometry) => {
                failures.push(format!("报告实体 {} 改变了几何类型", entity.id));
            }
            None => failures.push(format!("报告含有模型中不存在的实体 {}", entity.id)),
            _ => {}
        }
    }
    if input_entities.keys().collect::<Vec<_>>() != entities.keys().collect::<Vec<_>>() {
        failures.push("报告实体 ID 集合与输入模型不完整对应".into());
    }
    let mut constraints = BTreeMap::new();
    for constraint in &model.constraints {
        if constraints
            .insert(constraint.id, &constraint.kind)
            .is_some()
        {
            failures.push(format!("语料约束 id={} 重复", constraint.id));
        }
        if let ConstraintKind::Fixed { a } = constraint.kind {
            if input_entities.get(&a) != entities.get(&a) {
                failures.push(format!("fixed 约束 {} 的实体 {a} 被改变", constraint.id));
            }
        }
    }
    failures.extend(validate_model_domain(model, &input_entities));
    let diag = &report.diagnostics;
    let dof_total: u64 = model
        .entities
        .iter()
        .map(|e| match e.geometry {
            Geometry::Point2 { .. } => 2,
            Geometry::Line2 { .. } => 4,
            Geometry::Circle2 { .. } => 3,
            Geometry::Arc2 { .. } => 5,
            Geometry::Rigid3 { .. } => 6,
        })
        .sum();
    if u64::from(diag.dof_total) != dof_total || diag.dof_remaining > diag.dof_total {
        failures.push("诊断 DOF 总量或剩余量与模型不符".into());
    }
    if diag.iterations > model.params.max_iterations {
        failures.push("报告迭代次数超过模型 max_iterations".into());
    }
    if (report.outcome == SolveOutcome::Inconsistent) == diag.conflicts.is_empty() {
        failures.push("outcome 与 conflicts 不符：仅 inconsistent 必须有冲突组".into());
    }
    for (name, groups) in [
        ("conflicts", &diag.conflicts),
        ("redundant_constraints", &diag.redundant_constraints),
    ] {
        for group in groups {
            let ids: BTreeSet<_> = group.constraint_ids.iter().copied().collect();
            if ids.is_empty()
                || ids.len() != group.constraint_ids.len()
                || ids.iter().any(|id| !constraints.contains_key(id))
            {
                failures.push(format!("{name} 包含空组、重复或不存在的约束 ID"));
            }
        }
    }
    let mut residual_ids = BTreeSet::new();
    for residual in &diag.residuals {
        if !residual.residual.is_finite()
            || !constraints.contains_key(&residual.constraint_id)
            || !residual_ids.insert(residual.constraint_id)
        {
            failures.push("residuals 含非有限数值、重复或不存在的约束 ID".into());
        }
        if report.outcome == SolveOutcome::Converged && residual.residual.abs() > tol {
            failures.push(format!(
                "converged 报告自身残差 {} 超过 tolerance",
                residual.constraint_id
            ));
        }
    }
    if let Some(maximum) = &diag.max_residual {
        let matching = diag
            .residuals
            .iter()
            .find(|r| r.constraint_id == maximum.constraint_id && r.residual == maximum.residual);
        let largest = diag
            .residuals
            .iter()
            .map(|r| r.residual.abs())
            .fold(0.0f64, f64::max);
        if !maximum.residual.is_finite() || matching.is_none() || maximum.residual.abs() != largest
        {
            failures.push("max_residual 与 residuals 的最大项不一致".into());
        }
    }
    if diag
        .jacobian_condition_estimate
        .is_some_and(|value| !value.is_finite() || value < 1.0)
    {
        failures.push("jacobian_condition_estimate 必须为空或有限且不小于 1".into());
    }
    for suggestion in &diag.suggestions {
        match suggestion.action {
            Action::RemoveConstraint { id } | Action::RelaxConstraint { id }
                if !constraints.contains_key(&id) =>
            {
                failures.push(format!("建议引用了不存在的约束 {id}"));
            }
            Action::AddConstraint { dof_remaining }
                if dof_remaining == 0 || dof_remaining != diag.dof_remaining =>
            {
                failures.push("add_constraint 建议的 DOF 与诊断不符".into());
            }
            _ => {}
        }
    }
    let mut joint_ids = BTreeSet::new();
    for joint in &diag.joints {
        let kind = constraints.get(&joint.constraint_id).copied();
        if !joint.value.is_finite()
            || !joint_ids.insert(joint.constraint_id)
            || kind.and_then(joint_tag) != Some(joint.joint_kind.as_str())
        {
            failures.push("joints 含非有限数值、重复 ID 或错误关节类型".into());
        }
    }
    let expected_residuals: BTreeSet<_> = model
        .constraints
        .iter()
        .filter(|c| !matches!(c.kind, ConstraintKind::Fixed { .. }))
        .map(|c| c.id)
        .collect();
    if expected_residuals != residual_ids {
        failures.push("报告缺少非 fixed 约束的 residuals，或含不应报告的约束 ID".into());
    }
    let expected_joints: BTreeSet<_> = model
        .constraints
        .iter()
        .filter(|c| joint_tag(&c.kind).is_some())
        .map(|c| c.id)
        .collect();
    if joint_ids != expected_joints {
        failures.push("报告缺少关节状态".into());
    }
    if report.outcome != SolveOutcome::Converged || !failures.is_empty() {
        return failures;
    }
    for constraint in &model.constraints {
        if let Err(reason) = check_constraint(constraint.kind, &entities, &constraints, tol) {
            failures.push(format!("独立几何检查：约束 {} {reason}", constraint.id));
        }
    }
    for joint in &diag.joints {
        if let Some(kind) = constraints.get(&joint.constraint_id) {
            let result = joint_value(**kind, &entities).and_then(|value| {
                let error = if joint.joint_kind == "prismatic" {
                    value - joint.value
                } else {
                    angle_difference(value, joint.value)
                };
                scalar(error, tol, "报告关节变量与几何不符")
            });
            if let Err(reason) = result {
                failures.push(format!("关节 {} {reason}", joint.constraint_id));
            }
        }
    }
    failures
}

fn geometry_tag(g: Geometry) -> &'static str {
    match g {
        Geometry::Point2 { .. } => "point2",
        Geometry::Line2 { .. } => "line2",
        Geometry::Circle2 { .. } => "circle2",
        Geometry::Arc2 { .. } => "arc2",
        Geometry::Rigid3 { .. } => "rigid3",
    }
}

fn finite_geometry(g: Geometry) -> bool {
    match g {
        Geometry::Point2 { x, y } => x.is_finite() && y.is_finite(),
        Geometry::Line2 { start, end } => [start.x, start.y, end.x, end.y]
            .iter()
            .all(|v| v.is_finite()),
        Geometry::Circle2 { center, radius } => {
            [center.x, center.y, radius].iter().all(|v| v.is_finite())
        }
        Geometry::Arc2 {
            center,
            radius,
            start_angle,
            end_angle,
        } => [center.x, center.y, radius, start_angle, end_angle]
            .iter()
            .all(|v| v.is_finite()),
        Geometry::Rigid3 { pose } => array(pose.translation)
            .iter()
            .chain(pose.rotation.vector.iter())
            .all(|v| v.is_finite()),
    }
}

fn joint_tag(kind: &ConstraintKind) -> Option<&'static str> {
    match kind {
        ConstraintKind::Revolute { .. } => Some("revolute"),
        ConstraintKind::Cylindrical { .. } => Some("cylindrical"),
        ConstraintKind::Prismatic { .. } => Some("prismatic"),
        _ => None,
    }
}

type Entities = BTreeMap<u32, Geometry>;
type Constraints<'a> = BTreeMap<u32, &'a ConstraintKind>;
type V = [f64; 3];

// Reports are only meaningful for supported model domains. Tool-error corpus
// cases bypass this oracle; a candidate cannot turn invalid/unsupported input
// into a solve-level report by returning plausible geometry.
fn validate_model_domain(model: &Model, entities: &Entities) -> Vec<String> {
    let mut failures = Vec::new();
    let has_rigid = entities
        .values()
        .any(|g| matches!(g, Geometry::Rigid3 { .. }));
    if has_rigid
        && entities
            .values()
            .any(|g| !matches!(g, Geometry::Rigid3 { .. }))
    {
        failures.push("语料模型混合 2D 与 rigid3 实体，不能返回求解层报告".into());
    }
    for entity in &model.entities {
        if !finite_geometry(entity.geometry)
            || matches!(entity.geometry, Geometry::Circle2 { radius, .. } | Geometry::Arc2 { radius, .. } if radius < 0.0)
        {
            failures.push(format!("语料实体 {} 几何数值无效", entity.id));
        }
    }
    for constraint in &model.constraints {
        let pair = match constraint.kind {
            ConstraintKind::Distance {
                a,
                b,
                a_anchor,
                b_anchor,
                ..
            } => {
                if matches!(entities.get(&a), Some(Geometry::Point2 { .. })) {
                    if b.is_some() || a_anchor.is_some() || b_anchor.is_some() {
                        failures.push(format!(
                            "约束 {} 的 2D distance 不支持 b 或 3D anchor",
                            constraint.id
                        ));
                    }
                    None
                } else {
                    b.map(|b| (a, b)).or_else(|| {
                        failures.push(format!("约束 {} 的 3D distance 缺少 b", constraint.id));
                        None
                    })
                }
            }
            ConstraintKind::Angle { a, b, .. } => b.map(|b| (a, b)).or_else(|| {
                failures.push(format!("约束 {} 的 3D angle 缺少 b", constraint.id));
                None
            }),
            ConstraintKind::Mate { a, b, .. }
            | ConstraintKind::Coaxial { a, b, .. }
            | ConstraintKind::Revolute { a, b, .. }
            | ConstraintKind::Cylindrical { a, b, .. }
            | ConstraintKind::Prismatic { a, b, .. }
            | ConstraintKind::Spherical { a, b, .. } => Some((a, b)),
            ConstraintKind::Fixed { a } => {
                if !matches!(entities.get(&a), Some(Geometry::Rigid3 { .. })) {
                    failures.push(format!("约束 {} 的 fixed 实体不是 rigid3", constraint.id));
                }
                None
            }
            ConstraintKind::Transmission { .. } => None,
            _ => {
                failures.push(format!("约束 {} 使用当前不支持的类型", constraint.id));
                None
            }
        };
        if let Some((a, b)) = pair {
            if a == b
                || [a, b]
                    .iter()
                    .any(|id| !matches!(entities.get(id), Some(Geometry::Rigid3 { .. })))
            {
                failures.push(format!(
                    "约束 {} 必须引用两个不同且存在的 rigid3 实体",
                    constraint.id
                ));
            }
        }
    }
    failures
}

fn check_constraint(
    kind: ConstraintKind,
    entities: &Entities,
    constraints: &Constraints<'_>,
    tol: f64,
) -> Result<(), String> {
    match kind {
        ConstraintKind::Fixed { a } => {
            pose(entities, a)?;
            Ok(()) // Exact invariance was checked against the input above.
        }
        ConstraintKind::Distance {
            a,
            b,
            value,
            a_anchor,
            b_anchor,
        } => {
            if !value.is_finite() || value < 0.0 {
                return Err("distance 的目标必须是有限非负数".into());
            }
            let distance = match (entities.get(&a), b) {
                (Some(Geometry::Point2 { x, y }), None) => x.hypot(*y),
                (Some(Geometry::Rigid3 { .. }), Some(b)) => norm(sub(
                    point(pose(entities, a)?, a_anchor.map(array).unwrap_or([0.0; 3]))?,
                    point(pose(entities, b)?, b_anchor.map(array).unwrap_or([0.0; 3]))?,
                )),
                _ => return Err("distance 使用了当前不支持的实体组合".into()),
            };
            scalar(distance - value, tol, "锚点距离不满足目标")
        }
        ConstraintKind::Angle {
            a,
            b: Some(b),
            value,
            a_dir,
            b_dir,
        } => {
            if !value.is_finite() || !(0.0..=PI).contains(&value) {
                return Err("angle 的目标必须在 [0, π]".into());
            }
            let (Some(a_dir), Some(b_dir)) = (a_dir, b_dir) else {
                return Err("angle 缺少必须的 a_dir 或 b_dir".into());
            };
            let da = direction(pose(entities, a)?, array(a_dir))?;
            let db = direction(pose(entities, b)?, array(b_dir))?;
            let angle = norm(cross(da, db)).atan2(dot(da, db));
            scalar(angle - value, tol, "真实方向夹角不满足目标")
        }
        ConstraintKind::Mate {
            a,
            b,
            a_plane,
            b_plane,
        } => {
            let pa = pose(entities, a)?;
            let pb = pose(entities, b)?;
            let na = direction(pa, array(a_plane.normal))?;
            let nb = direction(pb, array(b_plane.normal))?;
            vector(add(na, nb), tol, "贴合法向没有反向")?;
            scalar(
                dot(
                    sub(
                        point(pa, array(a_plane.origin))?,
                        point(pb, array(b_plane.origin))?,
                    ),
                    nb,
                ),
                tol,
                "贴合平面没有重合",
            )
        }
        ConstraintKind::Coaxial {
            a,
            b,
            a_axis,
            b_axis,
        }
        | ConstraintKind::Cylindrical {
            a,
            b,
            a_axis,
            b_axis,
            ..
        } => coaxial(entities, a, b, a_axis, b_axis, tol).map(|_| ()),
        ConstraintKind::Revolute {
            a,
            b,
            a_axis,
            b_axis,
            offset,
            drive,
            ..
        } => {
            let (pa, pb, axis) = coaxial(entities, a, b, a_axis, b_axis, tol)?;
            scalar(
                dot(sub(pa, pb), axis) - offset.unwrap_or(0.0),
                tol,
                "转动副轴向间距不符",
            )?;
            if let Some(target) = drive {
                scalar(
                    angle_difference(joint_value(kind, entities)?, target),
                    tol,
                    "转动副驱动角不符",
                )?;
            }
            Ok(())
        }
        ConstraintKind::Prismatic {
            a,
            b,
            a_axis,
            b_axis,
            a_ref,
            b_ref,
            roll,
            drive,
        } => {
            coaxial(entities, a, b, a_axis, b_axis, tol)?;
            match (a_ref, b_ref, roll) {
                (None, None, None) => {
                    scalar(
                        theta(entities, a, b, a_axis, b_axis, None, None)?,
                        tol,
                        "移动副缺省零滚转不符",
                    )?;
                }
                (Some(_), Some(_), Some(target))
                    if target.is_finite() && (0.0..=PI).contains(&target) =>
                {
                    scalar(
                        angle_difference(
                            theta(entities, a, b, a_axis, b_axis, a_ref, b_ref)?,
                            target,
                        ),
                        tol,
                        "移动副有向滚转不符",
                    )?;
                }
                _ => return Err("移动副滚转必须同时提供两个参考方向和 roll，或全部省略".into()),
            }
            if let Some(target) = drive {
                scalar(
                    joint_value(kind, entities)? - target,
                    tol,
                    "移动副驱动位移不符",
                )?;
            }
            Ok(())
        }
        ConstraintKind::Spherical {
            a,
            b,
            a_point,
            b_point,
        } => vector(
            sub(
                point(pose(entities, a)?, array(a_point))?,
                point(pose(entities, b)?, array(b_point))?,
            ),
            tol,
            "球副锚点没有重合",
        ),
        ConstraintKind::Transmission {
            joint_a,
            joint_b,
            ratio,
        } => {
            if joint_a == joint_b || !ratio.is_finite() {
                return Err("传动要求不同的转动副及有限速比".into());
            }
            let revolute = |id| match constraints.get(&id) {
                Some(kind @ ConstraintKind::Revolute { .. }) => Ok(**kind),
                _ => Err("传动引用了不存在的转动副".to_string()),
            };
            let ta = joint_value(revolute(joint_a)?, entities)?;
            let tb = joint_value(revolute(joint_b)?, entities)?;
            scalar(angle_difference(tb, ratio * ta), tol, "传动角度不满足速比")
        }
        _ => Err("使用了当前不支持的约束组合，却报告 converged".into()),
    }
}

fn coaxial(
    entities: &Entities,
    a: u32,
    b: u32,
    a_axis: Axis3,
    b_axis: Axis3,
    tol: f64,
) -> Result<(V, V, V), String> {
    let a = pose(entities, a)?;
    let b = pose(entities, b)?;
    let da = direction(a, array(a_axis.direction))?;
    let db = direction(b, array(b_axis.direction))?;
    let pa = point(a, array(a_axis.origin))?;
    let pb = point(b, array(b_axis.origin))?;
    vector(sub(da, db), tol, "轴方向没有同向对齐")?;
    vector(cross(sub(pa, pb), db), tol, "两根轴没有共线")?;
    Ok((pa, pb, da))
}

fn joint_value(kind: ConstraintKind, entities: &Entities) -> Result<f64, String> {
    match kind {
        ConstraintKind::Revolute {
            a,
            b,
            a_axis,
            b_axis,
            a_ref,
            b_ref,
            ..
        }
        | ConstraintKind::Cylindrical {
            a,
            b,
            a_axis,
            b_axis,
            a_ref,
            b_ref,
        } => theta(entities, a, b, a_axis, b_axis, a_ref, b_ref),
        ConstraintKind::Prismatic {
            a,
            b,
            a_axis,
            b_axis,
            ..
        } => {
            let a = pose(entities, a)?;
            let b = pose(entities, b)?;
            Ok(dot(
                sub(
                    point(b, array(b_axis.origin))?,
                    point(a, array(a_axis.origin))?,
                ),
                direction(a, array(a_axis.direction))?,
            ))
        }
        _ => Err("不是受支持的关节类型".into()),
    }
}

fn theta(
    entities: &Entities,
    a: u32,
    b: u32,
    a_axis: Axis3,
    b_axis: Axis3,
    a_ref: Option<Vec3>,
    b_ref: Option<Vec3>,
) -> Result<f64, String> {
    let pa = pose(entities, a)?;
    let pb = pose(entities, b)?;
    let da = unit(array(a_axis.direction))?;
    let db = unit(array(b_axis.direction))?;
    let ua = rotate(pa, reference(da, a_ref)?)?;
    let ub = rotate(pb, reference(db, b_ref)?)?;
    let axis = rotate(pa, da)?;
    let sine = dot(axis, cross(ua, ub));
    let cosine = dot(ua, ub);
    if sine == 0.0 && cosine == 0.0 {
        return Err("关节参考方向在返回位形处无法定义转角".into());
    }
    Ok(wrap(sine.atan2(cosine)))
}

fn reference(axis: V, given: Option<Vec3>) -> Result<V, String> {
    if let Some(given) = given {
        let given = unit(array(given))?;
        let projection = sub(given, scale(axis, dot(axis, given)));
        if norm(projection) <= 1e-12 {
            return Err("关节参考方向平行于轴或为零".into());
        }
        unit(projection)
    } else {
        // This is the documented deterministic default, not a solver helper.
        let mut index = 0;
        for i in 1..3 {
            if axis[i].abs() < axis[index].abs() {
                index = i;
            }
        }
        let mut basis = [0.0; 3];
        basis[index] = 1.0;
        unit(cross(axis, basis))
    }
}

fn pose(entities: &Entities, id: u32) -> Result<Pose3, String> {
    match entities.get(&id) {
        Some(Geometry::Rigid3 { pose }) => Ok(*pose),
        _ => Err(format!("实体 {id} 不存在或不是 rigid3")),
    }
}

fn point(pose: Pose3, local: V) -> Result<V, String> {
    let result = add(array(pose.translation), rotate(pose, local)?);
    if result.iter().all(|v| v.is_finite()) {
        Ok(result)
    } else {
        Err("变换后的点不是有限值".into())
    }
}

fn direction(pose: Pose3, local: V) -> Result<V, String> {
    unit(rotate(pose, unit(local)?)?)
}

// Quaternion reconstruction differs from the product's Rodrigues matrices.
fn rotate(pose: Pose3, local: V) -> Result<V, String> {
    let rotation = pose.rotation.vector;
    let angle = norm(rotation);
    if !angle.is_finite() || local.iter().any(|v| !v.is_finite()) {
        return Err("旋转或局部几何不是有限值".into());
    }
    if angle == 0.0 {
        return Ok(local);
    }
    let half = angle / 2.0;
    let q = scale(unit(rotation)?, half.sin());
    let q_cross = cross(q, local);
    let result = add(
        local,
        scale(add(scale(q_cross, half.cos()), cross(q, q_cross)), 2.0),
    );
    if result.iter().all(|v| v.is_finite()) {
        Ok(result)
    } else {
        Err("旋转后的几何不是有限值".into())
    }
}

fn array(v: Vec3) -> V {
    [v.x, v.y, v.z]
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(v: V, k: f64) -> V {
    [v[0] * k, v[1] * k, v[2] * k]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(v: V) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}
fn unit(v: V) -> Result<V, String> {
    let largest = v.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
    if !largest.is_finite() || largest == 0.0 || v.iter().any(|x| !x.is_finite()) {
        return Err("方向必须是有限非零向量".into());
    }
    let scaled = [v[0] / largest, v[1] / largest, v[2] / largest];
    Ok(scale(scaled, 1.0 / norm(scaled)))
}
fn wrap(angle: f64) -> f64 {
    let value = angle % TAU;
    if value > PI {
        value - TAU
    } else if value <= -PI {
        value + TAU
    } else {
        value
    }
}
fn angle_difference(a: f64, b: f64) -> f64 {
    wrap(wrap(a) - wrap(b))
}
fn scalar(error: f64, tol: f64, what: &str) -> Result<(), String> {
    if error.is_finite() && error.abs() <= tol {
        Ok(())
    } else {
        Err(format!("{what}：误差 {error:e} 超过容差 {tol:e}"))
    }
}
fn vector(error: V, tol: f64, what: &str) -> Result<(), String> {
    for value in error {
        scalar(value, tol, what)?;
    }
    Ok(())
}
