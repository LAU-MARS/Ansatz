//! 求解入口与 2D 闭式路径。
//!
//! 含 rigid3 实体的模型分流到 [`crate::assembly`]。纯 2D 模型目前只支持
//! 「2D 点 + 到原点的距离约束」，用闭式解 `p' = p · (d / ‖p‖)`——通用 2D
//! 草图约束接入后由通用引擎取代本路径。
//!
//! 闭式解用 libm 的 hypot 求模长，极端尺度下先缩放分量以免平方溢出。
//! 已满足全部约束的输入原样返回，保证结果回灌的逐位幂等。
//!
//! DOF 计数：每个非零目标距离留下圆周上的 1 DOF；零距离的解集只有
//! 原点，留下 0 DOF（这里不能用原点处退化的一阶梯度推断解集维数）。

use crate::model::{ConstraintKind, EntityId, Geometry, Model, SolveOutcome};
use crate::report::{
    Action, ConstraintGroup, ConstraintResidual, Diagnostics, SolveError, SolveReport, Suggestion,
};
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

/// 求解入口：接收模型，返回结构化结果。
///
/// - `Ok(report)`：求解层结论（含 `Inconsistent` 等失败结局 + 完整诊断）。
/// - `Err(error)`：工具级错误（模型无效 / 能力未实现），不是求解失败。
pub fn solve(model: &Model) -> Result<SolveReport, SolveError> {
    validate(model)?;

    if model
        .entities
        .iter()
        .any(|e| matches!(e.geometry, Geometry::Rigid3 { .. }))
    {
        return crate::assembly::solve(model);
    }

    // 1. 收集受支持的约束：Point2 到原点的距离，按目标点分组。
    let mut groups: BTreeMap<EntityId, Vec<(u32, f64)>> = BTreeMap::new();
    for constraint in &model.constraints {
        let ConstraintKind::Distance {
            a,
            b,
            value,
            a_anchor,
            b_anchor,
        } = &constraint.kind
        else {
            return Err(SolveError::UnsupportedConstraint {
                constraint_id: constraint.id,
                constraint_kind: constraint.kind.kind_name().into(),
                reason: String::from(
                    "2D 草图目前只实现「Point2 到原点的距离约束」，其余约束类型尚未提供。",
                ),
            });
        };
        let Some(entity) = model.entity(*a) else {
            return Err(SolveError::UnknownEntity {
                constraint_id: constraint.id,
                entity_id: *a,
            });
        };
        if !matches!(entity.geometry, Geometry::Point2 { .. }) {
            return Err(SolveError::UnsupportedConstraint {
                constraint_id: constraint.id,
                constraint_kind: "distance".into(),
                reason: String::from("距离约束的目标实体不是 Point2。"),
            });
        }
        if b.is_some() {
            return Err(SolveError::UnsupportedConstraint {
                constraint_id: constraint.id,
                constraint_kind: "distance".into(),
                reason: String::from("2D 草图目前仅支持到原点的距离（b 字段需为 null）。"),
            });
        }
        if a_anchor.is_some() || b_anchor.is_some() {
            return Err(SolveError::UnsupportedConstraint {
                constraint_id: constraint.id,
                constraint_kind: "distance".into(),
                reason: String::from("锚点（a_anchor/b_anchor）是 3D 装配语义，2D 点不支持。"),
            });
        }
        groups.entry(*a).or_default().push((constraint.id, *value));
    }

    // 2. DOF 计数（见模块注释）。
    let dof_total: u32 = model.entities.iter().map(|e| e.geometry.dof()).sum();
    let removed_dof: u32 = groups
        .values()
        .map(|constraints| {
            if constraints.iter().all(|&(_, d)| d == 0.0) {
                2
            } else {
                1
            }
        })
        .sum();
    let dof_remaining = dof_total.saturating_sub(removed_dof);
    let tol = model.params.tolerance;

    // 3. 逐点闭式求解 + 冲突/冗余分析。
    let n_groups = groups.len();
    // 条件数估计：单条归一化约束行的 κ 精确为 1；出现退化/重复即置 None。
    let mut condition = Some(1.0f64);
    let mut conflicts: Vec<ConstraintGroup> = Vec::new();
    let mut redundant: Vec<ConstraintGroup> = Vec::new();
    let mut residuals: Vec<ConstraintResidual> = Vec::new();
    let mut suggestions: Vec<Suggestion> = Vec::new();
    let mut solved_entities = model.entities.clone();

    for entity in &mut solved_entities {
        let Geometry::Point2 { x, y } = &mut entity.geometry else {
            continue;
        };
        let Some(constraints) = groups.remove(&entity.id) else {
            continue;
        };

        let d_min = constraints
            .iter()
            .fold(f64::INFINITY, |m, &(_, d)| m.min(d));
        let d_max = constraints
            .iter()
            .fold(f64::NEG_INFINITY, |m, &(_, d)| m.max(d));
        let all_ids: Vec<u32> = constraints.iter().map(|&(cid, _)| cid).collect();

        if d_min < 0.0 {
            let negative_ids: Vec<u32> = constraints
                .iter()
                .filter(|&(_, d)| *d < 0.0)
                .map(|&(cid, _)| cid)
                .collect();
            conflicts.push(ConstraintGroup {
                constraint_ids: negative_ids.clone(),
                human_message: format!(
                    "约束 {negative_ids:?} 的距离参数为负值：距离约束要求 value ≥ 0，\
                     这些约束不可能被任何几何满足。"
                ),
            });
            for &cid in &negative_ids {
                suggestions.push(Suggestion {
                    action: Action::RelaxConstraint { id: cid },
                    human_message: format!("把约束 {cid} 的距离值改为非负数。"),
                });
            }
        } else if (d_max - d_min) * 0.5 > tol {
            conflicts.push(ConstraintGroup {
                constraint_ids: all_ids.clone(),
                human_message: format!(
                    "约束 {all_ids:?} 对同一实体给出不一致的距离要求（最小 {d_min:e}，\
                     最大 {d_max:e}），没有距离能同时落在每条约束的容差 {tol:e} 内。"
                ),
            });
            for &cid in &all_ids[1..] {
                suggestions.push(Suggestion {
                    action: Action::RemoveConstraint { id: cid },
                    human_message: format!(
                        "删除约束 {cid}，或把它的距离值改到 {d_min:e}（与其他约束一致），\
                         其余约束才可能同时满足。"
                    ),
                });
            }
        } else if constraints.len() > 1 {
            redundant.push(ConstraintGroup {
                constraint_ids: all_ids.clone(),
                human_message: format!(
                    "约束 {all_ids:?} 相互重复（线性相关）：它们约束的是同一个标量 ‖p‖，\
                     删除其中任意一条都不改变解集。"
                ),
            });
            let last = all_ids[all_ids.len() - 1];
            suggestions.push(Suggestion {
                action: Action::RemoveConstraint { id: last },
                human_message: format!("约束 {last} 是冗余的，可以删除。"),
            });
        }

        // 多条距离的容差区间相交时，必要时选中点；否则仍取最大距离的
        // 特解来报告诚实残差。负距离不可行，其特解取原点。
        let d_ref = if d_min >= 0.0 && d_max - d_min > tol && (d_max - d_min) * 0.5 <= tol {
            d_min + (d_max - d_min) * 0.5
        } else {
            d_max.max(0.0)
        };
        let r = libm::hypot(*x, *y);
        let already_satisfied =
            r.is_finite() && constraints.iter().all(|&(_, d)| (r - d).abs() <= tol);
        if !already_satisfied {
            // 先检查模长再投影，防止一个已经满足的解因舍入反复移动。
            // 通常尺度保留原有计算顺序；模长/比例溢出或下溢时使用缩放。
            let factor = d_ref / r;
            let (nx, ny) = if r >= f64::MIN_POSITIVE
                && r.is_finite()
                && factor.is_finite()
                && (factor >= f64::MIN_POSITIVE || d_ref == 0.0)
            {
                (*x * factor, *y * factor)
            } else {
                let scale = x.abs().max(y.abs());
                if scale > 0.0 {
                    let sx = *x / scale;
                    let sy = *y / scale;
                    let length = libm::hypot(sx, sy);
                    ((sx / length) * d_ref, (sy / length) * d_ref)
                } else {
                    // 原点没有方向：确定性选择 +x。
                    (d_ref, 0.0)
                }
            };
            *x = nx;
            *y = ny;
        }
        let solved_radius = libm::hypot(*x, *y);
        for &(cid, d) in &constraints {
            let residual = solved_radius - d;
            if !residual.is_finite() {
                return Err(SolveError::InvalidModel {
                    reason: format!(
                        "约束 {cid} 的距离残差超出 f64 可表示范围；请缩放模型的长度单位。"
                    ),
                });
            }
            residuals.push(ConstraintResidual {
                constraint_id: cid,
                residual,
                human_message: format!(
                    "约束 {cid} 在返回的特解处残差为 {residual:e}（即 ‖p‖ − {d:e}）。"
                ),
            });
        }
        if constraints.len() != 1 || r == 0.0 || solved_radius == 0.0 {
            // 行重复（秩亏）或原点退化（梯度为零）时条件数无意义。
            condition = None;
        }
    }
    if n_groups == 0 {
        condition = None; // 空雅可比谈不上条件数。
    }

    if dof_remaining > 0 {
        suggestions.push(Suggestion {
            action: Action::AddConstraint { dof_remaining },
            human_message: format!(
                "模型还有 {dof_remaining} 个自由度未被约束。以「点到原点的距离」为例：\
                 它只固定了模长，点仍可沿圆周滑动，需要再施加角度或坐标类约束。"
            ),
        });
    }

    let max_residual = residuals
        .iter()
        .max_by(|a, b| a.residual.abs().total_cmp(&b.residual.abs()))
        .cloned();
    let satisfied = max_residual
        .as_ref()
        .map_or(true, |r| r.residual.abs() <= tol);

    let outcome = if !conflicts.is_empty() {
        SolveOutcome::Inconsistent
    } else if satisfied {
        SolveOutcome::Converged
    } else {
        SolveOutcome::NotConverged
    };

    Ok(SolveReport {
        outcome,
        entities: solved_entities,
        diagnostics: Diagnostics {
            dof_total,
            dof_remaining,
            iterations: 0,
            conflicts,
            redundant_constraints: redundant,
            residuals,
            max_residual,
            suggestions,
            joints: Vec::new(),
            jacobian_condition_estimate: condition,
        },
    })
}

/// 结构与数值校验：必须在分流前执行，Rust API 与 JSON 外壳共用。
fn validate(model: &Model) -> Result<(), SolveError> {
    if !model.params.tolerance.is_finite() || model.params.tolerance <= 0.0 {
        return Err(SolveError::InvalidModel {
            reason: String::from("params.tolerance 必须是有限的正数。"),
        });
    }
    let mut seen_entities = BTreeSet::new();
    for entity in &model.entities {
        if !seen_entities.insert(entity.id) {
            return Err(SolveError::InvalidModel {
                reason: format!("实体 id {} 重复，id 必须在模型内唯一。", entity.id),
            });
        }
        let context = format!("实体 {}（{}）", entity.id, entity.geometry.kind_name());
        match entity.geometry {
            Geometry::Point2 { x, y } => finite(&[x, y], &context)?,
            Geometry::Line2 { start, end } => {
                finite(&[start.x, start.y, end.x, end.y], &context)?;
            }
            Geometry::Circle2 { center, radius } => {
                finite(&[center.x, center.y, radius], &context)?;
                nonnegative_radius(radius, &context)?;
            }
            Geometry::Arc2 {
                center,
                radius,
                start_angle,
                end_angle,
            } => {
                finite(
                    &[center.x, center.y, radius, start_angle, end_angle],
                    &context,
                )?;
                nonnegative_radius(radius, &context)?;
            }
            Geometry::Rigid3 { pose } => {
                finite_vecs(&[pose.translation], &context)?;
                finite(&pose.rotation.vector, &context)?;
            }
        }
    }
    let mut seen_constraints = BTreeSet::new();
    for constraint in &model.constraints {
        if !seen_constraints.insert(constraint.id) {
            return Err(SolveError::InvalidModel {
                reason: format!("约束 id {} 重复，id 必须在模型内唯一。", constraint.id),
            });
        }
        let context = format!("约束 {}（{}）", constraint.id, constraint.kind.kind_name());
        match constraint.kind {
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
                finite(&[value], &context)?;
                finite_optional_vecs(&[a_anchor, b_anchor], &context)?;
            }
            ConstraintKind::Mate {
                a_plane, b_plane, ..
            } => finite_vecs(
                &[
                    a_plane.origin,
                    a_plane.normal,
                    b_plane.origin,
                    b_plane.normal,
                ],
                &context,
            )?,
            ConstraintKind::Coaxial { a_axis, b_axis, .. } => {
                finite_axes(a_axis, b_axis, &context)?;
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
                finite_axes(a_axis, b_axis, &context)?;
                finite_optional_vecs(&[a_ref, b_ref], &context)?;
                finite_optional(&[offset, drive], &context)?;
            }
            ConstraintKind::Cylindrical {
                a_axis,
                b_axis,
                a_ref,
                b_ref,
                ..
            } => {
                finite_axes(a_axis, b_axis, &context)?;
                finite_optional_vecs(&[a_ref, b_ref], &context)?;
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
                finite_axes(a_axis, b_axis, &context)?;
                finite_optional_vecs(&[a_ref, b_ref], &context)?;
                finite_optional(&[roll, drive], &context)?;
            }
            ConstraintKind::Spherical {
                a_point, b_point, ..
            } => finite_vecs(&[a_point, b_point], &context)?,
            ConstraintKind::Transmission { ratio, .. } => finite(&[ratio], &context)?,
            ConstraintKind::Coincident { .. }
            | ConstraintKind::Collinear { .. }
            | ConstraintKind::Parallel { .. }
            | ConstraintKind::Perpendicular { .. }
            | ConstraintKind::Tangent { .. }
            | ConstraintKind::Symmetric { .. }
            | ConstraintKind::Fixed { .. } => {}
        }
    }
    Ok(())
}

fn finite(values: &[f64], context: &str) -> Result<(), SolveError> {
    if values.iter().any(|value| !value.is_finite()) {
        return Err(SolveError::InvalidModel {
            reason: format!("{context} 的数值参数必须全部有限，不能包含 NaN 或无穷大。"),
        });
    }
    Ok(())
}

fn finite_optional(values: &[Option<f64>], context: &str) -> Result<(), SolveError> {
    for value in values.iter().flatten() {
        finite(&[*value], context)?;
    }
    Ok(())
}

fn finite_vecs(values: &[crate::model::Vec3], context: &str) -> Result<(), SolveError> {
    for value in values {
        finite(&[value.x, value.y, value.z], context)?;
    }
    Ok(())
}

fn finite_optional_vecs(
    values: &[Option<crate::model::Vec3>],
    context: &str,
) -> Result<(), SolveError> {
    for value in values.iter().flatten() {
        finite_vecs(&[*value], context)?;
    }
    Ok(())
}

fn finite_axes(
    a: crate::model::Axis3,
    b: crate::model::Axis3,
    context: &str,
) -> Result<(), SolveError> {
    finite_vecs(&[a.origin, a.direction, b.origin, b.direction], context)
}

fn nonnegative_radius(radius: f64, context: &str) -> Result<(), SolveError> {
    if radius < 0.0 {
        return Err(SolveError::InvalidModel {
            reason: format!("{context} 的半径必须非负。"),
        });
    }
    Ok(())
}
