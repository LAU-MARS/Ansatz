//! 骨架阶段的占位求解入口。
//!
//! **边界（严格遵守）**：只实现「2D 点 + 到原点的距离约束」这一个平凡算例，
//! 用闭式解 `p' = p · (d / ‖p‖)`，不写通用 Newton 迭代。目的是让端到端
//! 管线与诊断框架以真实形态跑通；通用数值求解（阻尼 Newton / Levenberg–
//! Marquardt、稀疏雅可比、秩分析）属于后续阶段，届时只替换本模块，
//! 数据模型与诊断结构不动。
//!
//! 闭式解的数值性质（与根 Cargo.toml 的位级可复现性承诺对应）：
//! 只用到 `+ · / sqrt`，全部是 IEEE 754 正确舍入运算，因此结果在
//! x86-64 / aarch64 / wasm32 上逐位一致（见 `tests/parity`）。
//!
//! DOF 计数说明：骨架阶段用通用近似——每实体按 [`Geometry::dof`] 计总数，
//! 每个受支持的距离约束贡献秩 1（约束同一标量 `‖p‖` 的重复约束合并计秩 1）。
//! 通用求解器接入后这里替换为真实的雅可比秩分析。

use crate::model::{ConstraintKind, EntityId, Geometry, Model, SolveOutcome};
use crate::report::{
    Action, ConstraintResidual, Diagnostics, RedundancyGroup, SolveError, SolveReport, Suggestion,
};
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

/// 求解入口：接收模型，返回结构化结果。
///
/// - `Ok(report)`：求解层结论（含 `Inconsistent` 等失败结局 + 完整诊断）。
/// - `Err(error)`：工具级错误（模型无效 / 能力未实现），不是求解失败。
///
/// 路径分发：模型含 rigid3 实体 → [`crate::assembly`]（3D 装配，LM 数值求解）；
/// 纯 2D 模型 → 本模块的闭式解路径（位级契约由 tests/parity 冻结，保持不变）。
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
                    "骨架阶段只实现「Point2 到原点的距离约束」，其余约束类型在后续阶段提供。",
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
                reason: String::from("骨架阶段仅支持到原点的距离（b 字段需为 null）。"),
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

    // 2. DOF 计数（通用近似，见模块注释）。
    let dof_total: u32 = model.entities.iter().map(|e| e.geometry.dof()).sum();
    let constrained_points = groups.len() as u32;
    let dof_remaining = dof_total.saturating_sub(constrained_points);
    let tol = model.params.tolerance;

    // 3. 逐点闭式求解 + 冗余/冲突分析。
    let n_groups = groups.len();
    let mut inconsistent = false;
    // 条件数估计：单条归一化约束行的 κ 精确为 1；出现退化/重复即置 None。
    let mut condition = Some(1.0f64);
    let mut redundant_groups: Vec<RedundancyGroup> = Vec::new();
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
            // 距离为负：几何上不可能满足。
            let negative_ids: Vec<u32> = constraints
                .iter()
                .filter(|&(_, d)| *d < 0.0)
                .map(|&(cid, _)| cid)
                .collect();
            inconsistent = true;
            redundant_groups.push(RedundancyGroup {
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
        } else if d_max - d_min > tol {
            // 同一点的距离要求不一致：互相冲突。
            inconsistent = true;
            redundant_groups.push(RedundancyGroup {
                constraint_ids: all_ids.clone(),
                human_message: format!(
                    "约束 {all_ids:?} 对同一实体给出不一致的距离要求（最小 {d_min:e}，\
                     最大 {d_max:e}，相差超过容差 {tol:e}），不可能同时满足。"
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
            // 距离一致但重复：线性相关（冗余）。
            redundant_groups.push(RedundancyGroup {
                constraint_ids: all_ids.clone(),
                human_message: format!(
                    "约束 {all_ids:?} 相互重复（线性相关）：它们约束的是同一个标量 ‖p‖，\
                     删除其中任何一条都不改变解集。"
                ),
            });
            for &cid in &all_ids[1..] {
                suggestions.push(Suggestion {
                    action: Action::RemoveConstraint { id: cid },
                    human_message: format!("约束 {cid} 是冗余的，可以删除。"),
                });
            }
        }

        // 闭式特解：p' = p · (d_ref / ‖p‖)。d_ref 取组内最大值（各组内一致时
        // 即唯一值；冲突/负值时也已标记 Inconsistent，特解仅用于计算诚实残差）。
        // ‖p‖ = 0 时方向退化：按 +x 轴取特解（确定性选择，保证跨平台一致）。
        // sqrt 一律走 libm（纯 Rust、正确舍入）：同一份代码在所有目标上运行，
        // 位级一致由构造保证，而不仅靠 IEEE 语义约定。
        let d_ref = d_max;
        let r = libm::sqrt(*x * *x + *y * *y);
        let (nx, ny) = if r > 0.0 {
            (*x * (d_ref / r), *y * (d_ref / r))
        } else {
            (d_ref, 0.0)
        };
        *x = nx;
        *y = ny;
        for &(cid, d) in &constraints {
            let residual = libm::sqrt(nx * nx + ny * ny) - d;
            residuals.push(ConstraintResidual {
                constraint_id: cid,
                residual,
                human_message: format!(
                    "约束 {cid} 在返回的特解处残差为 {residual:e}（即 ‖p‖ − {d:e}）。"
                ),
            });
        }
        if constraints.len() != 1 || r == 0.0 {
            // 行重复（秩亏）或原点退化（梯度为零）时条件数无意义。
            condition = None;
        }
    }
    if n_groups == 0 {
        condition = None; // 空雅可比谈不上条件数。
    }

    // 4. 欠约束建议。
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

    // 5. 结局判定。优先级：Inconsistent > Overconstrained > Underconstrained > Converged
    //    （「无解」比「有冗余」「欠定」对使用者更紧急；均为事实陈述，可并存，
    //     完整信息在 Diagnostics 里）。
    let outcome = if inconsistent {
        SolveOutcome::Inconsistent
    } else if !redundant_groups.is_empty() {
        SolveOutcome::Overconstrained
    } else if dof_remaining > 0 {
        SolveOutcome::Underconstrained
    } else if max_residual
        .as_ref()
        .is_some_and(|r| r.residual.abs() > tol)
    {
        // 闭式解理论上残差为 0（± ulp）；保留数值防御性检查。
        SolveOutcome::Inconsistent
    } else {
        SolveOutcome::Converged
    };

    let entities =
        if outcome == SolveOutcome::Underconstrained && !model.params.allow_underconstrained {
            // 参数要求不接受欠约束解：实体原样返回输入，特解仅体现在残差诊断中。
            model.entities.clone()
        } else {
            solved_entities
        };

    Ok(SolveReport {
        outcome,
        entities,
        diagnostics: Diagnostics {
            dof_total,
            dof_remaining,
            joints: Vec::new(),
            redundant_constraints: redundant_groups,
            residuals,
            max_residual,
            suggestions,
            jacobian_condition_estimate: condition,
        },
    })
}

/// 结构校验：实体/约束 id 在模型内必须唯一。
fn validate(model: &Model) -> Result<(), SolveError> {
    let mut seen_entities = BTreeSet::new();
    for entity in &model.entities {
        if !seen_entities.insert(entity.id) {
            return Err(SolveError::InvalidModel {
                reason: format!("实体 id {} 重复，id 必须在模型内唯一。", entity.id),
            });
        }
    }
    let mut seen_constraints = BTreeSet::new();
    for constraint in &model.constraints {
        if !seen_constraints.insert(constraint.id) {
            return Err(SolveError::InvalidModel {
                reason: format!("约束 id {} 重复，id 必须在模型内唯一。", constraint.id),
            });
        }
    }
    Ok(())
}
