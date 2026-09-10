//! 3D 装配约束求解：6-DOF 刚体（Rigid3）的 mate / coaxial / distance / angle / fixed。
//!
//! 数值内核：**Levenberg–Marquardt** 阻尼最小二乘，解析雅可比（SO(3) 指数映射
//! 的右雅可比，见 `so3` 模块），线性解算用 `linalg` 的手写 Cholesky——
//! 不依赖任何外部线性代数 crate，所有目标编译同一份代码，位级一致由构造保证。
//!
//! 诊断：终态雅可比经单边 Jacobi SVD 做秩分析——`dof_remaining = 自由参数数 − 秩`；
//! 冗余约束用贪心增量秩判定（按约束在模型中的顺序，确定性）；条件数估计
//! `σ_max/σ_min(非零)`。结局优先级与 2D 路径一致：
//! Inconsistent > Overconstrained > Underconstrained > Converged。
//!
//! 残差定义（世界系，n/d 均为单位化）：
//! - mate：`[dot(pA−pB, nB); nA+nB]`（4 行，秩 3：法向平移 1 + 姿态 2，
//!   nA+nB 三分量因单位约束秩亏 1，LM 天然处理）
//! - coaxial：`[dA×dB; (pA−pB)×dB]`（6 行，秩 4）
//! - distance：`‖tA−tB‖ − v`（1 行，原点间距）
//! - angle：`dot(RA·a_dir, RB·b_dir) − cos(v)`（1 行）
//! - fixed：结构性消参（0 行），重复 fixed 由贪心秩判定捕获。

use crate::linalg::{rank_from_sv, singular_values, svd_lm_step};
use crate::model::{Constraint, ConstraintKind, Entity, Geometry, Model, SolveOutcome};
use crate::report::{
    Action, ConstraintResidual, Diagnostics, RedundancyGroup, SolveError, SolveReport, Suggestion,
};
use crate::so3::{
    hat, mat3_mul_mat, mat3_mul_vec, mat3_transpose, right_jacobian, rotation, v3, Mat3,
};
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

/// 一个刚体在求解器中的槽位。
struct BodySlot {
    /// 在 model.entities 中的下标。
    entity_idx: usize,
    /// 自由参数偏移（6 个：t 3 + ω 3）；fixed 体为 None。
    param_off: Option<usize>,
    /// 输入位姿 (t, ω)。自由体在迭代中被 x 覆盖；fixed 体始终用它
    /// （绝不能用零占位——旋转矩阵为零会把世界方向算成零向量）。
    pose: ([f64; 3], [f64; 3]),
}

/// 预归一化的约束块描述。
enum BlockDesc {
    Mate {
        cid: u32,
        a: usize,
        b: usize,
        p_a: [f64; 3],
        n_a: [f64; 3],
        p_b: [f64; 3],
        n_b: [f64; 3],
    },
    Coaxial {
        cid: u32,
        a: usize,
        b: usize,
        p_a: [f64; 3],
        d_a: [f64; 3],
        p_b: [f64; 3],
        d_b: [f64; 3],
    },
    Distance {
        cid: u32,
        a: usize,
        b: usize,
        value: f64,
    },
    Angle {
        cid: u32,
        a: usize,
        b: usize,
        a_dir: [f64; 3],
        b_dir: [f64; 3],
        cos_v: f64,
    },
    Fixed {
        cid: u32,
    },
}

impl BlockDesc {
    fn cid(&self) -> u32 {
        match self {
            BlockDesc::Mate { cid, .. }
            | BlockDesc::Coaxial { cid, .. }
            | BlockDesc::Distance { cid, .. }
            | BlockDesc::Angle { cid, .. }
            | BlockDesc::Fixed { cid } => *cid,
        }
    }

    fn rows(&self) -> usize {
        match self {
            BlockDesc::Mate { .. } => 4,
            BlockDesc::Coaxial { .. } => 6,
            BlockDesc::Distance { .. } | BlockDesc::Angle { .. } => 1,
            BlockDesc::Fixed { .. } => 0,
        }
    }

    fn kind_name(&self) -> &'static str {
        match self {
            BlockDesc::Mate { .. } => "mate",
            BlockDesc::Coaxial { .. } => "coaxial",
            BlockDesc::Distance { .. } => "distance",
            BlockDesc::Angle { .. } => "angle",
            BlockDesc::Fixed { .. } => "fixed",
        }
    }
}

/// 模型含 rigid3 实体时走本路径（纯 2D 模型仍走闭式解路径，位级不变）。
pub(crate) fn solve(model: &Model) -> Result<SolveReport, SolveError> {
    // ── 1. 收集刚体，校验实体类型 ─────────────────────────────────
    let mut slots: Vec<BodySlot> = Vec::new();
    let mut fixed: Vec<u32> = Vec::new(); // fixed 的实体 id
    for (idx, entity) in model.entities.iter().enumerate() {
        if !matches!(entity.geometry, Geometry::Rigid3 { .. }) {
            return Err(SolveError::InvalidModel {
                reason: format!(
                    "3D 装配模型只允许 rigid3 实体，实体 {} 是 {}。\
                     请把 2D 草图实体放进单独的模型。",
                    entity.id,
                    entity.geometry.kind_name()
                ),
            });
        }
        slots.push(BodySlot {
            entity_idx: idx,
            param_off: None,
            pose: rigid3_pose(entity),
        });
    }
    let id_to_slot: Vec<(u32, usize)> = slots
        .iter()
        .enumerate()
        .map(|(i, s)| (model.entities[s.entity_idx].id, i))
        .collect();
    let find_slot = |id: u32| -> Option<usize> {
        id_to_slot
            .iter()
            .find(|(eid, _)| *eid == id)
            .map(|(_, i)| *i)
    };
    for constraint in &model.constraints {
        if let ConstraintKind::Fixed { a } = &constraint.kind {
            if find_slot(*a).is_none() {
                return Err(SolveError::UnknownEntity {
                    constraint_id: constraint.id,
                    entity_id: *a,
                });
            }
            fixed.push(*a);
        }
    }

    // ── 2. 校验并构建约束块（含归一化）────────────────────────────
    let tol = model.params.tolerance;
    let mut blocks: Vec<BlockDesc> = Vec::new();
    // 同对距离的冲突预检（可证明的 Inconsistent）
    /// （无序刚体对 -> [(约束 id, 距离值)]）的别名，见下方冲突预检。
    type DistancePairs = Vec<((usize, usize), Vec<(u32, f64)>)>;
    let mut distance_pairs: DistancePairs = Vec::new();
    for constraint in &model.constraints {
        let c: &Constraint = constraint;
        let desc = match &c.kind {
            ConstraintKind::Fixed { .. } => {
                BlockDesc::Fixed { cid: c.id } // 实体存在性已在上面检查
            }
            ConstraintKind::Mate {
                a,
                b,
                a_plane,
                b_plane,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                BlockDesc::Mate {
                    cid: c.id,
                    a: sa,
                    b: sb,
                    p_a: [a_plane.origin.x, a_plane.origin.y, a_plane.origin.z],
                    n_a: unit(
                        c,
                        &[a_plane.normal.x, a_plane.normal.y, a_plane.normal.z],
                        "a_plane.normal",
                    )?,
                    p_b: [b_plane.origin.x, b_plane.origin.y, b_plane.origin.z],
                    n_b: unit(
                        c,
                        &[b_plane.normal.x, b_plane.normal.y, b_plane.normal.z],
                        "b_plane.normal",
                    )?,
                }
            }
            ConstraintKind::Coaxial {
                a,
                b,
                a_axis,
                b_axis,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                BlockDesc::Coaxial {
                    cid: c.id,
                    a: sa,
                    b: sb,
                    p_a: [a_axis.origin.x, a_axis.origin.y, a_axis.origin.z],
                    d_a: unit(
                        c,
                        &[a_axis.direction.x, a_axis.direction.y, a_axis.direction.z],
                        "a_axis.direction",
                    )?,
                    p_b: [b_axis.origin.x, b_axis.origin.y, b_axis.origin.z],
                    d_b: unit(
                        c,
                        &[b_axis.direction.x, b_axis.direction.y, b_axis.direction.z],
                        "b_axis.direction",
                    )?,
                }
            }
            ConstraintKind::Distance { a, b, value } => {
                let Some(b_id) = b else {
                    return Err(SolveError::UnsupportedConstraint {
                        constraint_id: c.id,
                        constraint_kind: "distance".into(),
                        reason: String::from(
                            "3D 装配的距离约束需要两个刚体（b 不能为空）；到坐标原点的距离仅支持 2D 点。",
                        ),
                    });
                };
                let (sa, sb) = two_slots(c, *a, *b_id, &find_slot)?;
                if *value < 0.0 {
                    return Ok(inconsistent_negative_distance(model, c.id, *value));
                }
                let key = if sa < sb { (sa, sb) } else { (sb, sa) };
                if let Some(entry) = distance_pairs.iter_mut().find(|(k, _)| *k == key) {
                    entry.1.push((c.id, *value));
                } else {
                    distance_pairs.push((key, Vec::from([(c.id, *value)])));
                }
                BlockDesc::Distance {
                    cid: c.id,
                    a: sa,
                    b: sb,
                    value: *value,
                }
            }
            ConstraintKind::Angle {
                a,
                b,
                value,
                a_dir,
                b_dir,
            } => {
                let Some(b_id) = b else {
                    return Err(SolveError::UnsupportedConstraint {
                        constraint_id: c.id,
                        constraint_kind: "angle".into(),
                        reason: String::from("3D 装配的角度约束需要 a、b 两个刚体（b 不能为空）。"),
                    });
                };
                let (Some(da), Some(db)) = (a_dir, b_dir) else {
                    return Err(SolveError::UnsupportedConstraint {
                        constraint_id: c.id,
                        constraint_kind: "angle".into(),
                        reason: String::from(
                            "3D 装配的角度约束需要 a_dir 与 b_dir（各自体坐标系内的参考方向）。",
                        ),
                    });
                };
                if !(0.0..=core::f64::consts::PI).contains(value) {
                    return Err(SolveError::InvalidModel {
                        reason: format!(
                            "约束 {}（angle）的 value 必须在 [0, π] 弧度内，得到 {value:e}。",
                            c.id
                        ),
                    });
                }
                let (sa, sb) = two_slots(c, *a, *b_id, &find_slot)?;
                BlockDesc::Angle {
                    cid: c.id,
                    a: sa,
                    b: sb,
                    a_dir: [da.x, da.y, da.z],
                    b_dir: [db.x, db.y, db.z],
                    cos_v: libm::cos(*value),
                }
            }
            other => {
                return Err(SolveError::UnsupportedConstraint {
                    constraint_id: c.id,
                    constraint_kind: other.kind_name().into(),
                    reason: String::from("该 2D 草图约束类型不适用于 3D 装配（rigid3 实体）。"),
                });
            }
        };
        blocks.push(desc);
    }

    // 冲突距离预检：同一对刚体的距离要求不一致（可证明无解）
    for (_, values) in &distance_pairs {
        let d_min = values.iter().fold(f64::INFINITY, |m, &(_, d)| m.min(d));
        let d_max = values.iter().fold(f64::NEG_INFINITY, |m, &(_, d)| m.max(d));
        if d_max - d_min > tol {
            return Ok(inconsistent_conflicting_distances(
                model,
                values.iter().map(|&(cid, _)| cid).collect(),
                d_min,
                d_max,
                tol,
            ));
        }
        // 距离一致但成对重复 → 贪心秩判定稍后给出冗余组
    }

    // ── 3. 自由参数布局：fixed 体 0 参数，其余 6 个 ────────────────
    let mut n = 0usize;
    for slot in &mut slots {
        let id = model.entities[slot.entity_idx].id;
        if fixed.contains(&id) {
            slot.param_off = None;
        } else {
            slot.param_off = Some(n);
            n += 6;
        }
    }
    let dof_total = (slots.len() * 6) as u32;

    // 初始状态 = 输入位姿
    let mut x: Vec<f64> = alloc::vec![0.0; n];
    for slot in &slots {
        if let Some(off) = slot.param_off {
            x[off] = slot.pose.0[0];
            x[off + 1] = slot.pose.0[1];
            x[off + 2] = slot.pose.0[2];
            x[off + 3] = slot.pose.1[0];
            x[off + 4] = slot.pose.1[1];
            x[off + 5] = slot.pose.1[2];
        }
    }

    // ── 4. LM 迭代 ────────────────────────────────────────────────
    let m: usize = blocks.iter().map(|b| b.rows()).sum();
    let (mut r, mut j) = eval(&x, &slots, &blocks);
    let mut converged = r.iter().all(|v| v.abs() <= tol);
    let mut iterations = 0u32;
    let mut lambda = 1e-3f64;
    let max_iter = model.params.max_iterations;
    while !converged && iterations < max_iter && n > 0 {
        iterations += 1;
        // LM 步用 SVD 阻尼伪逆（svd_lm_step）：装配约束的残差行天然线性
        // 相关（mate 的 nA+nB 秩 2/3、coaxial 的叉积秩 4/6），正规方程在
        // λ 小时病态（条件数 ~1/σ² → 步长放大 → 拒绝 → λ 震荡爬行）；
        // SVD 形式把零奇异方向截断，秩亏下仍二次收敛。
        let r_norm2: f64 = r.iter().map(|v| v * v).sum();
        let mut accepted = false;
        for _ in 0..8 {
            let d = svd_lm_step(&j, &r, m, n, lambda);
            let x_new: Vec<f64> = x.iter().zip(d.iter()).map(|(a, b)| a + b).collect();
            let (r_new, j_new) = eval(&x_new, &slots, &blocks);
            let r_new_norm2: f64 = r_new.iter().map(|v| v * v).sum();
            if r_new_norm2 < r_norm2 {
                x = x_new;
                r = r_new;
                j = j_new;
                converged = r.iter().all(|v| v.abs() <= tol);
                lambda = (lambda / 2.0).max(1e-14);
                accepted = true;
                break;
            }
            lambda = (lambda * 8.0).min(1e14);
            if lambda >= 1e14 {
                break;
            }
        }
        if !accepted {
            break; // 卡在局部极小：如实按未收敛报告
        }
    }

    // ── 5. 秩分析与诊断 ───────────────────────────────────────────
    let (rank, condition, redundant) = analyze_rank(&j, m, n, &blocks);
    let dof_remaining = (n - rank) as u32;

    let mut residuals: Vec<ConstraintResidual> = Vec::new();
    {
        let mut row = 0usize;
        for block in &blocks {
            let rows = block.rows();
            let block_max = r[row..row + rows]
                .iter()
                .fold(0.0f64, |mx, v| mx.max(v.abs()));
            if rows > 0 {
                residuals.push(ConstraintResidual {
                    constraint_id: block.cid(),
                    residual: block_max,
                    human_message: format!(
                        "约束 {}（{}）在返回位姿处残差 ∞-范数为 {block_max:e}（0 表示完全满足）。",
                        block.cid(),
                        block.kind_name()
                    ),
                });
            }
            row += rows;
        }
    }
    let max_residual = residuals
        .iter()
        .max_by(|a, b| a.residual.abs().total_cmp(&b.residual.abs()))
        .cloned();

    let mut suggestions: Vec<Suggestion> = Vec::new();
    for cid in &redundant {
        suggestions.push(Suggestion {
            action: Action::RemoveConstraint { id: *cid },
            human_message: format!(
                "约束 {cid} 的方程与在它之前的约束线性相关（重复限制），删除它不改变解集。"
            ),
        });
    }
    if dof_remaining > 0 {
        suggestions.push(Suggestion {
            action: Action::AddConstraint { dof_remaining },
            human_message: format!(
                "装配体还有 {dof_remaining} 个自由度未被约束。可用的 3D 装配约束：\
                 mate 消去 3 个、coaxial 消去 4 个、distance / angle 各消去 1 个、\
                 fixed 消去 6 个。"
            ),
        });
    }

    // ── 6. 结局判定（与 2D 路径同优先级）─────────────────────────
    let outcome = if !converged {
        if n == 0 {
            // 没有任何自由参数而残差仍超容差：可证明不一致
            SolveOutcome::Inconsistent
        } else {
            // 未在迭代上限内收敛（或 LM 卡在局部极小）：如实报告，
            // 不臆断「不一致」——初值不佳与真冲突在此不可区分
            SolveOutcome::MaxIterations
        }
    } else if !redundant.is_empty() {
        SolveOutcome::Overconstrained
    } else if dof_remaining > 0 {
        SolveOutcome::Underconstrained
    } else {
        SolveOutcome::Converged
    };

    // ── 7. 输出实体（欠约束且不接受特解时原样返回）────────────────
    let mut entities = model.entities.clone();
    if !(outcome == SolveOutcome::Underconstrained && !model.params.allow_underconstrained) {
        for slot in &slots {
            if let Some(off) = slot.param_off {
                if let Geometry::Rigid3 { pose } = &mut entities[slot.entity_idx].geometry {
                    pose.translation.x = x[off];
                    pose.translation.y = x[off + 1];
                    pose.translation.z = x[off + 2];
                    pose.rotation.vector = [x[off + 3], x[off + 4], x[off + 5]];
                }
            }
        }
    }

    Ok(SolveReport {
        outcome,
        entities,
        diagnostics: Diagnostics {
            dof_total,
            dof_remaining,
            redundant_constraints: redundant
                .iter()
                .map(|&cid| RedundancyGroup {
                    constraint_ids: Vec::from([cid]),
                    human_message: format!(
                        "约束 {cid} 的方程组与在它之前的约束线性相关：\
                         它对装配体施加了重复的限制，删除它不改变解集。"
                    ),
                })
                .collect(),
            residuals,
            max_residual,
            suggestions,
            jacobian_condition_estimate: condition,
        },
    })
}

// ══════════════════════════ 残差与雅可比 ══════════════════════════

/// 求值：给定参数向量，返回残差 (m) 与雅可比 (m×n，行主序)。
fn eval(x: &[f64], slots: &[BodySlot], blocks: &[BlockDesc]) -> (Vec<f64>, Vec<f64>) {
    let m: usize = blocks.iter().map(|b| b.rows()).sum();
    let n = x.len();
    let mut r = alloc::vec![0.0; m];
    let mut j = alloc::vec![0.0; m * n];

    // 每个体缓存 (t, ω, R, J_r)：自由体取自 x，fixed 体取自输入位姿
    let mut bodies: Vec<([f64; 3], [f64; 3], Mat3, Mat3)> = Vec::with_capacity(slots.len());
    for slot in slots {
        match slot.param_off {
            Some(off) => bodies.push((
                [x[off], x[off + 1], x[off + 2]],
                [x[off + 3], x[off + 4], x[off + 5]],
                rotation([x[off + 3], x[off + 4], x[off + 5]]),
                right_jacobian([x[off + 3], x[off + 4], x[off + 5]]),
            )),
            None => {
                // fixed：用输入位姿求值（雅可比不写，R/J_r 仍须正确以算世界量）
                let (t, w) = slot.pose;
                bodies.push((t, w, rotation(w), right_jacobian(w)))
            }
        }
    }

    // 局部点/方向的世界量及其对 (t, ω) 的雅可比
    // P = t + R·p：∂P/∂t = I，∂P/∂ω = −R·hat(p)·J_r
    // D = R·d：    ∂D/∂t = 0，∂D/∂ω = −R·hat(d)·J_r
    fn world_point(body: &([f64; 3], [f64; 3], Mat3, Mat3), p: [f64; 3]) -> [f64; 3] {
        v3::add(body.0, mat3_mul_vec(&body.2, p))
    }
    fn point_jac(body: &([f64; 3], [f64; 3], Mat3, Mat3), p: [f64; 3]) -> Mat3 {
        // −R·hat(p)·J_r
        let m = mat3_mul_mat(&mat3_mul_mat(&body.2, &hat(p)), &body.3);
        [
            [-m[0][0], -m[0][1], -m[0][2]],
            [-m[1][0], -m[1][1], -m[1][2]],
            [-m[2][0], -m[2][1], -m[2][2]],
        ]
    }
    fn world_dir(body: &([f64; 3], [f64; 3], Mat3, Mat3), d: [f64; 3]) -> [f64; 3] {
        mat3_mul_vec(&body.2, d)
    }
    fn dir_jac(body: &([f64; 3], [f64; 3], Mat3, Mat3), d: [f64; 3]) -> Mat3 {
        point_jac(body, d) // 结构相同：−R·hat(d)·J_r
    }

    // 把 1×3 行向量写进 J[row][col..col+3]
    fn write_row(j: &mut [f64], n: usize, row: usize, col: usize, v: [f64; 3]) {
        j[row * n + col] = v[0];
        j[row * n + col + 1] = v[1];
        j[row * n + col + 2] = v[2];
    }
    // 把 3×3 块 ∂r_vec/∂(参数三元组) 写进 J[row0..row0+3][col..col+3]
    fn write_mat(j: &mut [f64], n: usize, row0: usize, col: usize, m: &Mat3) {
        for i in 0..3 {
            for k in 0..3 {
                j[(row0 + i) * n + col + k] = m[i][k];
            }
        }
    }
    // 矩阵转置作用于向量：Mᵀ·v
    fn tmul(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
        mat3_mul_vec(&mat3_transpose(m), v)
    }

    let mut row = 0usize;
    for block in blocks {
        match *block {
            BlockDesc::Fixed { .. } => {}
            BlockDesc::Distance {
                cid: _,
                a,
                b,
                value,
            } => {
                let pa = bodies[a];
                let pb = bodies[b];
                let dt = v3::sub(pa.0, pb.0);
                let dist = v3::norm(dt);
                r[row] = dist - value;
                if dist > 0.0 {
                    let u = v3::scale(dt, 1.0 / dist);
                    if let Some(off) = slots[a].param_off {
                        write_row(&mut j, n, row, off, u);
                    }
                    if let Some(off) = slots[b].param_off {
                        let neg = v3::scale(u, -1.0);
                        write_row(&mut j, n, row, off, neg);
                    }
                } // dist == 0：梯度未定义，J 行保持 0（条件数按退化处理）
            }
            BlockDesc::Angle {
                cid: _,
                a,
                b,
                a_dir,
                b_dir,
                cos_v,
            } => {
                let da_w = world_dir(&bodies[a], a_dir);
                let db_w = world_dir(&bodies[b], b_dir);
                r[row] = v3::dot(da_w, db_w) - cos_v;
                if let Some(off) = slots[a].param_off {
                    // ∂r/∂ω_a = (∂da_w/∂ω_a)ᵀ·db_w（dir_jac 已含负号）
                    let g = tmul(&dir_jac(&bodies[a], a_dir), db_w);
                    write_row(&mut j, n, row, off + 3, g);
                }
                if let Some(off) = slots[b].param_off {
                    let g = tmul(&dir_jac(&bodies[b], b_dir), da_w);
                    write_row(&mut j, n, row, off + 3, g);
                }
            }
            BlockDesc::Mate {
                cid: _,
                a,
                b,
                p_a,
                n_a,
                p_b,
                n_b,
            } => {
                let pa_w = world_point(&bodies[a], p_a);
                let na_w = world_dir(&bodies[a], n_a);
                let pb_w = world_point(&bodies[b], p_b);
                let nb_w = world_dir(&bodies[b], n_b);
                let dp = v3::sub(pa_w, pb_w);
                // r1 = dot(dp, nb_w)
                r[row] = v3::dot(dp, nb_w);
                // r2..4 = na_w + nb_w
                let nsum = v3::add(na_w, nb_w);
                r[row + 1] = nsum[0];
                r[row + 2] = nsum[1];
                r[row + 3] = nsum[2];

                if let Some(off) = slots[a].param_off {
                    // ∂r1/∂t_a = nb_w；∂r1/∂ω_a = (∂pa_w/∂ω_a)ᵀ·nb_w
                    write_row(&mut j, n, row, off, nb_w);
                    let g = tmul(&point_jac(&bodies[a], p_a), nb_w);
                    write_row(&mut j, n, row, off + 3, g);
                    // ∂(na_w)/∂ω_a = dir_jac
                    write_mat(&mut j, n, row + 1, off + 3, &dir_jac(&bodies[a], n_a));
                }
                if let Some(off) = slots[b].param_off {
                    let neg_nb = v3::scale(nb_w, -1.0);
                    write_row(&mut j, n, row, off, neg_nb);
                    // ∂r1/∂ω_b = −(∂pb_w/∂ω_b)ᵀ·nb_w + (∂nb_w/∂ω_b)ᵀ·dp
                    let g1 = tmul(&point_jac(&bodies[b], p_b), nb_w);
                    let g2 = tmul(&dir_jac(&bodies[b], n_b), dp);
                    write_row(&mut j, n, row, off + 3, v3::sub(g2, g1));
                    write_mat(&mut j, n, row + 1, off + 3, &dir_jac(&bodies[b], n_b));
                }
            }
            BlockDesc::Coaxial {
                cid: _,
                a,
                b,
                p_a,
                d_a,
                p_b,
                d_b,
            } => {
                let pa_w = world_point(&bodies[a], p_a);
                let da_w = world_dir(&bodies[a], d_a);
                let pb_w = world_point(&bodies[b], p_b);
                let db_w = world_dir(&bodies[b], d_b);
                let dp = v3::sub(pa_w, pb_w);
                // r_c = da_w × db_w（3 行）
                let rc = v3::cross(da_w, db_w);
                r[row] = rc[0];
                r[row + 1] = rc[1];
                r[row + 2] = rc[2];
                // r_p = dp × db_w（3 行）
                let rp = v3::cross(dp, db_w);
                r[row + 3] = rp[0];
                r[row + 4] = rp[1];
                r[row + 5] = rp[2];

                // ∂(u×v) = −hat(v)·∂u + hat(u)·∂v
                if let Some(off) = slots[a].param_off {
                    // r_c 对 ω_a：−hat(db_w)·∂da_w/∂ω_a = hat(db_w)·R·hat(d_a)·J_r
                    let m_c = mat3_mul_mat(
                        &mat3_mul_mat(&hat(db_w), &bodies[a].2),
                        &mat3_mul_mat(&hat(d_a), &bodies[a].3),
                    );
                    write_mat(&mut j, n, row, off + 3, &m_c);
                    // r_p 对 t_a：−hat(db_w)
                    let neg_hat = hat(db_w);
                    write_mat(
                        &mut j,
                        n,
                        row + 3,
                        off,
                        &[
                            [-neg_hat[0][0], -neg_hat[0][1], -neg_hat[0][2]],
                            [-neg_hat[1][0], -neg_hat[1][1], -neg_hat[1][2]],
                            [-neg_hat[2][0], -neg_hat[2][1], -neg_hat[2][2]],
                        ],
                    );
                    // r_p 对 ω_a：hat(db_w)·R·hat(p_a)·J_r
                    let m_p = mat3_mul_mat(
                        &mat3_mul_mat(&hat(db_w), &bodies[a].2),
                        &mat3_mul_mat(&hat(p_a), &bodies[a].3),
                    );
                    write_mat(&mut j, n, row + 3, off + 3, &m_p);
                }
                if let Some(off) = slots[b].param_off {
                    // r_c 对 ω_b：hat(da_w)·∂db_w/∂ω_b = −hat(da_w)·R·hat(d_b)·J_r
                    let m_c = mat3_mul_mat(
                        &mat3_mul_mat(&hat(da_w), &bodies[b].2),
                        &mat3_mul_mat(&hat(d_b), &bodies[b].3),
                    );
                    let m_c = [
                        [-m_c[0][0], -m_c[0][1], -m_c[0][2]],
                        [-m_c[1][0], -m_c[1][1], -m_c[1][2]],
                        [-m_c[2][0], -m_c[2][1], -m_c[2][2]],
                    ];
                    write_mat(&mut j, n, row, off + 3, &m_c);
                    // r_p 对 t_b：+hat(db_w)
                    write_mat(&mut j, n, row + 3, off, &hat(db_w));
                    // r_p 对 ω_b：−hat(db_w)·R·hat(p_b)·J_r − hat(dp)·R·hat(d_b)·J_r
                    let m1 = mat3_mul_mat(
                        &mat3_mul_mat(&hat(db_w), &bodies[b].2),
                        &mat3_mul_mat(&hat(p_b), &bodies[b].3),
                    );
                    let m2 = mat3_mul_mat(
                        &mat3_mul_mat(&hat(dp), &bodies[b].2),
                        &mat3_mul_mat(&hat(d_b), &bodies[b].3),
                    );
                    let m_p = [
                        [
                            -(m1[0][0] + m2[0][0]),
                            -(m1[0][1] + m2[0][1]),
                            -(m1[0][2] + m2[0][2]),
                        ],
                        [
                            -(m1[1][0] + m2[1][0]),
                            -(m1[1][1] + m2[1][1]),
                            -(m1[1][2] + m2[1][2]),
                        ],
                        [
                            -(m1[2][0] + m2[2][0]),
                            -(m1[2][1] + m2[2][1]),
                            -(m1[2][2] + m2[2][2]),
                        ],
                    ];
                    write_mat(&mut j, n, row + 3, off + 3, &m_p);
                }
            }
        }
        row += block.rows();
    }
    (r, j)
}

/// 秩 / 条件数 / 冗余块分析。
/// 返回 (rank, 条件数估计, 冗余约束 id 列表——按模型顺序的贪心增量秩判定)。
fn analyze_rank(
    j: &[f64],
    m: usize,
    n: usize,
    blocks: &[BlockDesc],
) -> (usize, Option<f64>, Vec<u32>) {
    if m == 0 || n == 0 {
        return (0, None, Vec::new());
    }
    let sv = singular_values(j, m, n);
    let rank = rank_from_sv(&sv, m, n);
    let condition = if rank > 0 {
        Some(sv[0] / sv[rank - 1])
    } else {
        None
    };
    // 贪心冗余：按块顺序累加行，秩不增长的块（且行数 > 0）判定为冗余
    let mut redundant = Vec::new();
    let mut row0 = 0usize;
    let mut prev_rank = 0usize;
    for block in blocks {
        let rows = block.rows();
        if rows == 0 {
            continue; // fixed：结构性消参，无行可谈
        }
        let sub_m = row0 + rows;
        let sub_sv = singular_values(&j[..sub_m * n], sub_m, n);
        let sub_rank = rank_from_sv(&sub_sv, sub_m, n);
        if sub_rank == prev_rank {
            redundant.push(block.cid());
        }
        prev_rank = sub_rank;
        row0 = sub_m;
    }
    (rank, condition, redundant)
}

// ══════════════════════════ 可证明的不一致捷径 ══════════════════════════

/// 负距离：几何上不可能满足（镜像 2D 路径语义）。
fn inconsistent_negative_distance(model: &Model, cid: u32, value: f64) -> SolveReport {
    SolveReport {
        outcome: SolveOutcome::Inconsistent,
        entities: model.entities.clone(),
        diagnostics: Diagnostics {
            dof_total: (model.entities.len() * 6) as u32,
            dof_remaining: (model.entities.len() * 6) as u32,
            redundant_constraints: Vec::from([RedundancyGroup {
                constraint_ids: Vec::from([cid]),
                human_message: format!(
                    "约束 {cid} 的距离参数为负值 {value:e}：距离约束要求 value ≥ 0，\
                     不可能被任何位姿满足。"
                ),
            }]),
            residuals: Vec::from([ConstraintResidual {
                constraint_id: cid,
                residual: -value,
                human_message: format!(
                    "约束 {cid}（distance）在输入位姿处残差为 {:e}（距离为负，无解）。",
                    -value
                ),
            }]),
            max_residual: Some(ConstraintResidual {
                constraint_id: cid,
                residual: -value,
                human_message: format!(
                    "约束 {cid}（distance）在输入位姿处残差为 {:e}（距离为负，无解）。",
                    -value
                ),
            }),
            suggestions: Vec::from([Suggestion {
                action: Action::RelaxConstraint { id: cid },
                human_message: format!("把约束 {cid} 的距离值改为非负数。"),
            }]),
            jacobian_condition_estimate: None,
        },
    }
}

/// 同对刚体的距离要求冲突（镜像 2D 路径语义）。
fn inconsistent_conflicting_distances(
    model: &Model,
    cids: Vec<u32>,
    d_min: f64,
    d_max: f64,
    tol: f64,
) -> SolveReport {
    let residuals: Vec<ConstraintResidual> = cids
        .iter()
        .map(|&cid| ConstraintResidual {
            constraint_id: cid,
            residual: d_max - d_min,
            human_message: format!(
                "约束 {cid}（distance）与其余同对距离约束的要求相差 {:e}，\
                 超过容差 {tol:e}，不可能同时满足。",
                d_max - d_min
            ),
        })
        .collect();
    let max_residual = residuals.first().cloned();
    SolveReport {
        outcome: SolveOutcome::Inconsistent,
        entities: model.entities.clone(),
        diagnostics: Diagnostics {
            dof_total: (model.entities.len() * 6) as u32,
            dof_remaining: (model.entities.len() * 6) as u32,
            redundant_constraints: Vec::from([RedundancyGroup {
                constraint_ids: cids.clone(),
                human_message: format!(
                    "约束 {cids:?} 对同一对刚体给出不一致的距离要求（最小 {d_min:e}，\
                     最大 {d_max:e}），不可能同时满足。"
                ),
            }]),
            residuals,
            max_residual,
            suggestions: cids[1..]
                .iter()
                .map(|&cid| Suggestion {
                    action: Action::RemoveConstraint { id: cid },
                    human_message: format!(
                        "删除约束 {cid}，或把它的距离值改到 {d_min:e}（与其他约束一致），\
                         其余约束才可能同时满足。"
                    ),
                })
                .collect(),
            jacobian_condition_estimate: None,
        },
    }
}

// ══════════════════════════ 小工具 ══════════════════════════

fn rigid3_pose(entity: &Entity) -> ([f64; 3], [f64; 3]) {
    let Geometry::Rigid3 { pose } = &entity.geometry else {
        unreachable!("装配路径只接受 rigid3 实体（已在入口校验）");
    };
    (
        [pose.translation.x, pose.translation.y, pose.translation.z],
        pose.rotation.vector,
    )
}

fn two_slots(
    c: &Constraint,
    a: u32,
    b: u32,
    find_slot: &dyn Fn(u32) -> Option<usize>,
) -> Result<(usize, usize), SolveError> {
    let Some(sa) = find_slot(a) else {
        return Err(SolveError::UnknownEntity {
            constraint_id: c.id,
            entity_id: a,
        });
    };
    let Some(sb) = find_slot(b) else {
        return Err(SolveError::UnknownEntity {
            constraint_id: c.id,
            entity_id: b,
        });
    };
    Ok((sa, sb))
}

fn unit(c: &Constraint, v: &[f64; 3], field: &str) -> Result<[f64; 3], SolveError> {
    let norm = v3::norm(*v);
    if norm == 0.0 {
        return Err(SolveError::InvalidModel {
            reason: format!("约束 {} 的 {field} 是零向量，无法定义平面/轴。", c.id),
        });
    }
    Ok(v3::scale(*v, 1.0 / norm))
}

// ══════════════════════════ 单元测试 ══════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// 中心差分雅可比交叉验证：解析实现 vs 数值微分。
    /// 这是装配雅可比正确性的地基测试——符号/约定错误在这里暴露。
    #[test]
    fn analytic_jacobian_matches_finite_differences() {
        // 确定性的非平凡位姿（两个自由体）
        let x: Vec<f64> = Vec::from([
            0.30, -0.20, 0.10, 0.05, -0.03, 0.02, // A: t + ω
            1.10, 0.40, -0.60, -0.08, 0.11, 0.04, // B: t + ω
        ]);
        let make_blocks = || {
            Vec::from([
                BlockDesc::Mate {
                    cid: 1,
                    a: 0,
                    b: 1,
                    p_a: [0.20, 0.10, -0.30],
                    n_a: unit3([1.0, 2.0, 3.0]),
                    p_b: [-0.10, 0.25, 0.05],
                    n_b: unit3([-1.0, 0.5, 2.0]),
                },
                BlockDesc::Coaxial {
                    cid: 2,
                    a: 0,
                    b: 1,
                    p_a: [0.0, 0.05, 0.10],
                    d_a: unit3([0.0, 1.0, 1.0]),
                    p_b: [0.10, -0.05, 0.0],
                    d_b: unit3([1.0, 0.0, 0.5]),
                },
                BlockDesc::Distance {
                    cid: 3,
                    a: 0,
                    b: 1,
                    value: 1.5,
                },
                BlockDesc::Angle {
                    cid: 4,
                    a: 0,
                    b: 1,
                    a_dir: [1.0, 0.0, 0.0],
                    b_dir: [0.0, 1.0, 0.0],
                    cos_v: 0.5,
                },
            ])
        };
        let slots = Vec::from([
            BodySlot {
                entity_idx: 0,
                param_off: Some(0),
                pose: ([0.0; 3], [0.0; 3]),
            },
            BodySlot {
                entity_idx: 1,
                param_off: Some(6),
                pose: ([0.0; 3], [0.0; 3]),
            },
        ]);
        let blocks = make_blocks();
        let (r0, j0) = eval(&x, &slots, &blocks);
        let m = r0.len();
        let n = x.len();
        let h = 1e-6;
        for col in 0..n {
            let mut xp = x.clone();
            let mut xm = x.clone();
            xp[col] += h;
            xm[col] -= h;
            let (rp, _) = eval(&xp, &slots, &blocks);
            let (rm, _) = eval(&xm, &slots, &blocks);
            for row in 0..m {
                let fd = (rp[row] - rm[row]) / (2.0 * h);
                let an = j0[row * n + col];
                assert!(
                    (fd - an).abs() < 1e-5,
                    "J[{row}][{col}] 解析 {an:e} vs 差分 {fd:e}（差 {:e}）",
                    (fd - an).abs()
                );
            }
        }
    }

    fn unit3(v: [f64; 3]) -> [f64; 3] {
        let n = v3::norm(v);
        v3::scale(v, 1.0 / n)
    }
}
