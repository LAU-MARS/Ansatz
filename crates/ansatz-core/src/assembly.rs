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
use alloc::boxed::Box;
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
        p_a: [f64; 3],
        p_b: [f64; 3],
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
    /// 关节块（revolute / cylindrical / prismatic）：coaxial 行 + 可选的
    /// 轴向间距行（revolute）/ 滚转角行（prismatic）/ 驱动行。
    Joint {
        cid: u32,
        /// "revolute" | "cylindrical" | "prismatic"
        jk: &'static str,
        m: JointCtx,
        /// revolute：轴向锚点间距目标。
        offset: f64,
        /// prismatic 滚转锁定：(a_ref, b_ref, cos(roll))。
        roll: Option<([f64; 3], [f64; 3], f64)>,
        /// 驱动目标（关节变量，以初始位形为零点）。
        drive: Option<f64>,
    },
    Spherical {
        cid: u32,
        a: usize,
        b: usize,
        p_a: [f64; 3],
        p_b: [f64; 3],
    },
    /// 传动耦合：θ_b − ratio·θ_a = 0（θ 均为初始相对关节变量）。
    Transmission {
        cid: u32,
        ma: Box<JointCtx>,
        mb: Box<JointCtx>,
        ratio: f64,
    },
}

/// 求值中单体的紧凑视图：(t, ω, R, J_r)。
type BodyView = ([f64; 3], [f64; 3], Mat3, Mat3);

/// 关节变量测量结果的梯度：(平移行, 旋转行)。
type JointGrad = ([f64; 3], [f64; 3]);

/// 关节测量上下文：轴几何 + 垂直参考方向 + 初始标定（θ₀/δ₀）。
/// 关节变量定义为**相对初始位形**的增量——这是驱动与传动的零点约定。
struct JointCtx {
    a: usize,
    b: usize,
    p_a: [f64; 3],
    d_a: [f64; 3],
    p_b: [f64; 3],
    d_b: [f64; 3],
    /// a 体上垂直于轴的单位参考方向（确定性构造）。
    ref_a: [f64; 3],
    /// b 体上垂直于轴的单位参考方向。
    ref_b: [f64; 3],
    /// 初始原始角（atan2 值，未减零点）。
    theta0: f64,
    /// 初始轴向间距。
    delta0: f64,
}

/// 确定性地取一个与 d 垂直的单位向量：与 |d| 最小的坐标轴做叉积。
fn perp_ref(d: [f64; 3]) -> [f64; 3] {
    let e = if d[0].abs() <= d[1].abs() && d[0].abs() <= d[2].abs() {
        [1.0, 0.0, 0.0]
    } else if d[1].abs() <= d[2].abs() {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let c = v3::cross(d, e);
    let n = v3::norm(c);
    v3::scale(c, 1.0 / n)
}

impl BlockDesc {
    fn cid(&self) -> u32 {
        match self {
            BlockDesc::Mate { cid, .. }
            | BlockDesc::Coaxial { cid, .. }
            | BlockDesc::Distance { cid, .. }
            | BlockDesc::Angle { cid, .. }
            | BlockDesc::Joint { cid, .. }
            | BlockDesc::Spherical { cid, .. }
            | BlockDesc::Transmission { cid, .. }
            | BlockDesc::Fixed { cid } => *cid,
        }
    }

    fn rows(&self) -> usize {
        match self {
            BlockDesc::Mate { .. } => 4,
            BlockDesc::Coaxial { .. } => 6,
            BlockDesc::Distance { .. } | BlockDesc::Angle { .. } => 1,
            BlockDesc::Fixed { .. } => 0,
            BlockDesc::Joint {
                jk, roll, drive, ..
            } => {
                6 + if *jk == "revolute" { 1 } else { 0 }
                    + if roll.is_some() { 1 } else { 0 }
                    + if drive.is_some() { 1 } else { 0 }
            }
            BlockDesc::Spherical { .. } => 3,
            BlockDesc::Transmission { .. } => 1,
        }
    }

    fn kind_name(&self) -> &'static str {
        match self {
            BlockDesc::Mate { .. } => "mate",
            BlockDesc::Coaxial { .. } => "coaxial",
            BlockDesc::Distance { .. } => "distance",
            BlockDesc::Angle { .. } => "angle",
            BlockDesc::Fixed { .. } => "fixed",
            BlockDesc::Joint { jk, .. } => jk,
            BlockDesc::Spherical { .. } => "spherical",
            BlockDesc::Transmission { .. } => "transmission",
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
            ConstraintKind::Distance {
                a,
                b,
                value,
                a_anchor,
                b_anchor,
            } => {
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
                    p_a: anchor(a_anchor),
                    p_b: anchor(b_anchor),
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
            ConstraintKind::Revolute {
                a,
                b,
                a_axis,
                b_axis,
                offset,
                drive,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                let m = joint_ctx(&slots, sa, sb, a_axis, b_axis)?;
                BlockDesc::Joint {
                    cid: c.id,
                    jk: "revolute",
                    m,
                    offset: offset.unwrap_or(0.0),
                    roll: None,
                    drive: *drive,
                }
            }
            ConstraintKind::Cylindrical {
                a,
                b,
                a_axis,
                b_axis,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                let m = joint_ctx(&slots, sa, sb, a_axis, b_axis)?;
                BlockDesc::Joint {
                    cid: c.id,
                    jk: "cylindrical",
                    m,
                    offset: 0.0,
                    roll: None,
                    drive: None,
                }
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
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                let m = joint_ctx(&slots, sa, sb, a_axis, b_axis)?;
                let roll_data = match (a_ref, b_ref, roll) {
                    (Some(ra), Some(rb), Some(v)) => {
                        if !(0.0..=core::f64::consts::PI).contains(v) {
                            return Err(SolveError::InvalidModel {
                                reason: format!(
                                    "约束 {}（prismatic）的 roll 必须在 [0, π] 弧度内，得到 {v:e}。",
                                    c.id
                                ),
                            });
                        }
                        Some(([ra.x, ra.y, ra.z], [rb.x, rb.y, rb.z], libm::cos(*v)))
                    }
                    _ => None, // 参考方向缺省：退化为圆柱副语义
                };
                BlockDesc::Joint {
                    cid: c.id,
                    jk: "prismatic",
                    m,
                    offset: 0.0,
                    roll: roll_data,
                    drive: *drive,
                }
            }
            ConstraintKind::Spherical {
                a,
                b,
                a_point,
                b_point,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                BlockDesc::Spherical {
                    cid: c.id,
                    a: sa,
                    b: sb,
                    p_a: [a_point.x, a_point.y, a_point.z],
                    p_b: [b_point.x, b_point.y, b_point.z],
                }
            }
            ConstraintKind::Transmission {
                joint_a,
                joint_b,
                ratio,
            } => {
                // 两个被耦合的关节必须是本模型中已声明的 revolute
                let find_revolute = |want: u32| -> Option<(
                    usize,
                    usize,
                    crate::model::Axis3,
                    crate::model::Axis3,
                )> {
                    for c2 in &model.constraints {
                        if c2.id == want {
                            if let ConstraintKind::Revolute {
                                a,
                                b,
                                a_axis,
                                b_axis,
                                ..
                            } = &c2.kind
                            {
                                let sa = find_slot(*a)?;
                                let sb = find_slot(*b)?;
                                return Some((sa, sb, *a_axis, *b_axis));
                            }
                        }
                    }
                    None
                };
                let Some((aa, ab, ax_a, ax_b)) = find_revolute(*joint_a) else {
                    return Err(SolveError::InvalidModel {
                        reason: format!(
                            "传动约束 {} 引用的 joint_a={} 不存在或不是 revolute 关节。",
                            c.id, joint_a
                        ),
                    });
                };
                let Some((ba, bb, bx_a, bx_b)) = find_revolute(*joint_b) else {
                    return Err(SolveError::InvalidModel {
                        reason: format!(
                            "传动约束 {} 引用的 joint_b={} 不存在或不是 revolute 关节。",
                            c.id, joint_b
                        ),
                    });
                };
                let ma = joint_ctx(&slots, aa, ab, &ax_a, &ax_b)?;
                let mb = joint_ctx(&slots, ba, bb, &bx_a, &bx_b)?;
                BlockDesc::Transmission {
                    cid: c.id,
                    ma: Box::new(ma),
                    mb: Box::new(mb),
                    ratio: *ratio,
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

    // 关节状态：收敛位形下逐关节测量（θ/δ 以初始位形为零点）
    let mut joints: Vec<crate::report::JointState> = Vec::new();
    for block in &blocks {
        if let BlockDesc::Joint {
            cid, jk, m, drive, ..
        } = block
        {
            let value = if *jk == "prismatic" {
                measure_delta(
                    &bodies_now(&x, &slots, m.a),
                    &bodies_now(&x, &slots, m.b),
                    m.p_a,
                    m.d_a,
                    m.p_b,
                )
                .0 - m.delta0
            } else {
                let (theta, _, _) = measure_theta(
                    &bodies_now(&x, &slots, m.a),
                    &bodies_now(&x, &slots, m.b),
                    m.p_a,
                    m.d_a,
                    m.ref_a,
                    m.p_b,
                    m.d_b,
                    m.ref_b,
                );
                angle_pi(theta - m.theta0)
            };
            joints.push(crate::report::JointState {
                constraint_id: *cid,
                joint_kind: (*jk).into(),
                value,
                human_message: format!(
                    "关节 {cid}（{jk}）的当前变量为 {value:e}（以初始位形为零点{}）。",
                    if drive.is_some() {
                        "；驱动目标已施加"
                    } else {
                        ""
                    }
                ),
            });
        }
    }

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
            joints,
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
                p_a,
                p_b,
            } => {
                // 锚点式：r = ‖P_A(p_a) − P_B(p_b)‖ − value（锚点缺省即原点，
                // 与 0.2.0 的原点距语义逐位一致）
                let pa_w = world_point(&bodies[a], p_a);
                let pb_w = world_point(&bodies[b], p_b);
                let dt = v3::sub(pa_w, pb_w);
                let dist = v3::norm(dt);
                r[row] = dist - value;
                if dist > 0.0 {
                    let u = v3::scale(dt, 1.0 / dist);
                    if let Some(off) = slots[a].param_off {
                        // ∂r/∂t_a = u；∂r/∂ω_a = (∂pa_w/∂ω_a)ᵀ·u
                        write_row(&mut j, n, row, off, u);
                        let g = tmul(&point_jac(&bodies[a], p_a), u);
                        write_row(&mut j, n, row, off + 3, g);
                    }
                    if let Some(off) = slots[b].param_off {
                        let neg = v3::scale(u, -1.0);
                        write_row(&mut j, n, row, off, neg);
                        let g = tmul(&point_jac(&bodies[b], p_b), u);
                        write_row(&mut j, n, row, off + 3, v3::scale(g, -1.0));
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
            BlockDesc::Joint {
                cid: _,
                jk,
                ref m,
                offset,
                ref roll,
                drive,
            } => {
                // ── 行 0..6：coaxial 行（同轴 + 横向偏移）──
                let pa_w = world_point(&bodies[m.a], m.p_a);
                let da_w = world_dir(&bodies[m.a], m.d_a);
                let pb_w = world_point(&bodies[m.b], m.p_b);
                emit_coaxial_rows(
                    &mut r, &mut j, n, row, slots, &bodies, m.a, m.b, m.p_a, m.d_a, m.p_b, m.d_b,
                );
                let mut rr = row + 6;
                // ── revolute：轴向间距行 r = (pA_w − pB_w)·n − offset ──
                if jk == "revolute" {
                    let dp = v3::sub(pa_w, pb_w);
                    let n_w = da_w;
                    r[rr] = v3::dot(dp, n_w) - offset;
                    if let Some(off) = slots[m.a].param_off {
                        write_row(&mut j, n, rr, off, n_w);
                        let pa_j = point_jac(&bodies[m.a], m.p_a);
                        let n_j = dir_jac(&bodies[m.a], m.d_a);
                        let g = v3::add(tmul(&pa_j, n_w), tmul(&n_j, dp));
                        write_row(&mut j, n, rr, off + 3, g);
                    }
                    if let Some(off) = slots[m.b].param_off {
                        write_row(&mut j, n, rr, off, v3::scale(n_w, -1.0));
                        let g = tmul(&point_jac(&bodies[m.b], m.p_b), n_w);
                        write_row(&mut j, n, rr, off + 3, v3::scale(g, -1.0));
                    }
                    rr += 1;
                }
                // ── prismatic 滚转锁定：Angle 机制（a_ref/b_ref/cos(roll)）──
                if let Some((ra, rb, cos_v)) = roll.as_ref() {
                    let ua = world_dir(&bodies[m.a], *ra);
                    let ub = world_dir(&bodies[m.b], *rb);
                    r[rr] = v3::dot(ua, ub) - cos_v;
                    if let Some(off) = slots[m.a].param_off {
                        let g = tmul(&dir_jac(&bodies[m.a], *ra), ub);
                        write_row(&mut j, n, rr, off + 3, g);
                    }
                    if let Some(off) = slots[m.b].param_off {
                        let g = tmul(&dir_jac(&bodies[m.b], *rb), ua);
                        write_row(&mut j, n, rr, off + 3, g);
                    }
                    rr += 1;
                }
                // ── 驱动行：关节变量（初始为零点）到目标 ──
                if let Some(target) = drive.as_ref() {
                    if jk == "prismatic" {
                        let (delta, (gat, gaw), (gbt, gbw)) =
                            measure_delta(&bodies[m.a], &bodies[m.b], m.p_a, m.d_a, m.p_b);
                        r[rr] = (delta - m.delta0) - target;
                        if let Some(off) = slots[m.a].param_off {
                            write_row(&mut j, n, rr, off, gat);
                            write_row(&mut j, n, rr, off + 3, gaw);
                        }
                        if let Some(off) = slots[m.b].param_off {
                            write_row(&mut j, n, rr, off, gbt);
                            write_row(&mut j, n, rr, off + 3, gbw);
                        }
                    } else {
                        let (theta, ga, gb) = measure_theta(
                            &bodies[m.a],
                            &bodies[m.b],
                            m.p_a,
                            m.d_a,
                            m.ref_a,
                            m.p_b,
                            m.d_b,
                            m.ref_b,
                        );
                        r[rr] = angle_pi(theta - m.theta0) - target;
                        if let Some(off) = slots[m.a].param_off {
                            write_row(&mut j, n, rr, off + 3, ga);
                        }
                        if let Some(off) = slots[m.b].param_off {
                            write_row(&mut j, n, rr, off + 3, gb);
                        }
                    }
                    rr += 1;
                }
                let _ = rr;
            }
            BlockDesc::Spherical {
                cid: _,
                a,
                b,
                p_a,
                p_b,
            } => {
                // 锚点重合：r = pA_w − pB_w（3 行，秩 3）
                let pa_w = world_point(&bodies[a], p_a);
                let pb_w = world_point(&bodies[b], p_b);
                let d3 = v3::sub(pa_w, pb_w);
                r[row] = d3[0];
                r[row + 1] = d3[1];
                r[row + 2] = d3[2];
                if let Some(off) = slots[a].param_off {
                    write_row(&mut j, n, row, off, [1.0, 0.0, 0.0]);
                    write_row(&mut j, n, row + 1, off, [0.0, 1.0, 0.0]);
                    write_row(&mut j, n, row + 2, off, [0.0, 0.0, 1.0]);
                    let g = point_jac(&bodies[a], p_a);
                    write_mat(&mut j, n, row, off + 3, &g);
                }
                if let Some(off) = slots[b].param_off {
                    write_row(&mut j, n, row, off, [-1.0, 0.0, 0.0]);
                    write_row(&mut j, n, row + 1, off, [0.0, -1.0, 0.0]);
                    write_row(&mut j, n, row + 2, off, [0.0, 0.0, -1.0]);
                    let g = point_jac(&bodies[b], p_b);
                    let neg = [
                        [-g[0][0], -g[0][1], -g[0][2]],
                        [-g[1][0], -g[1][1], -g[1][2]],
                        [-g[2][0], -g[2][1], -g[2][2]],
                    ];
                    write_mat(&mut j, n, row, off + 3, &neg);
                }
            }
            BlockDesc::Transmission {
                cid: _,
                ref ma,
                ref mb,
                ratio,
            } => {
                // r = θ_b − ratio·θ_a（θ 均为初始相对关节变量）
                let (ta, ga_a, ga_b) = measure_theta(
                    &bodies[ma.a],
                    &bodies[ma.b],
                    ma.p_a,
                    ma.d_a,
                    ma.ref_a,
                    ma.p_b,
                    ma.d_b,
                    ma.ref_b,
                );
                let (tb, gb_a, gb_b) = measure_theta(
                    &bodies[mb.a],
                    &bodies[mb.b],
                    mb.p_a,
                    mb.d_a,
                    mb.ref_a,
                    mb.p_b,
                    mb.d_b,
                    mb.ref_b,
                );
                r[row] = angle_pi(tb - mb.theta0) - ratio * angle_pi(ta - ma.theta0);
                // 梯度：dθ_b − ratio·dθ_a。中间体可能同时是两个关节的端点
                //（如行星轮系），贡献必须**累加**而非覆盖。
                let mut add_row = |col: usize, v: [f64; 3]| {
                    for k in 0..3 {
                        j[row * n + col + k] += v[k];
                    }
                };
                if let Some(off) = slots[ma.a].param_off {
                    add_row(off + 3, v3::scale(ga_a, -ratio));
                }
                if let Some(off) = slots[ma.b].param_off {
                    add_row(off + 3, v3::scale(ga_b, -ratio));
                }
                if let Some(off) = slots[mb.a].param_off {
                    add_row(off + 3, gb_a);
                }
                if let Some(off) = slots[mb.b].param_off {
                    add_row(off + 3, gb_b);
                }
            }
        }
        row += block.rows();
    }
    (r, j)
}

/// 按当前参数取向量的体求值元组（自由体取 x，fixed 体取输入位姿）。
fn bodies_now(x: &[f64], slots: &[BodySlot], i: usize) -> ([f64; 3], [f64; 3], Mat3, Mat3) {
    match slots[i].param_off {
        Some(off) => (
            [x[off], x[off + 1], x[off + 2]],
            [x[off + 3], x[off + 4], x[off + 5]],
            rotation([x[off + 3], x[off + 4], x[off + 5]]),
            right_jacobian([x[off + 3], x[off + 4], x[off + 5]]),
        ),
        None => {
            let (t, w) = slots[i].pose;
            (t, w, rotation(w), right_jacobian(w))
        }
    }
}

/// 把 atan2 的 (−π, π] 主值差折回同区间（关节变量的连续化）。
fn angle_pi(x: f64) -> f64 {
    // x 可能略越界（浮点），用两次取模折回
    let mut v = x;
    while v > core::f64::consts::PI {
        v -= 2.0 * core::f64::consts::PI;
    }
    while v <= -core::f64::consts::PI {
        v += 2.0 * core::f64::consts::PI;
    }
    v
}

/// 发射 coaxial 的 6 行残差与雅可比（Joint 块复用 Coaxial 的机制）。
#[allow(clippy::too_many_arguments)]
fn emit_coaxial_rows(
    r: &mut [f64],
    j: &mut [f64],
    n: usize,
    row: usize,
    slots: &[BodySlot],
    bodies: &[([f64; 3], [f64; 3], Mat3, Mat3)],
    a: usize,
    b: usize,
    p_a: [f64; 3],
    d_a: [f64; 3],
    p_b: [f64; 3],
    d_b: [f64; 3],
) {
    let pa_w = v3::add(bodies[a].0, mat3_mul_vec(&bodies[a].2, p_a));
    let da_w = mat3_mul_vec(&bodies[a].2, d_a);
    let pb_w = v3::add(bodies[b].0, mat3_mul_vec(&bodies[b].2, p_b));
    let db_w = mat3_mul_vec(&bodies[b].2, d_b);
    let dp = v3::sub(pa_w, pb_w);
    let rc = v3::cross(da_w, db_w);
    r[row] = rc[0];
    r[row + 1] = rc[1];
    r[row + 2] = rc[2];
    let rp = v3::cross(dp, db_w);
    r[row + 3] = rp[0];
    r[row + 4] = rp[1];
    r[row + 5] = rp[2];
    let write_mat = |j: &mut [f64], row0: usize, col: usize, m: &Mat3| {
        for i in 0..3 {
            for k in 0..3 {
                j[(row0 + i) * n + col + k] = m[i][k];
            }
        }
    };
    if let Some(off) = slots[a].param_off {
        let m_c = mat3_mul_mat(
            &mat3_mul_mat(&hat(db_w), &bodies[a].2),
            &mat3_mul_mat(&hat(d_a), &bodies[a].3),
        );
        write_mat(j, row, off + 3, &m_c);
        let neg_hat = hat(db_w);
        write_mat(
            j,
            row + 3,
            off,
            &[
                [-neg_hat[0][0], -neg_hat[0][1], -neg_hat[0][2]],
                [-neg_hat[1][0], -neg_hat[1][1], -neg_hat[1][2]],
                [-neg_hat[2][0], -neg_hat[2][1], -neg_hat[2][2]],
            ],
        );
        let m_p = mat3_mul_mat(
            &mat3_mul_mat(&hat(db_w), &bodies[a].2),
            &mat3_mul_mat(&hat(p_a), &bodies[a].3),
        );
        write_mat(j, row + 3, off + 3, &m_p);
    }
    if let Some(off) = slots[b].param_off {
        let m_c = mat3_mul_mat(
            &mat3_mul_mat(&hat(da_w), &bodies[b].2),
            &mat3_mul_mat(&hat(d_b), &bodies[b].3),
        );
        let m_c = [
            [-m_c[0][0], -m_c[0][1], -m_c[0][2]],
            [-m_c[1][0], -m_c[1][1], -m_c[1][2]],
            [-m_c[2][0], -m_c[2][1], -m_c[2][2]],
        ];
        write_mat(j, row, off + 3, &m_c);
        write_mat(j, row + 3, off, &hat(db_w));
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
        write_mat(j, row + 3, off + 3, &m_p);
    }
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
            joints: Vec::new(),
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
            joints: Vec::new(),
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

/// 锚点取值：None -> 原点。
fn anchor(v: &Option<crate::model::Vec3>) -> [f64; 3] {
    match v {
        None => [0.0, 0.0, 0.0],
        Some(p) => [p.x, p.y, p.z],
    }
}

/// 构建关节测量上下文：归一轴几何 + 确定性垂直参考 + 初始标定。
fn joint_ctx(
    slots: &[BodySlot],
    a: usize,
    b: usize,
    a_axis: &crate::model::Axis3,
    b_axis: &crate::model::Axis3,
) -> Result<JointCtx, SolveError> {
    let da = unit_raw(&[a_axis.direction.x, a_axis.direction.y, a_axis.direction.z])?;
    let db = unit_raw(&[b_axis.direction.x, b_axis.direction.y, b_axis.direction.z])?;
    let p_a = [a_axis.origin.x, a_axis.origin.y, a_axis.origin.z];
    let p_b = [b_axis.origin.x, b_axis.origin.y, b_axis.origin.z];
    let ref_a = perp_ref(da);
    let ref_b = perp_ref(db);
    // 初始标定：用输入位姿计算 θ₀ 与 δ₀
    let ba = body_pose(&slots[a]);
    let bb = body_pose(&slots[b]);
    let (theta0, _, _) = measure_theta(&ba, &bb, p_a, da, ref_a, p_b, db, ref_b);
    let delta0 = measure_delta(&ba, &bb, p_a, da, p_b).0;
    Ok(JointCtx {
        a,
        b,
        p_a,
        d_a: da,
        p_b,
        d_b: db,
        ref_a,
        ref_b,
        theta0,
        delta0,
    })
}

/// 单位化（无约束上下文版本）。
fn unit_raw(v: &[f64; 3]) -> Result<[f64; 3], SolveError> {
    let n = v3::norm(*v);
    if n == 0.0 {
        return Err(SolveError::InvalidModel {
            reason: String::from("关节轴方向不能是零向量。"),
        });
    }
    Ok(v3::scale(*v, 1.0 / n))
}

/// 体的当前位姿（仅用于构建期的初始标定：自由体取输入位姿）。
fn body_pose(slot: &BodySlot) -> ([f64; 3], [f64; 3], Mat3, Mat3) {
    let (t, w) = slot.pose;
    (t, w, rotation(w), right_jacobian(w))
}

/// 关节角测量（原始 atan2 值）：两垂直参考方向绕轴的有向夹角。
/// 返回 (θ_raw, 对 a 体 ω 的梯度行, 对 b 体 ω 的梯度行)。
///
/// θ = atan2(s, c)，s = (uA×uB)·n，c = uA·uB；
/// uA = RA·ref_a，uB = RB·ref_b，n = RA·d_a。
/// 链式法则全部复用 point/dir 雅可比（经 FD 交叉验证的机器）。
#[allow(clippy::too_many_arguments)]
fn measure_theta(
    ba: &BodyView,
    bb: &BodyView,
    _p_a: [f64; 3],
    d_a: [f64; 3],
    ref_a: [f64; 3],
    _p_b: [f64; 3],
    _d_b: [f64; 3],
    ref_b: [f64; 3],
) -> (f64, [f64; 3], [f64; 3]) {
    let (ja, jb) = (right_jacobian(ba.1), right_jacobian(bb.1));
    // 世界量
    let ua = mat3_mul_vec(&ba.2, ref_a);
    let ub = mat3_mul_vec(&bb.2, ref_b);
    let n = mat3_mul_vec(&ba.2, d_a);
    let cx = v3::cross(ua, ub);
    let s = v3::dot(cx, n);
    let c = v3::dot(ua, ub);
    let theta = libm::atan2(s, c);
    let denom = s * s + c * c;
    if denom < 1e-30 {
        // 参考方向平行/反平行：测量退化（关节绕轴 ±π 处），梯度置零并如实报告
        return (theta, [0.0; 3], [0.0; 3]);
    }
    let k = 1.0 / denom;
    // dθ = (c·ds − s·dc)·k
    // ds|ωA = M_Aᵀ(uB×n) + N_Aᵀ(uA×uB)；dc|ωA = M_Aᵀ·uB
    // ds|ωB = M_Bᵀ(n×uA)；dc|ωB = M_Bᵀ·uA
    // 其中 M_A = ∂uA/∂ωA = −RA·hat(ref_a)·J_A，N_A = ∂n/∂ωA 同构
    // ∂uA/∂ωA = −R·hat(ref_a)·J_A（右扰动约定，负号与 eval 的 point/dir_jac 一致）
    let neg3 = |m: &Mat3| {
        [
            [-m[0][0], -m[0][1], -m[0][2]],
            [-m[1][0], -m[1][1], -m[1][2]],
            [-m[2][0], -m[2][1], -m[2][2]],
        ]
    };
    let m_a = neg3(&mat3_mul_mat(&mat3_mul_mat(&ba.2, &hat(ref_a)), &ja));
    let n_a = neg3(&mat3_mul_mat(&mat3_mul_mat(&ba.2, &hat(d_a)), &ja));
    let m_b = neg3(&mat3_mul_mat(&mat3_mul_mat(&bb.2, &hat(ref_b)), &jb));
    let ga = {
        let t1 = mat3_mul_vec(&mat3_transpose(&m_a), v3::cross(ub, n));
        let t2 = mat3_mul_vec(&mat3_transpose(&n_a), cx);
        let ds = v3::add(t1, t2);
        let dc = mat3_mul_vec(&mat3_transpose(&m_a), ub);
        v3::scale(v3::sub(v3::scale(ds, c), v3::scale(dc, s)), k)
    };
    let gb = {
        let ds = mat3_mul_vec(&mat3_transpose(&m_b), v3::cross(n, ua));
        let dc = mat3_mul_vec(&mat3_transpose(&m_b), ua);
        v3::scale(v3::sub(v3::scale(ds, c), v3::scale(dc, s)), k)
    };
    (theta, ga, gb)
}

/// 轴向间距测量：δ = (pB_w − pA_w)·n。返回 (δ, 对 a 的 (t,ω) 梯度, 对 b 的)。
fn measure_delta(
    ba: &BodyView,
    bb: &BodyView,
    p_a: [f64; 3],
    d_a: [f64; 3],
    p_b: [f64; 3],
) -> (f64, JointGrad, JointGrad) {
    let ja = right_jacobian(ba.1);
    let jb = right_jacobian(bb.1);
    let pa_w = v3::add(ba.0, mat3_mul_vec(&ba.2, p_a));
    let pb_w = v3::add(bb.0, mat3_mul_vec(&bb.2, p_b));
    let n = mat3_mul_vec(&ba.2, d_a);
    let dp = v3::sub(pb_w, pa_w);
    let delta = v3::dot(dp, n);
    // ∂δ/∂tB = n；∂δ/∂tA = −n
    // ∂δ/∂ωB = P_Bᵀ·n（P_B = ∂pb_w/∂ωB）
    // ∂δ/∂ωA = −P_Aᵀ·n + N_Aᵀ·dp（N_A = ∂n/∂ωA）
    let p_a_jac = mat3_mul_mat(&mat3_mul_mat(&ba.2, &hat(p_a)), &ja);
    let p_b_jac = mat3_mul_mat(&mat3_mul_mat(&bb.2, &hat(p_b)), &jb);
    let n_jac = mat3_mul_mat(&mat3_mul_mat(&ba.2, &hat(d_a)), &ja);
    // 真导数带负号（−R·hat·J）：∂δ/∂ωA = +p_a_jacᵀ·n − n_jacᵀ·dp；∂δ/∂ωB = −p_b_jacᵀ·n
    let ga_w = v3::sub(
        mat3_mul_vec(&mat3_transpose(&p_a_jac), n),
        mat3_mul_vec(&mat3_transpose(&n_jac), dp),
    );
    let gb_w = v3::scale(mat3_mul_vec(&mat3_transpose(&p_b_jac), n), -1.0);
    (delta, (v3::scale(n, -1.0), ga_w), (n, gb_w))
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
                    p_a: [0.15, -0.05, 0.20],
                    p_b: [0.05, 0.10, -0.15],
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

    /// 新块类型（revolute/cylindrical/prismatic/spherical/transmission/drive）
    /// 的解析雅可比 vs 中心差分。三体模型：关节 J1 连 0-1，J2 连 1-2，
    /// 传动耦合 J1 与 J2。
    #[test]
    fn analytic_jacobian_matches_fd_joints_and_transmission() {
        let x: Vec<f64> = Vec::from([
            0.10, 0.05, -0.02, 0.01, -0.02, 0.03, // body 0
            0.60, 0.20, 0.30, 0.05, 0.02, -0.10, // body 1
            1.10, -0.30, 0.50, -0.06, 0.12, 0.08, // body 2
        ]);
        let mk_ctx = |a: usize, b: usize| {
            let d_a = unit3([0.0, 0.0, 1.0]);
            let d_b = unit3([0.1, 0.1, 1.0]);
            JointCtx {
                a,
                b,
                p_a: [0.0, 0.0, 0.0],
                d_a,
                p_b: [0.05, -0.05, 0.0],
                d_b,
                ref_a: perp_ref(d_a),
                ref_b: perp_ref(d_b),
                theta0: 0.0,
                delta0: 0.0,
            }
        };
        let blocks = Vec::from([
            BlockDesc::Joint {
                cid: 1,
                jk: "revolute",
                m: mk_ctx(0, 1),
                offset: 0.15,
                roll: None,
                drive: Some(0.30),
            },
            BlockDesc::Joint {
                cid: 2,
                jk: "prismatic",
                m: mk_ctx(1, 2),
                offset: 0.0,
                roll: Some(([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], libm::cos(0.4))),
                drive: Some(0.20),
            },
            BlockDesc::Joint {
                cid: 3,
                jk: "cylindrical",
                m: mk_ctx(0, 2),
                offset: 0.0,
                roll: None,
                drive: None,
            },
            BlockDesc::Spherical {
                cid: 4,
                a: 0,
                b: 2,
                p_a: [0.10, 0.20, 0.30],
                p_b: [-0.05, 0.15, 0.25],
            },
            BlockDesc::Transmission {
                cid: 5,
                ma: Box::new(mk_ctx(0, 1)),
                mb: Box::new(mk_ctx(1, 2)),
                ratio: -0.5,
            },
        ]);
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
            BodySlot {
                entity_idx: 2,
                param_off: Some(12),
                pose: ([0.0; 3], [0.0; 3]),
            },
        ]);
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
