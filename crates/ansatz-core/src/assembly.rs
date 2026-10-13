//! 3D 装配约束求解：6-DOF 刚体（Rigid3）的 mate / coaxial / distance / angle /
//! fixed，以及关节族（revolute / cylindrical / prismatic / spherical）与传动耦合。
//!
//! 数值内核：**Levenberg–Marquardt**，步长用 SVD 阻尼伪逆（`linalg`，分解本体
//! 由 nalgebra 提供），解析雅可比（SO(3) 三维局部切空间增量）。
//!
//! 诊断（全部基于终态雅可比 J 与残差 r）：
//! - `dof_remaining = 自由参数数 − rank(J)`；条件数 `σ_max/σ_min(非零)`。
//! - **冲突**：仅报告已证实的矛盾（负距离、固定体上的不满足约束，或相同锚点
//!   的互斥距离）。非线性驻点和左零空间残差不构成全局无解证明。
//! - **冗余**：约束块 B 可删 ⇔ 去掉 B 的行后 rank(J) 不变；可删块之间
//!   若相互重叠（合起来带来的秩小于各自贡献之和）则归入同组。判定与
//!   约束书写顺序无关。
//!
//! 残差定义（世界系，n/d 均为单位化）：
//! - mate：`[dot(pA−pB, nB); nA+nB]`（4 行，秩 3）
//! - coaxial：`[dA−dB; (pA−pB)×dB]`（6 行，秩 4，同向）
//! - distance：`‖PA−PB‖ − v`（1 行，锚点间距）
//! - angle：真实夹角减目标；0/π 用方向对齐向量（避免端点秩退化）。
//! - fixed：结构性消参（0 行）；同一刚体重复 fixed 直接报冗余。

use crate::linalg::{equilibrate_columns, rank_from_sv, singular_values, svd_lm_step};
use crate::model::{Constraint, ConstraintKind, Entity, Geometry, Model, SolveOutcome, Vec3};
use crate::report::{
    Action, ConstraintGroup, ConstraintResidual, Diagnostics, SolveError, SolveReport, Suggestion,
};
use crate::so3::{hat, mat3_mul_mat, mat3_mul_vec, mat3_transpose, retract, rotation, v3, Mat3};
use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

/// 一个刚体在求解器中的槽位。
struct BodySlot {
    /// 在 model.entities 中的下标。
    entity_idx: usize,
    /// 位姿存储偏移（t 3 + 旋转向量 3）；雅可比列和求解步使用局部三维转动增量。
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
        value: f64,
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
        /// prismatic 有向滚转锁定：(a_ref, b_ref, roll)。
        roll: Option<([f64; 3], [f64; 3], f64)>,
        /// 驱动目标（绝对关节变量）。
        drive: Option<f64>,
    },
    Spherical {
        cid: u32,
        a: usize,
        b: usize,
        p_a: [f64; 3],
        p_b: [f64; 3],
    },
    /// 传动耦合：θ_b − ratio·θ_a = 0（θ 为绝对关节角）。
    Transmission {
        cid: u32,
        ma: Box<JointCtx>,
        mb: Box<JointCtx>,
        ratio: f64,
    },
}

/// 求值中单体的紧凑视图：(t, ω, R, I)。局部转动增量的右雅可比为 I。
type BodyView = ([f64; 3], [f64; 3], Mat3, Mat3);

/// 关节变量测量结果的梯度：(平移行, 旋转行)。
type JointGrad = ([f64; 3], [f64; 3]);

/// 关节测量上下文：轴几何 + 垂直于轴的零位参考方向（均在体坐标系内）。
/// 关节变量只由当前位姿决定，不记录任何输入位形。
struct JointCtx {
    a: usize,
    b: usize,
    p_a: [f64; 3],
    d_a: [f64; 3],
    p_b: [f64; 3],
    d_b: [f64; 3],
    /// a 体上垂直于轴的单位零位参考方向。
    ref_a: [f64; 3],
    /// b 体上垂直于轴的单位零位参考方向。
    ref_b: [f64; 3],
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
            BlockDesc::Distance { value: 0.0, .. } => 3,
            BlockDesc::Angle { value, .. } if *value == 0.0 || *value == core::f64::consts::PI => 3,
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

    /// 块涉及的刚体槽位。
    fn bodies(&self) -> Vec<usize> {
        match self {
            BlockDesc::Mate { a, b, .. }
            | BlockDesc::Coaxial { a, b, .. }
            | BlockDesc::Distance { a, b, .. }
            | BlockDesc::Angle { a, b, .. }
            | BlockDesc::Spherical { a, b, .. } => Vec::from([*a, *b]),
            BlockDesc::Joint { m, .. } => Vec::from([m.a, m.b]),
            BlockDesc::Transmission { ma, mb, .. } => Vec::from([ma.a, ma.b, mb.a, mb.b]),
            BlockDesc::Fixed { .. } => Vec::new(),
        }
    }
}

/// 模型含 rigid3 实体时走本路径。
pub(crate) fn solve(model: &Model) -> Result<SolveReport, SolveError> {
    // ── 1. 收集刚体，校验实体类型 ─────────────────────────────────
    let mut slots: Vec<BodySlot> = Vec::new();
    let mut fixed: Vec<(u32, u32)> = Vec::new(); // (约束 id, 实体 id)
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
            fixed.push((constraint.id, *a));
        }
    }

    // ── 2. 校验并构建约束块（含归一化）────────────────────────────
    let tol = model.params.tolerance;
    let mut blocks: Vec<BlockDesc> = Vec::new();
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
                    a_dir: unit(c, &[da.x, da.y, da.z], "a_dir")?,
                    b_dir: unit(c, &[db.x, db.y, db.z], "b_dir")?,
                    value: *value,
                }
            }
            ConstraintKind::Revolute {
                a,
                b,
                a_axis,
                b_axis,
                offset,
                a_ref,
                b_ref,
                drive,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                let m = joint_ctx(c.id, sa, sb, a_axis, b_axis, a_ref, b_ref)?;
                BlockDesc::Joint {
                    cid: c.id,
                    jk: "revolute",
                    m,
                    offset: offset.unwrap_or(0.0),
                    roll: None,
                    drive: drive.map(angle_pi),
                }
            }
            ConstraintKind::Cylindrical {
                a,
                b,
                a_axis,
                b_axis,
                a_ref,
                b_ref,
            } => {
                let (sa, sb) = two_slots(c, *a, *b, &find_slot)?;
                let m = joint_ctx(c.id, sa, sb, a_axis, b_axis, a_ref, b_ref)?;
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
                let m = joint_ctx(c.id, sa, sb, a_axis, b_axis, a_ref, b_ref)?;
                let roll_data = match (a_ref, b_ref, roll) {
                    (Some(_), Some(_), Some(v)) => {
                        if !(0.0..=core::f64::consts::PI).contains(v) {
                            return Err(SolveError::InvalidModel {
                                reason: format!(
                                    "约束 {}（prismatic）的 roll 必须在 [0, π] 弧度内，得到 {v:e}。",
                                    c.id
                                ),
                            });
                        }
                        Some((m.ref_a, m.ref_b, *v))
                    }
                    (None, None, None) => Some((m.ref_a, m.ref_b, 0.0)),
                    _ => {
                        return Err(SolveError::InvalidModel {
                            reason: format!(
                                "约束 {}（prismatic）的 a_ref、b_ref 和 roll 必须一起提供。",
                                c.id
                            ),
                        })
                    }
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
                if joint_a == joint_b {
                    return Err(SolveError::InvalidModel {
                        reason: format!(
                            "传动约束 {} 的 joint_a 与 joint_b 都是 {}：传动必须耦合两个不同的关节。",
                            c.id, joint_a
                        ),
                    });
                }
                // 两个被耦合的关节必须是本模型中已声明的 revolute
                let find_revolute = |want: u32| -> Result<JointCtx, SolveError> {
                    for c2 in &model.constraints {
                        if c2.id != want {
                            continue;
                        }
                        if let ConstraintKind::Revolute {
                            a,
                            b,
                            a_axis,
                            b_axis,
                            a_ref,
                            b_ref,
                            ..
                        } = &c2.kind
                        {
                            let (sa, sb) = two_slots(c2, *a, *b, &find_slot)?;
                            return joint_ctx(c2.id, sa, sb, a_axis, b_axis, a_ref, b_ref);
                        }
                    }
                    Err(SolveError::InvalidModel {
                        reason: format!(
                            "传动约束 {} 引用的关节 {want} 不存在或不是 revolute 关节。",
                            c.id
                        ),
                    })
                };
                let ma = find_revolute(*joint_a)?;
                let mb = find_revolute(*joint_b)?;
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

    // ── 3. 自由参数布局：fixed 体 0 参数，其余 6 个 ────────────────
    let mut n = 0usize;
    for slot in &mut slots {
        let id = model.entities[slot.entity_idx].id;
        if fixed.iter().any(|&(_, eid)| eid == id) {
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
    ensure_finite(&r, &j)?;
    let mut converged = satisfied(&x, &slots, &blocks, &r, tol);
    let mut iterations = 0u32;
    let mut lambda = 1e-3f64;
    let max_iter = model.params.max_iterations;
    let initial_ranges = row_ranges(&blocks);
    let proven_at_input = find_conflicts(&x, &r, tol, &blocks, &initial_ranges, &slots);
    while !converged && proven_at_input.is_empty() && iterations < max_iter && n > 0 {
        iterations += 1;
        // LM 步用 SVD 阻尼伪逆（svd_lm_step）：装配约束的残差行天然线性
        // 相关（mate 的 nA+nB 秩 2/3、coaxial 的叉积秩 4/6），正规方程在
        // λ 小时病态（条件数 ~1/σ² → 步长放大 → 拒绝 → λ 震荡爬行）；
        // SVD 形式把零奇异方向截断，秩亏下仍二次收敛。
        let r_norm = residual_norm(&r);
        let (scaled_j, divisors) = equilibrate_columns(&j, m, n)?;
        let mut accepted = false;
        for _ in 0..8 {
            let mut d = svd_lm_step(&scaled_j, &r, m, n, lambda)?;
            for (v, divisor) in d.iter_mut().zip(&divisors) {
                *v /= divisor;
            }
            let x_new = apply_step(&x, &slots, &d);
            if !x_new.iter().all(|v| v.is_finite()) {
                lambda = (lambda * 8.0).min(1e14);
                continue;
            }
            let (r_new, j_new) = eval(&x_new, &slots, &blocks);
            if r_new.iter().chain(&j_new).all(|v| v.is_finite()) && residual_norm(&r_new) < r_norm {
                x = x_new;
                r = r_new;
                j = j_new;
                converged = satisfied(&x, &slots, &blocks, &r, tol);
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
            // Anti-aligned direction residuals can be stationary maxima.
            // Try a bounded, deterministic local rotation; never call this
            // stall a proof that the nonlinear model has no solution.
            if let Some((xn, rn, jn)) = escape_alignment(&x, &slots, &blocks, r_norm) {
                x = xn;
                r = rn;
                j = jn;
                converged = satisfied(&x, &slots, &blocks, &r, tol);
                lambda = 1e-3;
            } else {
                break;
            }
        }
    }

    // ── 5. 秩、冲突与冗余 ─────────────────────────────────────────
    let (diagnostic_j, _) = equilibrate_columns(&j, m, n)?;
    let sv = singular_values(&diagnostic_j, m, n)?;
    let rank = rank_from_sv(&sv, m, n);
    let condition = if rank > 0 {
        Some(sv[0] / sv[rank - 1])
    } else {
        None
    };
    let dof_remaining = (n - rank) as u32;
    let ranges = row_ranges(&blocks);

    let conflict_blocks = if converged {
        Vec::new()
    } else {
        find_conflicts(&x, &r, tol, &blocks, &ranges, &slots)
    };
    let mut in_conflict = alloc::vec![false; blocks.len()];
    for group in &conflict_blocks {
        for &k in group {
            in_conflict[k] = true;
        }
    }
    let redundant_blocks = if n > 0 && converged {
        redundant_groups(&diagnostic_j, n, &sv, &blocks, &ranges, &in_conflict)?
    } else {
        Vec::new()
    };

    let mut residuals: Vec<ConstraintResidual> = Vec::new();
    for (block, range) in blocks.iter().zip(&ranges) {
        if range.is_empty() {
            continue;
        }
        let block_max = r[range.clone()]
            .iter()
            .fold(0.0f64, |mx, v| mx.max(v.abs()));
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
    let max_residual = residuals
        .iter()
        .max_by(|a, b| a.residual.abs().total_cmp(&b.residual.abs()))
        .cloned();

    let mut suggestions: Vec<Suggestion> = Vec::new();
    let mut conflicts: Vec<ConstraintGroup> = Vec::new();
    for group in &conflict_blocks {
        let ids: Vec<u32> = group.iter().map(|&k| blocks[k].cid()).collect();
        let worst = group
            .iter()
            .flat_map(|&k| r[ranges[k].clone()].iter())
            .fold(0.0f64, |mx, v| mx.max(v.abs()));
        conflicts.push(ConstraintGroup {
            constraint_ids: ids.clone(),
            human_message: format!(
                "约束 {ids:?} 存在已证实的矛盾：负距离、固定体上的约束不满足，或相同锚点距离目标互斥；\
                 返回位形的组内最大残差为 {worst:e}。"
            ),
        });
        let last = ids[ids.len() - 1];
        suggestions.push(if ids.len() == 1 {
            Suggestion {
                action: Action::RelaxConstraint { id: last },
                human_message: format!("调整约束 {last} 的参数（或它引用的几何），使其可被满足。"),
            }
        } else {
            Suggestion {
                action: Action::RemoveConstraint { id: last },
                human_message: format!(
                    "先从约束 {last} 排查冲突组 {ids:?}；修改后重新求解，组内可能仍有其他矛盾。"
                ),
            }
        });
    }

    let mut redundant: Vec<ConstraintGroup> = Vec::new();
    for (eid, ids) in duplicate_fixed(&fixed) {
        redundant.push(ConstraintGroup {
            constraint_ids: ids.clone(),
            human_message: format!("约束 {ids:?} 重复固定了同一个刚体 {eid}，保留一条即可。"),
        });
    }
    for group in &redundant_blocks {
        let ids: Vec<u32> = group.iter().map(|&k| blocks[k].cid()).collect();
        let human_message = if ids.len() == 1 {
            format!(
                "约束 {} 在当前位形的线性化中不增加秩；删除后须重新求解确认几何行为。",
                ids[0]
            )
        } else {
            format!("约束 {ids:?} 在当前位形的线性化中相互冗余；不能全部删除，修改后须重新验证。")
        };
        redundant.push(ConstraintGroup {
            constraint_ids: ids,
            human_message,
        });
    }
    for group in &redundant {
        let last = group.constraint_ids[group.constraint_ids.len() - 1];
        suggestions.push(Suggestion {
            action: Action::RemoveConstraint { id: last },
            human_message: format!(
                "约束 {last} 是冗余的，可以删除；删除后重新求解可确认是否仍有冗余。"
            ),
        });
    }
    if dof_remaining > 0 {
        suggestions.push(Suggestion {
            action: Action::AddConstraint { dof_remaining },
            human_message: format!(
                "装配体还有 {dof_remaining} 个自由度未被约束。可用的 3D 装配约束：\
                 fixed 消去 6 个、revolute / prismatic 各消去 5 个、coaxial / cylindrical \
                 各消去 4 个、mate / spherical 各消去 3 个；distance / angle 的贡献需按目标和位形判断；\
                 关节上的 drive 再消去 1 个。"
            ),
        });
    }

    let mut joints: Vec<crate::report::JointState> = Vec::new();
    for block in &blocks {
        let BlockDesc::Joint {
            cid, jk, m, drive, ..
        } = block
        else {
            continue;
        };
        let (ba, bb) = (bodies_now(&x, &slots, m.a), bodies_now(&x, &slots, m.b));
        let value = if *jk == "prismatic" {
            measure_delta(&ba, &bb, m.p_a, m.d_a, m.p_b).0
        } else {
            measure_theta(&ba, &bb, m.d_a, m.ref_a, m.ref_b).0
        };
        let drive_note = match drive {
            Some(t) => format!("；驱动目标 {t:e}"),
            None => String::new(),
        };
        joints.push(crate::report::JointState {
            constraint_id: *cid,
            joint_kind: (*jk).into(),
            value,
            human_message: format!("关节 {cid}（{jk}）的当前变量为 {value:e}{drive_note}。"),
        });
    }

    // ── 6. 结局：只描述返回位形是否满足约束 ──────────────────────
    let outcome = if !conflicts.is_empty() {
        SolveOutcome::Inconsistent
    } else if converged {
        SolveOutcome::Converged
    } else {
        SolveOutcome::NotConverged
    };

    let mut entities = model.entities.clone();
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

    Ok(SolveReport {
        outcome,
        entities,
        diagnostics: Diagnostics {
            dof_total,
            dof_remaining,
            iterations,
            conflicts,
            redundant_constraints: redundant,
            residuals,
            max_residual,
            suggestions,
            joints,
            jacobian_condition_estimate: condition,
        },
    })
}

// ══════════════════════════ 冲突与冗余分析 ══════════════════════════

const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn residual_norm(r: &[f64]) -> f64 {
    r.iter().fold(0.0, |norm, v| libm::hypot(norm, *v))
}

fn ensure_finite(r: &[f64], j: &[f64]) -> Result<(), SolveError> {
    if r.iter().chain(j).all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(SolveError::InvalidModel {
            reason: "模型的坐标或参数在求值中溢出，无法产生有限的残差/雅可比。".into(),
        })
    }
}

fn apply_step(x: &[f64], slots: &[BodySlot], d: &[f64]) -> Vec<f64> {
    let mut out = x.to_vec();
    for slot in slots {
        if let Some(off) = slot.param_off {
            for k in 0..3 {
                out[off + k] += d[off + k];
            }
            let w = retract(
                [x[off + 3], x[off + 4], x[off + 5]],
                [d[off + 3], d[off + 4], d[off + 5]],
            );
            out[off + 3..off + 6].copy_from_slice(&w);
        }
    }
    out
}

fn escape_alignment(
    x: &[f64],
    slots: &[BodySlot],
    blocks: &[BlockDesc],
    cost: f64,
) -> Option<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    for off in slots.iter().filter_map(|s| s.param_off).take(8) {
        for axis in 0..3 {
            let mut d = alloc::vec![0.0; x.len()];
            d[off + 3 + axis] = core::f64::consts::FRAC_PI_2;
            let xn = apply_step(x, slots, &d);
            let (rn, jn) = eval(&xn, slots, blocks);
            if rn.iter().chain(&jn).all(|v| v.is_finite()) && residual_norm(&rn) < cost {
                return Some((xn, rn, jn));
            }
        }
    }
    None
}

fn block_satisfied(x: &[f64], slots: &[BodySlot], block: &BlockDesc, r: &[f64], tol: f64) -> bool {
    if matches!(block, BlockDesc::Distance { value, .. } if *value < 0.0) {
        return false;
    }
    if !r.iter().all(|v| v.is_finite() && v.abs() <= tol) {
        return false;
    }
    if let BlockDesc::Angle {
        a,
        b,
        a_dir,
        b_dir,
        value,
        ..
    } = block
    {
        let ua = mat3_mul_vec(&bodies_now(x, slots, *a).2, *a_dir);
        let ub = mat3_mul_vec(&bodies_now(x, slots, *b).2, *b_dir);
        let theta = libm::atan2(v3::norm(v3::cross(ua, ub)), v3::dot(ua, ub));
        return (theta - value).abs() <= tol;
    }
    if matches!(block, BlockDesc::Distance { value: 0.0, .. }) {
        return residual_norm(r) <= tol;
    }
    true
}

fn satisfied(x: &[f64], slots: &[BodySlot], blocks: &[BlockDesc], r: &[f64], tol: f64) -> bool {
    blocks
        .iter()
        .zip(row_ranges(blocks))
        .all(|(b, range)| block_satisfied(x, slots, b, &r[range], tol))
}

/// 每个约束块在 J / r 中的行区间。
fn row_ranges(blocks: &[BlockDesc]) -> Vec<core::ops::Range<usize>> {
    let mut row = 0usize;
    blocks
        .iter()
        .map(|b| {
            let start = row;
            row += b.rows();
            start..row
        })
        .collect()
}

/// 只保留 `keep[k]` 为真的块的行，按绝对阈值 `tau` 计秩。
fn rank_of(
    j: &[f64],
    n: usize,
    ranges: &[core::ops::Range<usize>],
    keep: &[bool],
    tau: f64,
) -> Result<usize, SolveError> {
    let mut sub: Vec<f64> = Vec::new();
    let mut rows = 0usize;
    for (range, &k) in ranges.iter().zip(keep) {
        if k {
            sub.extend_from_slice(&j[range.start * n..range.end * n]);
            rows += range.len();
        }
    }
    Ok(singular_values(&sub, rows, n)?
        .iter()
        .filter(|&&s| s > tau)
        .count())
}

/// Proven conflicts only. A nonlinear stationary residual is not a proof.
fn find_conflicts(
    x: &[f64],
    r: &[f64],
    tol: f64,
    blocks: &[BlockDesc],
    ranges: &[core::ops::Range<usize>],
    slots: &[BodySlot],
) -> Vec<Vec<usize>> {
    let mut proofs: Vec<Vec<usize>> = Vec::new();
    for (k, block) in blocks.iter().enumerate() {
        if block.bodies().iter().all(|&s| slots[s].param_off.is_none())
            && !block_satisfied(x, slots, block, &r[ranges[k].clone()], tol)
        {
            proofs.push(alloc::vec![k]);
        }
        let BlockDesc::Distance {
            a,
            b,
            value,
            p_a,
            p_b,
            ..
        } = block
        else {
            continue;
        };
        if *value < 0.0 {
            proofs.push(alloc::vec![k]);
            continue;
        }
        for (other_k, other) in blocks[..k].iter().enumerate() {
            let BlockDesc::Distance {
                a: oa,
                b: ob,
                value: ov,
                p_a: opa,
                p_b: opb,
                ..
            } = other
            else {
                continue;
            };
            let same = (a == oa && b == ob && p_a == opa && p_b == opb)
                || (a == ob && b == oa && p_a == opb && p_b == opa);
            // Both absolute-tolerance intervals must be disjoint.
            if same && (value - ov).abs() / 2.0 > tol {
                proofs.push(alloc::vec![other_k, k]);
            }
        }
    }
    components(proofs.len(), |a, b| {
        proofs[a].iter().any(|v| proofs[b].contains(v))
    })
    .into_iter()
    .map(|group| {
        let mut ids: Vec<usize> = group
            .into_iter()
            .flat_map(|g| proofs[g].iter().copied())
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids
    })
    .collect()
}

/// 冗余组（块下标），与约束书写顺序无关：
/// - 块 B 可删 ⇔ 去掉 B 的行后秩不变；
/// - 记 K 为全部不可删块，B 的贡献 c(B) = rank(K∪B) − rank(K)；
///   可删块 B、C 同组 ⇔ rank(K∪B∪C) < rank(K) + c(B) + c(C)（相互重叠）。
fn redundant_groups(
    j: &[f64],
    n: usize,
    sv: &[f64],
    blocks: &[BlockDesc],
    ranges: &[core::ops::Range<usize>],
    excluded: &[bool],
) -> Result<Vec<Vec<usize>>, SolveError> {
    let Some(&smax) = sv.first() else {
        return Ok(Vec::new());
    };
    let tau = smax * 1e-10;
    let all: Vec<bool> = ranges.iter().map(|r| !r.is_empty()).collect();
    let full = rank_of(j, n, ranges, &all, tau)?;
    let mut removable = Vec::new();
    for k in 0..blocks.len() {
        if !all[k] || excluded[k] {
            continue;
        }
        let mut keep = all.clone();
        keep[k] = false;
        if rank_of(j, n, ranges, &keep, tau)? == full {
            removable.push(k);
        }
    }
    if removable.is_empty() {
        return Ok(Vec::new());
    }
    let mut base = all.clone();
    for &k in &removable {
        base[k] = false;
    }
    let rank_base = rank_of(j, n, ranges, &base, tau)?;
    let with = |extra: &[usize]| {
        let mut keep = base.clone();
        for &k in extra {
            keep[k] = true;
        }
        rank_of(j, n, ranges, &keep, tau)
    };
    let contrib: Vec<usize> = removable
        .iter()
        .map(|&k| {
            with(&[k])?
                .checked_sub(rank_base)
                .ok_or_else(|| SolveError::InvalidModel {
                    reason: "数值秩在添加约束后降低，无法可靠分析冗余；请缩放模型。".into(),
                })
        })
        .collect::<Result<_, _>>()?;
    let mut links = alloc::vec![false; removable.len() * removable.len()];
    for a in 0..removable.len() {
        for b in a + 1..removable.len() {
            links[a * removable.len() + b] =
                with(&[removable[a], removable[b]])? < rank_base + contrib[a] + contrib[b];
        }
    }
    Ok(
        components(removable.len(), |a, b| links[a * removable.len() + b])
            .into_iter()
            .map(|g| g.into_iter().map(|i| removable[i]).collect())
            .collect(),
    )
}

/// 无向图 `0..len` 的连通分量；每个分量内升序，分量按最小成员排序。
fn components(len: usize, linked: impl Fn(usize, usize) -> bool) -> Vec<Vec<usize>> {
    let mut label: Vec<usize> = (0..len).collect();
    for a in 0..len {
        for b in (a + 1)..len {
            if label[a] != label[b] && linked(a, b) {
                let (keep, drop) = (label[a].min(label[b]), label[a].max(label[b]));
                for l in label.iter_mut() {
                    if *l == drop {
                        *l = keep;
                    }
                }
            }
        }
    }
    (0..len)
        .filter(|&i| label[i] == i)
        .map(|root| (0..len).filter(|&i| label[i] == root).collect())
        .collect()
}

/// 同一刚体上的重复 fixed：(实体 id, 约束 id 列表)。
fn duplicate_fixed(fixed: &[(u32, u32)]) -> Vec<(u32, Vec<u32>)> {
    let mut out: Vec<(u32, Vec<u32>)> = Vec::new();
    for &(cid, eid) in fixed {
        match out.iter_mut().find(|(e, _)| *e == eid) {
            Some((_, ids)) => ids.push(cid),
            None => out.push((eid, Vec::from([cid]))),
        }
    }
    out.retain(|(_, ids)| ids.len() > 1);
    out
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
                IDENTITY,
            )),
            None => {
                // fixed：用输入位姿求值（雅可比不写，R/J_r 仍须正确以算世界量）
                let (t, w) = slot.pose;
                bodies.push((t, w, rotation(w), IDENTITY))
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
                if value == 0.0 {
                    r[row..row + 3].copy_from_slice(&dt);
                    for (s, p, sign) in [(a, p_a, 1.0), (b, p_b, -1.0)] {
                        if let Some(off) = slots[s].param_off {
                            let mut g = point_jac(&bodies[s], p);
                            for k in 0..3 {
                                j[(row + k) * n + off + k] = sign;
                                for v in &mut g[k] {
                                    *v *= sign;
                                }
                            }
                            write_mat(&mut j, n, row, off + 3, &g);
                        }
                    }
                    row += block.rows();
                    continue;
                }
                r[row] = dist - value;
                {
                    // At coincident anchors choose a deterministic one-sided
                    // direction, rather than pretending a zero Jacobian proves
                    // the positive-distance target is impossible.
                    let u = if dist > 0.0 {
                        [dt[0] / dist, dt[1] / dist, dt[2] / dist]
                    } else {
                        [1.0, 0.0, 0.0]
                    };
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
                }
            }
            BlockDesc::Angle {
                cid: _,
                a,
                b,
                a_dir,
                b_dir,
                value,
            } => {
                let da_w = world_dir(&bodies[a], a_dir);
                let db_w = world_dir(&bodies[b], b_dir);
                if value == 0.0 || value == core::f64::consts::PI {
                    let sign = if value == 0.0 { -1.0 } else { 1.0 };
                    let d = v3::add(da_w, v3::scale(db_w, sign));
                    r[row..row + 3].copy_from_slice(&d);
                    if let Some(off) = slots[a].param_off {
                        write_mat(&mut j, n, row, off + 3, &dir_jac(&bodies[a], a_dir));
                    }
                    if let Some(off) = slots[b].param_off {
                        let mut g = dir_jac(&bodies[b], b_dir);
                        for vals in &mut g {
                            for v in vals {
                                *v *= sign;
                            }
                        }
                        write_mat(&mut j, n, row, off + 3, &g);
                    }
                } else {
                    let sine = v3::norm(v3::cross(da_w, db_w));
                    let cosine = v3::dot(da_w, db_w);
                    r[row] = libm::atan2(sine, cosine) - value;
                    let (ga, gb) = if sine > 1e-12 {
                        (
                            v3::scale(tmul(&dir_jac(&bodies[a], a_dir), db_w), -1.0 / sine),
                            v3::scale(tmul(&dir_jac(&bodies[b], b_dir), da_w), -1.0 / sine),
                        )
                    } else {
                        let u = perp_ref(da_w);
                        (
                            v3::scale(tmul(&dir_jac(&bodies[a], a_dir), u), -1.0),
                            v3::scale(
                                tmul(&dir_jac(&bodies[b], b_dir), u),
                                if cosine >= 0.0 { 1.0 } else { -1.0 },
                            ),
                        )
                    };
                    if let Some(off) = slots[a].param_off {
                        write_row(&mut j, n, row, off + 3, ga);
                    }
                    if let Some(off) = slots[b].param_off {
                        write_row(&mut j, n, row, off + 3, gb);
                    }
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
                a,
                b,
                p_a,
                d_a,
                p_b,
                d_b,
                ..
            } => {
                emit_coaxial_rows(
                    &mut r, &mut j, n, row, slots, &bodies, a, b, p_a, d_a, p_b, d_b,
                );
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
                // Signed roll has a nonzero derivative at the usual zero lock.
                if let Some((ra, rb, target)) = roll.as_ref() {
                    let (theta, ga, gb) =
                        measure_theta(&bodies[m.a], &bodies[m.b], m.d_a, *ra, *rb);
                    r[rr] = angle_pi(theta - target);
                    if let Some(off) = slots[m.a].param_off {
                        write_row(&mut j, n, rr, off + 3, ga);
                    }
                    if let Some(off) = slots[m.b].param_off {
                        write_row(&mut j, n, rr, off + 3, gb);
                    }
                    rr += 1;
                }
                // ── 驱动行：绝对关节变量到目标 ──
                if let Some(target) = drive.as_ref() {
                    if jk == "prismatic" {
                        let (delta, (gat, gaw), (gbt, gbw)) =
                            measure_delta(&bodies[m.a], &bodies[m.b], m.p_a, m.d_a, m.p_b);
                        r[rr] = delta - target;
                        if let Some(off) = slots[m.a].param_off {
                            write_row(&mut j, n, rr, off, gat);
                            write_row(&mut j, n, rr, off + 3, gaw);
                        }
                        if let Some(off) = slots[m.b].param_off {
                            write_row(&mut j, n, rr, off, gbt);
                            write_row(&mut j, n, rr, off + 3, gbw);
                        }
                    } else {
                        let (theta, ga, gb) =
                            measure_theta(&bodies[m.a], &bodies[m.b], m.d_a, m.ref_a, m.ref_b);
                        r[rr] = angle_pi(theta - target);
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
                // r = θ_b − ratio·θ_a（绝对关节角，折回 (−π, π]）
                let (ta, ga_a, ga_b) =
                    measure_theta(&bodies[ma.a], &bodies[ma.b], ma.d_a, ma.ref_a, ma.ref_b);
                let (tb, gb_a, gb_b) =
                    measure_theta(&bodies[mb.a], &bodies[mb.b], mb.d_a, mb.ref_a, mb.ref_b);
                // Reduce the potentially huge product before subtracting the
                // measured angle, which would otherwise be rounded away.
                r[row] = angle_pi(tb - angle_pi(ratio * ta));
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
            IDENTITY,
        ),
        None => {
            let (t, w) = slots[i].pose;
            (t, w, rotation(w), IDENTITY)
        }
    }
}

/// 把 atan2 的 (−π, π] 主值差折回同区间（关节变量的连续化）。
fn angle_pi(x: f64) -> f64 {
    if x > -core::f64::consts::PI && x <= core::f64::consts::PI {
        return x;
    }
    let mut v = libm::fmod(x, 2.0 * core::f64::consts::PI);
    if v > core::f64::consts::PI {
        v -= 2.0 * core::f64::consts::PI;
    }
    if v <= -core::f64::consts::PI {
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
    let rc = v3::sub(da_w, db_w);
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
        let mut m_c = mat3_mul_mat(&bodies[a].2, &mat3_mul_mat(&hat(d_a), &bodies[a].3));
        for vals in &mut m_c {
            for v in vals {
                *v = -*v;
            }
        }
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
        let m_c = mat3_mul_mat(&bodies[b].2, &mat3_mul_mat(&hat(d_b), &bodies[b].3));
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
    if sa == sb {
        return Err(SolveError::InvalidModel {
            reason: format!(
                "约束 {}（{}）的 a 与 b 都是实体 {a}：两体约束必须连接两个不同的刚体。",
                c.id,
                c.kind.kind_name()
            ),
        });
    }
    Ok((sa, sb))
}

/// 锚点取值：None -> 原点。
fn anchor(v: &Option<crate::model::Vec3>) -> [f64; 3] {
    match v {
        None => [0.0, 0.0, 0.0],
        Some(p) => [p.x, p.y, p.z],
    }
}

/// 构建关节测量上下文：归一轴几何 + 零位参考方向（显式给出则投影到
/// 垂直于轴的平面再归一，缺省按轴方向确定性选取）。
fn joint_ctx(
    cid: u32,
    a: usize,
    b: usize,
    a_axis: &crate::model::Axis3,
    b_axis: &crate::model::Axis3,
    a_ref: &Option<Vec3>,
    b_ref: &Option<Vec3>,
) -> Result<JointCtx, SolveError> {
    let unit_axis = |axis: &crate::model::Axis3, field: &str| {
        let d = [axis.direction.x, axis.direction.y, axis.direction.z];
        normalize_direction(d, cid, field)
    };
    let da = unit_axis(a_axis, "a_axis.direction")?;
    let db = unit_axis(b_axis, "b_axis.direction")?;
    let zero_ref = |given: &Option<Vec3>, d: [f64; 3], field: &str| match given {
        None => Ok(perp_ref(d)),
        Some(v) => {
            let v = normalize_direction([v.x, v.y, v.z], cid, field)?;
            let proj = v3::sub(v, v3::scale(d, v3::dot(v, d)));
            let n = v3::norm(proj);
            if n <= 1e-12 * v3::norm(v) || n == 0.0 {
                return Err(SolveError::InvalidModel {
                    reason: format!(
                        "约束 {cid} 的 {field} 与关节轴平行（或为零向量），无法定义关节零位。"
                    ),
                });
            }
            Ok(v3::scale(proj, 1.0 / n))
        }
    };
    Ok(JointCtx {
        a,
        b,
        p_a: [a_axis.origin.x, a_axis.origin.y, a_axis.origin.z],
        d_a: da,
        p_b: [b_axis.origin.x, b_axis.origin.y, b_axis.origin.z],
        d_b: db,
        ref_a: zero_ref(a_ref, da, "a_ref")?,
        ref_b: zero_ref(b_ref, db, "b_ref")?,
    })
}

/// 关节角测量：两零位参考方向绕 a 轴的有向夹角，主值 (−π, π]。
/// 返回 (θ, 对 a 体 ω 的梯度行, 对 b 体 ω 的梯度行)。
///
/// θ = atan2(s, c)，s = (uA×uB)·n，c = uA·uB；
/// uA = RA·ref_a，uB = RB·ref_b，n = RA·d_a。
/// 链式法则全部复用 point/dir 雅可比（经 FD 交叉验证的机器）。
fn measure_theta(
    ba: &BodyView,
    bb: &BodyView,
    d_a: [f64; 3],
    ref_a: [f64; 3],
    ref_b: [f64; 3],
) -> (f64, [f64; 3], [f64; 3]) {
    let (ja, jb) = (ba.3, bb.3);
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
        // uB 与 a 轴平行（同轴尚未满足时可能出现）：角度无定义，梯度置零
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
    let ja = ba.3;
    let jb = bb.3;
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
    normalize_direction(*v, c.id, field)
}

fn normalize_direction(v: [f64; 3], cid: u32, field: &str) -> Result<[f64; 3], SolveError> {
    let max = v.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if max == 0.0 {
        return Err(SolveError::InvalidModel {
            reason: format!("约束 {cid} 的 {field} 是零向量，无法定义方向。"),
        });
    }
    let scaled = [v[0] / max, v[1] / max, v[2] / max];
    Ok(v3::scale(scaled, 1.0 / v3::norm(scaled)))
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
                    value: core::f64::consts::FRAC_PI_3,
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
            let mut dp = alloc::vec![0.0; n];
            let mut dm = alloc::vec![0.0; n];
            dp[col] = h;
            dm[col] = -h;
            let xp = apply_step(&x, &slots, &dp);
            let xm = apply_step(&x, &slots, &dm);
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
                roll: Some(([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.4)),
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
            let mut dp = alloc::vec![0.0; n];
            let mut dm = alloc::vec![0.0; n];
            dp[col] = h;
            dm[col] = -h;
            let xp = apply_step(&x, &slots, &dp);
            let xm = apply_step(&x, &slots, &dm);
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
