//! 线性代数薄层：SVD（秩分析）与 SVD 阻尼伪逆 LM 步。
//!
//! ## 为什么现在站在 nalgebra 上
//!
//! 分解本体用 [nalgebra](https://nalgebra.org)（纯 Rust、无 BLAS/平台分发、
//! no_std 经 libm）——位级可复现的前提它全部满足，且被 Rust 机器人生态
//! 充分验证。本模块保留的是**施加方式**：
//! - `svd_lm_step` 的阻尼伪逆公式与 λ 策略是求解器语义，不随库变；
//! - 秩阈值取相对 1e-10，应用于列尺度均衡后的局部切空间雅可比；
//! - 残差/雅可比的组装顺序在 assembly 模块，固定不变。
//!
//! ## 为什么 LM 循环与 SO(3) 不引库
//!
//! 优化框架（argmin 等）会抢走收敛判据与迭代语义的控制权——那是本项目
//! 的对外契约（MaxIterations 的含义、tolerance 的作用位置）。SO(3) 的
//! 旋转存储与局部增量复合由 so3 模块统一实现，保留自写。

use alloc::vec::Vec;

use crate::report::SolveError;
use nalgebra::{linalg::SVD, DMatrix, DVector, Dyn};

/// 动态矩阵别名。
type DynMat = DMatrix<f64>;

/// A decomposition may consume less than one LM iteration, but it must still
/// have its own nonzero budget: nalgebra's zero budget means unlimited.
const SVD_ITERATION_BUDGET: usize = 4096;

fn numeric_error(reason: &str) -> SolveError {
    SolveError::InvalidModel {
        reason: reason.into(),
    }
}

fn validate_matrix(a: &[f64], m: usize, n: usize) -> Result<(), SolveError> {
    if m.checked_mul(n) != Some(a.len())
        || [m, n].iter().any(|&dim| {
            dim.checked_mul(core::mem::size_of::<f64>())
                .map_or(true, |bytes| bytes > isize::MAX as usize)
        })
    {
        return Err(numeric_error(
            "线性代数矩阵尺寸与数据长度不匹配，或尺寸超出可表示范围。",
        ));
    }
    if !a.iter().all(|v| v.is_finite()) {
        return Err(numeric_error("线性代数矩阵必须只包含有限数值。"));
    }
    Ok(())
}

fn bounded_svd(
    matrix: DynMat,
    compute_u: bool,
    compute_v: bool,
    budget: usize,
) -> Result<SVD<f64, Dyn, Dyn>, SolveError> {
    if budget == 0 {
        return Err(numeric_error("SVD 迭代预算必须非零，不能使用无限迭代。"));
    }
    let sv = matrix
        .try_svd(compute_u, compute_v, 5.0 * f64::EPSILON, budget)
        .ok_or_else(|| {
            numeric_error("SVD 达到数值迭代预算，未能完成分解；无法可靠求解或诊断秩。")
        })?;
    if !sv.singular_values.iter().all(|v| v.is_finite())
        || sv
            .u
            .as_ref()
            .is_some_and(|u| !u.iter().all(|v| v.is_finite()))
        || sv
            .v_t
            .as_ref()
            .is_some_and(|v| !v.iter().all(|v| v.is_finite()))
    {
        return Err(numeric_error("SVD 分解超出有限数值范围；请缩放模型。"));
    }
    Ok(sv)
}

/// Normalize each translation/rotation triplet by its largest absolute entry.
/// A shared scale preserves the metric within a 3D vector, rather than
/// magnifying nearly unconstrained individual axes. Keep the divisors so
/// an LM step can be converted back to physical units.
/// Rank comparisons use the same equilibrated full matrix for every subset.
pub fn equilibrate_columns(
    a: &[f64],
    m: usize,
    n: usize,
) -> Result<(Vec<f64>, Vec<f64>), SolveError> {
    validate_matrix(a, m, n)?;
    let mut divisors = alloc::vec![1.0f64; n];
    for start in (0..n).step_by(3) {
        let end = (start + 3).min(n);
        let max = (0..m).fold(0.0f64, |v, row| {
            (start..end).fold(v, |v, col| v.max(a[row * n + col].abs()))
        });
        if max > 0.0 {
            divisors[start..end].fill(max);
        }
    }
    let mut scaled = a.to_vec();
    for row in 0..m {
        for col in 0..n {
            scaled[row * n + col] /= divisors[col];
        }
    }
    Ok((scaled, divisors))
}

/// 从行主序切片构造 m×n 矩阵。
fn from_row_major(a: &[f64], m: usize, n: usize) -> DynMat {
    let mut mat = DynMat::zeros(m, n);
    for i in 0..m {
        for j in 0..n {
            mat[(i, j)] = a[i * n + j];
        }
    }
    mat
}

/// 奇异值（降序）。空矩阵返回空。
pub fn singular_values(a: &[f64], m: usize, n: usize) -> Result<Vec<f64>, SolveError> {
    validate_matrix(a, m, n)?;
    if m == 0 || n == 0 {
        return Ok(Vec::new());
    }
    let sv = bounded_svd(from_row_major(a, m, n), false, false, SVD_ITERATION_BUDGET)?;
    let mut out = sv.singular_values.as_slice().to_vec();
    out.sort_by(|x, y| y.total_cmp(x));
    Ok(out)
}

/// SVD 阻尼伪逆的 LM 步：`d = −Σ_j v_j · σ_j·(u_jᵀr)/(σ_j² + λ)`，
/// 仅 σ_j > σ_max·1e-10 的方向参与（零奇异方向截断，秩亏稳健）。
///
/// 装配约束的残差行天然线性相关（mate 秩 3/4、coaxial 秩 4/6），正规方程
/// 在 λ 小时病态（条件数 ~1/σ² → 步长放大 → 拒绝 → λ 震荡爬行）；SVD
/// 形式对秩亏二次收敛。λ 仅阻尼非零方向，0-奇异方向完全不产生步长。
pub fn svd_lm_step(
    a: &[f64],
    r: &[f64],
    m: usize,
    n: usize,
    lambda: f64,
) -> Result<Vec<f64>, SolveError> {
    validate_matrix(a, m, n)?;
    if r.len() != m || !r.iter().all(|v| v.is_finite()) || !lambda.is_finite() || lambda < 0.0 {
        return Err(numeric_error(
            "LM 的残差长度必须匹配矩阵，残差和非负阻尼必须全部有限。",
        ));
    }
    if m == 0 || n == 0 {
        return Ok(alloc::vec![0.0; n]);
    }
    let mat = from_row_major(a, m, n);
    let sv = bounded_svd(mat.clone(), false, true, SVD_ITERATION_BUDGET)?;
    let smax = sv.singular_values.get(0).copied().unwrap_or(0.0);
    let mut d = alloc::vec![0.0; n];
    if smax <= 0.0 {
        return Ok(d); // 零雅可比：无步可走
    }
    let tau_cut = smax * 1e-10;
    let Some(v_t) = sv.v_t else {
        return Err(numeric_error("SVD 未返回 LM 步所需的右奇异向量。"));
    };
    let rv = DVector::from_row_slice(r);
    for j in 0..n.min(v_t.nrows()) {
        let sigma = sv.singular_values.get(j).copied().unwrap_or(0.0);
        if sigma <= tau_cut {
            continue;
        }
        // v_j = v_t 的第 j 行（转置）；u_j = (J·v_j)/σ_j，u_jᵀr = (J·v_j)ᵀr/σ_j
        let vj = v_t.row(j).transpose();
        let utr = (&mat * &vj).dot(&rv) / sigma;
        let coef = sigma * utr / (sigma * sigma + lambda);
        for i in 0..n {
            d[i] -= vj[(i, 0)] * coef;
        }
    }
    if !d.iter().all(|v| v.is_finite()) {
        return Err(numeric_error("LM 步超出有限数值范围；请缩放模型。"));
    }
    Ok(d)
}

/// 残差在 J 的左零空间（range(J) 的正交补）上的投影 `r − U_r·U_rᵀ·r`，
/// U_r 取 σ > σ_max·1e-10 的左奇异向量。
///
/// 这一部分残差无法通过一阶参数调整减小。非线性驻点处非零并不
/// 证明全局无解；此投影只能作为局部数值信息。
pub fn left_null_projection(
    a: &[f64],
    r: &[f64],
    m: usize,
    n: usize,
) -> Result<Vec<f64>, SolveError> {
    validate_matrix(a, m, n)?;
    if r.len() != m || !r.iter().all(|v| v.is_finite()) {
        return Err(numeric_error("投影残差必须长度匹配矩阵且全部有限。"));
    }
    let mut out = r.to_vec();
    if m == 0 || n == 0 {
        return Ok(out);
    }
    let sv = bounded_svd(from_row_major(a, m, n), true, false, SVD_ITERATION_BUDGET)?;
    let smax = sv.singular_values.get(0).copied().unwrap_or(0.0);
    let Some(u) = sv.u else {
        return Err(numeric_error("SVD 未返回投影所需的左奇异向量。"));
    };
    if smax <= 0.0 {
        return Ok(out);
    }
    let tau_cut = smax * 1e-10;
    let rv = DVector::from_row_slice(r);
    for j in 0..u.ncols() {
        let sigma = sv.singular_values.get(j).copied().unwrap_or(0.0);
        if sigma <= tau_cut {
            continue;
        }
        let uj = u.column(j);
        let coef = uj.dot(&rv);
        for i in 0..m {
            out[i] -= uj[i] * coef;
        }
    }
    if !out.iter().all(|v| v.is_finite()) {
        return Err(numeric_error("残差投影超出有限数值范围；请缩放模型。"));
    }
    Ok(out)
}

/// 由奇异值序列计算秩：σ_i > σ_max · 1e-10 视为非零（相对阈值）。
///
/// 阈值选型：经典 LAPACK 式 `σ_max·max(m,n)·EPS` 约在 1e-14·σ_max 量级，
/// 贴着 SVD 的数值噪声地板（实测 12×12 链式装配秩多报 1）。1e-10 的相对
/// 阈值对装配雅可比（真实非零 σ ≫ 1e-10·σ_max）既稳健又不失分辨力。
pub fn rank_from_sv(sv: &[f64], _m: usize, _n: usize) -> usize {
    let Some(&smax) = sv.first() else { return 0 };
    if smax <= 0.0 {
        return 0;
    }
    let tau = smax * 1e-10;
    sv.iter().filter(|&&s| s > tau).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exhausted_svd_budget_is_an_explicit_error() {
        let matrix = from_row_major(&[1.0, 2.0, 3.0, 4.0, 6.0, 5.0, 8.0, 7.0, 9.0], 3, 3);
        let error = bounded_svd(matrix, true, true, 1).expect_err("one iteration is insufficient");
        assert!(
            matches!(error, SolveError::InvalidModel { ref reason } if reason.contains("预算"))
        );
    }

    #[test]
    fn zero_budget_cannot_enable_unbounded_svd() {
        let error = bounded_svd(DynMat::identity(2, 2), true, true, 0).unwrap_err();
        assert!(matches!(error, SolveError::InvalidModel { .. }));
    }

    #[test]
    fn public_numeric_methods_reject_invalid_shapes_and_nonfinite_data() {
        for (data, m, n) in [
            (&[1.0][..], 2, 2),
            (&[][..], usize::MAX, 2),
            (&[][..], 0, usize::MAX),
            (&[f64::NAN][..], 1, 1),
            (&[f64::INFINITY][..], 1, 1),
        ] {
            assert!(singular_values(data, m, n).is_err());
            assert!(equilibrate_columns(data, m, n).is_err());
            assert!(svd_lm_step(data, &[], m, n, 1e-3).is_err());
            assert!(left_null_projection(data, &[], m, n).is_err());
        }
        assert!(svd_lm_step(&[1.0], &[], 1, 1, 1e-3).is_err());
        assert!(svd_lm_step(&[1.0], &[f64::NAN], 1, 1, 1e-3).is_err());
        assert!(svd_lm_step(&[1.0], &[1.0], 1, 1, -1.0).is_err());
        assert!(left_null_projection(&[1.0], &[], 1, 1).is_err());
        assert!(left_null_projection(&[1.0], &[f64::INFINITY], 1, 1).is_err());
    }

    #[test]
    fn bounded_svd_preserves_valid_rank_and_lm_step() {
        let matrix = [2.0, 0.0, 0.0, 3.0];
        let sv = singular_values(&matrix, 2, 2).unwrap();
        assert_eq!(sv, [3.0, 2.0]);
        assert_eq!(rank_from_sv(&sv, 2, 2), 2);
        let step = svd_lm_step(&matrix, &[4.0, 9.0], 2, 2, 0.0).unwrap();
        assert!((step[0] + 2.0).abs() < 1e-14);
        assert!((step[1] + 3.0).abs() < 1e-14);
        let projection = left_null_projection(&matrix, &[4.0, 9.0], 2, 2).unwrap();
        assert!(projection.iter().all(|v| v.abs() < 1e-14));
    }
}
