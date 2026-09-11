//! 线性代数薄层：SVD（秩分析）与 SVD 阻尼伪逆 LM 步。
//!
//! ## 为什么现在站在 nalgebra 上
//!
//! 分解本体用 [nalgebra](https://nalgebra.org)（纯 Rust、无 BLAS/平台分发、
//! no_std 经 libm）——位级可复现的前提它全部满足，且被 Rust 机器人生态
//! 充分验证。本模块保留的是**施加方式**：
//! - `svd_lm_step` 的阻尼伪逆公式与 λ 策略是求解器语义，不随库变；
//! - 秩阈值取相对 1e-10（选型理由见 `rank_from_sv`）；
//! - 残差/雅可比的组装顺序在 assembly 模块，固定不变。
//!
//! ## 为什么 LM 循环与 SO(3) 不引库
//!
//! 优化框架（argmin 等）会抢走收敛判据与迭代语义的控制权——那是本项目
//! 的对外契约（MaxIterations 的含义、tolerance 的作用位置）。SO(3) 的
//! exp/J_r 是 60 行参数化核心，也是 `Rotation3` 的切换缝，保留自写。

use alloc::vec::Vec;

use nalgebra::{DMatrix, DVector};

/// 动态矩阵别名。
type DynMat = DMatrix<f64>;

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
pub fn singular_values(a: &[f64], m: usize, n: usize) -> Vec<f64> {
    if m == 0 || n == 0 {
        return Vec::new();
    }
    let sv = from_row_major(a, m, n).svd(false, false);
    let mut out = sv.singular_values.as_slice().to_vec();
    out.sort_by(|x, y| y.partial_cmp(x).unwrap_or(core::cmp::Ordering::Equal));
    out
}

/// SVD 阻尼伪逆的 LM 步：`d = −Σ_j v_j · σ_j·(u_jᵀr)/(σ_j² + λ)`，
/// 仅 σ_j > σ_max·1e-10 的方向参与（零奇异方向截断，秩亏稳健）。
///
/// 装配约束的残差行天然线性相关（mate 秩 3/4、coaxial 秩 4/6），正规方程
/// 在 λ 小时病态（条件数 ~1/σ² → 步长放大 → 拒绝 → λ 震荡爬行）；SVD
/// 形式对秩亏二次收敛。λ 仅阻尼非零方向，0-奇异方向完全不产生步长。
pub fn svd_lm_step(a: &[f64], r: &[f64], m: usize, n: usize, lambda: f64) -> Vec<f64> {
    if m == 0 || n == 0 {
        return alloc::vec![0.0; n];
    }
    let mat = from_row_major(a, m, n);
    let sv = mat.clone().svd(false, true); // 只需 V（右奇异向量）
    let smax = sv.singular_values.get(0).copied().unwrap_or(0.0);
    let mut d = alloc::vec![0.0; n];
    if smax <= 0.0 {
        return d; // 零雅可比：无步可走
    }
    let tau_cut = smax * 1e-10;
    let Some(v_t) = sv.v_t else {
        return d;
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
    d
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
