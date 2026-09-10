//! 手写稠密线性代数：Cholesky 求解（LM 步）与单边 Jacobi 奇异值（秩分析）。
//!
//! 为什么不用 nalgebra/ndarray：core 的依赖面必须保持「serde + libm」，
//! 且**手写实现 = 所有目标上编译同一份代码**——位级一致由构造保证，
//! 没有平台 BLAS 的方差。规模定位是装配体（参数 ≤ 数百），稠密足够。
//!
//! 全部函数确定性：固定扫描顺序、固定阈值，无数据依赖的分支以外的自由度。

use alloc::vec::Vec;

/// 求解 (A + λ·diag(A))·x = rhs，A 为对称正定（Marquardt 阻尼形式）。
/// 失败（非正定）返回 None，由调用方增大 λ 重试。
pub fn solve_damped(a: &[f64], lambda: f64, rhs: &[f64], n: usize) -> Option<Vec<f64>> {
    // 组装 A + λ·diag(A)（diag 取 max(a_ii, 1e-12) 防零）
    let mut m = a.to_vec();
    for i in 0..n {
        let d = if m[i * n + i] > 1e-12 {
            m[i * n + i]
        } else {
            1e-12
        };
        m[i * n + i] += lambda * d;
    }
    cholesky_solve(&m, rhs, n)
}

/// Cholesky 分解求解对称正定线性方程组（行主序 n×n）。非正定返回 None。
pub fn cholesky_solve(a: &[f64], rhs: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = alloc::vec![0.0; n * n];
    // 分解 A = L·Lᵀ
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i * n + j];
            for k in 0..j {
                sum -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if sum <= 0.0 {
                    return None;
                }
                l[i * n + i] = libm::sqrt(sum);
            } else {
                l[i * n + j] = sum / l[j * n + j];
            }
        }
    }
    // 前代 L·y = rhs
    let mut y = alloc::vec![0.0; n];
    for i in 0..n {
        let mut sum = rhs[i];
        for k in 0..i {
            sum -= l[i * n + k] * y[k];
        }
        y[i] = sum / l[i * n + i];
    }
    // 回代 Lᵀ·x = y
    let mut x = alloc::vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = y[i];
        for k in (i + 1)..n {
            sum -= l[k * n + i] * x[k];
        }
        x[i] = sum / l[i * n + i];
    }
    Some(x)
}

/// 单边 Jacobi 求奇异值：对 m×n 矩阵（行主序，m ≥ 1）的列做正交化扫描，
/// 收敛后第 j 列的范数即 σ_j。返回**降序**奇异值。
///
/// 固定 60 轮扫描上限 + 固定收敛阈值，跨平台确定性。
pub fn singular_values(a: &[f64], m: usize, n: usize) -> Vec<f64> {
    // 列主序工作副本（对列做旋转更直接）
    let mut cols = alloc::vec![0.0; m * n];
    for i in 0..m {
        for j in 0..n {
            cols[j * m + i] = a[i * n + j];
        }
    }
    let mut off = 1.0;
    let mut sweep = 0;
    while off > 1e-15 && sweep < 60 {
        off = 0.0;
        sweep += 1;
        for p in 0..n {
            for q in (p + 1)..n {
                let cp = &cols[p * m..(p + 1) * m];
                let cq = &cols[q * m..(q + 1) * m];
                let mut app = 0.0;
                let mut aqq = 0.0;
                let mut apq = 0.0;
                for i in 0..m {
                    app += cp[i] * cp[i];
                    aqq += cq[i] * cq[i];
                    apq += cp[i] * cq[i];
                }
                off += apq * apq;
                if apq.abs() <= 1e-30 * libm::sqrt(app * aqq).max(1e-300) {
                    continue;
                }
                // Jacobi 旋转角：最小化两列的内积
                let tau = (aqq - app) / (2.0 * apq);
                let t = if tau >= 0.0 {
                    1.0 / (tau + libm::sqrt(1.0 + tau * tau))
                } else {
                    1.0 / (tau - libm::sqrt(1.0 + tau * tau))
                };
                let c = 1.0 / libm::sqrt(1.0 + t * t);
                let s = c * t;
                for i in 0..m {
                    let (x, y) = (cols[p * m + i], cols[q * m + i]);
                    cols[p * m + i] = c * x - s * y;
                    cols[q * m + i] = s * x + c * y;
                }
            }
        }
    }
    let mut sigmas: Vec<f64> = (0..n)
        .map(|j| {
            let col = &cols[j * m..(j + 1) * m];
            let mut s2 = 0.0;
            for v in col {
                s2 += v * v;
            }
            libm::sqrt(s2)
        })
        .collect();
    // 降序（稳定排序，确定性）
    sigmas.sort_by(|a, b| b.partial_cmp(a).unwrap_or(core::cmp::Ordering::Equal));
    sigmas
}

/// 由奇异值序列计算秩：σ_i > σ_max · 1e-10 视为非零（相对阈值）。
///
/// 阈值选型：经典 LAPACK 式 `σ_max·max(m,n)·EPS` 约在 1e-14·σ_max 量级，
/// 而单边 Jacobi 对小奇异值的绝对误差可达 ~EPS·σ_max，二者贴边会导致
/// 本应为零的 σ 被计入（实测：12×12 链式装配秩多报 1）。1e-10 的相对
/// 阈值对装配雅可比（真实非零 σ ≫ 1e-10·σ_max）既稳健又不失分辨力。
pub fn rank_from_sv(sv: &[f64], _m: usize, _n: usize) -> usize {
    let Some(&smax) = sv.first() else { return 0 };
    if smax <= 0.0 {
        return 0;
    }
    let tau = smax * 1e-10;
    sv.iter().filter(|&&s| s > tau).count()
}

/// 单边 Jacobi SVD + 阻尼伪逆的 LM 步：
/// `d = −Σ_j v_j · σ_j·(u_jᵀr)/(σ_j² + λ)`，仅 σ_j > σ_max·1e-10 的方向参与。
///
/// 相比正规方程 (JᵀJ+λD) 的 Cholesky 解法，此形式对秩亏雅可比天然稳健：
/// 零奇异方向被截断而不是被 1/(≈0) 放大——装配约束（mate/coaxial 的
/// 残差行天然线性相关）恰恰是秩亏系统，正规方程在 λ 小时病态导致收敛爬行。
/// 旋转同时累积到 V（n×n，列 j = 右奇异向量），σ_j = 变换后第 j 列的范数。
pub fn svd_lm_step(a: &[f64], r: &[f64], m: usize, n: usize, lambda: f64) -> Vec<f64> {
    // 列主序工作副本 + V = I
    let mut cols = alloc::vec![0.0; m * n];
    for i in 0..m {
        for j in 0..n {
            cols[j * m + i] = a[i * n + j];
        }
    }
    let mut v = alloc::vec![0.0; n * n]; // 行主序 V，V[i][j] = v[i*n+j]
    for i in 0..n {
        v[i * n + i] = 1.0;
    }
    // 单边 Jacobi 扫描（与 singular_values 相同的确定性参数）
    let mut off = 1.0;
    let mut sweep = 0;
    while off > 1e-15 && sweep < 60 {
        off = 0.0;
        sweep += 1;
        for p in 0..n {
            for q in (p + 1)..n {
                let mut app = 0.0;
                let mut aqq = 0.0;
                let mut apq = 0.0;
                for i in 0..m {
                    let cp = cols[p * m + i];
                    let cq = cols[q * m + i];
                    app += cp * cp;
                    aqq += cq * cq;
                    apq += cp * cq;
                }
                off += apq * apq;
                if apq.abs() <= 1e-30 * libm::sqrt(app * aqq).max(1e-300) {
                    continue;
                }
                let tau = (aqq - app) / (2.0 * apq);
                let t = if tau >= 0.0 {
                    1.0 / (tau + libm::sqrt(1.0 + tau * tau))
                } else {
                    1.0 / (tau - libm::sqrt(1.0 + tau * tau))
                };
                let c = 1.0 / libm::sqrt(1.0 + t * t);
                let s = c * t;
                for i in 0..m {
                    let (x, y) = (cols[p * m + i], cols[q * m + i]);
                    cols[p * m + i] = c * x - s * y;
                    cols[q * m + i] = s * x + c * y;
                }
                for i in 0..n {
                    let (x, y) = (v[i * n + p], v[i * n + q]);
                    v[i * n + p] = c * x - s * y;
                    v[i * n + q] = s * x + c * y;
                }
            }
        }
    }
    // σ_j 与阻尼伪逆步
    let sigmas: Vec<f64> = (0..n)
        .map(|j| {
            let col = &cols[j * m..(j + 1) * m];
            let mut s2 = 0.0;
            for val in col {
                s2 += val * val;
            }
            libm::sqrt(s2)
        })
        .collect();
    let smax = sigmas.iter().cloned().fold(0.0f64, f64::max);
    let tau_cut = smax * 1e-10;
    let mut d = alloc::vec![0.0; n];
    if smax <= 0.0 {
        return d; // 零雅可比：无步可走
    }
    for j in 0..n {
        let sigma = sigmas[j];
        if sigma <= tau_cut {
            continue;
        }
        // u_jᵀr = (J·v_j)ᵀr / σ_j = col_jᵀ·r / σ_j
        let col = &cols[j * m..(j + 1) * m];
        let mut utr = 0.0;
        for i in 0..m {
            utr += col[i] * r[i];
        }
        utr /= sigma;
        // d -= v_j · σ_j·utr/(σ_j²+λ)
        let coef = sigma * utr / (sigma * sigma + lambda);
        for i in 0..n {
            d[i] -= v[i * n + j] * coef;
        }
    }
    d
}
