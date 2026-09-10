//! SO(3) 指数映射数学：Rodrigues 旋转、右雅可比、hat 映射。
//!
//! 全部经 libm（sin/cos/sqrt），保证跨平台位级一致（见根 Cargo.toml 的
//! profile 注释与 tests/parity）。

/// 3×3 矩阵，行主序。
pub type Mat3 = [[f64; 3]; 3];

/// 反对称矩阵 hat(v)：hat(v)·x = v × x。
pub fn hat(v: [f64; 3]) -> Mat3 {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}

/// 矩阵-向量乘。
pub fn mat3_mul_vec(m: &Mat3, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// 矩阵-矩阵乘。
pub fn mat3_mul_mat(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

/// 矩阵转置。
pub fn mat3_transpose(m: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[j][i] = m[i][j];
        }
    }
    out
}

/// 指数映射的旋转矩阵（Rodrigues）：R(ω)，ω = θ·û。
///
/// R = I + (sinθ/θ) K + ((1−cosθ)/θ²) K²，K = hat(ω)。
/// θ < `SERIES_SWITCH` 时切换到泰勒级数（sinθ/θ → 1 − θ²/6 + θ⁴/120，
/// (1−cosθ)/θ² → 1/2 − θ²/24 + θ⁴/720），避免 0/0；阈值固定，结果确定。
pub fn rotation(omega: [f64; 3]) -> Mat3 {
    let theta2 = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
    let k = hat(omega);
    let (a, b) = if theta2 < SERIES_SWITCH * SERIES_SWITCH {
        // 小角度：级数展开（θ² 项）
        (1.0 - theta2 / 6.0, 0.5 - theta2 / 24.0)
    } else {
        let theta = libm::sqrt(theta2);
        (libm::sin(theta) / theta, (1.0 - libm::cos(theta)) / theta2)
    };
    let k2 = mat3_mul_mat(&k, &k);
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = (if i == j { 1.0 } else { 0.0 }) + a * k[i][j] + b * k2[i][j];
        }
    }
    r
}

/// SO(3) 的**右雅可比** J_r(ω)：满足 exp([ω+δ]×) ≈ exp([ω]×)·exp([J_r(ω)δ]×)。
///
/// J_r = I − ((1−cosθ)/θ²)·hat(ω) + ((θ−sinθ)/θ³)·hat(ω)²；
/// J_r(0) = I。小角度用级数（(1−cosθ)/θ² → 1/2−θ²/24，(θ−sinθ)/θ³ → 1/6−θ²/120）。
pub fn right_jacobian(omega: [f64; 3]) -> Mat3 {
    let theta2 = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
    let k = hat(omega);
    let k2 = mat3_mul_mat(&k, &k);
    let (a, b) = if theta2 < SERIES_SWITCH * SERIES_SWITCH {
        (0.5 - theta2 / 24.0, 1.0 / 6.0 - theta2 / 120.0)
    } else {
        let theta = libm::sqrt(theta2);
        (
            (1.0 - libm::cos(theta)) / theta2,
            (theta - libm::sin(theta)) / (theta2 * theta),
        )
    };
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for c in 0..3 {
            out[i][c] = (if i == c { 1.0 } else { 0.0 }) - a * k[i][c] + b * k2[i][c];
        }
    }
    out
}

/// 小角度级数切换阈值（平方前）；固定常量保证确定性。
const SERIES_SWITCH: f64 = 1e-4;

/// 三维点/向量便捷运算（避免引入外部线性代数 crate）。
pub mod v3 {
    /// 点积。
    pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
    /// 叉积。
    pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }
    /// a − b。
    pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }
    /// a + b。
    pub fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
    }
    /// 标量乘。
    pub fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
        [a[0] * s, a[1] * s, a[2] * s]
    }
    /// 欧几里得范数（libm::sqrt，确定性）。
    pub fn norm(a: [f64; 3]) -> f64 {
        libm::sqrt(dot(a, a))
    }
}
