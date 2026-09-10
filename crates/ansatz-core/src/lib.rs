//! # ansatz-core
//!
//! Ansatz 几何约束求解器核心。定位是**基础设施而不是工具**：一个 Rust 核心，
//! 多个薄壳（native / wasm / node / cli）复用同一套算法与诊断。
//!
//! ## 设计约束（对本 crate 的硬性要求）
//!
//! - **`no_std` 友好**：不依赖任何宿主相关 crate，不出现
//!   `#[cfg(target_arch = "wasm32")]` 之类的平台分叉——平台差异全部由外壳吸收。
//! - **浮点一律 `f64`**，严格遵守 IEEE 754 语义；不做任何等价于
//!   `-ffast-math` 的假设，不手写 FMA 融合。这保证同一输入在
//!   x86-64 / aarch64 / wasm32 上逐位一致（见根 Cargo.toml 的 profile 注释
//!   与 `tests/parity` 位模式测试框架）。
//! - **结构化诊断是一等公民**：求解结果不只回答「解出来了没有」，
//!   还要回答「为什么没解出来」——自由度计数、冗余/冲突约束组、每条约束的
//!   残差、机器可读的修复建议，且每条诊断都带同时面向人类与 LLM 的
//!   `human_message`。
//!
//! ## 当前阶段（骨架）
//!
//! [`solver::solve`] 只处理一个平凡算例：**2D 点 + 到原点的距离约束**，
//! 使用闭式解 `p' = p · (d / ‖p‖)`，不包含通用 Newton 迭代。它的职责是让
//! 端到端管线（cli / ffi / wasm / node / schema / CI）全部以真实形态跑通；
//! 通用数值求解属于后续阶段，届时只替换 `solver` 模块。

#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

pub mod model;
pub mod report;
pub mod solver;

pub use model::{
    Constraint, ConstraintKind, Entity, EntityId, Geometry, Model, Point2, Pose3, Rotation3,
    SolveOutcome, SolveParams, Vec3,
};
pub use report::{
    Action, ConstraintResidual, Diagnostics, Envelope, ErrorPayload, RedundancyGroup, SolveError,
    SolveReport, Suggestion,
};
pub use solver::solve;

/// 核心版本号，供各外壳导出给宿主做兼容性判断。
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
