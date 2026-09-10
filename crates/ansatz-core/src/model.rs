//! 求解器输入的数据模型：实体、约束、求解参数。
//!
//! 所有类型在 `serde` feature 下提供双向序列化；JSON 形状即
//! `schema/model.schema.json` 契约，枚举一律 `{"type": "snake_case", ...}` 内部标签。

use alloc::string::String;
use alloc::vec::Vec;

/// 实体的稳定 id，模型内唯一，约束通过它引用实体。
pub type EntityId = u32;

/// 2D 点。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

/// 3D 向量（平移、方向等）。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 3D 旋转的参数化：**指数映射（旋转向量）** `v = θ·û`。
///
/// # 为什么选指数映射而不是四元数
///
/// - 四元数是 4 参数表示 3 个自由度，求解时必须额外引入单位范数约束，
///   等于给系统加了一个非线性方程；且其雅可比存在冗余列，牛顿类求解器
///   的法方程矩阵 `JᵀJ` 必然奇异，需要额外的流形投影/正交化处理。
/// - 指数映射是**最小参数化**（3 参数 ↔ 3 自由度）：无冗余列、无额外约束，
///   `θ → 0` 时 `v → 0` 连续，没有四元数的 ±q 双重覆盖问题，也没有
///   欧拉角的万向锁。
///
/// # 切换抽象点
///
/// 本类型是旋转参数化的唯一入口：求解器只经由 `Rotation3`（及其未来的
/// 切空间/雅可比接口）访问旋转，不直接触碰内部表示。若后续需要换成
/// 四元数 + 流形投影等表示，只需替换本结构体与对应的导数实现，
/// 数据模型其余部分不动。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Rotation3 {
    /// 旋转向量：方向为旋转轴（单位化后），模长为旋转角（弧度）。
    pub vector: [f64; 3],
}

/// 3D 刚体位姿 = 平移 + 旋转（指数映射）。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Pose3 {
    pub translation: Vec3,
    pub rotation: Rotation3,
}

/// 几何实体。每个变体自带全部几何参数（骨架阶段不做参数共享/引用式建模）。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)
)]
pub enum Geometry {
    /// 平面上的点（2 DOF）。
    Point2 { x: f64, y: f64 },
    /// 线段，由两端点表示（4 DOF）。
    Line2 { start: Point2, end: Point2 },
    /// 圆（3 DOF：圆心 2 + 半径 1）。
    Circle2 { center: Point2, radius: f64 },
    /// 圆弧（5 DOF：圆 3 + 起止角 2），角度为弧度。
    Arc2 {
        center: Point2,
        radius: f64,
        start_angle: f64,
        end_angle: f64,
    },
    /// 3D 刚体（6 DOF），位姿用 [`Pose3`]（平移 + 指数映射旋转）。
    Rigid3 { pose: Pose3 },
}

impl Geometry {
    /// 实体的自由度（连续参数个数）。
    ///
    /// Point2=2；Line2 按两端点表示=4（若将来换成「点 + 角度」表示则为 3，
    /// 届时只需修改这里与求解器）；Circle2=3；Arc2=5；Rigid3=6。
    pub fn dof(&self) -> u32 {
        match self {
            Geometry::Point2 { .. } => 2,
            Geometry::Line2 { .. } => 4,
            Geometry::Circle2 { .. } => 3,
            Geometry::Arc2 { .. } => 5,
            Geometry::Rigid3 { .. } => 6,
        }
    }

    /// 稳定的种类名，与 serde 序列化的 `type` 标签一致，用于诊断消息。
    pub fn kind_name(&self) -> &'static str {
        match self {
            Geometry::Point2 { .. } => "point2",
            Geometry::Line2 { .. } => "line2",
            Geometry::Circle2 { .. } => "circle2",
            Geometry::Arc2 { .. } => "arc2",
            Geometry::Rigid3 { .. } => "rigid3",
        }
    }
}

/// 模型中的一个实体：稳定 id + 几何数据。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Entity {
    pub id: EntityId,
    pub geometry: Geometry,
}

/// 约束种类与参数。
///
/// 2D 草图约束与 3D 装配约束共用本枚举。骨架阶段的求解器只支持
/// [`ConstraintKind::Distance`] 的 `b = None`（到原点）形式，其余类型
/// 会被 [`crate::solve`](crate::solver::solve) 以
/// [`SolveError::UnsupportedConstraint`](crate::report::SolveError::UnsupportedConstraint)
/// 拒绝——这是「能力未实现」的工具级错误，不是求解失败。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)
)]
pub enum ConstraintKind {
    /// 两实体重合（点-点，或点落在曲线/曲面上）。
    Coincident { a: EntityId, b: EntityId },
    /// 三点共线（c 落在 a、b 确定的直线上）。
    Collinear {
        a: EntityId,
        b: EntityId,
        c: EntityId,
    },
    /// 两直线平行。
    Parallel { a: EntityId, b: EntityId },
    /// 两元素垂直。
    Perpendicular { a: EntityId, b: EntityId },
    /// 相切。
    Tangent { a: EntityId, b: EntityId },
    /// 距离。`b = None` 表示到坐标原点（**骨架阶段唯一受支持的算例**）。
    Distance {
        a: EntityId,
        b: Option<EntityId>,
        value: f64,
    },
    /// 角度（弧度）。`b = None` 表示相对参考系。
    Angle {
        a: EntityId,
        b: Option<EntityId>,
        value: f64,
    },
    /// a 与 b 关于 about（点/线/面）对称。
    Symmetric {
        a: EntityId,
        b: EntityId,
        about: EntityId,
    },
    /// 固定 a 的当前位形（不参与求解）。
    Fixed { a: EntityId },
    /// 3D 装配：两面贴合（法向相反、面上重合，6-DOF 刚体配合）。
    Mate { a: EntityId, b: EntityId },
    /// 3D 装配：同轴。
    Coaxial { a: EntityId, b: EntityId },
}

impl ConstraintKind {
    /// 稳定的种类名，与 serde 序列化的 `type` 标签一致，用于诊断消息。
    pub fn kind_name(&self) -> &'static str {
        match self {
            ConstraintKind::Coincident { .. } => "coincident",
            ConstraintKind::Collinear { .. } => "collinear",
            ConstraintKind::Parallel { .. } => "parallel",
            ConstraintKind::Perpendicular { .. } => "perpendicular",
            ConstraintKind::Tangent { .. } => "tangent",
            ConstraintKind::Distance { .. } => "distance",
            ConstraintKind::Angle { .. } => "angle",
            ConstraintKind::Symmetric { .. } => "symmetric",
            ConstraintKind::Fixed { .. } => "fixed",
            ConstraintKind::Mate { .. } => "mate",
            ConstraintKind::Coaxial { .. } => "coaxial",
        }
    }
}

/// 模型中的一条约束：稳定 id（模型内唯一）+ 种类参数 + 可选人类标签。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Constraint {
    pub id: u32,
    pub kind: ConstraintKind,
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    pub label: Option<String>,
}

fn default_tolerance() -> f64 {
    1e-9
}

fn default_max_iterations() -> u32 {
    100
}

fn default_allow_underconstrained() -> bool {
    true
}

/// 求解参数。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct SolveParams {
    /// 绝对容差：同时用作残差收敛判据与「距离值是否一致」的冲突判据。
    #[cfg_attr(feature = "serde", serde(default = "default_tolerance"))]
    pub tolerance: f64,
    /// 最大迭代数（骨架阶段闭式解不迭代，字段保留为契约的一部分）。
    #[cfg_attr(feature = "serde", serde(default = "default_max_iterations"))]
    pub max_iterations: u32,
    /// 是否接受欠约束解：为 `true` 时报告 `Underconstrained` 并附特解；
    /// 为 `false` 时同样报告 `Underconstrained`，但实体原样返回输入。
    #[cfg_attr(feature = "serde", serde(default = "default_allow_underconstrained"))]
    pub allow_underconstrained: bool,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            max_iterations: default_max_iterations(),
            allow_underconstrained: default_allow_underconstrained(),
        }
    }
}

/// 求解器输入：实体 + 约束 + 参数。
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Model {
    pub entities: Vec<Entity>,
    pub constraints: Vec<Constraint>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub params: SolveParams,
}

impl Model {
    /// 按 id 查找实体。
    pub fn entity(&self, id: EntityId) -> Option<&Entity> {
        self.entities.iter().find(|e| e.id == id)
    }
}

/// 求解结局。这是**求解层**的结果（区别于工具层错误 [`crate::SolveError`]）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(rename_all = "snake_case")
)]
pub enum SolveOutcome {
    /// 在容差内满足全部约束。
    Converged,
    /// 约束一致但自由度未耗尽（附特解或原样返回，取决于参数）。
    Underconstrained,
    /// 约束一致但存在冗余（线性相关），解集仍可能非空。
    Overconstrained,
    /// 约束互相冲突，无解（附冲突组与修复建议）。
    Inconsistent,
    /// 达到最大迭代数仍未收敛（骨架阶段不会出现，保留为契约的一部分）。
    MaxIterations,
}

impl SolveOutcome {
    /// 稳定字符串名，与 serde 序列化一致。
    pub fn as_str(&self) -> &'static str {
        match self {
            SolveOutcome::Converged => "converged",
            SolveOutcome::Underconstrained => "underconstrained",
            SolveOutcome::Overconstrained => "overconstrained",
            SolveOutcome::Inconsistent => "inconsistent",
            SolveOutcome::MaxIterations => "max_iterations",
        }
    }
}
