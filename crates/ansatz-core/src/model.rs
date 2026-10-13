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
/// JSON 使用紧凑的三维旋转向量。它是姿态存储格式，并非全局无奇异
/// 的求解坐标：2π 等位置的全局参数雅可比会退化。
/// 求解内部采用三维 SO(3) 局部增量并复合旋转，DOF 分析也使用该切空间。
/// 四元数存储配合三维局部增量同样无需添加单位范数约束。
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

/// 3D 装配用的**体坐标系**平面：过 origin、法向 normal（不必单位化，求解时归一）。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Plane3 {
    pub origin: Vec3,
    pub normal: Vec3,
}

/// 3D 装配用的**体坐标系**轴：过 origin、方向 direction（不必单位化，求解时归一）。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Axis3 {
    pub origin: Vec3,
    pub direction: Vec3,
}

/// 几何实体。每个变体自带全部几何参数（不做参数共享/引用式建模）。
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
/// 2D 草图约束与 3D 装配约束共用本枚举。2D 路径目前只支持
/// [`ConstraintKind::Distance`] 的 `b = None`（到原点）形式；3D 路径支持
/// distance / angle / fixed / mate / coaxial / 关节族 / transmission。
/// 其余组合会被 [`crate::solve`](crate::solver::solve) 以
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
    /// 距离。2D：`b = None` 表示到坐标原点。3D 装配：a、b 为两个 Rigid3，
    /// 距离在**锚点**之间测量（体坐标系；缺省为各自原点）。
    Distance {
        a: EntityId,
        b: Option<EntityId>,
        value: f64,
        /// a 体上的锚点（体坐标系）；缺省 (0,0,0)。
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        a_anchor: Option<Vec3>,
        /// b 体上的锚点（体坐标系）；缺省 (0,0,0)。
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        b_anchor: Option<Vec3>,
    },
    /// 角度（弧度）。2D：`b = None` 表示相对参考系（未实现）。
    /// 3D 装配：a、b 为两个 Rigid3，`a_dir`/`b_dir` 为非零参考方向
    /// （求解时归一化）。容差按真实夹角误差（弧度）检查；0/π 的方向对齐
    /// 消去两个转动自由度，其他角度通常消去一个。
    Angle {
        a: EntityId,
        b: Option<EntityId>,
        value: f64,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        a_dir: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        b_dir: Option<Vec3>,
    },
    /// a 与 b 关于 about（点/线/面）对称。
    Symmetric {
        a: EntityId,
        b: EntityId,
        about: EntityId,
    },
    /// 固定 a 的当前位形（不参与求解，自由度被结构性消去）。
    Fixed { a: EntityId },
    /// 3D 装配：两面贴合——两面在世界系重合且法向相反（消 3 个自由度：
    /// 法向平移 1 + 法向转动 2）。平面几何在各自**体坐标系**内给出。
    Mate {
        a: EntityId,
        b: EntityId,
        /// a 体上的贴合面（体坐标系）。
        a_plane: Plane3,
        /// b 体上的贴合面（体坐标系）。
        b_plane: Plane3,
    },
    /// 3D 装配：同轴——两轴共线且同向（消 4 个自由度：横向平移 2 + 偏轴转动 2；
    /// 沿轴滑动与绕轴转动仍自由）。轴几何在各自**体坐标系**内给出。
    Coaxial {
        a: EntityId,
        b: EntityId,
        /// a 体上的轴（体坐标系）。
        a_axis: Axis3,
        /// b 体上的轴（体坐标系）。
        b_axis: Axis3,
    },
    /// 3D 装配：**转动副**（revolute）——同轴 + 轴向锚点间距锁定，
    /// 消 5 留 1（绕轴转动）。`offset` 为两轴原点沿轴的间距（缺省 0）。
    ///
    /// 关节角 θ 是**位姿的函数**：从 `R_a·a_ref` 绕 a 轴转到 `R_b·b_ref` 的
    /// 有向角，取主值 (−π, π]。`a_ref`/`b_ref` 是各自体坐标系内的零位参考
    /// 方向（会被投影到垂直于轴的平面再归一）；缺省时按轴方向确定性地选取。
    /// `drive` 是 θ 的绝对目标（弧度），因此把求解结果原样再解一次不会移动。
    Revolute {
        a: EntityId,
        b: EntityId,
        a_axis: Axis3,
        b_axis: Axis3,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        offset: Option<f64>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        a_ref: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        b_ref: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        drive: Option<f64>,
    },
    /// 3D 装配：**圆柱副**（cylindrical）——同轴，消 4 留 2
    /// （沿轴滑动 + 绕轴转动）。约束行与 coaxial 相同，诊断按关节报告 θ，
    /// θ 的定义与零位参考同 revolute。
    Cylindrical {
        a: EntityId,
        b: EntityId,
        a_axis: Axis3,
        b_axis: Axis3,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        a_ref: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        b_ref: Option<Vec3>,
    },
    /// 3D 装配：**移动副**（prismatic）——圆柱副 + 姿态滚转锁定，
    /// 消 5 留 1（沿轴平动）。滚转通过一对垂直于轴的参考方向锁定
    /// （`roll` 为绕 a 轴的有向夹角目标，取 [0, π]，弧度）。
    /// a_ref/b_ref/roll 必须一起给出；全部缺省则确定性选取参考方向并锁定 roll=0。
    /// 参考方向先投影到各自轴的法平面并归一化。
    /// 关节变量 δ = (b 轴原点 − a 轴原点)·(a 轴方向)，世界系下的有向距离；
    /// `drive` 是 δ 的绝对目标。
    Prismatic {
        a: EntityId,
        b: EntityId,
        a_axis: Axis3,
        b_axis: Axis3,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        a_ref: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        b_ref: Option<Vec3>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        roll: Option<f64>,
        #[cfg_attr(
            feature = "serde",
            serde(default, skip_serializing_if = "Option::is_none")
        )]
        drive: Option<f64>,
    },
    /// 3D 装配：**球副**（spherical）——两锚点重合，消 3 留 3（姿态自由）。
    Spherical {
        a: EntityId,
        b: EntityId,
        /// a 体上的球心锚点（体坐标系）。
        a_point: Vec3,
        /// b 体上的球心锚点（体坐标系）。
        b_point: Vec3,
    },
    /// 3D 装配：**传动耦合**（齿轮/带轮）——两个已存在的 revolute 关节的
    /// 关节变量按速比联动：`θ_b = ratio · θ_a`（θ 为 revolute 文档定义的
    /// 绝对关节角；残差折回 (−π, π]）。两个关节的零位由各自的
    /// `a_ref`/`b_ref` 决定。`joint_a`/`joint_b` 是两个不同 revolute 约束的 id。
    Transmission {
        joint_a: u32,
        joint_b: u32,
        /// 速比：θ_b 每随 θ_a 变化 1 弧度变化 ratio 弧度（外啮合为负）。
        ratio: f64,
    },
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
            ConstraintKind::Revolute { .. } => "revolute",
            ConstraintKind::Cylindrical { .. } => "cylindrical",
            ConstraintKind::Prismatic { .. } => "prismatic",
            ConstraintKind::Spherical { .. } => "spherical",
            ConstraintKind::Transmission { .. } => "transmission",
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

/// 求解参数。
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct SolveParams {
    /// 正有限绝对容差：长度按长度单位，角度按弧度检查。
    /// 同锚点距离仅在各自容差区间不相交时判为已证实冲突。
    #[cfg_attr(feature = "serde", serde(default = "default_tolerance"))]
    pub tolerance: f64,
    /// 3D 路径 LM 的最大迭代数（2D 闭式路径不迭代）。
    #[cfg_attr(feature = "serde", serde(default = "default_max_iterations"))]
    pub max_iterations: u32,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            max_iterations: default_max_iterations(),
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

/// 求解结局：只回答「返回的位形是否满足全部约束」。这是**求解层**的结果
/// （区别于工具层错误 [`crate::SolveError`]）。
///
/// 欠约束（`diagnostics.dof_remaining > 0`）与冗余
/// （`diagnostics.redundant_constraints` 非空）是与结局正交的事实，
/// 不折叠进本枚举：一个完美满足但仍可拖动的草图就是 `Converged`。
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
    /// 约束互相冲突：`diagnostics.conflicts` 给出冲突组与修复建议。
    Inconsistent,
    /// 未收敛，且未能证明冲突（迭代上限、退化初值或非线性停滞）。
    NotConverged,
}

impl SolveOutcome {
    /// 稳定字符串名，与 serde 序列化一致。
    pub fn as_str(&self) -> &'static str {
        match self {
            SolveOutcome::Converged => "converged",
            SolveOutcome::Inconsistent => "inconsistent",
            SolveOutcome::NotConverged => "not_converged",
        }
    }
}
