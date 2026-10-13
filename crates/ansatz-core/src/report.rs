//! 求解报告与结构化诊断——项目差异化的核心。
//!
//! 设计原则：不只回答「解出来了没有」，还要回答「为什么没解出来」。
//! 每条诊断（冗余组、残差、建议）都带 `human_message`，措辞同时面向
//! 人类与 LLM：陈述事实（哪些约束、差多少）、给出可执行的下一步。

use crate::model::{Entity, EntityId, SolveOutcome};
use alloc::string::{String, ToString};
use alloc::vec::Vec;

/// 一组相互牵连的约束。出现在两个位置，含义不同：
///
/// - `Diagnostics::conflicts`：组内约束不可能同时满足；
/// - `Diagnostics::redundant_constraints`：组内约束相互一致但线性相关，
///   在当前位形的线性化中不增加秩；删除后须重新验证几何行为。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct ConstraintGroup {
    /// 组内约束 id（按模型中的顺序）。
    pub constraint_ids: Vec<u32>,
    pub human_message: String,
}

/// 单条约束在返回解（或特解）处的残差。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct ConstraintResidual {
    pub constraint_id: u32,
    /// 残差值（约束方程左端减右端；距离约束即 `‖p‖ − d`）。
    pub residual: f64,
    pub human_message: String,
}

/// 机器可读的修复建议动作。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)
)]
pub enum Action {
    /// 删除指定约束（冗余，或冲突组中的多余一方）。
    RemoveConstraint { id: u32 },
    /// 放宽指定约束（调整参数值使其与其余约束相容）。
    RelaxConstraint { id: u32 },
    /// 补充约束以消除 `dof_remaining` 个剩余自由度。
    AddConstraint { dof_remaining: u32 },
}

/// 一条诊断建议：机器可读动作 + 人类/LLM 可读解释。
///
/// JSON 形状（flatten 拍平，便于人与 LLM 直接阅读）：
/// `{"action":"remove_constraint","id":5,"human_message":"…"}`。
/// 注意 serde 的 flatten 与 deny_unknown_fields 不兼容，故本类型不加该属性。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Suggestion {
    #[cfg_attr(feature = "serde", serde(flatten))]
    pub action: Action,
    pub human_message: String,
}

/// 关节变量的当前值（按关节约束逐个报告，机器人学的一等公民）。
///
/// revolute/cylindrical：绕轴转角 θ（弧度，主值 (−π, π]，零位由
/// `a_ref`/`b_ref` 定义）；prismatic：沿轴位移 δ（长度）。两者都只依赖
/// 返回的位姿，`drive` 收敛后该值即驱动目标。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct JointState {
    /// 对应的关节约束 id。
    pub constraint_id: u32,
    /// "revolute" | "cylindrical" | "prismatic"
    pub joint_kind: String,
    /// 当前关节变量（弧度或长度），只由返回的位姿决定。
    pub value: f64,
    pub human_message: String,
}

/// 结构化诊断。
#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct Diagnostics {
    /// 模型总自由度（各实体参数数之和）。
    pub dof_total: u32,
    /// 返回位形处的剩余自由度：自由参数数 − rank(J)。
    pub dof_remaining: u32,
    /// 实际执行的迭代数（2D 闭式路径恒为 0）。
    pub iterations: u32,
    /// 冲突组：组内约束不可能同时满足。非空 ⇔ 结局为 `Inconsistent`。
    pub conflicts: Vec<ConstraintGroup>,
    /// 冗余组：组内约束一致但线性相关（与约束书写顺序无关）。
    pub redundant_constraints: Vec<ConstraintGroup>,
    /// 每条约束在返回解处的残差。
    pub residuals: Vec<ConstraintResidual>,
    /// 残差绝对值最大的那条约束。
    pub max_residual: Option<ConstraintResidual>,
    /// 机器可读的修复建议。
    pub suggestions: Vec<Suggestion>,
    /// 关节变量状态（revolute/cylindrical/prismatic 各报一条）。
    pub joints: Vec<JointState>,
    /// 雅可比条件数估计：`κ(J) = σ_max / σ_min(非零)`。
    /// 3D 取自局部旋转切空间、平移/旋转三元组尺度均衡后的终态雅可比；
    /// 这是均衡矩阵的非零谱条件数，不能代表原始物理单位下的条件数。
    /// 2D 闭式路径对单条约束精确为 `1.0`，
    /// 行退化（原点处梯度为零）或重复约束时为 `None`。
    pub jacobian_condition_estimate: Option<f64>,
}

/// 求解报告：结局 + 求解后的实体 + 诊断。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct SolveReport {
    pub outcome: SolveOutcome,
    /// 求解后的实体。欠约束时是解集中的一个特解。
    pub entities: Vec<Entity>,
    pub diagnostics: Diagnostics,
}

/// 工具级错误：模型本身有问题或能力未实现，属于「工具错误」而非「求解失败」。
/// 外壳把它映射到各自错误通道（CLI stderr + exit 2 / FFI 错误信封 / wasm-node 错误信封）。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)
)]
pub enum SolveError {
    /// 约束引用了不存在的实体。
    UnknownEntity {
        constraint_id: u32,
        entity_id: EntityId,
    },
    /// 约束类型在当前阶段不受支持。
    UnsupportedConstraint {
        constraint_id: u32,
        constraint_kind: String,
        reason: String,
    },
    /// 模型结构无效（如 id 重复）。
    InvalidModel { reason: String },
}

impl SolveError {
    /// 稳定错误名，与 serde 序列化的 `kind` 标签一致。
    pub fn kind_name(&self) -> &'static str {
        match self {
            SolveError::UnknownEntity { .. } => "unknown_entity",
            SolveError::UnsupportedConstraint { .. } => "unsupported_constraint",
            SolveError::InvalidModel { .. } => "invalid_model",
        }
    }

    /// 面向人类与 LLM 的错误说明。
    pub fn human_message(&self) -> String {
        match self {
            SolveError::UnknownEntity {
                constraint_id,
                entity_id,
            } => {
                alloc::format!(
                    "约束 {constraint_id} 引用了不存在的实体 {entity_id}，请检查实体 id。"
                )
            }
            SolveError::UnsupportedConstraint {
                constraint_id,
                constraint_kind,
                reason,
            } => alloc::format!(
                "约束 {constraint_id}（类型 `{constraint_kind}`）在当前阶段不受支持：{reason}"
            ),
            SolveError::InvalidModel { reason } => alloc::format!("模型无效：{reason}"),
        }
    }
}

impl core::fmt::Display for SolveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.human_message())
    }
}

/// 外壳错误信封的负载：`{ kind, message }`。
/// kind 取值：`unknown_entity` / `unsupported_constraint` / `invalid_model`
/// （来自 [`SolveError`]，输入已按 schema 解析成功但语义无效），以及外壳自产的
/// `invalid_json`（输入无法按模型 schema 解析，字符串与结构化 API 一致）/
/// `invalid_utf8` / `invalid_argument` / `panic` 等。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(deny_unknown_fields)
)]
pub struct ErrorPayload {
    pub kind: String,
    pub message: String,
}

impl ErrorPayload {
    /// 由核心错误构造（外壳做错误映射时使用）。
    pub fn from_solve_error(err: &SolveError) -> Self {
        Self {
            kind: err.kind_name().to_string(),
            message: err.human_message(),
        }
    }
}

/// ffi / wasm / node 外壳统一的返回信封：
/// 成功 → `{"ok":true,"result":<SolveReport>}`，
/// 失败 → `{"ok":false,"error":{"kind":...,"message":...}}`。
///
/// 定义在 core 是为了保证三个壳的 JSON 形状永远一致（它们之间只做类型转换，
/// 不各造信封）；CLI 不用信封（stdout 报告 / stderr 错误），见其文档。
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct Envelope {
    pub ok: bool,
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub result: Option<SolveReport>,
    #[cfg_attr(feature = "serde", serde(skip_serializing_if = "Option::is_none"))]
    pub error: Option<ErrorPayload>,
}

impl Envelope {
    /// 成功信封。
    pub fn ok(report: SolveReport) -> Self {
        Self {
            ok: true,
            result: Some(report),
            error: None,
        }
    }

    /// 错误信封。
    pub fn error(kind: &str, message: &str) -> Self {
        Self {
            ok: false,
            result: None,
            error: Some(ErrorPayload {
                kind: kind.to_string(),
                message: message.to_string(),
            }),
        }
    }

    /// 错误信封（由核心错误构造）。
    pub fn from_solve_error(err: &SolveError) -> Self {
        Self {
            ok: false,
            result: None,
            error: Some(ErrorPayload::from_solve_error(err)),
        }
    }
}
