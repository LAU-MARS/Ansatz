/**
 * Ansatz 结构化 API 的 TypeScript 类型（与 schema/*.schema.json 契约一一对应）。
 *
 * wasm-pack 生成的 ansatz_wasm.d.ts 对 JsValue 参数只能标 any；
 * 接入方在 tsconfig 中引用本文件即可获得完整类型：
 *
 *   import type { AnsatzModel, SolveEnvelope } from 'ansatz-wasm/types/ansatz';
 *   import init, { solve } from 'ansatz-wasm';
 *   await init();
 *   const env: SolveEnvelope = solve(model as AnsatzModel);
 */

export type EntityId = number;

export interface Point2 {
  x: number;
  y: number;
}

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/** JSON 旋转向量 v = θ·û（并非全局唯一）；求解更新使用 SO(3) 三维局部切空间。 */
export interface Rotation3 {
  vector: [number, number, number];
}

export interface Pose3 {
  translation: Vec3;
  rotation: Rotation3;
}

/** 体坐标系平面：过 origin、法向 normal（不必单位化）。 */
export interface Plane3 {
  origin: Vec3;
  normal: Vec3;
}

/** 体坐标系轴：过 origin、方向 direction（不必单位化）。 */
export interface Axis3 {
  origin: Vec3;
  direction: Vec3;
}

export type Geometry =
  | { type: 'point2'; x: number; y: number }
  | { type: 'line2'; start: Point2; end: Point2 }
  | { type: 'circle2'; center: Point2; radius: number }
  | {
      type: 'arc2';
      center: Point2;
      radius: number;
      start_angle: number;
      end_angle: number;
    }
  | { type: 'rigid3'; pose: Pose3 };

export interface Entity {
  id: EntityId;
  geometry: Geometry;
}

/** 数值输入必须有限；方向/法向必须非零，求解时归一化。 */
export type ConstraintKind =
  | { type: 'coincident'; a: EntityId; b: EntityId }
  | { type: 'collinear'; a: EntityId; b: EntityId; c: EntityId }
  | { type: 'parallel'; a: EntityId; b: EntityId }
  | { type: 'perpendicular'; a: EntityId; b: EntityId }
  | { type: 'tangent'; a: EntityId; b: EntityId }
  | {
      type: 'distance';
      a: EntityId;
      /** 2D：null 表示到原点；3D：必须是另一个 rigid3。 */
      b: EntityId | null;
      /** 零距离表示重合：2D 原点锁定消 2 DOF，3D 锚点重合消 3 DOF。负距离是冲突。 */
      value: number;
      a_anchor?: Vec3;
      b_anchor?: Vec3;
    }
  | {
      type: 'angle';
      a: EntityId;
      b: EntityId | null;
      /** 弧度，3D 下取 [0, π]；按真实弧度容差检查，精确 0/π 各消 2 个旋转 DOF。 */
      value: number;
      /** 3D 必须提供两方向；长度不改变夹角语义。 */
      a_dir?: Vec3;
      b_dir?: Vec3;
    }
  | { type: 'symmetric'; a: EntityId; b: EntityId; about: EntityId }
  | { type: 'fixed'; a: EntityId }
  | { type: 'mate'; a: EntityId; b: EntityId; a_plane: Plane3; b_plane: Plane3 }
  /** 同轴要求两轴共线且同向；反向轴不满足。 */
  | { type: 'coaxial'; a: EntityId; b: EntityId; a_axis: Axis3; b_axis: Axis3 }
  | {
      /**
       * 转动副。关节角 θ = 从 R_a·a_ref 绕 a 轴转到 R_b·b_ref 的有向角（主值）；
       * 只依赖位姿。drive 是 θ 的绝对目标（弧度），有限巨大值亦按 2π 有界折回。
       */
      type: 'revolute';
      a: EntityId;
      b: EntityId;
      a_axis: Axis3;
      b_axis: Axis3;
      offset?: number;
      a_ref?: Vec3;
      b_ref?: Vec3;
      drive?: number;
    }
  | {
      type: 'cylindrical';
      a: EntityId;
      b: EntityId;
      a_axis: Axis3;
      b_axis: Axis3;
      a_ref?: Vec3;
      b_ref?: Vec3;
    }
  | {
      /**
       * 移动副，消 5 留 1。δ = (b 轴原点 − a 轴原点)·(a 轴方向)；drive 是 δ 的绝对目标。
       * a_ref、b_ref、roll 必须一起提供；全部缺省选确定性参考并锁零滚转，不退化为圆柱副。
       */
      type: 'prismatic';
      a: EntityId;
      b: EntityId;
      a_axis: Axis3;
      b_axis: Axis3;
      a_ref?: Vec3;
      b_ref?: Vec3;
      roll?: number;
      drive?: number;
    }
  | { type: 'spherical'; a: EntityId; b: EntityId; a_point: Vec3; b_point: Vec3 }
  | {
      /** θ_b − ratio·θ_a 按 2π 折回；joint_a/joint_b 是两个不同 revolute 约束的 id。 */
      type: 'transmission';
      joint_a: number;
      joint_b: number;
      ratio: number;
    };

export interface Constraint {
  id: number;
  kind: ConstraintKind;
  label?: string;
}

export interface SolveParams {
  /** 有限正数；长度残差和真实弧度误差的绝对容差。@default 1e-9 */
  tolerance?: number;
  /** @default 100 */
  max_iterations?: number;
}

export interface AnsatzModel {
  entities: Entity[];
  constraints: Constraint[];
  params?: SolveParams;
}

/**
 * 只描述返回位形是否满足约束。欠约束（dof_remaining > 0）与冗余
 * （redundant_constraints 非空）是正交事实，见 Diagnostics。
 * inconsistent 需要直接矛盾证明；非线性驻点的未满足模型仍为 not_converged。
 */
export type SolveOutcome = 'converged' | 'inconsistent' | 'not_converged';

export interface ConstraintGroup {
  constraint_ids: number[];
  human_message: string;
}

export interface ConstraintResidual {
  constraint_id: number;
  residual: number;
  human_message: string;
}

export interface JointState {
  constraint_id: number;
  joint_kind: 'revolute' | 'cylindrical' | 'prismatic';
  value: number;
  human_message: string;
}

export type Suggestion =
  | { action: 'remove_constraint'; id: number; human_message: string }
  | { action: 'relax_constraint'; id: number; human_message: string }
  | { action: 'add_constraint'; dof_remaining: number; human_message: string };

export interface Diagnostics {
  dof_total: number;
  /** 自由参数数减列均衡后局部雅可比 rank；为返回位姿处的一阶局部信息。 */
  dof_remaining: number;
  iterations: number;
  /** 直接可证矛盾组（负距离/固定体不满足/同锚点容差区间不交）；非空 ⇔ inconsistent。 */
  conflicts: ConstraintGroup[];
  /** 满足位姿处的一阶冗余组（与书写顺序无关），不证明删除后全局非线性解集不变。 */
  redundant_constraints: ConstraintGroup[];
  residuals: ConstraintResidual[];
  max_residual: ConstraintResidual | null;
  suggestions: Suggestion[];
  joints: JointState[];
  /** 3D：平移/旋转列组均衡后的局部雅可比 σ_max/σ_min(非零)。 */
  jacobian_condition_estimate: number | null;
}

export interface SolveReport {
  outcome: SolveOutcome;
  entities: Entity[];
  diagnostics: Diagnostics;
}

export interface ErrorPayload {
  /** unknown_entity / unsupported_constraint / invalid_model / invalid_json / ... */
  kind: string;
  message: string;
}

/** solve / solveJson 的统一信封：成功 { ok: true, result }，失败 { ok: false, error }。 */
export type SolveEnvelope =
  | { ok: true; result: SolveReport }
  | { ok: false; error: ErrorPayload };
