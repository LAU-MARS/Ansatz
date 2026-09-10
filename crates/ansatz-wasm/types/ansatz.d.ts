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

/** 3D 旋转：指数映射（旋转向量）v = θ·û，3 参数 ↔ 3 自由度，无冗余。 */
export interface Rotation3 {
  vector: [number, number, number];
}

export interface Pose3 {
  translation: Vec3;
  rotation: Rotation3;
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

export type ConstraintKind =
  | { type: 'coincident'; a: EntityId; b: EntityId }
  | { type: 'collinear'; a: EntityId; b: EntityId; c: EntityId }
  | { type: 'parallel'; a: EntityId; b: EntityId }
  | { type: 'perpendicular'; a: EntityId; b: EntityId }
  | { type: 'tangent'; a: EntityId; b: EntityId }
  | { type: 'distance'; a: EntityId; b: EntityId | null; value: number }
  | { type: 'angle'; a: EntityId; b: EntityId | null; value: number }
  | { type: 'symmetric'; a: EntityId; b: EntityId; about: EntityId }
  | { type: 'fixed'; a: EntityId }
  | { type: 'mate'; a: EntityId; b: EntityId }
  | { type: 'coaxial'; a: EntityId; b: EntityId };

export interface Constraint {
  id: number;
  kind: ConstraintKind;
  label?: string;
}

export interface SolveParams {
  /** @default 1e-9 */
  tolerance?: number;
  /** @default 100 */
  max_iterations?: number;
  /** @default true */
  allow_underconstrained?: boolean;
}

export interface AnsatzModel {
  entities: Entity[];
  constraints: Constraint[];
  params?: SolveParams;
}

export type SolveOutcome =
  | 'converged'
  | 'underconstrained'
  | 'overconstrained'
  | 'inconsistent'
  | 'max_iterations';

export interface RedundancyGroup {
  constraint_ids: number[];
  human_message: string;
}

export interface ConstraintResidual {
  constraint_id: number;
  residual: number;
  human_message: string;
}

export type Suggestion =
  | { action: 'remove_constraint'; id: number; human_message: string }
  | { action: 'relax_constraint'; id: number; human_message: string }
  | { action: 'add_constraint'; dof_remaining: number; human_message: string };

export interface Diagnostics {
  dof_total: number;
  dof_remaining: number;
  redundant_constraints: RedundancyGroup[];
  residuals: ConstraintResidual[];
  max_residual: ConstraintResidual | null;
  suggestions: Suggestion[];
  jacobian_condition_estimate: number | null;
}

export interface SolveReport {
  outcome: SolveOutcome;
  entities: Entity[];
  diagnostics: Diagnostics;
}

export interface ErrorPayload {
  kind: string;
  message: string;
}

/** solve / solveJson 的统一信封：成功 { ok: true, result }，失败 { ok: false, error }。 */
export type SolveEnvelope =
  | { ok: true; result: SolveReport }
  | { ok: false; error: ErrorPayload };
