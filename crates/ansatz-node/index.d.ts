/**
 * Ansatz Node 原生壳类型声明。接口与 ansatz-wasm 同名同签名
 * （solve / solveJson / version），宿主可在 wasm 与原生之间无痛切换。
 *
 * 结构化类型与 ansatz-wasm/types/ansatz.d.ts 是同一份 schema 契约
 * （schema/*.schema.json），此处直接复用，保证两壳类型永远一致。
 * 正式发包时由打包步骤把该文件一并复制进包内。
 */

import type { AnsatzModel, SolveEnvelope } from '../ansatz-wasm/types/ansatz';

export declare function solve(model: AnsatzModel | unknown): SolveEnvelope;

/** 与 FFI `ansatz_solve_json` 同形的 JSON 信封字符串。 */
export declare function solveJson(modelJson: string): string;

export declare function version(): string;
