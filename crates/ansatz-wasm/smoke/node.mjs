// Node 冒烟测试：加载 wasm-pack --target nodejs 产物（pkg-node），
// 对 core tests/parity 的两个固定算例做「位级一致性」断言——
// 与 Rust 测试用的是同一份期望文件（十六进制位模式，非 epsilon 比较）。
//
// 运行前提：
//   wasm-pack build crates/ansatz-wasm --release --target nodejs --out-dir pkg-node
//   node crates/ansatz-wasm/smoke/node.mjs

import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);
const wasm = require(path.join(here, '..', 'pkg-node', 'ansatz_wasm.js'));

const PARITY_DIR = path.join(here, '..', '..', 'ansatz-core', 'tests', 'parity');

// f64 -> 0x… 位模式（与 Rust 的 to_bits 一致，大端规范序）
const buf = new ArrayBuffer(8);
const f64 = new Float64Array(buf);
const u64 = new BigUint64Array(buf);
const hexOf = (n) => {
  f64[0] = n;
  return '0x' + u64[0].toString(16).padStart(16, '0').toUpperCase();
};
const bitsOf = (n) => {
  f64[0] = n;
  return u64[0];
};
const parseHex = (s) => BigInt(s);

console.log(`wasm version() = ${wasm.version()}`);

for (const name of ['case1', 'case2']) {
  const model = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.model.json`), 'utf8'));
  const expected = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.expected.json`), 'utf8'));

  // 1) 字符串 API（与 FFI 同形的 JSON 信封）
  const envelope = JSON.parse(wasm.solveJson(JSON.stringify(model)));
  assert.equal(envelope.ok, true, `${name}: envelope.ok`);
  const report = envelope.result;
  assert.equal(report.outcome, expected.outcome, `${name}: outcome`);

  // 2) 位级断言：wasm 输出必须与期望位模式逐位一致
  for (let i = 0; i < expected.entities.length; i++) {
    const g = report.entities[i].geometry;
    const w = expected.entities[i];
    assert.equal(g.type, 'point2');
    assert.equal(bitsOf(g.x), parseHex(w.x), `${name}.x: ${hexOf(g.x)} != ${w.x}`);
    assert.equal(bitsOf(g.y), parseHex(w.y), `${name}.y: ${hexOf(g.y)} != ${w.y}`);
  }
  const diag = report.diagnostics;
  assert.equal(diag.dof_total, expected.dof_total);
  assert.equal(diag.dof_remaining, expected.dof_remaining);
  const maxAbs = Math.abs(diag.max_residual.residual);
  assert.equal(bitsOf(maxAbs), parseHex(expected.max_residual_abs), `${name}.max_residual_abs`);
  assert.equal(
    bitsOf(diag.jacobian_condition_estimate),
    parseHex(expected.jacobian_condition_estimate),
    `${name}.cond`
  );

  // 3) 结构化 API（serde-wasm-bindgen 路径）与字符串 API 结果一致
  const structured = wasm.solve(model);
  assert.equal(structured.ok, true);
  assert.equal(structured.result.outcome, expected.outcome);
  const g = structured.result.entities[0].geometry;
  assert.equal(bitsOf(g.x), parseHex(expected.entities[0].x), `${name} structured.x`);
  assert.equal(bitsOf(g.y), parseHex(expected.entities[0].y), `${name} structured.y`);

  console.log(`${name}: 位级一致（x=${hexOf(g.x)}）`);
}

// 4) 错误路径：非法 JSON / 冲突模型
const bad = JSON.parse(wasm.solveJson('{ not json'));
assert.equal(bad.ok, false);
assert.equal(bad.error.kind, 'invalid_json');

const conflict = wasm.solve({
  entities: [{ id: 1, geometry: { type: 'point2', x: 3, y: 4 } }],
  constraints: [
    { id: 1, kind: { type: 'distance', a: 1, b: null, value: 3 } },
    { id: 2, kind: { type: 'distance', a: 1, b: null, value: 5 } },
  ],
});
assert.equal(conflict.ok, true); // 求解层结论
assert.equal(conflict.result.outcome, 'inconsistent');
assert.deepEqual(conflict.result.diagnostics.redundant_constraints[0].constraint_ids, [1, 2]);
assert.ok(conflict.result.diagnostics.suggestions.length > 0);

console.log('error paths: ok=false(invalid_json) / inconsistent-with-diagnostics');
console.log('wasm smoke: ALL PASS');
