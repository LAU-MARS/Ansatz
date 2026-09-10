// Node 原生壳（napi-rs）冒烟测试：定位 cargo 构建出的 cdylib，复制为 .node 加载，
// 断言与 ansatz-wasm 壳同名同签名，并对 core tests/parity 算例做位级一致断言。
//
// 运行前提：cargo build --release -p ansatz-node
//   node crates/ansatz-node/smoke/native.mjs

import assert from 'node:assert/strict';
import { readFileSync, copyFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import path from 'node:path';
import os from 'node:os';

const here = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(here, '..', '..', '..');
const PARITY_DIR = path.join(ROOT, 'crates', 'ansatz-core', 'tests', 'parity');

// 与 wasm 冒烟相同 API：solve / solveJson / version
const libNames = {
  win32: 'ansatz_node.dll',
  linux: 'libansatz_node.so',
  darwin: 'libansatz_node.dylib',
};
const libPath = path.join(ROOT, 'target', 'release', libNames[process.platform]);
const nodePath = path.join(os.tmpdir(), 'ansatz-node-smoke', 'ansatz.node');
mkdirSync(path.dirname(nodePath), { recursive: true });
copyFileSync(libPath, nodePath); // Node 的 require 只认 .node 后缀
const require = createRequire(import.meta.url);
const native = require(nodePath);

const buf = new ArrayBuffer(8);
const f64 = new Float64Array(buf);
const u64 = new BigUint64Array(buf);
const bitsOf = (n) => {
  f64[0] = n;
  return u64[0];
};
const parseHex = (s) => BigInt(s);

console.log(`native version() = ${native.version()}`);
assert.match(native.version(), /^\d+\.\d+\.\d+/);

for (const name of ['case1', 'case2']) {
  const model = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.model.json`), 'utf8'));
  const expected = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.expected.json`), 'utf8'));

  // 字符串 API：与 FFI/wasm 同形的 JSON 信封
  const envelope = JSON.parse(native.solveJson(JSON.stringify(model)));
  assert.equal(envelope.ok, true, `${name}: envelope.ok`);
  assert.equal(envelope.result.outcome, expected.outcome);

  // 位级断言（与 Rust/wasm 测试同一份期望文件）
  const g = envelope.result.entities[0].geometry;
  assert.equal(bitsOf(g.x), parseHex(expected.entities[0].x), `${name}.x`);
  assert.equal(bitsOf(g.y), parseHex(expected.entities[0].y), `${name}.y`);
  const diag = envelope.result.diagnostics;
  assert.equal(bitsOf(Math.abs(diag.max_residual.residual)), parseHex(expected.max_residual_abs));
  assert.equal(
    bitsOf(diag.jacobian_condition_estimate),
    parseHex(expected.jacobian_condition_estimate)
  );

  // 结构化 API 与字符串 API 一致
  const structured = native.solve(model);
  assert.equal(structured.ok, true);
  assert.equal(bitsOf(structured.result.entities[0].geometry.x), parseHex(expected.entities[0].x));
  console.log(`${name}: 位级一致`);
}

// 错误路径
const bad = JSON.parse(native.solveJson('{ oops'));
assert.equal(bad.ok, false);
assert.equal(bad.error.kind, 'invalid_json');
const unsupported = native.solve({
  entities: [{ id: 1, geometry: { type: 'point2', x: 0, y: 0 } }],
  constraints: [{ id: 1, kind: { type: 'fixed', a: 1 } }],
});
assert.equal(unsupported.ok, false);
assert.equal(unsupported.error.kind, 'unsupported_constraint');

console.log('native smoke: ALL PASS');
