// Node 原生壳（napi-rs）冒烟测试：定位 cargo 构建出的 cdylib，复制为 .node 加载，
// 断言与 ansatz-wasm 壳同名同签名，并对 core tests/parity 算例做位级一致断言。
//
// 运行前提：cargo build [--release] [-p ansatz-node] [--target <triple>]
//   node crates/ansatz-node/smoke/native.mjs
// 产物目录自动发现（target/release、target/<triple>/release、…/debug）；
// 也可用 ANSATZ_LIB_DIR 显式指定。

import assert from 'node:assert/strict';
import { readFileSync, copyFileSync, mkdirSync, existsSync, readdirSync } from 'node:fs';
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

/** 定位 cdylib：显式 env > 宿主默认 > --target <triple> 目录（release 优先于 debug）。 */
function findLib() {
  const libName = libNames[process.platform];
  if (process.env.ANSATZ_LIB_DIR) {
    return path.join(process.env.ANSATZ_LIB_DIR, libName);
  }
  const targetDir = path.join(ROOT, 'target');
  if (!existsSync(targetDir)) return null;
  // '' = 宿主默认（target/release）；其余条目里，三元组目录（如
  // x86_64-unknown-linux-gnu）是显式 --target 构建的落点。
  // release 一律优先于 debug：避免加载陈旧的 host debug 构建造成假绿。
  const subdirs = ['', ...readdirSync(targetDir).filter((e) => e.includes('-'))];
  for (const profile of ['release', 'debug']) {
    for (const sub of subdirs) {
      const p = path.join(targetDir, sub, profile, libName);
      if (existsSync(p)) return p;
    }
  }
  return null;
}

const libPath = findLib();
if (!libPath) {
  console.error(
    `找不到 ansatz-node cdylib（已搜索 target/ 及其三元组子目录）。先 cargo build -p ansatz-node，或用 ANSATZ_LIB_DIR 指定。`
  );
  process.exit(1);
}
const nodePath = path.join(os.tmpdir(), 'ansatz-node-smoke', 'ansatz.node');
mkdirSync(path.dirname(nodePath), { recursive: true });
copyFileSync(libPath, nodePath); // Node 的 require 只认 .node 后缀
const require = createRequire(import.meta.url);
const native = require(nodePath);
console.log(`native lib: ${libPath}`);

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

for (const name of ['case1', 'case2', 'case3', 'case4']) {
  const model = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.model.json`), 'utf8'));
  const expected = JSON.parse(readFileSync(path.join(PARITY_DIR, `${name}.expected.json`), 'utf8'));

  // 字符串 API：与 FFI/wasm 同形的 JSON 信封
  const envelope = JSON.parse(native.solveJson(JSON.stringify(model)));
  assert.equal(envelope.ok, true, `${name}: envelope.ok`);
  assert.equal(envelope.result.outcome, expected.outcome);

  // 位级断言（与 Rust/wasm 测试同一份期望文件）：按实体类型分派
  for (let i = 0; i < expected.entities.length; i++) {
    const g = envelope.result.entities[i].geometry;
    const w = expected.entities[i];
    if (g.type === 'point2') {
      assert.equal(bitsOf(g.x), parseHex(w.x), `${name}.x`);
      assert.equal(bitsOf(g.y), parseHex(w.y), `${name}.y`);
    } else if (g.type === 'rigid3') {
      const t = g.pose.translation, rv = g.pose.rotation.vector;
      const fields = [['tx', t.x], ['ty', t.y], ['tz', t.z], ['rx', rv[0]], ['ry', rv[1]], ['rz', rv[2]]];
      for (const [f, val] of fields) {
        assert.equal(bitsOf(val), parseHex(w.pose[f]), `${name}.${f}`);
      }
    } else {
      assert.fail(`未支持的 parity 实体类型 ${g.type}`);
    }
  }
  const diag = envelope.result.diagnostics;
  assert.equal(bitsOf(Math.abs(diag.max_residual.residual)), parseHex(expected.max_residual_abs));
  assert.equal(
    bitsOf(diag.jacobian_condition_estimate),
    parseHex(expected.jacobian_condition_estimate)
  );

  // 结构化 API 与字符串 API 一致（首个实体的首个数值字段）
  const structured = native.solve(model);
  assert.equal(structured.ok, true);
  const sg = structured.result.entities[0].geometry;
  const firstNum = sg.type === 'point2' ? sg.x : sg.pose.translation.x;
  const wg = expected.entities[0];
  const wantFirst = wg.x ?? wg.pose.tx;
  assert.equal(bitsOf(firstNum), parseHex(wantFirst));
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
