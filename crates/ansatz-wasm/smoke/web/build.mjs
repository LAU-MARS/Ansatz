// 生成单文件冒烟 HTML：把 wasm-bindgen 胶水与 base64 的 .wasm 全部内联。
// 产物 index.html 不依赖任何外部文件，双击 / file:// 直接可用（initSync 免 fetch）。
//
// 重新生成（修改 wasm 壳后）：
//   wasm-pack build crates/ansatz-wasm --release --target web --out-dir pkg-web
//   node crates/ansatz-wasm/smoke/web/build.mjs

import { readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const pkg = path.join(here, '..', '..', 'pkg-web');

const glue = readFileSync(path.join(pkg, 'ansatz_wasm.js'), 'utf8');
// 内联到 <script type="module"> 时去掉模块导出语句，函数声明仍在模块作用域可用
const inlinedGlue = glue.replace(/export \{ initSync, __wbg_init as default \};/, '/* (export 语句已移除：内联场景无需模块导出) */');
if (inlinedGlue === glue) throw new Error('胶水结构与预期不符，请检查 pkg-web/ansatz_wasm.js 的导出行');

const wasmB64 = readFileSync(path.join(pkg, 'ansatz_wasm_bg.wasm')).toString('base64');

const model = {
  entities: [{ id: 1, geometry: { type: 'point2', x: 3.0, y: 4.0 } }],
  constraints: [
    { id: 1, kind: { type: 'distance', a: 1, b: null, value: 10.0 }, label: '点到原点距离' },
  ],
};

const html = `<!doctype html>
<!-- Ansatz wasm 单文件冒烟页（由 smoke/web/build.mjs 生成，勿手改）。
     特性：wasm 以 base64 内嵌 + initSync 同步实例化，无 fetch/fetch 失败问题，
     file:// 协议直接打开即可；单线程，不依赖 SharedArrayBuffer / COOP-COEP。 -->
<html lang="zh">
<head>
<meta charset="utf-8">
<title>Ansatz wasm smoke</title>
<style>
  body { font-family: ui-monospace, Consolas, monospace; margin: 2rem; background: #101418; color: #d8dee9; }
  h1 { font-size: 1.1rem; }
  pre { background: #1b2129; padding: 1rem; border-radius: 6px; overflow: auto; white-space: pre-wrap; }
  .ok { color: #7fd962; }
</style>
</head>
<body>
<h1>Ansatz · wasm smoke（单文件）</h1>
<pre id="out">wasm 加载中…</pre>
<script type="module">
/* ===================== wasm-bindgen 胶水（由 pkg-web/ansatz_wasm.js 内联） ===================== */
${inlinedGlue}
/* ===================== wasm 二进制（base64 内嵌） ===================== */
const WASM_B64 = "${wasmB64}";

const out = document.getElementById('out');
try {
  const bin = atob(WASM_B64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  initSync(bytes); // 同步实例化：无 fetch、无跨域、file:// 可用

  const model = ${JSON.stringify(model)};
  const lines = [];
  lines.push('version(): ' + version());

  // 1) 字符串 API（与 FFI 同形的 JSON 信封）
  const env = JSON.parse(solveJson(JSON.stringify(model)));
  lines.push('solveJson(): ok=' + env.ok + ', outcome=' + env.result.outcome);
  const g = env.result.entities[0].geometry;
  lines.push('  solved point = (' + g.x + ', ' + g.y + ')   // 期望 (6, 8)');
  lines.push('  dof: ' + env.result.diagnostics.dof_total + ' -> ' + env.result.diagnostics.dof_remaining);
  lines.push('  suggestion: ' + env.result.diagnostics.suggestions[0].human_message);

  // 2) 结构化 API
  const env2 = solve(model);
  const g2 = env2.result.entities[0].geometry;
  lines.push('solve():     ok=' + env2.ok + ', point=(' + g2.x + ', ' + g2.y + ')');

  const pass = env.ok && g.x === 6 && g.y === 8 && env2.ok && g2.x === 6 && g2.y === 8;
  lines.push(pass ? 'SMOKE PASS ✓' : 'SMOKE FAIL ✗');
  out.textContent = lines.join('\\n');
  out.className = pass ? 'ok' : '';
} catch (e) {
  out.textContent = 'SMOKE FAIL ✗\\n' + (e && e.stack || e);
}
</script>
</body>
</html>
`;

const outFile = path.join(here, 'index.html');
writeFileSync(outFile, html);
console.log(`written ${outFile} (${(html.length / 1024).toFixed(0)} KB)`);
