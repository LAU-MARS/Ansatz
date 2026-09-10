/**
 * Prepare the publishable `ansatz-wasm` npm package (nodejs target).
 *
 * Steps:
 *   1. wasm-pack build crates/ansatz-wasm --target nodejs --out-dir pkg-node
 *   2. polish the generated package.json (repo metadata, keywords, files)
 *   3. copy in the handwritten model types (types/ansatz.d.ts) and the
 *      npm-facing README
 *
 * Usage:
 *   node scripts/npm-prepare.mjs            # build + polish into pkg-node/
 *   node scripts/npm-prepare.mjs --check v0.1.0   # CI: verify tag == version
 *
 * Publishing itself is done by CI (.github/workflows/npm-publish.yml) with the
 * NPM_TOKEN secret; locally you can also `npm publish crates/ansatz-wasm/pkg-node`
 * after `npm login`.
 */
import { copyFileSync, mkdirSync, readFileSync, writeFileSync, existsSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const crate = path.join(root, 'crates', 'ansatz-wasm')
const pkg = path.join(crate, 'pkg-node')

const checkTag = (() => {
  const i = process.argv.indexOf('--check')
  return i >= 0 ? process.argv[i + 1] : null
})()

const build = spawnSync(
  // 单字符串命令避免 args+shell 组合的注入告警；路径带引号防空格
  `wasm-pack build "${crate}" --release --target nodejs --out-dir pkg-node --out-name ansatz_wasm`,
  { stdio: 'inherit', shell: true, cwd: root },
)
if (build.status !== 0) {
  console.error('wasm-pack build failed')
  process.exit(build.status ?? 1)
}

const generated = JSON.parse(readFileSync(path.join(pkg, 'package.json'), 'utf8'))

// Polish: keep wasm-pack's name/version/main/types/files (source of truth = the
// Cargo workspace version), add the metadata npm consumers expect.
const polished = {
  name: 'ansatz-wasm',
  version: generated.version,
  description:
    'Ansatz geometric constraint solver core (wasm, Node): structured diagnostics-first, bit-reproducible f64, no SharedArrayBuffer/COOP-COEP',
  license: 'MIT',
  repository: {
    type: 'git',
    url: 'git+https://github.com/LAU-MARS/Ansatz.git',
    directory: 'crates/ansatz-wasm',
  },
  homepage: 'https://github.com/LAU-MARS/Ansatz#readme',
  bugs: { url: 'https://github.com/LAU-MARS/Ansatz/issues' },
  keywords: [
    'geometric-constraint-solver',
    'cad',
    'sketch',
    'solver',
    'constraint',
    'wasm',
    'rust',
  ],
  files: [
    'ansatz_wasm.js',
    'ansatz_wasm_bg.wasm',
    'ansatz_wasm.d.ts',
    'ansatz_wasm_bg.wasm.d.ts',
    'types/ansatz.d.ts',
    'README.md',
    'LICENSE',
  ],
  main: 'ansatz_wasm.js',
  types: 'ansatz_wasm.d.ts',
  sideEffects: false,
  collaborators: ['Ansatz contributors'],
}

for (const [src, dest] of [
  ['types/ansatz.d.ts', 'types/ansatz.d.ts'],
  ['npm/README.md', 'README.md'],
  // 根 LICENSE（npm 包完整性；wasm-pack 只看 crate 目录所以它自己找不到）
  ['../../LICENSE', 'LICENSE'],
]) {
  const from = path.join(crate, src)
  const to = path.join(pkg, dest)
  if (!existsSync(from)) {
    console.error(`missing ${from} — run from the repo root`)
    process.exit(1)
  }
  mkdirSync(path.dirname(to), { recursive: true })
  copyFileSync(from, to)
}

writeFileSync(path.join(pkg, 'package.json'), JSON.stringify(polished, null, 2) + '\n')
console.log(`pkg-node polished: ansatz-wasm@${polished.version}`)

if (checkTag) {
  if (checkTag.replace(/^v/, '') !== polished.version) {
    console.error(`tag ${checkTag} does not match package version ${polished.version}`)
    process.exit(1)
  }
  console.log(`tag check ok: ${checkTag} == v${polished.version}`)
}
