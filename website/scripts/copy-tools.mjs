import { copyFileSync, existsSync, mkdirSync, rmSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const site = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = join(site, '..', 'etc', 'config-tool');
const dst = join(site, 'static', 'tools');

const files = ['dashboard.html', 'teledec.js', 'conf-editor.html'];
const missing = files.filter((f) => !existsSync(join(src, f)));
if (missing.length) {
  console.error(`copy-tools: missing ${missing.map((f) => join(src, f)).join(', ')}`);
  process.exit(1);
}
mkdirSync(dst, { recursive: true });
for (const f of files) copyFileSync(join(src, f), join(dst, f));

// Board presets for "Load preset", regenerated from config/ so the copy can't drift.
const presetsDst = join(dst, 'presets');
rmSync(presetsDst, { recursive: true, force: true });
const { generatePresets } = await import(pathToFileURL(join(src, 'gen_presets.mjs')).href);
const presets = generatePresets(presetsDst);
console.log(`copy-tools: ${files.join(', ')} + ${presets.length} presets -> static/tools/`);
