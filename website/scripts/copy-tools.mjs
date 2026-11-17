import { copyFileSync, existsSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const site = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = join(site, '..', 'etc', 'config-tool');
const dst = join(site, 'static', 'tools');

const files = ['dashboard.html', 'teledec.js'];
const missing = files.filter((f) => !existsSync(join(src, f)));
if (missing.length) {
  console.error(`copy-tools: missing ${missing.map((f) => join(src, f)).join(', ')}`);
  process.exit(1);
}
mkdirSync(dst, { recursive: true });
for (const f of files) copyFileSync(join(src, f), join(dst, f));
console.log(`copy-tools: ${files.join(', ')} -> static/tools/`);
