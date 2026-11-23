#!/usr/bin/env node
// Mirrors the board configs under config/ into a presets/ dir the editor can
// fetch over http(s) ("Load preset"). Generated, never committed: run
// `node etc/config-tool/gen_presets.mjs` after a config change; the website
// build regenerates its own copy via scripts/copy-tools.mjs.
import { cpSync, mkdirSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve, sep } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const TOOL = dirname(fileURLToPath(import.meta.url));
export const REPO = resolve(TOOL, '..', '..');

// Device captures under config/dl/ are not shipped presets.
const SKIP = ['dl'];

export function findPresets(configDir = join(REPO, 'config')){
  const found = [];
  (function walk(dir){
    for (const e of readdirSync(dir, { withFileTypes: true })){
      if (!e.isDirectory()) continue;
      const p = join(dir, e.name);
      if (e.name === 'conf'){
        const files = readdirSync(p).filter(f => f.endsWith('.conf')).sort();
        const id = relative(configDir, dir).split(sep).join('/');
        if (files.length && !SKIP.includes(id.split('/')[0])) found.push({ id, dir, files });
        continue;   // nothing but *.conf lives in a conf/ dir
      }
      walk(p);
    }
  })(configDir);
  return found.sort((a, b) => a.id.localeCompare(b.id));
}

export function generatePresets(outDir, configDir = join(REPO, 'config')){
  const presets = findPresets(configDir).map(({ id, dir, files }) => {
    mkdirSync(join(outDir, id, 'conf'), { recursive: true });
    for (const f of files) cpSync(join(dir, 'conf', f), join(outDir, id, 'conf', f));
    return { id, label: id, files };
  });
  rmSync(join(outDir, 'index.json'), { force: true });
  writeFileSync(join(outDir, 'index.json'), JSON.stringify({ presets }, null, 1) + '\n');
  return presets;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href){
  const out = process.argv[2] || join(TOOL, 'presets');
  const presets = generatePresets(out);
  console.log(`gen_presets: ${presets.length} presets, ${presets.reduce((n, p) => n + p.files.length, 0)} files -> ${out}`);
}
