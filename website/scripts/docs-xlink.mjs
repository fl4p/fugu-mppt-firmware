// Runs the docs-xlink check over every page, independent of the build cache. Exit 1 on a finding or parse failure.
import fs from 'node:fs';
import path from 'node:path';
import {unified} from 'unified';
import parse from 'remark-parse';
import gfm from 'remark-gfm';
import fm from 'remark-frontmatter';
import math from 'remark-math';
import mdx from 'remark-mdx';
import {check, docsDir} from '../src/remark/docs-xlink.mjs';

let bad = 0;
for (const f of fs.readdirSync(docsDir, {recursive: true}).map(String).sort()) {
  if (!/\.mdx?$/.test(f)) continue;
  const full = path.join(docsDir, f);
  const proc = unified().use(parse).use(gfm).use(fm).use(math);
  if (f.endsWith('.mdx')) proc.use(mdx);
  let msg;
  try { msg = check(proc.parse(fs.readFileSync(full, 'utf8')), full); }
  catch (e) { msg = `[docs-xlink] ${f}: parse failed: ${e.message}`; }
  if (msg) { console.log(msg); bad++; }
}
process.exit(bad ? 1 : 0);
