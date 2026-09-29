// Rewrites relative links that leave the docs tree (source, configs, internal notes) to their GitHub URL.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {visit} from 'unist-util-visit';

const websiteDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const repoRoot = path.dirname(websiteDir);
const docsDir = path.join(websiteDir, 'docs');
// Internal material that the public site must not link to.
const unpublished = ['doc/lab', 'doc/research', 'doc/reviews', 'doc/superpowers', 'plans', '.claude',
  'doc/Test Cases.MD', 'doc/NOTES.MD', 'doc/vibe.md', 'doc/commerce.md', 'doc/web.MD'];

export default function repoLinks({repoUrl}) {
  return (tree, file) => {
    const dir = path.dirname(file.path);
    visit(tree, ['link', 'definition'], (node) => {
      const url = node.url;
      if (!url || /^([a-z]+:|#|\/)/i.test(url)) return;
      const [target, hash] = url.split('#');
      if (!target) return;
      const abs = path.resolve(dir, decodeURIComponent(target));
      if (abs.startsWith(docsDir + path.sep)) return;
      const rel = path.relative(repoRoot, abs);
      if (rel.startsWith('..')) return;
      if (!fs.existsSync(abs)) throw new Error(`${file.path}: link to missing file ${url}`);
      if (unpublished.some((d) => rel === d || rel.startsWith(d + path.sep)))
        throw new Error(`${file.path}: link to unpublished ${rel}`);
      node.url = `${repoUrl}/blob/main/${rel.split(path.sep).map(encodeURIComponent).join('/')}` +
        (hash ? `#${hash}` : '');
    });
  };
}
