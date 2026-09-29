// Fails the build when a published page reads like a research scratchpad or an agent transcript.
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {visit} from 'unist-util-visit';
import {toString} from 'mdast-util-to-string';

const docsDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../docs');

const SCRATCH_HEADING = /^\s*(\d+[a-z]?\.?\s*)?(bottom line|tl;?\s*dr|provenance|search record|independent review|(source )?access log|review notes?)\b/i;
const TRANSCRIPT_LINE = /^\s*(LLM|User|Assistant|Human|AI)\s*>/m;
const AI_NAME = /\b(codex|claude|chatgpt|gpt-\d)\b/i;
const PLACEHOLDER = /LLM generated placeholder/i;

export default function docsLint({allowAiNames = []} = {}) {
  return (tree, file) => {
    const rel = path.relative(docsDir, file.path);
    const problems = [];
    visit(tree, 'heading', (node) => {
      const text = toString(node);
      if (SCRATCH_HEADING.test(text)) problems.push(`heading "${text}"`);
    });
    visit(tree, 'text', (node) => {
      if (TRANSCRIPT_LINE.test(node.value)) problems.push(`transcript line "${node.value.trim().slice(0, 60)}"`);
      if (PLACEHOLDER.test(node.value)) problems.push('LLM-placeholder line (not shown on published pages)');
      const m = !allowAiNames.includes(rel) && node.value.match(AI_NAME);
      if (m) problems.push(`"${m[0]}" in prose`);
    });
    if (!problems.length) return;
    const msg = `${rel}: reads like a scratchpad, not documentation (CLAUDE.md "Documentation site"): ${problems.join('; ')}`;
    // Production builds fail; the dev server only warns so one page can't take the whole site down mid-edit.
    if (process.env.NODE_ENV === 'production') throw new Error(msg);
    console.warn(`[docs-lint] ${msg}`);
  };
}
