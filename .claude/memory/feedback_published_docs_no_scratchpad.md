---
name: feedback-published-docs-no-scratchpad
description: "Published website/docs pages must read as curated documentation, not research scratchpads (no \"Bottom line\" lead, no provenance/review/access-log sections)"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 9e23fd13-3af5-47da-8f79-207149c4bd53
  modified: 2026-09-29T16:06:49.988Z
---

Pages under website/docs/ must read like reference documentation. Research-process artifacts are not published:
no "Bottom line"/TL;DR as the first heading, no "Provenance and search record", no "Independent review" (AI review
notes), no "Source access log", no numbered scratch-note structure. Literature-based pages are proper surveys:
intro/scope, findings per topic in prose, implications, limitations, and a numbered References list with full
citations verified against the sources (local archive ~/dev/pv/ee/lit/bat for battery literature).

**Why:** Fab, 2026-09-29, on internals/lfp-longevity: "this reads like a scratchpad … '6a. Independent review' is just
ai slop … make it a proper survey with references … 'Bottom line' as first head-line looks like slop too." The pages
came from doc/ research notes that were published verbatim during the Docusaurus migration.

Also no `*this document is an LLM generated placeholder*` line on published pages (Fab, 2026-09-29: "remove");
it stays on unpublished doc/ notes. docs-lint rejects it.

**How to apply:** keep the research record (search log, reviews, access log) unpublished in doc/research/, and
write the published page from it. Check this before moving any doc/ note into website/docs. See
[[reference-lfp-longevity-research]].
