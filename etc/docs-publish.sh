#!/bin/bash
# Commit files onto origin/main without touching the shared checkout's index or HEAD, build the docs from
# that commit in a clean worktree, push by sha and wait for the Docs deploy.
#
#   etc/docs-publish.sh -m MSG [-n] [--no-build] [--no-wait] [--force] PATH... [--from SRC=PATH]...
#
# PATH is repo-relative; its content comes from the working file, or from SRC with --from (use that
# when the working file also holds someone else's uncommitted edits: copy origin's version, edit it,
# publish the copy). Refuses content equal to origin, and content that would drop lines origin added
# since this checkout's HEAD (a stale base); --force overrides the latter. -n builds but does not push.
set -u
cd "$(git rev-parse --show-toplevel)" || exit 2

msg="" dry=0 build=1 wait=1 force=0
paths=() srcs=()
while [ $# -gt 0 ]; do
  case "$1" in
    -m) msg="$2"; shift 2 ;;
    -n|--dry-run) dry=1; shift ;;
    --no-build) build=0; shift ;;
    --no-wait) wait=0; shift ;;
    --force) force=1; shift ;;
    --from) paths+=("${2#*=}"); srcs+=("${2%%=*}"); shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    -*) echo "unknown option $1" >&2; exit 2 ;;
    *) paths+=("$1"); srcs+=("$1"); shift ;;
  esac
done
[ -n "$msg" ] && [ ${#paths[@]} -gt 0 ] || { sed -n '2,10p' "$0"; exit 2; }

PY_STALE='
import difflib, sys
h, o, c = (open(f, errors="replace").read().splitlines() for f in sys.argv[1:4])
cs = set(c)
n = 0
for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, h, o, autojunk=False).get_opcodes():
    old, new = [x for x in h[i1:i2] if x.strip()], [x for x in o[j1:j2] if x.strip()]
    if tag == "equal" or not (old or new):
        continue
    if all(x in cs for x in old) and not any(x in cs for x in new):
        n += 1
print(n)
'

tmp=$(mktemp -d /tmp/docs-publish.XXXX)
trap 'git worktree remove --force "$tmp/wt" >/dev/null 2>&1; rm -rf "$tmp"' EXIT

git fetch -q origin || { echo "fetch failed" >&2; exit 1; }

# per-file checks against origin and HEAD
bad=0
for i in "${!paths[@]}"; do
  p=${paths[$i]} s=${srcs[$i]}
  [ -f "$s" ] || { echo "$p: no such file $s" >&2; bad=1; continue; }
  if git show "origin/main:${p}" > "$tmp/o" 2>/dev/null; then :; else : > "$tmp/o"; fi
  if cmp -s "$s" "$tmp/o"; then echo "$p: same as origin/main, nothing to publish" >&2; bad=1; continue; fi
  if git show "HEAD:${p}" > "$tmp/h" 2>/dev/null && ! cmp -s "$tmp/h" "$tmp/o"; then
    # hunks of HEAD..origin the content still has in HEAD's form: committing it would revert them.
    # A hunk whose lines the content rewrote (neither form present) counts as edited, not stale.
    lost=$(python3 -c "$PY_STALE" "$tmp/h" "$tmp/o" "$s")
    if [ "$lost" -gt 0 ]; then
      echo "$p: built on a stale base, would revert $lost change(s) origin/main made since HEAD." >&2
      echo "  refresh it: git show origin/main:$p > $p, re-apply your edit (or --force)" >&2
      [ $force = 1 ] || bad=1
    fi
  fi
  [ "$s" = "$p" ] && ! git diff --quiet HEAD -- "$p" 2>/dev/null && [ -x ~/dev/bin/who-touched ] &&
    ~/dev/bin/who-touched "$p" --since 24h 2>/dev/null | grep -v '^no edits' | sed "s|^|  $p: |" | head -5
  echo "$p: $(git diff --no-index --numstat "$tmp/o" "$s" | awk '{print "+"$1" -"$2}') vs origin/main"
done
[ $bad = 0 ] || exit 1

mkcommit() { # $1 = parent
  export GIT_INDEX_FILE="$tmp/index"
  rm -f "$GIT_INDEX_FILE"
  git read-tree "$1" || return 1
  for i in "${!paths[@]}"; do
    git update-index --add --cacheinfo "100644,$(git hash-object -w "${srcs[$i]}"),${paths[$i]}" || return 1
  done
  git commit-tree "$(git write-tree)" -p "$1" -m "$msg"
  local rc=$?; unset GIT_INDEX_FILE; return $rc
}

base=$(git rev-parse origin/main)
new=$(mkcommit "$base") || { echo "commit failed" >&2; exit 1; }
git diff --stat "$base" "$new"

if [ $build = 1 ]; then
  git worktree add -q --detach "$tmp/wt" "$new" || exit 1
  # a copy, not symlinks: symlinked node_modules resolve back into this checkout and build its sources
  cp -c -R website/node_modules "$tmp/wt/website/" 2>/dev/null || cp -R website/node_modules "$tmp/wt/website/" || exit 1
  (cd "$tmp/wt/website" && npx docusaurus build --out-dir "$tmp/site" > "$tmp/build.log" 2>&1)
  rc=$?
  cp "$tmp/build.log" /tmp/docs-publish-build.log
  if [ $rc != 0 ]; then echo "docs build failed (rc=$rc), log: /tmp/docs-publish-build.log" >&2; tail -20 "$tmp/build.log" >&2; exit 1; fi
  grep -i 'warn' "$tmp/build.log" | grep -v -e ExperimentalWarning -e trace-warnings | head -10
  echo "docs build ok"
  git worktree remove --force "$tmp/wt"
fi

[ $dry = 1 ] && { echo "dry run: built $new, not pushed"; exit 0; }

for try in 1 2 3 4 5; do
  if git push -q origin "$new:refs/heads/main" 2>"$tmp/push.err"; then echo "pushed $new"; break; fi
  [ $try = 5 ] && { cat "$tmp/push.err" >&2; exit 1; }
  sleep $((try * 2)); git fetch -q origin || continue
  over=$(git diff --name-only "$base" origin/main -- "${paths[@]}")
  [ -z "$over" ] || { echo "origin/main changed these files meanwhile, re-check them: $over" >&2; exit 1; }
  base=$(git rev-parse origin/main)
  new=$(mkcommit "$base") || exit 1
done

[ $wait = 1 ] || exit 0
git diff --quiet "$base" "$new" -- website etc/config-tool/conf-editor.html etc/config-tool/dashboard.html \
  etc/config-tool/teledec.js && { echo "no docs paths changed, no deploy to wait for"; exit 0; }
for i in $(seq 1 60); do
  st=$(gh run list --workflow Docs -L 8 --json headSha,status,conclusion 2>/dev/null |
    python3 -c '
import json, subprocess, sys
for r in json.load(sys.stdin):
    if subprocess.run(["git", "merge-base", "--is-ancestor", sys.argv[1], r["headSha"]]).returncode == 0:
        print(r["status"], r["conclusion"], r["headSha"][:8]); break
' "$new")
  case "$st" in
    "completed success"*) echo "deployed: $st"; exit 0 ;;
    completed*) echo "deploy failed: $st" >&2; exit 1 ;;
  esac
  git fetch -q origin 2>/dev/null; sleep 20
done
echo "deploy not finished after 20 min: ${st:-no run yet}" >&2; exit 1
