#!/bin/bash
# Commit files onto origin/main without touching the shared checkout's index or HEAD, build the docs from
# that commit in a clean worktree, push by sha and wait for the Docs deploy.
#
#   etc/docs-publish.sh -m MSG [-n] [--no-build] [--no-wait] [--force] PATH... [--from SRC=PATH]...
#
# PATH is repo-relative; its content comes from the working file, or from SRC with --from (use that
# when the working file also holds someone else's uncommitted edits: copy origin's version, edit it,
# publish the copy). Refuses content equal to origin, and content that lacks changes origin made since
# this checkout's merge-base with it (a stale base); --force overrides that when the content changes the
# same lines on purpose. -n builds but does not push.
set -u
top=$(git rev-parse --show-toplevel) || exit 2

msg="" dry=0 build=1 wait=1 force=0
paths=() srcs=()
abs() { case "$1" in /*) echo "$1" ;; *) echo "$PWD/$1" ;; esac; }
while [ $# -gt 0 ]; do
  case "$1" in
    -m) msg="$2"; shift 2 ;;
    -n|--dry-run) dry=1; shift ;;
    --no-build) build=0; shift ;;
    --no-wait) wait=0; shift ;;
    --force) force=1; shift ;;
    --from) paths+=("${2#*=}"); srcs+=("$(abs "${2%%=*}")"); shift 2 ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    -*) echo "unknown option $1" >&2; exit 2 ;;
    *) paths+=("$1"); srcs+=("$top/$1"); shift ;;
  esac
done
[ -n "$msg" ] && [ ${#paths[@]} -gt 0 ] || { sed -n '2,11p' "$0"; exit 2; }
cd "$top" || exit 2

tmp=$(mktemp -d /tmp/docs-publish.XXXX)
trap 'git worktree remove --force "$tmp/wt" >/dev/null 2>&1; rm -rf "$tmp"' EXIT

git fetch -q origin || { echo "fetch failed" >&2; exit 1; }
mb=$(git merge-base HEAD origin/main) || { echo "no merge-base of HEAD and origin/main" >&2; exit 1; }

# per-file checks against origin and the merge-base
bad=0
for i in "${!paths[@]}"; do
  p=${paths[$i]} s=${srcs[$i]}
  [ -f "$s" ] || { echo "$p: no such file $s" >&2; bad=1; continue; }
  git show "origin/main:${p}" > "$tmp/o" 2>/dev/null || : > "$tmp/o"
  if cmp -s "$s" "$tmp/o"; then echo "$p: same as origin/main, nothing to publish" >&2; bad=1; continue; fi
  git show "${mb}:${p}" > "$tmp/b" 2>/dev/null || : > "$tmp/b"
  if ! cmp -s "$tmp/b" "$tmp/o"; then
    # 3-way: origin's changes since the merge-base must already be in the content
    git merge-file -p "$s" "$tmp/b" "$tmp/o" > "$tmp/m" 2>/dev/null
    rc=$?
    stale=0
    if [ $rc -gt 127 ]; then
      echo "$p: stale check failed (git merge-file rc=$rc)" >&2; bad=1
    elif [ $rc = 0 ]; then
      cmp -s "$tmp/m" "$s" || { stale=1; why="lacks"; }
    else
      # overlap: origin's changes outside the conflicting hunks must still be in the content
      git merge-file -p --ours "$s" "$tmp/b" "$tmp/o" > "$tmp/m" 2>/dev/null
      if ! cmp -s "$tmp/m" "$s"; then
        stale=1; why="lacks"
      elif [ -s "$tmp/b" ]; then
        # in the conflicting hunks: stale if the content is closer to the base than to origin
        d_o=$(git diff --no-index --numstat "$tmp/o" "$s" | awk '{n += $1 + $2} END {print n + 0}')
        d_b=$(git diff --no-index --numstat "$tmp/b" "$s" | awk '{n += $1 + $2} END {print n + 0}')
        [ "$d_o" -lt "$d_b" ] || { stale=1; why="changes the same lines as"; }
      fi
    fi
    if [ $stale = 1 ]; then
      echo "$p: $why changes origin/main made since this checkout's base (stale base?)." >&2
      echo "  copy origin's version (git show origin/main:$p > /tmp/x), re-apply your edit there and" >&2
      echo "  publish with --from /tmp/x=$p; --force if the overlap is intended" >&2
      [ $force = 1 ] || bad=1
    fi
  fi
  [ "$s" = "$top/$p" ] && ! git diff --quiet HEAD -- "$p" 2>/dev/null && [ -x ~/dev/bin/who-touched ] &&
    ~/dev/bin/who-touched "$p" --since 24h 2>/dev/null | grep -v '^no edits' | sed "s|^|  $p: |" | head -5
  echo "$p: $(git diff --no-index --numstat "$tmp/o" "$s" | awk '{print "+"$1" -"$2}') vs origin/main"
done
[ $bad = 0 ] || exit 1

mode_of() { # executable if it is on origin or the source is, else 100644
  local m
  m=$(git ls-tree origin/main -- "$1" | awk '{print $1}')
  if [ "$m" = 100755 ] || [ -x "$2" ]; then echo 100755; else echo 100644; fi
}

mkcommit() { # $1 = parent
  export GIT_INDEX_FILE="$tmp/index"
  rm -f "$GIT_INDEX_FILE"
  git read-tree "$1" || return 1
  for i in "${!paths[@]}"; do
    git update-index --add --cacheinfo "$(mode_of "${paths[$i]}" "${srcs[$i]}"),$(git hash-object -w "${srcs[$i]}"),${paths[$i]}" ||
      return 1
  done
  git commit-tree "$(git write-tree)" -p "$1" -m "$msg"
  local rc=$?; unset GIT_INDEX_FILE; return $rc
}

build_site() { # $1 = commit
  [ $build = 1 ] || return 0
  git worktree add -q --detach "$tmp/wt" "$1" || return 1
  # a copy, not symlinks: symlinked node_modules resolve back into this checkout and build its sources
  cp -c -R website/node_modules "$tmp/wt/website/" 2>/dev/null || cp -R website/node_modules "$tmp/wt/website/" || return 1
  (cd "$tmp/wt/website" && npx docusaurus build --out-dir "$tmp/site" > "$tmp/build.log" 2>&1)
  local rc=$?
  cp "$tmp/build.log" /tmp/docs-publish-build.log
  git worktree remove --force "$tmp/wt"
  rm -rf "$tmp/site"
  if [ $rc != 0 ]; then echo "docs build failed (rc=$rc), log: /tmp/docs-publish-build.log" >&2; tail -20 /tmp/docs-publish-build.log >&2; return 1; fi
  grep -i 'warn' /tmp/docs-publish-build.log | grep -v -e ExperimentalWarning -e trace-warnings | head -10
  echo "docs build ok ($(git rev-parse --short "$1"))"
}

base=$(git rev-parse origin/main)
new=$(mkcommit "$base") || { echo "commit failed" >&2; exit 1; }
git diff --stat "$base" "$new"
build_site "$new" || exit 1

if [ $dry = 1 ]; then
  [ $build = 1 ] && echo "dry run: built $new, not pushed" || echo "dry run: committed $new (not built), not pushed"
  exit 0
fi

pushed=0
for try in 1 2 3 4 5; do
  if git push -q origin "$new:refs/heads/main" 2>"$tmp/push.err"; then echo "pushed $new"; pushed=1; break; fi
  sleep $((try * 2))
  if git fetch -q origin; then
    git merge-base --is-ancestor "$new" origin/main && { echo "pushed $new (push reported an error, but it landed)"; pushed=1; break; }
  else
    continue
  fi
  [ $try = 5 ] && break
  over=$(git diff --name-only "$base" origin/main -- "${paths[@]}")
  [ -z "$over" ] || { echo "origin/main changed these files meanwhile, re-check them: $over" >&2; exit 1; }
  base=$(git rev-parse origin/main)
  new=$(mkcommit "$base") || exit 1
  build_site "$new" || exit 1
done
[ $pushed = 1 ] || { echo "push failed:" >&2; cat "$tmp/push.err" >&2; exit 1; }

[ $wait = 1 ] || exit 0
git diff --quiet "$base" "$new" -- website etc/config-tool/conf-editor.html etc/config-tool/dashboard.html \
  etc/config-tool/teledec.js etc/config-tool/pmini.js .github/workflows/docs.yml && { echo "no docs paths changed, no deploy to wait for"; exit 0; }
for i in $(seq 1 60); do
  git fetch -q origin 2>/dev/null
  # newest run that contains the commit; a cancelled run means a newer one superseded it
  st=$(gh run list --workflow Docs -L 10 --json headSha,status,conclusion 2>/dev/null |
    python3 -c '
import json, subprocess, sys
for r in json.load(sys.stdin):
    if subprocess.run(["git", "merge-base", "--is-ancestor", sys.argv[1], r["headSha"]]).returncode == 0:
        print(r["status"], r["conclusion"], r["headSha"][:8]); break
' "$new")
  case "$st" in
    "completed success"*) echo "deployed: $st"; exit 0 ;;
    "completed cancelled"*) ;;
    completed*) echo "deploy failed: $st" >&2; exit 1 ;;
  esac
  sleep 20
done
echo "deploy not finished after 20 min: ${st:-no run yet}" >&2; exit 1
