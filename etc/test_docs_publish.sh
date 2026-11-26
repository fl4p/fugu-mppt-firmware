#!/bin/bash
# Hermetic tests for etc/docs-publish.sh in a scratch repo (dry runs, no docs build). DOCS_PUBLISH=<path> tests another copy.
S=${DOCS_PUBLISH:-$(cd "$(dirname "$0")" && pwd)/docs-publish.sh}
fails=0
T=$(mktemp -d /tmp/dpt.XXXX); cd "$T" || exit 1
git init -q --bare -b main origin.git; git init -q -b main w; cd w; git remote add origin ../origin.git
git config user.email t@t; git config user.name t
seq 1 20 > a.txt; cp a.txt b.txt; cp a.txt c.txt; git add .; git commit -qm base; git push -q origin HEAD:main
git clone -q -b main ../origin.git ../o; cd ../o; git config user.email t@t; git config user.name t
sed -i '' 's/^10$/ten/; s/^2$/two/' a.txt; sed -i '' 's/^10$/ten/' b.txt; seq 100 130 > new.txt; git add .; git commit -qm o; git push -q origin HEAD:main
cd ../w; git fetch -q origin
run() { local want=$1; shift; out=$("$S" -m t -n --no-build "$@" 2>&1); rc=$?; 
  if { [ $want = ok ] && [ $rc = 0 ]; } || { [ $want = refuse ] && [ $rc != 0 ]; }; then echo "PASS $want: $*"; else fails=$((fails+1)); echo "FAIL $want (rc=$rc): $*"; echo "$out" | sed 's/^/    /'; fi; }
# T1 stale: lacks origin's change
seq 1 20 | sed 's/^1$/one/' > a.txt; run refuse a.txt
# T2 fresh: origin + edit
git show origin/main:a.txt | sed 's/^1$/one/' > a.txt; run ok a.txt
# T4 (finding 1): conflict on line 2, but reverts origin's line 10
seq 1 20 | sed 's/^2$/TWO/' > a.txt; run refuse a.txt
# T5 stale overlap: same line as origin, rest base, closer to base
seq 1 20 | sed 's/^10$/TEN/' > b.txt; run refuse b.txt
# T5b same, --force
run ok --force b.txt
# T6 (finding 3): file new on origin since merge-base, big rewrite
seq 500 540 > new.txt; run ok new.txt
# T3 relative --from from subdir
git checkout -q -- a.txt b.txt; mkdir -p sub; git show origin/main:b.txt | sed 's/^3$/three/' > sub/x; cd sub; run ok --from x=b.txt; cd ..
# mode
git show origin/main:c.txt | sed 's/^4$/four/' > c.txt; chmod +x c.txt; out=$("$S" -m t -n --no-build c.txt 2>&1); n=$(echo "$out" | grep -o 'committed [0-9a-f]*' | cut -d' ' -f2)
[ "$(git ls-tree "$n" c.txt | cut -c1-6)" = 100755 ] && echo "PASS mode 100755" || { fails=$((fails+1)); echo "FAIL mode"; echo "$out"; }
# push failure must not report success
git remote set-url --push origin /nonexistent; out=$("$S" -m t --no-build --no-wait c.txt 2>&1); rc=$?
[ $rc != 0 ] && echo "$out" | grep -q 'push failed' && echo "PASS push failure -> rc=$rc" || { fails=$((fails+1)); echo "FAIL push failure rc=$rc"; echo "$out"; }
rm -rf "$T"
exit $fails
