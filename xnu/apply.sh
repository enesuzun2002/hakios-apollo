#!/bin/bash
# Get Apple's XNU sources (the release that iOS 16.5 was built from) and put the HakiOS
# work on top of them: hakios.patch changes Apple's files, files/ holds the new ones.
# usage: xnu/apply.sh [DIR]              (default: $XNU_SRC, else xnu-src/xnu-8796.121.2 next to this directory)
# env:   XNU_URL   where to fetch from (default: Apple's repository on GitHub; an existing clone
#                  that has the tag will do: XNU_URL=file:///path/to/xnu)
# The result is a git tree on the branch "hakios" with the work uncommitted, which is how
# xnu/export.sh expects to find it. A directory that already exists is left alone. Each phone's own
# kernel work (devices/<name>/xnu/) goes on top.
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DIR=${1:-${XNU_SRC:-$HERE/../xnu-src/xnu-8796.121.2}}
URL=${XNU_URL:-https://github.com/apple-oss-distributions/xnu.git}
TAG=xnu-8796.121.2
BASE=c54f35ca767986246321eb901baf8f5ff7923f6a

[ -e "$DIR" ] && { echo "apply.sh: $DIR already exists; not touching it" >&2; exit 1; }
[ -s "$HERE/hakios.patch" ] && [ -d "$HERE/files" ] || { echo "apply.sh: no hakios.patch and files/ in $HERE" >&2; exit 1; }
mkdir -p "$(dirname "$DIR")"
DIR=$(cd "$(dirname "$DIR")" && pwd)/$(basename "$DIR")
git init -q "$DIR"
trap 'echo "apply.sh: failed; removing the incomplete $DIR" >&2; rm -rf "$DIR"' ERR
cd "$DIR"
git remote add origin "$URL"
git fetch -q --depth 1 origin "refs/tags/$TAG:refs/tags/$TAG"      # that one release, without the history
git checkout -q -b hakios "$TAG"
[ "$(git rev-parse HEAD)" = "$BASE" ] || { echo "apply.sh: $TAG is $(git rev-parse HEAD), expected $BASE" >&2; false; }
echo 'BUILD/' >> .git/info/exclude
git apply --whitespace=nowarn "$HERE/hakios.patch"
cp -a "$HERE/files/." .
# a phone's own kernel work, kept in its directory (devices/<name>/xnu/: a patch for Apple's files, files/ for
# its drivers), goes on top of the common work; xnu/export.sh puts it back there
for d in "$HERE"/../devices/*/xnu; do
	[ -d "$d" ] || continue
	for p in "$d"/*.patch; do if [ -s "$p" ]; then git apply --whitespace=nowarn "$p"; fi; done
	if [ -d "$d/files" ]; then cp -a "$d/files/." .; fi
	echo "apply.sh: + devices/$(basename "$(dirname "$d")")/xnu"
done
echo "apply.sh: $DIR: $TAG + $(grep -c '^diff --git' "$HERE/hakios.patch") changed and $(find "$HERE/files" -type f | wc -l) new files"
