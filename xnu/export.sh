#!/bin/bash
# Save the kernel work as it is in the working tree, in the two forms apply.sh puts back:
#   hakios.patch   the changes to Apple's files (against the tag the work sits on)
#   files/          the files that are not in Apple's tree, at their paths
# A phone's own work stays in its directory, devices/<name>/xnu/: its patch (kept by hand; it may change
# Apple's files and those of files/ too) is taken back out of what is saved here, and a new file that is
# already in its files/ is saved there, not in xnu/files/ (to add a driver of that phone, put a first copy
# of it in devices/<name>/xnu/files/ before exporting).
# usage: xnu/export.sh [KERNEL-TREE]     (default: $XNU_SRC, else xnu-src/xnu-8796.121.2)
# Run it before committing whenever the kernel tree has changed. Committed or not makes no
# difference there: everything is compared with the base commit.
set -e
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC=${1:-${XNU_SRC:-$HERE/../xnu-src/xnu-8796.121.2}}
BASE=c54f35ca767986246321eb901baf8f5ff7923f6a           # tag xnu-8796.121.2
# never part of the work: build output and what editors, patch and python leave behind
JUNK='^BUILD/|(^|/)__pycache__/|\.(orig|rej|pyc|o|swp)$|\.bak[0-9]*$|~$|(^|/)\.DS_Store$'

cd "$SRC"
git cat-file -e "$BASE^{commit}" 2>/dev/null || { echo "export.sh: $SRC is not a clone of xnu at $BASE" >&2; exit 1; }

# no renames: a moved file is a deletion in the patch and a new file in files/
# both are taken from a scratch index: the working tree with the phones' own patches taken back out
INDEX=$(mktemp)
trap 'rm -f "$INDEX"' EXIT
GIT_INDEX_FILE=$INDEX git read-tree "$BASE"
GIT_INDEX_FILE=$INDEX git add -A -- .
for p in "$HERE"/../devices/*/xnu/*.patch; do
	if [ -s "$p" ]; then GIT_INDEX_FILE=$INDEX git apply --cached -R --whitespace=nowarn "$p"; fi
done
GIT_INDEX_FILE=$INDEX git -c core.abbrev=12 diff --cached --binary --no-renames --no-color --no-ext-diff \
	--diff-filter=MDT "$BASE" -- . > "$HERE/hakios.patch.new"
mv "$HERE/hakios.patch.new" "$HERE/hakios.patch"

rm -rf "$HERE/files.new"
mkdir "$HERE/files.new"
{ git diff --no-renames --name-only --diff-filter=A "$BASE" -- .; git ls-files --others --exclude-standard; } |
	sort -u | grep -Ev "$JUNK" > "$HERE/files.list"
while IFS= read -r f; do
	[ -f "$f" ] || [ -L "$f" ] || continue      # added in a commit, deleted since
	dest=
	for d in "$HERE"/../devices/*/xnu; do
		[ -e "$d/files/$f" ] && { dest=$d/files; break; }
	done
	if [ -n "$dest" ]; then                     # a phone's own file, as it is
		mkdir -p "$dest/$(dirname "$f")"
		if [ "$(uname)" = Darwin ]; then cp -pP "$f" "$dest/$f"; else cp -P --preserve=mode "$f" "$dest/$f"; fi
	else                                        # the common work's, without what the phones' patches change in it
		GIT_INDEX_FILE=$INDEX git checkout-index --prefix="$HERE/files.new/" -- "$f"
	fi
done < "$HERE/files.list"
rm -rf "$HERE/files" "$HERE/files.list"
mv "$HERE/files.new" "$HERE/files"

echo "xnu/hakios.patch: $(grep -c '^diff --git' "$HERE/hakios.patch") files changed, $(wc -c < "$HERE/hakios.patch") bytes"
echo "xnu/files: $(find "$HERE/files" -type f | wc -l) new files, $(du -sk "$HERE/files" | cut -f1) KB"
