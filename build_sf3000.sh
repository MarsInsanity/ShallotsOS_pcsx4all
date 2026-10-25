#!/bin/sh
# Build ShallotsOS's PCSX4ALL (build/pcsx4all) for the SF3000 family.
#
# This repository holds only the SF3000 / ShallotsOS files. The rest of the
# emulator is gameblabla/pcsx4all at the commit named in UPSTREAM: it is
# fetched once into upstream/, copied to build/, and these files are laid
# over it. Needs the SF3000 toolchain in ~/sf3000-work/sf3000toolchain (see
# Makefile.sf3000), git and make.
set -e
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
. "$here/UPSTREAM"

if [ ! -d "$here/upstream/.git" ]; then
    git clone -q "$UPSTREAM_URL" "$here/upstream"
fi
if ! git -C "$here/upstream" cat-file -e "$UPSTREAM_COMMIT^{commit}" 2>/dev/null; then
    git -C "$here/upstream" fetch -q origin
fi

rm -rf "$here/build"
mkdir -p "$here/build"
git -C "$here/upstream" archive "$UPSTREAM_COMMIT" | tar -x -C "$here/build"
cp -a "$here/src/." "$here/build/src/"
cp "$here/Makefile.sf3000" "$here/build/"

make -C "$here/build" -f Makefile.sf3000 -j"$(nproc 2>/dev/null || echo 4)" 2>&1 | grep -v '^CC \|^CXX ' || true
test -x "$here/build/pcsx4all"
ls -la "$here/build/pcsx4all"
