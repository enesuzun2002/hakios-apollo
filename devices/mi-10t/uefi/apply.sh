#!/bin/bash
# Make ../Mu-Silicium-apollo: a clone of Project-Silicium/Mu-Silicium at the commit our port was made on,
# with the Mi 10T port (this directory) put into it. It is a tree of its own, apart from the POCO's
# Mu-Silicium/: this port's BootManager change differs. See README.md here for how to build it.
set -e
cd "$(dirname "$(readlink -f "$0")")"
MU=../../../Mu-Silicium-apollo
[ -e $MU ] && { echo "uefi/apply.sh: $MU already exists"; exit 1; }
git clone https://github.com/Project-Silicium/Mu-Silicium.git $MU
git -C $MU checkout -q "$(cat BASE)"
git -C $MU submodule update --init --recursive
git -C $MU apply --whitespace=nowarn "$PWD/mu-silicium.patch"
cp -r files/. $MU/
echo "uefi/apply.sh: done; next: devices/mi-10t/uefi/README.md"
