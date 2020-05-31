#!/bin/bash
pushd `git rev-parse --show-toplevel`
make -j`nproc`
res=$?
popd
exit $res
