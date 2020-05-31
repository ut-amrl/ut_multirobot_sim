#!/bin/bash
pushd `git rev-parse --show-toplevel`
make -j`nproc`
res=$?
popd
if [ $res -ne 0 ]; then
    echo -e "\n\n\n\n"
    echo "Build failed!"
    exit $res
fi

scripts/lint_diff.sh
res=$?
exit $res
