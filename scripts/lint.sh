#!/bin/bash
scripts/cpplint.py $(find src/ -name '*.cpp' -o -name '*.h')
ret=$?
if [ $ret -ne 0 ]; then
    echo -e "CI Lint Failures!!!";
    exit -1;
fi
echo -e "CI Lint Passed...";
exit 0;
