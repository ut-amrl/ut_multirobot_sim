#!/bin/bash

diff-lines() {
    local path=
    local line=
    while read; do
        esc=$'\033'
        if [[ $REPLY =~ ---\ (a/)?.* ]]; then
            continue
        elif [[ $REPLY =~ \+\+\+\ (b/)?([^[:blank:]$esc]+).* ]]; then
            path=${BASH_REMATCH[2]}
        elif [[ $REPLY =~ @@\ -[0-9]+(,[0-9]+)?\ \+([0-9]+)(,[0-9]+)?\ @@.* ]]; then
            line=${BASH_REMATCH[2]}
        elif [[ $REPLY =~ ^($esc\[[0-9;]+m)*([\ +-]) ]]; then
            echo "$path:$line"
            if [[ ${BASH_REMATCH[2]} != - ]]; then
                ((line++))
            fi
        fi
    done
}

rm -f /tmp/diff_lines_file;
touch /tmp/diff_lines_file
git diff -U0 | diff-lines >> /tmp/diff_lines_file
git diff --staged -U0 | diff-lines >> /tmp/diff_lines_file

if git rev-parse --verify HEAD >/dev/null 2>&1; then
    against=HEAD
else
    exit 1
fi

filters='-build/include_order,-build/namespaces,-legal/copyright,-runtime/references'

rm -f /tmp/lint_output_file;
touch /tmp/lint_output_file
for file in $(git diff-index --name-status $against -- | grep -E '\.[ch](pp)?$' | awk '{print $2}'); do
    echo $file
    scripts/cpplint.py --filter=$filters $file >> /tmp/lint_output_file 2>&1
done

rm -f /tmp/diff_errors_found
cat /tmp/diff_lines_file | while read diff_line; do
    cat /tmp/lint_output_file | while read lint_line; do
        if [[ $lint_line == $diff_line* ]]; then 
            echo $lint_line
            touch /tmp/diff_errors_found
        fi
    done
done

if [[ -f "/tmp/diff_errors_found" ]]; then
    exit 1
fi
