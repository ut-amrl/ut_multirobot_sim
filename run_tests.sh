#!/bin/bash
# Run
mkdir test_run
mkdir evals

python evaluator/runner.py mpdm_demo /home/jaholtz/code/cpp-pips/synthd/dipsl3/ `ls scenarios`
mv evals test_run/go_alone
mkdir evals

python evaluator/runner.py mpdm_ref /home/jaholtz/code/cpp-pips/synthd/dipsl3/ `ls scenarios`
mv evals test_run/reference
mkdir evals

python evaluator/runner.py mpdm_synth /home/jaholtz/code/cpp-pips/synthd/nice_best/ `ls scenarios`
mv evals test_run/nice
mkdir evals

python evaluator/runner.py mpdm_synth /home/jaholtz/code/docker/sldips/experiments/greedy/ `ls scenarios`
mv evals test_run/greedy
