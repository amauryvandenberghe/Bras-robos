#!/bin/bash
cd "$(dirname "$0")"
if [ $# -gt 0 ]; then f=etude_chassis_diff_L$1.log; else f=etude_chassis_diff.log; fi
nice -n 10 python3 etude_chassis_diff.py "$@" 2>&1 | tee $f
