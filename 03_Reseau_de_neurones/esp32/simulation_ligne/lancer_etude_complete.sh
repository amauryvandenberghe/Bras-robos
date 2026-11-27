#!/bin/bash
cd "$(dirname "$0")"
nice -n 10 python3 etude_complete.py 2>&1 | tee etude_complete.log
