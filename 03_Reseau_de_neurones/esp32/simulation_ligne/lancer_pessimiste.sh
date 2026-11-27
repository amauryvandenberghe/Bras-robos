#!/bin/bash
cd "$(dirname "$0")"
nice -n 10 python3 etude_pessimiste.py 2>&1 | tee etude_pessimiste.log
