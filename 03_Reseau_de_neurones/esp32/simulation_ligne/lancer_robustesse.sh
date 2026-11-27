#!/bin/bash
cd "$(dirname "$0")"
( echo "##### reglages pour 0,8 m/s"; nice -n 10 python3 etude_robustesse.py 0.8 "0.025,0,0.002,0" "1.6,1.0,0.05,0.04,255"; echo; echo "##### reglages pour 1 m/s"; nice -n 10 python3 etude_robustesse.py 1.0 "0.025,0,0.002,1" "1.6,3.5,0.05,0.04,170" ) 2>&1 | tee etude_robustesse.log
