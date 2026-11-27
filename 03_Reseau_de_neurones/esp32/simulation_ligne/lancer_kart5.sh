#!/bin/bash
# 28/11 : robot kart, 5 capteurs en fleche (geometrie puis distance)
cd "$(dirname "$0")"
nice -n 10 python3 etude_kart5.py 2>&1 | tee etude_kart5.log
