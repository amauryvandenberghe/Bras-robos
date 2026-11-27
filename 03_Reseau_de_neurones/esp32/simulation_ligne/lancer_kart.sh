#!/bin/bash
# 28/11 : robot kart (voie 136 mm) : L essieu -> capteurs
cd "$(dirname "$0")"
VOIE=136 nice -n 10 python3 etude_chassis_diff.py 95 110 125 2>&1 | tee etude_kart_voie136.log
