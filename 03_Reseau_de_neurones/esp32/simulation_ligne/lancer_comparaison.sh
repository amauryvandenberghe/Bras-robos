#!/bin/bash
cd "$(dirname "$0")"
nice -n 10 python3 comparaison_kart_actuel.py 2>&1 | tee comparaison_kart_actuel.log
