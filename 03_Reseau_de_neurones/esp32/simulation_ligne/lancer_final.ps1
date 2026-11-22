wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u -W ignore sim_ligne2.py final 70 2>&1 | tee reglages_finaux.log"
