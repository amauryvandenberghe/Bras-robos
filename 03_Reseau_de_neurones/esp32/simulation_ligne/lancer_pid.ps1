wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u -W ignore sim_ligne2.py pid 60 2>&1 | tee reglages_pid.log"
