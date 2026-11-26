wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u -W ignore grille_pid_22b.py 2>&1 | tee grille_pid_22b.log"
