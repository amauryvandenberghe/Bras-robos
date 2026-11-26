wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u -W ignore grille_vitesse2.py 2>&1 | tee grille_vitesse2.log"
