wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u -W ignore grille_vitesse.py 2>&1 | tee grille_vitesse.log"
