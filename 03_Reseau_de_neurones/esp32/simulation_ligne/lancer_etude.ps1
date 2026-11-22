wsl -d Ubuntu-22.04 -- bash -c "nice -n 10 python3 -u sim_ligne2.py etude 60 2>&1 | grep -v Warn | tee etude_capteurs.log"
