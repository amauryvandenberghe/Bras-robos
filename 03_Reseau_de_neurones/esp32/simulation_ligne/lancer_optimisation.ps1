wsl --cd . -- bash -c "nice -n 10 python3 -u sim_ligne.py optimiser 60 2>&1 | tee optimisation.log"
