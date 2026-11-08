#!/bin/bash
# Attend que fin_automatique.sh ait ecrit RESULTATS.txt (test Gazebo officiel termine), puis remplit le PDF recapitulatif.
cd "$(dirname "$0")"
DEST="/mnt/c/Users/amaur/Desktop/Bras robotisé"
touch /tmp/repere_document
echo "[$(date +%H:%M)] attente des resultats du test Gazebo officiel..."
until [ "$DEST/RESULTATS.txt" -nt /tmp/repere_document ] && [ -f modeles/politique_finale.pt ]; do sleep 60; done
sleep 120                                      # exporter_dossier.sh finit de copier
echo "[$(date +%H:%M)] $(head -2 "$DEST/RESULTATS.txt" | tail -1)"
python3 finaliser_document.py
echo "[$(date +%H:%M)] PDF a jour"
