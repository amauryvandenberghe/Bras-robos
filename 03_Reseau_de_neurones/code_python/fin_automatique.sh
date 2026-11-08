#!/bin/bash
# Enchaine tout seul la fin du travail, des que l'entrainement (simulateur rapide) est termine :
#   1. 20 sequences dans Gazebo pilotees par le reseau, l'expert note ses corrections (vraie physique) ;
#   2. affinage du reseau avec ces corrections ;
#   3. test officiel Gazebo (20 sequences, memes dispositions) du reseau avant et apres affinage ;
#   4. le meilleur devient modeles/politique_finale.pt et le dossier "Bras robotise" du Bureau est mis a jour.
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
echo "[$(date +%H:%M)] attente de la fin de l'entrainement..."
until grep -q "=== Fin" modeles/entrainement.log 2>/dev/null; do sleep 120; done
echo "[$(date +%H:%M)] entrainement termine : $(grep '=== Fin' modeles/entrainement.log | tail -1)"

bilan() {   # nombre de sequences reussies dans un fichier d'evaluation
  python3 -c "import json,sys; d=json.load(open(sys.argv[1])); s=d['sequences']; print(sum(x['reussite'] for x in s), len(s))" "$1"
}

cp modeles/politique_bras.pt modeles/politique_simulateur.pt
echo "[$(date +%H:%M)] 1. corrections de l'expert dans Gazebo"
bash gazebo_auto.sh 20 --collecter --part-expert 0 --graine 50000000 --modele modeles/politique_simulateur.pt

echo "[$(date +%H:%M)] 2. affinage avec les corrections Gazebo"
OUVRIERS=6 nice -n 10 python3 -u -W ignore entrainer_imitation.py --affiner

echo "[$(date +%H:%M)] 3. test officiel : reseau du simulateur"
bash gazebo_auto.sh 20 --graine 40000000 --modele modeles/politique_simulateur.pt
cp modeles/eval_gazebo.json modeles/eval_gazebo_simulateur.json
echo "[$(date +%H:%M)] 3. test officiel : reseau affine"
bash gazebo_auto.sh 20 --graine 40000000 --modele modeles/politique_affine.pt
cp modeles/eval_gazebo.json modeles/eval_gazebo_affine.json

read OK_S N_S < <(bilan modeles/eval_gazebo_simulateur.json)
read OK_A N_A < <(bilan modeles/eval_gazebo_affine.json)
if [ "$OK_A" -ge "$OK_S" ]; then MEILLEUR=politique_affine.pt; EVAL=eval_gazebo_affine.json
else MEILLEUR=politique_simulateur.pt; EVAL=eval_gazebo_simulateur.json; fi
cp modeles/$MEILLEUR modeles/politique_finale.pt
cp modeles/$EVAL modeles/eval_gazebo.json
RES="Test Gazebo (20 sequences de 3 balles, dispositions jamais vues) : reseau du simulateur $OK_S/$N_S, reseau affine $OK_A/$N_A -> retenu : $MEILLEUR"
echo "[$(date +%H:%M)] $RES"

echo "[$(date +%H:%M)] 4. mise a jour du dossier du Bureau"
bash exporter_dossier.sh modeles/politique_finale.pt
DEST="/mnt/c/Users/amaur/Desktop/Bras robotisé"
cp modeles/eval_gazebo_simulateur.json modeles/eval_gazebo_affine.json "$DEST/03_Reseau_de_neurones/modele/1_RESEAU_FINAL (a utiliser)/"
{ echo "$(date '+%d/%m/%Y %H:%M')"; echo "$RES"; echo; grep '>>>' modeles/entrainement.log; } \
  > "$DEST/RESULTATS.txt"
echo "[$(date +%H:%M)] termine"
