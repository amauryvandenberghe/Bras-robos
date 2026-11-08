#!/bin/bash
# Essais de prise : bash essai_prise.sh "masse:frottement" ...   ex. : bash essai_prise.sh 0.002:0.3 0.002:0.8
cd "$(dirname "$0")"
for reglage in "$@"; do
  m=${reglage%%:*}
  f=${reglage#*:}
  [ "$f" = "$reglage" ] && f=2.0
  echo "===== balle de $m kg, frottement $f ====="
  FROTTEMENT_BALLE=$f MASSE_BALLE=$m python3 creer_scene.py > /dev/null
  bash avec_gazebo.sh essai_prise.py 2 2>&1 | grep -E 'essai|plan'
done
python3 creer_scene.py > /dev/null       # scene par defaut
