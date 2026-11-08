#!/bin/bash
# Rassemble tout le projet "bras 50 cm" dans Bureau\Bras robotise (a relancer apres chaque nouvel entrainement).
# Usage : bash exporter_dossier.sh [modele.pt]   (defaut : modeles/politique_bras.pt)
set -e
cd "$(dirname "$0")"
PKG=~/ros2_ws/src/mon_bras_robot
DEST="/mnt/c/Users/amaur/Desktop/Bras robotisé"
BUREAU=/mnt/c/Users/amaur/Desktop
MODELE=${1:-modeles/politique_bras.pt}

mkdir -p "$DEST"/{01_SolidWorks,02_Impression_3D,03_Reseau_de_neurones,04_Simulation_Gazebo,05_Documentation}

# 01 SolidWorks : pieces neuves + assemblage contraint (deja generes par bras_50cm/generer_assemblage_sw.py),
#    copies des pieces d'origine reprises (avant-bras, pince, servos)
# (01_SolidWorks/1_ASSEMBLAGE_FINAL (a utiliser) est ecrit par bras_50cm/generer_assemblage_sw.py + assembler.ps1)
ORIG="$DEST/01_SolidWorks/2_Pieces d'origine (copies de vos fichiers)"
mkdir -p "$ORIG"
for f in "Assemblage1.SLDASM" "bati.SLDPRT" "broche.SLDPRT" "petit engrenage pince.SLDASM" "pince.SLDPRT" \
         "pince_a_imprimer_deux_fois.SLDASM" "Servo Motor Micro MG90S.SLDPRT" "support 2.SLDPRT" "support 3.SLDPRT" \
         "support1.SLDPRT"; do
  [ -f "$BUREAU/$f" ] && cp -u "$BUREAU/$f" "$ORIG/"
done
[ -f "$BUREAU/ICAM/Pieces tipe/MG996R SBORKA.SLDASM" ] && cp -u "$BUREAU/ICAM/Pieces tipe/MG996R SBORKA.SLDASM" "$ORIG/"
[ -d "$BUREAU/ICAM/Pieces tipe/MG996R SERVO" ] && cp -ru "$BUREAU/ICAM/Pieces tipe/MG996R SERVO" "$ORIG/"
if [ -d "$BUREAU/Bras 50cm" ]; then                 # premiere version (assemblage sans contraintes) : rangee ici
  mkdir -p "$DEST/01_SolidWorks/3_Anciennes versions/3a_premiere version sans contraintes"
  cp -ru "$BUREAU/Bras 50cm/." "$DEST/01_SolidWorks/3_Anciennes versions/3a_premiere version sans contraintes/"
  rm -rf "$BUREAU/Bras 50cm"
fi

# 02 Impression 3D
cp -u $PKG/bras_50cm/pieces/{socle,plateau,bras,bride_coude,avant_bras,bride_poignet,corps_pince}.stl "$DEST/02_Impression_3D/"

# 03 Reseau de neurones : modele, carte ESP32, code d'entrainement
R="$DEST/03_Reseau_de_neurones"
# modele/1_RESEAU_FINAL : le seul reseau a utiliser (celui exporte pour l'ESP32) et ses tests Gazebo ;
# modele/2_Secours : l'ancien reseau final du 29/10 (rempli une fois, voir LISEZMOI.txt du dossier modele)
FINAL="$R/modele/1_RESEAU_FINAL (a utiliser)"
mkdir -p "$FINAL" "$R/code_python" "$R/esp32"
cp "$MODELE" "$FINAL/reseau_final.pt"
M_=modeles/mission
[ -f $M_/eval_gazebo_officielle_politique_mission_affine.json ] && cp $M_/eval_gazebo_officielle_politique_mission_affine.json "$FINAL/test_gazebo_scene_origine.json" || true
[ -f $M_/eval_gazebo_devant_politique_mission_affine.json ] && cp $M_/eval_gazebo_devant_politique_mission_affine.json "$FINAL/test_gazebo_balles_devant.json" || true
[ -f $M_/mission.log ] && cp $M_/mission.log "$FINAL/journal_apprentissage.log" || true
python3 -W ignore esp32/exporter_esp32.py "$MODELE" | grep -v '^\s*$' > "$R/esp32/verification_code_C.txt"
cp -r esp32/bras_esp32 "$R/esp32/"
cp esp32/exporter_esp32.py esp32/cerveau.h esp32/cerveau.cpp "$R/esp32/"
cp *.py *.sh LISEZMOI.md "$R/code_python/"

# 04 Simulation Gazebo (ROS 2 Humble) : modele du bras, scene, lancement, scripts CAO
S="$DEST/04_Simulation_Gazebo"
mkdir -p "$S/urdf" "$S/meshes/bras_50cm" "$S/worlds" "$S/launch" "$S/config" "$S/bras_50cm"
cp $PKG/urdf/bras_50cm.urdf "$S/urdf/"
cp $PKG/meshes/bras_50cm/*.stl "$S/meshes/bras_50cm/"
cp $PKG/meshes/{bras_trois,bras_quatre,vis_un_4,vis_deux_4,vis_trois_4}.stl "$S/meshes/"
cp $PKG/worlds/scene_trois_receptacles.world "$S/worlds/"
cp ~/ros2_ws/src/mon_bras_robot_moveit_config/launch/gazebo.launch.py "$S/launch/"
cp ~/ros2_ws/src/mon_bras_robot_moveit_config/config/ros2_controllers.yaml "$S/config/"
cp $PKG/bras_50cm/*.py $PKG/bras_50cm/*.ps1 "$S/bras_50cm/"
cp -r $PKG/bras_50cm/servos "$S/bras_50cm/"
cp $PKG/bras_50cm/pieces/masses.json $PKG/bras_50cm/pieces/verification_logements.txt "$S/bras_50cm/"

# 05 Documentation
D="$DEST/05_Documentation"
mkdir -p "$D/images/anciennes analyses"
cp -u $PKG/bras_50cm/pieces/*.png "$D/images/anciennes analyses/" 2>/dev/null || true
cp LISEZMOI.md "$D/LISEZMOI_reseau.md"
echo "dossier a jour : $DEST"
