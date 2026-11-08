# Bras 50 cm : 3 balles dans 3 réceptacles

Le bras actuel (`ia/`, réseau c4 à 93 %) reste intact. Ce dossier pilote le **nouveau bras** décrit dans
`../bras_50cm/` et `../urdf/bras_50cm.urdf`.

## Tâche (scène `worlds/scene_trois_receptacles.world`)
- Le bras est fixé au centre du dessus du robot roulant (200 × 200 mm, dessus à 80 mm).
- Une réglette de 5 trous au pas de 50 mm porte 3 balles de Ø23 mm.
- Deux petits réceptacles : demi-sphères de Ø100 mm, bord à 150 mm.
- Un grand réceptacle : demi-sphère de Ø200 mm, bord à **500 mm**, garé face au robot.
- Une balle par réceptacle, du réceptacle le plus proche des balles au plus éloigné.
- Comme sur le parcours (le robot se gare devant un réceptacle à la fois, bras replié entre deux) :
  - chaque balle part de la posture de repos ;
  - **seul le réceptacle visé est un obstacle**, le bras peut traverser les deux autres ;
  - dans Gazebo, ces deux autres sont affichés en fantômes (sans collision), avec les balles déjà déposées.
- Tout est tiré au hasard à chaque essai (`zone.py`).

## Le bras (dossier `../bras_50cm/`)
| Axe | Servo | Rôle |
|---|---|---|
| 1 | MG996R | lacet, ±90° |
| 2 | MG996R | épaule, bras de 25 cm |
| 3 | SG90 (MG90S conseillé) | coude, avant-bras actuel de 12,4 cm |
| 4 | SG90 | poignet |
| pince | MG90S | pince actuelle, inchangée |

Pièces neuves : socle, plateau tournant avec berceau d'épaule, bras de 25 cm, avant-bras, brides du coude et du poignet.
- Servos fixés par vis M3 et écrous logés dans des empreintes. Logements cotés sur les modèles SolidWorks fournis (`../bras_50cm/servos/`).
- Vérification par calcul : `../bras_50cm/verifier_logements.py`. Pas d'interférence, ni en place, ni sur la course du coude et du poignet.
- Le servo du poignet est tourné vers le coude : la paroi de la pince tourne autour de l'axe du poignet, à 17 mm et plus, du côté du bout.
- Fichiers `.SLDPRT`, `.STEP` et `.STL` dans **Bureau\Bras robotisé**, avec l'assemblage contraint `Bras 50cm.SLDASM` (servos compris).
- Dans Gazebo, l'avant-bras est encore l'ancien maillage : même cinématique, masse à 5 g près.

**Pourquoi 5 axes et pas 6** (étude `dimensionnement.py`) : le coude est le point faible.
- Un roulis de poignet ajouterait 15 g au bout du bras et ferait passer le coude de 53 % à 66 % de son couple de blocage, sans rien apporter à la tâche.
- La réglette est posée dans l'axe du robot : les mâchoires s'ouvrent sur le côté et ne touchent pas les balles voisines.

**Couples** (masses de l'URDF, `couples_urdf.py`) : épaule ≤ 3,3 kg·cm sur 9,4 (35 %), coude ≤ 1,0 kg·cm sur 1,8 (55 %), poignet ≤ 0,2 kg·cm.
- Dans Gazebo, les servos tiennent leurs postures les plus dures sans écart (`diag_servos.py`).
- Les limites de couple de l'URDF sont celles des vrais servos.
- Exception : la balle simulée pèse 50 g, car une balle de 2 g est éjectée par la pince dans Gazebo. Chaque servo reçoit donc en plus le couple de ces 48 g, au pire bras de levier.

## Comment le bras passe
L'expert (`expert.py`), dont le réseau apprend, suit toujours le même trajet :
1. Il ramène la pince à 8 cm de l'axe, à 26 cm de haut.
2. Il tourne à ce rayon : la colonne du grand réceptacle, à 11,5 cm au plus près, ne peut pas être heurtée.
3. Il ressort vers la cible.
4. Pour le grand réceptacle, il monte à 56 cm à ce rayon, puis avance pince inclinée de 24° vers le haut et lâche à 6 cm du centre.

## Commandes (dans `ia_50cm/`)
```bash
bash entrainer.sh                       # entraînement (simulateur rapide + DAgger)
bash gazebo_auto.sh 20                  # test Gazebo sans fenêtre, 20 séquences pilotées par le réseau
bash gazebo_auto.sh 5 --expert          # même chose, l'expert pilote
bash preparer_gazebo.sh                 # régénère URDF + scène après une modification
python3 essai_expert.py 40              # l'expert dans le simulateur rapide
python3 tolerance_vision.py modeles/politique_finale.pt 90   # precision de camera necessaire (3 mm : 99 %, 6 mm : 64 %)
```
Gazebo avec la fenêtre :
```bash
ros2 launch mon_bras_robot_moveit_config gazebo.launch.py robot:=bras_50cm monde:=scene_trois_receptacles
```

## Attention : règlement du challenge
Le dossier TIPe impose **une seule carte Arduino Uno** (32 Ko de mémoire flash, 2 Ko de RAM).
- Le réseau 512 × 512 compte environ 280 000 poids : il ne tient ni sur une Uno, ni sur un ESP32 sans PSRAM s'il reste en 32 bits.
- Il faudra soit un réseau beaucoup plus petit (environ 2 000 poids en 8 bits sur une Uno), soit faire valider l'ESP32 par le professeur référent du matériel.
