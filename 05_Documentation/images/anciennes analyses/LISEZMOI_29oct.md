# Bras robotisé 50 cm : tout le projet au même endroit

Bras à 5 axes posé sur le robot roulant (200 × 200 mm, dessus à 80 mm). Il prend les balles de Ø23 mm sur la réglette et les dépose :
- dans les deux réceptacles de 15 cm de haut ;
- puis dans le grand réceptacle, dont le bord est à **50 cm**.

Un réseau de neurones pilote les 4 servos du bras et la pince. Le code est prêt pour un ESP32.

**À lire en premier : `Recapitulatif_bras_robotise.pdf`**. Il contient tout le projet, et sa page 1 donne les commandes pour lancer la simulation.

## Contenu
| Dossier | Ce qu'il contient |
|---|---|
| `01_SolidWorks` | Pièces neuves (socle, plateau tournant, bras de 25 cm, avant-bras, brides du coude et du poignet) en `.SLDPRT` et `.STEP`. |
| | `Bras 50cm.SLDASM` : assemblage **contraint** de tout le bras (socle → pince), servos et palonniers compris. Chaque pivot est libre en rotation. |
| | Servos : `Servo MG996R` (lacet, épaule), `Servo MG90S` (coude, poignet, pince), `Palonnier MG996R`, tirés de vos modèles. |
| | `Version precedente (sans servos)` et `Ancienne version (sans contraintes)` : versions d'avant. |
| | `Pieces d'origine (copies)` : vos pièces et vos servos d'origine. |
| `02_Impression_3D` | STL des pièces neuves, en mm : socle, plateau, bras, avant-bras, bride du coude, bride du poignet. |
| `03_Reseau_de_neurones` | `modele/` : réseau entraîné (`politique_bras.pt`), historique, résultats Gazebo. |
| | `esp32/bras_esp32/` : programme Arduino pour ESP32 ; `esp32/verification_code_C.txt` : comparaison du code C et de Python. |
| | `code_python/` : entraînement, expert, simulateurs, tests Gazebo. |
| `04_Simulation_Gazebo` | URDF du bras, maillages, scène à trois réceptacles, lancement ROS 2, scripts CAO et dimensionnement. |
| `05_Documentation` | Images, notes détaillées (`LISEZMOI_reseau.md`). |

## Le bras
| Axe | Servo | Couple statique maxi | Part du couple de blocage |
|---|---|---|---|
| Lacet (±90°) | MG996R | 0 (pas de gravité) | 0 % |
| Épaule, bras 25 cm | MG996R | 3,3 kg·cm | 35 % |
| Coude, avant-bras 12,4 cm | SG90 (MG90S conseillé) | 1,0 kg·cm | 55 % |
| Poignet | SG90 | 0,2 kg·cm | 10 % |
| Pince | MG90S | – | – |

- **Fixations** : servos fixés par vis M3 et écrous logés dans des empreintes. Les SG90 du coude et du poignet sont plaqués par une bride avec 2 vis M3, car leurs pattes ont des trous de 2 mm.
- **Logements** : cotés sur vos modèles SolidWorks des servos, et vérifiés par calcul. Aucune interférence, ni servo en place, ni en tournant les joints (`04_Simulation_Gazebo/bras_50cm/verification_logements.txt`).
- **5 axes plutôt que 6** : un roulis de poignet ajouterait 15 g au bout du bras. Le coude passerait de 55 % à 66 % de son couple, sans rien apporter à la tâche.
- **Impression** : PLA, 4 périmètres, 20 % de remplissage. Masses : socle 46 g, plateau 31 g, bras 40 g, avant-bras 18,5 g, brides 1,7 g.

## Branchements ESP32 (programme `bras_esp32.ino`)
- **Servos** : broches 13, 12, 14, 27 et 26 pour le lacet, l'épaule, le coude, le poignet et la pince.
- **Alimentation** : 5 à 6 V séparée, au moins 5 A, masse commune avec l'ESP32.
- **Arduino IDE** : bibliothèque **ESP32Servo**, carte « ESP32 Dev Module », partition **Huge APP (3MB No OTA)**. Les poids du réseau occupent 1,1 Mo de mémoire flash.
- **Entrées sur le port série**, envoyées par le module de vision : `C x y z g` pour le point de lâcher, puis `B x y z` pour la position de la balle à chaque image.
- **Sortie** : `D` quand la balle est déposée. Le bras se replie, et le suivi de ligne peut reprendre.
- **À régler sur le vrai bras** : `SENS` et `DECALAGE_DEG`, sens et calage de chaque palonnier.

## Attention : règlement du challenge
Le dossier TIPe impose **une seule carte Arduino Uno**. L'ESP32 doit donc être validé par le professeur référent du matériel. Le réseau actuel (280 000 poids) ne tient pas sur une Uno, qui n'a que 32 Ko de mémoire.
