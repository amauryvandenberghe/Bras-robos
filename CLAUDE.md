# Projet « Bras robotisé 50 cm » : mémo de démarrage pour Claude

Version courte, chargée à chaque session. Tout le détail (géométrie, IA, ESP32, essais réels, décisions, pièges SolidWorks, commandes) est dans **`HISTORIQUE.md`**.
Dernière mise à jour : 09/11/2026.

## Comment travailler (économie de tokens)
1. **Question sur le code** : interroger d'abord le graphe (`graphify query "..."`, `graphify path "A" "B"`, `graphify explain "X"`, ou les outils Graphify en ligne sur `amauryvandenberghe/Bras-robos`). Ne lire ensuite que les lignes localisées, pas des fichiers entiers.
2. **Question sur une décision, un essai, un piège, une cote** : chercher par mot-clé (Grep) dans `HISTORIQUE.md`. Ne pas le lire en entier. Sections : 1 projet, 3 emplacements, 4 environnement et pièges, 5 bras, 6 IA et simulation, 7 ESP32, 8 localisation, 9 budget et règlement, 10 commandes, 11 décisions, 12 reste à faire et essais réels.
3. **Fin de session** : ajouter les nouveautés en fin de section 12 de `HISTORIQUE.md`. Ne modifier ce fichier que pour ce qui change le travail de toutes les sessions.
4. **Commits** : après chaque modification de ce dossier, `git add -A` puis `git commit -m "<résumé en français>"`. Le hook `.githooks/post-commit` met alors à jour le graphe local (`graphify update .`) et pousse vers GitHub (Graphify en ligne) tout seul. Si le hook ne s'exécute pas : `git config core.hooksPath .githooks`.

## Préférences de l'utilisateur (à respecter)
- Répondre **en français**. Travailler vite mais bien, sans attendre de validation quand la tâche est claire.
- Tâches longues (entraînement, tests Gazebo) dans des **onglets Terminal visibles** (`run_in_terminal`), pas en arrière-plan caché.
- PC portable (i5-1334U, 12 cœurs logiques, 7,6 Go dans WSL) : `nice -n 10`, **4 à 6 ouvriers**, Gazebo **sans fenêtre** (`gui:=false`) pour les tests.
- Démo **avec** fenêtre Gazebo : c'est l'utilisateur qui lance Gazebo lui-même et qui dit quand c'est prêt.
- Ne jamais saisir de clés API. Pas d'accès au bureau ICAM (Citrix).
- Explications concrètes et bilan honnête (ce qui est vérifié / ce qui ne l'est pas).

## Le projet en bref
- Challenge **TIPe PT** de l'ICAM (Lille), 2026-2027 : robot suiveur de ligne + **bras 5 axes** (lacet et épaule MG996R, coude, poignet, pince) qui prend des balles de 2 g et les dépose dans 3 réceptacles. Pilotage par un **réseau de neurones** (imitation + DAgger) entraîné en simulation, exporté en C pour un **ESP32**.
- **Code de référence : WSL** `~/ros2_ws/src/mon_bras_robot/` (`ia_50cm/`, `bras_50cm/`). Ce dossier du Bureau = livrable + copies. Le dépôt GitHub et Graphify ne contiennent **que le Bureau** ; pour le code actuel, vérifier le fichier WSL. La copie WSL de `CLAUDE.md` n'a pas été raccourcie.
- Versions à utiliser : `01_SolidWorks/1_VERSION_FINALE (a ouvrir)` ; `02_Impression_3D` (27 STL) ; `03_Reseau_de_neurones/modele/1_RESEAU_FINAL (a utiliser)` (`politique_finale.pt` dans WSL) ; programme ESP32 unique `03_Reseau_de_neurones/esp32/bras_robot/`.
- PDF `Recapitulatif_bras_robotise.pdf` : modifier `05_Documentation/source/Recapitulatif_bras_robotise.modele.html`, puis `python3 finaliser_document.py` (jamais le `.html` généré).

## État actuel (au 13/11)
- Bras monté, servos alimentés par un abaisseur de tension. Poignet probablement grillé (coupé : `ACTIF[3] = false`).
- Programme unique `esp32/bras_robot/` : `bras_robot.ino` + `servo.h`/`servo.cpp` (broches 21, 4, 13, 22, 16 pour base, épaule, coude, poignet, pince ; direction 17 ; calage et bornes 10–170°) + `reseau.h` (poids + calcul). **Ne jamais commander un servo hors 10–170° ni le laisser forcer en butée.** Ne pas utiliser la 23.
- En attente : plan coté du plateau, puis apprentissage « réglettes pleines ». Budget 78/60 hors ESP32 : c'est à l'utilisateur d'arbitrer.
- Programme ESP32 du parcours (suivi de ligne, repères) : ne pas l'écrire sans nouvelle demande.

## Pièges à connaître à chaque session (détails : `HISTORIQUE.md` section 4)
- WSL : `python3` = ROS 2 + PyTorch ; `~/venv_cao/bin/python` = CadQuery, trimesh, pymupdf. ROS : `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`.
- PowerShell → WSL : écrire un script `.sh`/`.py` plutôt que `wsl -- bash -c "..."`. Chemins avec « é » : dans un `.ps1` encodé en UTF-8 avec BOM.
- L'application Claude est un paquet MSIX : ce qu'elle installe dans `%APPDATA%` est invisible du terminal de l'utilisateur. Ses outils s'installent depuis son terminal.
- SolidWorks : le fermer avant un script. Ne jamais relancer `robot_v3.ps1`, `direction_sw.ps1`, `capteurs_sw.ps1` (historiques).
- `exporter_dossier.sh` et `exporter_esp32.py` écraseraient ou ne mettraient pas à jour `bras_robot` (nom `B1`) : à adapter avant usage.

## Dépôt GitHub et Graphify en ligne (depuis le 08/11)
- Ce dossier est relié à `https://github.com/amauryvandenberghe/Bras-robos` (branche `main`). Graphify en ligne (espace `amaury-s-workspace`) indexe le dépôt GitHub, pas le dossier local : il ne voit que ce qui est poussé. S'il n'a pas repris le dernier push : bouton **Sync** sur la page Repositories de app.graphify.com.
- Ne pas pousser de secrets. `graphify-out/` et les fichiers temporaires sont exclus par `.gitignore`.
- Réglages du dépôt : `core.autocrlf=false`, `core.filemode=false`, `core.hooksPath=.githooks` (fichiers enregistrés en 100755, ne pas « corriger »).

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- When graphify-out/graph.json exists, run `graphify query "<question>"` before your first grep, search or source-file read for a codebase question, and put this rule in every subagent prompt that explores code. Read raw files only for lines the graph or an earlier read already located. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
- A committed graphify-out/graph.json is rewritten whole on every build, so a large diff to it in a pull request is expected. Do not flag it as churn or suggest skipping the graph rebuild or its hooks.
