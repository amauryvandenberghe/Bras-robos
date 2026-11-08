#!/bin/bash
# Entrainement propre depuis zero (regles definitives). L'entrainement precedent est archive dans modeles/archive_<date>/.
cd "$(dirname "$0")"
ARCHIVE=modeles/archive_$(date +%Y%m%d_%H%M)
mkdir -p "$ARCHIVE"
mv modeles/*.pt modeles/*.npz modeles/*.json modeles/*.log "$ARCHIVE"/ 2>/dev/null
echo "ancien entrainement archive dans $ARCHIVE"
OUVRIERS=${OUVRIERS:-5} bash entrainer.sh
