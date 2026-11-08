"""Affine le reseau complete (modeles/mission/politique_mission.pt) avec les corrections de l'expert notees dans Gazebo
(modeles/mission/donnees_gazebo_mission.npz, ecrites par evaluer_mission_gazebo.py --collecter).
Comme l'affinage du reseau final (entrainer_imitation.py --affiner) : 15 epoques, lr 3e-4, corrections Gazebo x5.
Tous les exemples deja appris restent : anciens (x1), anciennes corrections Gazebo (x5), complement mission (x2).
Usage : OUVRIERS=6 nice -n 10 python3 -u -W ignore affiner_mission.py
Sortie : modeles/mission/politique_mission_affine.pt (+ bilan au simulateur rapide dans mission.log)
"""
import os

import numpy as np
import torch

import entrainer_imitation as ei
import entrainer_mission as em
from politique import Reseau

ICI = os.path.dirname(os.path.abspath(__file__))
M, D = os.path.join(ICI, 'modeles'), em.DOSSIER


def main():
    ei.OUVRIERS = em.OUVRIERS
    poids = torch.load(os.path.join(D, 'politique_mission.pt'), map_location='cpu')
    reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    reseau.load_state_dict(poids)
    parts = [(np.load(os.path.join(M, 'donnees.npz')), 1.0), (np.load(os.path.join(M, 'donnees_gazebo.npz')), 5.0),
             (np.load(os.path.join(D, 'donnees_mission.npz')), 2.0),
             (np.load(os.path.join(D, 'donnees_gazebo_mission.npz')), 5.0)]
    X = np.concatenate([p['X'] for p, _ in parts])
    Y = np.concatenate([p['Y'] for p, _ in parts])
    source = np.concatenate([np.full(len(p['X']), w) for p, w in parts]).astype(np.float32)
    em.journal(f'=== Affinage Gazebo du reseau complete : {len(parts[3][0]["X"])} nouvelles corrections Gazebo (x5), '
               f'{len(X)} exemples au total ===')
    perte = ei.entrainer(reseau, X, Y, epoques=15, lr=3e-4, poids_source=source)
    etat = {k: v.clone() for k, v in reseau.state_dict().items()}
    torch.save(etat, os.path.join(D, 'politique_mission_affine.pt'))
    b = em.bilan(em.jouer(etat, list(range(20_000_000, 20_000_000 + em.N_EVAL)), 0.0, False))
    orig = ei.bilan_eval(em.jouer_origine(etat))[0]
    em.journal(f'  erreur {perte:.4f} | essais mission {b} | scene d\'origine (240 essais) {orig * 100:.0f} % '
               f'-> modeles/mission/politique_mission_affine.pt')


if __name__ == '__main__':
    main()
