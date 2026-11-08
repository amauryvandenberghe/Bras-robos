"""Angles dont la tache a besoin, sur le vrai bras (URDF sans butees) : sert a choisir la fenetre de 180 degres
de chaque servo (BUTEES de generer_urdf.py), et verifie que toutes les poses sont atteignables.

Poses : prise au sol (pince verticale), lacher au-dessus des petits receptacles (bord a 15 cm), passage par
l'interieur (rayon 9 cm, 26 cm de haut), lacher au-dessus du grand receptacle (bord a 50 cm, pince ~horizontale).
Usage : python3 verifier_tache.py
"""
import os
import subprocess
import sys
import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
URDF_LARGE = '/tmp/bras_50cm_large.urdf'
subprocess.run([os.path.expanduser('~/venv_cao/bin/python'), os.path.join(ICI, 'generer_urdf.py'), '--large', URDF_LARGE],
               check=True, capture_output=True)
from cinematique import Bras  # noqa: E402

b = Bras(URDF_LARGE)
rng = np.random.default_rng(0)
poses = []
for _ in range(150):
    th = rng.uniform(-np.radians(85), np.radians(85))
    u = np.array([np.cos(th), np.sin(th)])
    r = rng.uniform(0.17, 0.36)
    poses += [('sol', np.r_[r * u, 0.0148], -1.0, 0.05), ('sol+6cm', np.r_[r * u, 0.075], -1.0, 0.05)]
    r = rng.uniform(0.20, 0.34)
    poses += [('petit', np.r_[r * u, 0.18], -1.0, 0.05), ('passage', np.r_[r * u, 0.21], -1.0, 0.05)]
    poses += [('interieur', np.r_[0.09 * u, 0.26], -1.0, 0.02)]
    th3 = rng.uniform(-np.radians(15), np.radians(15))
    u3 = np.array([np.cos(th3), np.sin(th3)])
    d = rng.uniform(0.215, 0.235)
    poses += [('grand_interieur', np.r_[0.09 * u3, 0.54], 0.0, 0.0),
              ('grand', np.r_[(d - 0.06) * u3, 0.54], 0.0, 0.3)]

par_type = {}
q = np.array([0, 0.5, 1.2, 0.5])
for nom, cible, approche, poids in poses:
    q2, err = b.ik(cible, q, approche_z=approche, poids_approche=poids)
    p, ap = b.outil(q2)
    # ecart d'inclinaison de la pince (degres) par rapport a l'inclinaison demandee
    ecart = abs(np.degrees(np.arcsin(np.clip(ap[2], -1, 1)) - np.arcsin(approche))) if poids else 0.0
    par_type.setdefault(nom, []).append((q2, err, ecart))
    if err < 1e-3:
        q = q2
tous = []
for nom, l in par_type.items():
    Q = np.array([x[0] for x in l]); E = np.array([x[1] for x in l]); A = np.array([x[2] for x in l])
    ok = E < 1e-3
    if not ok.any():
        print(f'{nom:16s} AUCUNE pose atteinte (erreur mini {E.min() * 1000:.1f} mm)')
        continue
    tous.append(Q[ok])
    print(f'{nom:16s} atteintes {ok.sum()}/{len(l)}  ecart d inclinaison moyen {A[ok].mean():5.1f} deg  '
          f'q min {np.round(Q[ok].min(0), 2)}  q max {np.round(Q[ok].max(0), 2)}')
Q = np.vstack(tous)
print('TOUT : q min', np.round(Q.min(0), 2), ' q max', np.round(Q.max(0), 2), ' etendue (deg)',
      np.round(np.degrees(Q.max(0) - Q.min(0))))
