"""Profil du MG90S autour de sa sortie : rayon de matiere a chaque hauteur (repere du modele, z vers la sortie).
A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402

s = C.modele_servo('mg90s')
print('repere du modele : axe de sortie en (0, 0), bout de cannelure z =', C.SG_REF_Z, ', dessus du boitier z =', C.SG_DESSUS)
for z in np.arange(-9.0, -3.0, 0.25):
    sec = s.intersect(C.boite(-15, 15, -15, 15, z - 0.01, z + 0.01))
    if sec.Volume() <= 0:
        print(f'  z = {z:6.2f} : rien')
        continue
    # rayon maxi de la matiere a moins de 8 mm de l'axe (bossage / cannelure), et presence pres de l'axe
    pts = []
    for f in sec.Faces():
        for v in f.Vertices():
            pts.append((v.X, v.Y))
    pts = np.array(pts)
    r = np.hypot(pts[:, 0], pts[:, 1])
    pres = r[r < 9]
    print(f'  z = {z:6.2f} : aire {sec.Volume() / 0.02:7.1f} mm2, rayon maxi pres de l\'axe {pres.max() if len(pres) else 0:5.2f} mm')
