"""Ou la broche touche-t-elle le servo MG90S ? (repere de la broche). A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import cadquery as cq
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402

nom = 'broche_coude'
d = C.PALONNIERS[nom]
P = C.pose_palonnier(nom)
Pi = np.linalg.inv(P)
M = Pi @ np.linalg.inv(C.T_joint(d['joint'])) @ C.SERVOS[d['servo']][2]     # modele du servo -> repere broche
servo = C._dans(M, C.modele_servo('mg90s'))
b = max(cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, 'broche.step')).val().Solids(), key=lambda s: s.Volume())
for i, s in enumerate(servo.Solids()):
    bb = s.BoundingBox()
    print(f'servo corps {i} : volume {s.Volume():.0f}, x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} '
          f'z {bb.zmin:.2f}..{bb.zmax:.2f}')
    v = b.intersect(s)
    if v.Volume() > 0.01:
        bb = v.BoundingBox()
        print(f'   -> commun {v.Volume():.1f} mm3 : x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} '
              f'z {bb.zmin:.2f}..{bb.zmax:.2f}')
bb = b.BoundingBox()
print(f'broche (corps principal) : z {bb.zmin:.2f}..{bb.zmax:.2f}')
# coupe : rayon de la matiere de la broche a differentes hauteurs
for z in (-2.7, -2.0, -1.0, -0.5, 0.0, 0.5):
    sec = b.intersect(C.boite(-20, 20, -20, 20, z - 0.01, z + 0.01))
    if sec.Volume() > 0:
        bs = sec.BoundingBox()
        print(f'  broche a z = {z:5.2f} : x {bs.xmin:.2f}..{bs.xmax:.2f}, aire {sec.Volume() / 0.02:.1f} mm2')
