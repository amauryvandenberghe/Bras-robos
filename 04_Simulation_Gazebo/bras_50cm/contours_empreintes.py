"""Contours 2D : fond des empreintes (poches) des supports d'origine, et silhouettes de arm01 / broche (vues de dessus),
pour comprendre comment l'utilisateur fixait les palonniers. A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import cadquery as cq
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

EXP = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/'
SORTIE = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/palonniers/'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402


def aretes_xy(face):
    pts = []
    for e in face.Edges():
        ts = np.linspace(0, 1, 30)
        pts.append(np.array([(e.positionAt(t).x, e.positionAt(t).y) for t in ts]))
    return pts


fig, axs = plt.subplots(1, 5, figsize=(22, 5))
for ax, nom in zip(axs, ('support1', 'support_2', 'support_3')):
    s = cq.importers.importStep(EXP + nom + '.step').val()
    for f in s.Faces():
        n, c = f.normalAt(), f.Center()
        if f.geomType() == 'PLANE' and abs(n.z) > 0.99:
            col = 'r' if abs(c.z - 5) < 0.01 else ('0.7' if abs(c.z) < 0.01 else '0.4')
            for p in aretes_xy(f):
                ax.plot(p[:, 0], p[:, 1], color=col, lw=1)
    for f in s.Faces():
        if f.geomType() == 'CYLINDER':
            cy = f._geomAdaptor().Cylinder()
            l = cy.Location()
            bb = f.BoundingBox()
            print(f'{nom} : cylindre r={cy.Radius():.2f} centre ({l.X():.2f}, {l.Y():.2f}) z {bb.zmin:.1f}..{bb.zmax:.1f}')
    for f in s.Faces():
        n, c = f.normalAt(), f.Center()
        if f.geomType() == 'PLANE' and abs(c.z - 5) < 0.01 and n.z < -0.99:
            bb = f.BoundingBox()
            print(f'{nom} : fond d\'empreinte (z = 5, ouverte vers z = 0) aire {f.Area():.0f} mm2, '
                  f'x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f}')
    ax.set_aspect('equal'); ax.set_title(nom + ' (rouge : fond des poches z=5)'); ax.grid(alpha=0.3)

origine = cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, 'mg996r_origine.step')).val()
for ax, (nom, forme) in zip(axs[3:], (('arm01', origine.Solids()[6]),
                                      ('broche', cq.importers.importStep(EXP + 'broche.step').val()))):
    for f in forme.Faces():
        if f.geomType() == 'PLANE' and abs(f.normalAt().z) > 0.99:
            for p in aretes_xy(f):
                ax.plot(p[:, 0], p[:, 1], 'k', lw=0.6)
    bb = forme.BoundingBox()
    ax.set_title(f'{nom} : {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(SORTIE + 'contours.png', dpi=80)
print('->', SORTIE + 'contours.png')
