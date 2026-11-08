"""Regarde les pieces d'origine de l'utilisateur qui transmettent la rotation des servos : "broche" (SG90),
"Servo MG996R arm01" (MG996R) et les supports du bras d'origine (leur empreinte pour ces pieces).
Rendu de plusieurs vues + dimensions. A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import cadquery as cq
import matplotlib
import numpy as np
import trimesh

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

EXP = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/'
SORTIE = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/palonniers/'
os.makedirs(SORTIE, exist_ok=True)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402


def maillage(forme):
    v, f = forme.tessellate(0.05, 0.2)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in v]), np.array(f))


def rendu(nom, m, vues=((30, -60), (90, -90), (0, -90), (0, 0))):
    fig = plt.figure(figsize=(16, 4.2))
    for i, (el, az) in enumerate(vues):
        ax = fig.add_subplot(1, len(vues), i + 1, projection='3d')
        tri = m.vertices[m.faces]
        n = m.face_normals
        lum = np.clip(0.35 + 0.65 * np.abs(n @ np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])), 0, 1)
        ax.add_collection3d(Poly3DCollection(tri, facecolors=plt.cm.Oranges(0.3 + 0.6 * lum), edgecolor='none'))
        lo, hi = m.bounds
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(el, az)
        ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
        ax.set_title(f'{nom} (el {el}, az {az})', fontsize=8)
    fig.tight_layout()
    fig.savefig(SORTIE + nom + '.png', dpi=70)
    plt.close(fig)


def decrire(nom, forme):
    bb = forme.BoundingBox()
    print(f'== {nom} : {len(forme.Solids())} solide(s), volume {forme.Volume():.0f} mm3, boite x {bb.xmin:.2f}..{bb.xmax:.2f} '
          f'y {bb.ymin:.2f}..{bb.ymax:.2f} z {bb.zmin:.2f}..{bb.zmax:.2f}')
    # faces planes : position et aire (pour reperer epaisseurs, empreintes)
    plans = {}
    for f in forme.Faces():
        if f.geomType() == 'PLANE':
            n = f.normalAt()
            cle = (round(n.x, 2), round(n.y, 2), round(n.z, 2))
            c = f.Center()
            d = round(c.x * n.x + c.y * n.y + c.z * n.z, 2)
            plans.setdefault(cle, {}).setdefault(d, 0)
            plans[cle][d] += f.Area()
    for n, ds in plans.items():
        if abs(n[0]) + abs(n[1]) + abs(n[2]) > 0.99 and max(abs(x) for x in n) > 0.99:
            print(f'   plans normale {n} : ' + ', '.join(f'{d}({a:.0f})' for d, a in sorted(ds.items())))
    cyl = {}
    for f in forme.Faces():
        if f.geomType() == 'CYLINDER':
            r = round(f._geomAdaptor().Cylinder().Radius(), 2)
            cyl[r] = cyl.get(r, 0) + 1
    print('   rayons de cylindres (nombre de faces) :', dict(sorted(cyl.items())))


formes = {}
for nom in ('broche', 'support1', 'support_2', 'support_3', 'bati'):
    formes[nom] = cq.importers.importStep(EXP + nom + '.step').val()
origine = cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, 'mg996r_origine.step')).val()
sol = origine.Solids()
for i, s in enumerate(sol):
    bb = s.BoundingBox()
    print(f'MG996R SBORKA solide {i} : volume {s.Volume():.0f}, z {bb.zmin:.1f}..{bb.zmax:.1f}, x {bb.xmin:.1f}..{bb.xmax:.1f}')
formes['arm01'] = sol[6]
for nom, f in formes.items():
    decrire(nom, f)
    rendu(nom, maillage(f))
print('rendus dans', SORTIE)
