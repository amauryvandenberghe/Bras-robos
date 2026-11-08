"""Ou la pince (maillage d'origine) touche-t-elle l'avant-bras quand le poignet tourne ?"""
import os
import sys
import numpy as np
import cadquery as cq
import trimesh
from scipy.spatial.transform import Rotation
ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
import cao_bras as C  # noqa: E402
from cinematique import Bras  # noqa: E402


def maillage(forme, tol=0.2):
    v, f = forme.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in v]), np.array(f))


b = Bras()
j4 = [j for j in b.joints.values() if j['nom'] == 'joint_base_bras_4'][0]
formes = {'avant_bras': cq.importers.importStep(os.path.join(C.SORTIE, 'avant_bras.step')).val(),
          'bride_poignet': cq.importers.importStep(os.path.join(C.SORTIE, 'bride_poignet.step')).val(),
          'servo_poignet': C.enveloppe_servo('servo_poignet')}
mail = {n: maillage(f) for n, f in formes.items()}
pince = trimesh.load(os.path.join(ICI, '..', 'meshes', 'bras_quatre.stl'))
pince.apply_scale(1000)
morceaux = sorted(pince.split(only_watertight=False), key=lambda c: -len(c.faces))
noms = {0: 'servo de pince', 2: 'corps de pince', 3: 'plateau du palonnier'}
for q in (-0.41, 0.0, 0.76, 1.55):
    M = j4['T'].copy(); M[:3, 3] *= 1000
    D = np.eye(4); D[:3, :3] = Rotation.from_rotvec(np.array(j4['axe']) * q).as_matrix()
    M = M @ D
    for k, mc in enumerate(morceaux[:5]):
        pts = mc.sample(3000)
        p = pts @ M[:3, :3].T + M[:3, 3]
        for n, m in mail.items():
            d = trimesh.proximity.signed_distance(m, p)
            dedans = d > 0.05
            if dedans.any():
                a, z = p[dedans].min(0), p[dedans].max(0)
                ac, zc = pts[dedans].min(0), pts[dedans].max(0)
                print(f'q4={q:5.2f} {noms.get(k, "morceau %d" % k):20s} dans {n:13s}: {dedans.sum():4d} pts, '
                      f'{d.max():.2f} mm ; avant-bras x[{a[0]:.1f} {z[0]:.1f}] y[{a[1]:.1f} {z[1]:.1f}] z[{a[2]:.1f} {z[2]:.1f}]'
                      f' ; pince x[{ac[0]:.1f} {zc[0]:.1f}] y[{ac[1]:.1f} {zc[1]:.1f}] z[{ac[2]:.1f} {zc[2]:.1f}]')
