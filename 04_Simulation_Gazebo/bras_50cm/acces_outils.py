"""Acces au montage : pour chaque vis, longueur libre derriere la tete pour l'outil, avec les seules pieces deja posees
a cette etape de la notice. Pour chaque ecrou, directions par lesquelles il peut entrer dans sa piece."""
import os
import sys
import numpy as np
import trimesh

P = os.path.expanduser('~/ros2_ws/src/mon_bras_robot/bras_50cm/')
sys.path.insert(0, P)
sys.path.insert(0, os.path.join(P, '..', 'ia_50cm'))
os.chdir(P)
import cao_bras as C  # noqa: E402
from cinematique import Bras, BRAS  # noqa: E402

b = Bras()
LIEN = dict(C.LIEN_PIECE, servo_lacet='base_link', servo_epaule='bras un', servo_coude='bras deux', servo_poignet='bras trois')
PATINS = ['patin_a', 'patin_b', 'patin_c']
HOME = [0, -0.189, 2.286, 1.17]
POSTURES = [HOME, [0, 0.0, 0.0, 0.0], [0, 0.8, 1.5, 0.0], [0, -0.5, 1.5, -1.0], [0, 0.5, 2.8, 1.5]]


def mesh(nom):
    if nom.startswith('servo'):
        v, f = C.enveloppe_servo(nom).tessellate(0.1, 0.3)
        return trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True)
    return trimesh.load(os.path.join('pieces', nom + '.stl'))


M = {n: mesh(n) for n in list(LIEN)}


def pose(lien, q):
    T = b.pose(lien, dict(zip(BRAS, q))).copy()
    T[:3, 3] *= 1000
    return T


def etape(nom, porteur):
    """Pieces deja presentes quand on serre cette vis (ordre de la notice)."""
    base = ['socle', 'servo_lacet']
    if nom.startswith('vis_lacet'):
        return ['socle', 'servo_lacet']
    if nom.startswith('vis_palonnier_lacet'):
        return ['plateau']
    if nom.startswith('vis_patin'):
        return base + ['plateau'] + PATINS
    if nom.startswith('vis_epaule'):
        return base + ['plateau', 'servo_epaule'] + PATINS
    if nom.startswith('vis_palonnier_epaule'):
        return ['bras']
    if nom.startswith('vis_axe_epaule'):
        return base + PATINS + ['plateau', 'servo_epaule', 'bras', 'axe_epaule']
    tronc = base + PATINS + ['plateau', 'servo_epaule', 'bras', 'axe_epaule']
    if nom.startswith('vis_bride_coude'):
        return tronc + ['servo_coude', 'bride_coude']
    if nom.startswith('vis_broche_coude'):
        return tronc + ['servo_coude', 'bride_coude', 'avant_bras']
    if nom.startswith('vis_plaque') or nom.startswith('vis_axe_coude'):
        return tronc + ['servo_coude', 'bride_coude', 'avant_bras', 'palier_coude']
    t2 = tronc + ['servo_coude', 'bride_coude', 'avant_bras', 'palier_coude']
    if nom.startswith('vis_bride_poignet'):
        return t2 + ['servo_poignet', 'bride_poignet']
    if nom.startswith('vis_broche_poignet'):
        return t2 + ['servo_poignet', 'bride_poignet', 'corps_pince']
    raise ValueError(nom)


def anneau(o, d, r, n=12):
    d = d / np.linalg.norm(d)
    a = np.cross(d, [0, 0, 1] if abs(d[2]) < 0.9 else [1, 0, 0])
    a /= np.linalg.norm(a)
    c = np.cross(d, a)
    pts = [o] + [o + r * (np.cos(t) * a + np.sin(t) * c) for t in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    return np.array(pts)


def libre(o, d, pieces, q, r=2.0, ignore=()):
    """Distance libre depuis o dans la direction d, entre les pieces, faisceau de rayon r."""
    mini, qui = 999.0, None
    pts = anneau(o, d, r)
    for n in pieces:
        if n in ignore:
            continue
        T = pose(LIEN[n], q)
        inv = np.linalg.inv(T)
        po = trimesh.transformations.transform_points(pts, inv)
        dl = inv[:3, :3] @ d
        loc, ray, _ = M[n].ray.intersects_location(po, np.tile(dl, (len(po), 1)), multiple_hits=False)
        if len(loc):
            dist = np.linalg.norm(loc - po[ray], axis=1)
            k = dist.min()
            if k < mini:
                mini, qui = k, n
    return mini, qui


print('=== VIS : longueur libre derriere la tete, outil de 4 mm (meilleure posture du bras) ; < 40 mm = difficile')
res = {}
for nom, typ, porteur, T in C.visserie():
    if typ.startswith('Ecrou') or typ.startswith('Rondelle'):
        continue
    h = 1.5 if typ == 'Vis 2x8' else 3.0
    pres = etape(nom, porteur)
    best = (-1, None, None)
    for q in POSTURES:
        Tw = pose(LIEN[porteur], q) @ T
        z = Tw[:3, 2]
        tete = Tw[:3, 3] - h * z
        # on part de 0,2 mm derriere la tete, en sens oppose a l'enfoncement
        d, qui = libre(tete - 0.2 * z, -z, pres, q)
        if d > best[0]:
            best = (d, qui, q)
    res[nom] = best
    print(f'  {nom:26s} {typ:10s} libre {min(best[0], 150):6.1f} mm  bute sur {best[1]}')

print('\n=== VIS CENTRALE DU MG996R DE LACET : a travers le plateau (epaule pas encore posee)')
T0 = pose('bras un', HOME)
for r in (1.5, 2.0, 2.75):
    pts = anneau(np.array([0, 0, 120.0]), np.array([0, 0, -1.0]), r)
    loc, ray, _ = M['plateau'].ray.intersects_location(pts, np.tile([0, 0, -1.0], (len(pts), 1)), multiple_hits=False)
    print(f'  rayon {r} : ', 'aucune matiere du plateau sur l\'axe' if not len(loc) else f'matiere a z = {loc[:, 2].max():.1f} (plateau, repere propre)')
Tp = C.pose_palonnier('palonnier_lacet')
print('  palonnier de lacet a z =', round(float(Tp[2, 3]), 2))

print('\n=== ECROUS : directions libres dans la piece qui le recoit (faisceau r = 3,2 = ecrou), au-dela de 12 mm')
DIRS = {'+x': (1, 0, 0), '-x': (-1, 0, 0), '+y': (0, 1, 0), '-y': (0, -1, 0), '+z': (0, 0, 1), '-z': (0, 0, -1)}
for nom, typ, porteur, T in C.visserie():
    if not typ.startswith('Ecrou') or porteur not in M:
        continue
    ok = []
    for k, d in DIRS.items():
        dl = np.array(d, float)
        pts = anneau(T[:3, 3], dl, 3.2)
        loc, ray, _ = M[porteur].ray.intersects_location(pts, np.tile(dl, (len(pts), 1)), multiple_hits=False)
        dist = 999 if len(loc) < 1 else np.linalg.norm(loc - pts[ray], axis=1).min()
        if dist > 12:
            ok.append(k)
    print(f'  {nom:22s} dans {porteur:12s} : {" ".join(ok) if ok else "AUCUNE DIRECTION"}')
