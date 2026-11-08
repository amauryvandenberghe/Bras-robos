"""Acces, deuxieme passage.
1. Chaque ecrou : on translate son cercle inscrit (r 2,6, epaisseur 2,3 : independant de l'orientation de l'hexagone) depuis sa place,
   tous les 15 degres dans son plan et le long de la vis. Bonne direction = aucun point dans la matiere sur 20 mm.
2. Vis centrale du lacet : outil incline vers +y, passe-t-il derriere le linteau de la face du plateau ?
3. Vis du bas du MG996R d'epaule : quel obstacle exact (jante), et quelle inclinaison suffit."""
import os
import sys
import numpy as np
import trimesh

P = os.path.expanduser('~/ros2_ws/src/mon_bras_robot/bras_50cm/')
sys.path.insert(0, P)
sys.path.insert(0, os.path.join(P, '..', 'ia_50cm'))
os.chdir(P)
import cao_bras as C  # noqa: E402

CH = {}


def m(n):
    if n not in CH:
        CH[n] = trimesh.load(os.path.join('pieces', n + '.stl'))
    return CH[n]


pts = []
for z in (-1.15, 0, 1.15):
    for r in (0, 1.3, 2.6):
        for t in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            pts.append((r * np.cos(t), r * np.sin(t), z))
pts = np.array(pts)
for nom, typ, porteur, T in C.visserie():
    if not typ.startswith('Ecrou') or porteur not in C.LIEN_PIECE:
        continue
    bons = []
    essais = [(f'{int(a)}', np.array([np.cos(np.radians(a)), np.sin(np.radians(a)), 0.0])) for a in np.arange(0, 360, 15)]
    essais += [('+vis', np.array([0, 0, 1.0])), ('-vis', np.array([0, 0, -1.0]))]
    depart = T[:3, :3] @ pts.T + T[:3, [3]]
    for lib, d in essais:
        dw = T[:3, :3] @ d
        ok = True
        for s in np.arange(0.5, 20.5, 0.5):
            if m(porteur).contains((depart + (s * dw)[:, None]).T).any():
                ok = False
                break
        if ok:
            bons.append(lib if 'vis' in lib else '(%.1f,%.1f,%.1f)' % tuple(np.round(dw, 1)))
    print(f'{nom:22s} dans {porteur:12s}: {" ".join(bons[:4]) if bons else "BLOQUE"}', flush=True)


def faisceau(o, d, r):
    d = d / np.linalg.norm(d)
    a = np.cross(d, [1, 0, 0])
    a /= np.linalg.norm(a)
    c = np.cross(d, a)
    return np.array([o] + [o + r * (np.cos(t) * a + np.sin(t) * c) for t in np.linspace(0, 2 * np.pi, 16, endpoint=False)]), d


print('\nVIS CENTRALE DU LACET : pivot (0,0,z0) sur la tete, outil incline vers +y, plateau seul')
for z0 in (0.0, 2.0):
    for r in (1.5, 2.0):
        lig = []
        for deg in (0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 15):
            d = np.array([0, np.sin(np.radians(deg)), np.cos(np.radians(deg))])
            o, dn = faisceau(np.array([0, 0, z0]), d, r)
            loc, ray, _ = m('plateau').ray.intersects_location(o, np.tile(dn, (len(o), 1)), multiple_hits=False)
            lg = 999 if not len(loc) else np.linalg.norm(loc - o[ray], axis=1).min()
            lig.append(f'{deg}:{min(lg, 150):.0f}')
        print(f'  z0={z0} tige de {2 * r:.0f} mm  angle:longueur libre  ' + ' '.join(lig))
print('  E_PLATEAU', C.E_PLATEAU, 'JANTE_H', C.JANTE_H, 'JANTE_RI', C.JANTE_RI, 'R_PLATEAU', C.R_PLATEAU)

print('\nVIS DU BAS DU MG996R D EPAULE (x=5, z=8, tete y=0.4 cote servo) : outil vers +y, incline vers le haut')
for r in (1.5, 2.0):
    lig = []
    for deg in (0, 1, 2, 3, 4, 5, 8):
        d = np.array([0, np.cos(np.radians(deg)), np.sin(np.radians(deg))])
        o, dn = faisceau(np.array([5, 3.6, 8.0]), d, r)
        loc, ray, _ = m('plateau').ray.intersects_location(o, np.tile(dn, (len(o), 1)), multiple_hits=False)
        lg = 999 if not len(loc) else np.linalg.norm(loc - o[ray], axis=1).min()
        lig.append(f'{deg}:{min(lg, 150):.0f}')
    print(f'  tige de {2 * r:.0f} mm  ' + ' '.join(lig))
