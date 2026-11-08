"""Prepare les pieces d'origine de l'utilisateur qui transmettent la rotation des servos, et leurs reperes :
  - servos/arm01.step   : palonnier "Servo MG996R arm01" (repere propre : moyeu sur l'axe z, dessous du moyeu z = 0,
                          dessus z = 5,5, bras le long de x) ;
  - servos/broche.step  : palonnier du SG90 "broche" (axe z ; z = 0 : fond de l'empreinte de cannelure = bout de la
                          cannelure du servo ; bras le long de +x ; le servo est du cote z < 0) ;
  - servos/mg996r_corps.step : corps "Servo MG996R" (sans l'arbre metallique), dans son repere propre ;
  - servos/pince_origine.step : pince d'origine ("pince", 2 corps : plaque + bloc des machoires).
Recalages (ICP sur des points de surface) ecrits dans servos/recalages.json :
  - "Servo MG996R" (repere propre) -> repere de "MG996R SBORKA" (celui des cotes MG_* de cao_bras.py) ;
  - pince d'origine -> repere du lien "bras quatre" (maillage meshes/bras_quatre.stl).
A lancer avec ~/venv_cao/bin/python."""
import json
import os
import shutil
import sys

import cadquery as cq
import numpy as np
import trimesh

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
import cao_bras as C  # noqa: E402

EXP = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/'
S = C.DOSSIER_SERVOS


def maillage(forme):
    v, f = forme.tessellate(0.05, 0.2)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in v]), np.array(f))


def _icp(src, arbre, dst, T, iterations=60):
    """ICP point a point (KD-tree) : rapide."""
    for _ in range(iterations):
        p = src @ T[:3, :3].T + T[:3, 3]
        _, i = arbre.query(p)
        q = dst[i]
        cp, cq_ = p.mean(0), q.mean(0)
        U, _, Vt = np.linalg.svd((p - cp).T @ (q - cq_))
        D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        R = Vt.T @ D @ U.T
        dT = np.eye(4)
        dT[:3, :3], dT[:3, 3] = R, cq_ - R @ cp
        T = dT @ T
    p = src @ T[:3, :3].T + T[:3, 3]
    return T, float(arbre.query(p)[0].mean())


def recaler(source, cible, n=4000):
    """Meilleure transformation rigide source -> cible (maillages) : departs tous les 30 degres autour de z avec
    retournements, ICP sur points (KD-tree). Renvoie (matrice 4x4, ecart moyen, 99e centile) en mm."""
    from scipy.spatial import cKDTree
    src = source.sample(n, seed=1)
    dst = cible.sample(4 * n, seed=2)
    arbre = cKDTree(dst)
    meilleur = None
    for flip in (np.diag([1, 1, 1]), np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])):
        for a in np.radians(np.arange(0, 360, 30)):
            R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]) @ flip
            T0 = np.eye(4)
            T0[:3, :3] = R
            T0[:3, 3] = dst.mean(0) - R @ src.mean(0)
            T, cout = _icp(src, arbre, dst, T0)
            if meilleur is None or cout < meilleur[1]:
                meilleur = (T, cout)
    M = _icp(src, arbre, dst, meilleur[0], 100)[0]
    d = np.abs(trimesh.proximity.signed_distance(cible, trimesh.transform_points(src[:1500], M)))
    return M, float(d.mean()), float(np.percentile(d, 99))


os.makedirs(S, exist_ok=True)
shutil.copy(EXP + 'Servo_MG996R_arm01.step', os.path.join(S, 'arm01.step'))
shutil.copy(EXP + 'broche.step', os.path.join(S, 'broche.step'))
shutil.copy(EXP + 'Servo_MG996R.step', os.path.join(S, 'mg996r_corps.step'))
shutil.copy(EXP + 'pince.step', os.path.join(S, 'pince_origine.step'))
rec = {}
# corps du MG996R -> repere SBORKA (solide 4 de l'export de l'assemblage)
corps = maillage(cq.importers.importStep(os.path.join(S, 'mg996r_corps.step')).val())
sborka = maillage(cq.importers.importStep(os.path.join(S, 'mg996r_origine.step')).val().Solids()[4])
M, ecart, p99 = recaler(corps, sborka)
rec['mg996r_corps_vers_sborka'] = M.tolist()
print(f'Servo MG996R -> SBORKA : ecart moyen {ecart:.3f} mm (99 % < {p99:.3f}) ; translation {np.round(M[:3, 3], 2).tolist()}, '
      f'rotation\n{np.round(M[:3, :3], 3)}')
# pince d'origine -> "bras quatre" (morceaux plaque + bloc du maillage, en mm)
morceaux = sorted(trimesh.load(os.path.join(ICI, '..', 'meshes', 'bras_quatre.stl')).split(only_watertight=False),
                  key=lambda c: -c.area)
garde = [c for c in morceaux if len(c.faces) not in (3850, 402, 152)]
cible = trimesh.util.concatenate(garde)
cible.apply_scale(1000)
pince = maillage(cq.importers.importStep(os.path.join(S, 'pince_origine.step')).val())
M, ecart, p99 = recaler(pince, cible)
rec['pince_vers_bras_quatre'] = M.tolist()
print(f'pince -> bras quatre : ecart moyen {ecart:.3f} mm (99 % < {p99:.3f}) ; translation {np.round(M[:3, 3], 2).tolist()}, '
      f'rotation\n{np.round(M[:3, :3], 3)}')
json.dump(rec, open(os.path.join(S, 'recalages.json'), 'w'), indent=1)
print('-> servos/recalages.json')
