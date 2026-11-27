"""28/11/2026 : robot KART (2 moteurs a l'arriere, voie 136 mm) avec 5 CAPTEURS EN FLECHE : ou les mettre ?
Cartes TCRT5000 (14,1 mm de large, element a 3,8 mm d'un bout, 27,4 de l'autre) -> disposition possible :
- capteur du MILIEU : carte le long du robot, element en pointe (x = L) ;
- paire 1 : cartes le long du robot, a cote de celle du milieu (ecart b1 >= 15,2 mm), recul r1 ;
- paire 2 : cartes en travers vers l'exterieur (comme les exterieurs actuels), b2 >= b1 + 12, recul r2.
Phase 1 : geometrie (b1, b2, r1, r2) a L = 110 mm, 0,6 et 1 m/s. Phase 2 : distance L avec la meilleure geometrie, 0,45 a 1 m/s.
Reference : les 4 capteurs actuels (7,6 / 19,5, recul 8). Meme piste, meme bruit que sim_differentiel.py.
Usage : python3 etude_kart5.py   (resultats : etude_kart5.log)
"""
import itertools
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, '.')
import sim_differentiel as D

VOIE = 136.0
PIDS = list(itertools.product((0.015, 0.025, 0.04), (0.0,), (0.0005, 0.002), (0.0, 1.0)))


def capteurs(L, g):
    if g[0] == 'ref4':
        return [(L - 8.0, 19.5), (L, 7.6), (L, -7.6), (L - 8.0, -19.5)]
    b1, b2, r1, r2 = g
    return [(L - r2, b2), (L - r1, b1), (L, 0.0), (L - r1, -b1), (L - r2, -b2)]


def evaluer(args):
    L, g, pid, v = args
    D.VOIE = VOIE
    D.L_CAPT = L
    D.CAPT = capteurs(L, g)
    r = D.eval_diff((pid, v))
    r['L'], r['g'] = L, g
    return r


def note(d):
    """Plus petit = mieux : trajets rates tres penalises, puis ecart moyen + pertes de ligne."""
    return (d['n'] - d['ok']) * 100.0 + d['rms'] + 2.0 * d['pertes']


def tableau(res, Ls, geoms, vitesses):
    lignes = []
    for L in Ls:
        for g in geoms:
            ds = [D.meilleur([r for r in res if r['L'] == L and r['g'] == g and r['v'] == v]) for v in vitesses]
            lignes.append((sum(note(d) for d in ds), L, g, ds))
    return sorted(lignes, key=lambda t: t[0])


def afficher(lignes, vitesses, n=None):
    print('note  | L   | geometrie (b1, b2, r1, r2) | ' + ' | '.join('%.2f m/s: ok RMS pertes' % v for v in vitesses))
    for s, L, g, ds in lignes[:n]:
        print('%5.1f | %3.0f | %-26s | ' % (s, L, g) + ' | '.join('%d/%d %5.1f %4.1f' % (d['ok'], d['n'], d['rms'], d['pertes']) for d in ds), flush=True)


if __name__ == '__main__':
    geoms = [('ref4',)]
    for b1, db2, r1, dr2 in itertools.product((15.2, 18.0), (12.0, 18.0, 24.0), (0.0, 4.0, 8.0), (4.0, 10.0)):
        geoms.append((b1, round(b1 + db2, 1), r1, r1 + dr2))
    v1 = (0.6, 1.0)
    taches = [(110.0, g, p, v) for g in geoms for v in v1 for p in PIDS]
    print('PHASE 1 : %d geometries, %d essais (x 4 trajets), voie %.0f mm, L 110' % (len(geoms), len(taches), VOIE), flush=True)
    with Pool(5) as pool:
        res = pool.map(evaluer, taches, chunksize=4)
    l1 = tableau(res, (110.0,), geoms, v1)
    afficher(l1, v1)
    best = [t[2] for t in l1 if t[2] != ('ref4',)][:2]
    v2 = (0.45, 0.6, 0.8, 1.0)
    Ls = (85.0, 95.0, 110.0, 125.0)
    taches = [(L, g, p, v) for L in Ls for g in best + [('ref4',)] for v in v2 for p in PIDS]
    print('\nPHASE 2 : distance essieu -> capteur du milieu, 2 meilleures geometries + reference (%d essais)' % len(taches), flush=True)
    with Pool(5) as pool:
        res2 = pool.map(evaluer, taches, chunksize=4)
    l2 = tableau(res2, Ls, best + [('ref4',)], v2)
    afficher(l2, v2)
    print('\nPID retenus pour la meilleure :')
    s, L, g, ds = l2[0]
    for v, d in zip(v2, ds):
        print('  %.2f m/s : %s' % (v, d['pid']))
