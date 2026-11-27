"""28/11/2026 : kart dans le cas PESSIMISTE (tous les defauts a la fois : moteurs lents tau 150 ms, retard de commande 20 ms,
desaccord des moteurs 10 %, seuil 0,25 m/s) : quel reglage et quelle vitesse restent surs ? 18 trajets par reglage.
Usage : python3 etude_pessimiste.py   (resultats : etude_pessimiste.log)
"""
import itertools
import sys
from multiprocessing import Pool

sys.path.insert(0, '.')
import etude_complete as E
import etude_robustesse as R

PESSIMISTE = dict(desaccord=0.10, seuil=0.25, retard=4, tau=0.15)


def tache(a):
    pid, v = a
    r = E.bilan([R.kart(pid, v, g, ra, m, **PESSIMISTE) for g in E.GRAINES2 for ra in E.RAYONS for m in E.MODELES])
    r.update(robot='kart', geo='5 (15/27, 0/4)', L=110.0, v=v, pid=pid)
    return r


if __name__ == '__main__':
    grille = list(itertools.product((0.008, 0.015, 0.025, 0.04), (0.0, 0.01), (0.001, 0.002, 0.004), (1.0, 2.0)))
    vs = (0.6, 0.8, 1.0)
    with Pool(5) as pool:
        res = pool.map(tache, [(p, v) for v in vs for p in grille], chunksize=2)
    for v in vs:
        print('=== %.2f m/s (cas pessimiste), 3 meilleurs reglages ===' % v)
        for r in E.meilleurs([r for r in res if r['v'] == v], 3):
            print(E.ligne(r))
