"""27/11/2026 : CHASSIS DIFFERENTIEL "compact" (roues sur les cotes, essieu sous le bras) : ou mettre les capteurs ?
Voie fixee par le carre de 200 mm : roues de 13,6 mm a l'exterieur du bati -> centres a +-93,2 mm, voie 186 mm.
On fait varier L = distance essieu -> elements interieurs de la fleche (les exterieurs 8 mm en arriere, comme sur le robot).
Le robot doit tenir dans 200 mm : essieu au centre -> L <= ~95 mm ; essieu recule de d mm -> L ~ 95 + d (mais le bras n'est plus
au-dessus de l'essieu). Meme piste, memes capteurs, meme bruit que sim_differentiel.py.
Usage : python3 etude_chassis_diff.py [L1 L2 ...]   (resultats du 27/11 : etude_chassis_diff.log ; L 102 retenu : etude_chassis_diff_L102.log)
"""
import itertools
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, '.')
import sim_differentiel as D

VOIE = float(os.environ.get('VOIE', 186.4))     # 28/11 : robot kart, VOIE=136 (lancer_kart.sh)


def regler(L):
    D.VOIE = VOIE
    D.L_CAPT = L
    D.CAPT = [(L - 8.0, 19.5), (L, 7.6), (L, -7.6), (L - 8.0, -19.5)]


def evaluer(args):
    L, pid, v = args
    regler(L)
    r = D.eval_diff((pid, v))
    r['L'] = L
    return r


if __name__ == '__main__':
    Ls = tuple(float(a) for a in sys.argv[1:]) or (70.0, 85.0, 95.0, 110.0, 125.0)
    vitesses = (0.45, 0.6, 0.8, 1.0)
    pids = list(itertools.product((0.008, 0.015, 0.025, 0.04), (0.0, 0.01), (0.0005, 0.001, 0.002), (0.0, 1.0)))
    taches = [(L, p, v) for L in Ls for v in vitesses for p in pids]
    print('%d essais (x 4 trajets), voie %.1f mm' % (len(taches), VOIE), flush=True)
    with Pool(5) as pool:
        res = pool.map(evaluer, taches, chunksize=4)
    print('\nL mm | vitesse | reussis  temps s  RMS mm  max mm  pertes  PID (kp m/s/mm, ki, kd, k_lent)')
    for L in Ls:
        for v in vitesses:
            d = D.meilleur([r for r in res if r['L'] == L and r['v'] == v])
            print('%4.0f |  %.2f   |  %d/%d   %6.1f   %5.1f  %6.1f  %5.1f   %s' % (L, v, d['ok'], d['n'], d['temps'], d['rms'], d['emax'], d['pertes'], d['pid']), flush=True)
        print()
