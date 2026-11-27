"""28/11/2026 : COMPARAISON robot ACTUEL (roue directrice + servo, 4 capteurs en fleche sur le bloc) / KART v2 (2 moteurs a l'arriere,
voie 136 mm, 5 capteurs en fleche a 110 mm de l'essieu : chassis_kart.py). Meme piste, memes trajets, meme bruit (sim_differentiel.py).
Actuel : 8 trajets (dont 4 avec un servo lent) ; kart : 4 trajets. Usage : python3 comparaison_kart_actuel.py
"""
import itertools
import sys
from multiprocessing import Pool

sys.path.insert(0, '.')
import sim_differentiel as D

L = 110.0
CAPT5 = [(L - 8.0, 33.2), (L - 4.0, 15.2), (L, 0.0), (L - 4.0, -15.2), (L - 8.0, -33.2)]


def kart(args):
    D.VOIE, D.L_CAPT, D.CAPT = 136.0, L, CAPT5
    return D.eval_diff(args)


if __name__ == '__main__':
    vitesses = (0.3, 0.45, 0.6, 0.8, 1.0, 1.2)
    g_kart = list(itertools.product((0.008, 0.015, 0.025, 0.04), (0.0, 0.01), (0.0005, 0.001, 0.002), (0.0, 1.0)))
    g_act = list(itertools.product((1.0, 1.6, 2.5), (1.0, 2.0, 3.5), (0.03, 0.05), (0.04, 0.06, 0.1)))
    tk = [(p, v) for v in vitesses for p in g_kart]
    ta = [(p, v) for v in vitesses for p in g_act]
    print('kart : %d essais ; actuel : %d essais' % (len(tk), len(ta)), flush=True)
    with Pool(5) as pool:
        rk = pool.map(kart, tk, chunksize=4)
        ra = pool.map(D.eval_actuel, ta, chunksize=2)
    print('\nvitesse | ACTUEL (roue directrice + servo, 4 capteurs)              | KART v2 (2 moteurs, 5 capteurs a 110 mm, voie 136)')
    print('  m/s   | reussis  temps s  RMS mm  max mm  pertes  PID                | reussis  temps s  RMS mm  max mm  pertes  PID')
    for v in vitesses:
        a = D.meilleur([r for r in ra if r['v'] == v]); k = D.meilleur([r for r in rk if r['v'] == v])
        print('  %.2f  |  %d/%d   %6.1f   %5.1f  %6.1f  %5.1f   %-20s |  %d/%d   %6.1f   %5.1f  %6.1f  %5.1f   %s' % (
            v, a['ok'], a['n'], a['temps'], a['rms'], a['emax'], a['pertes'], str(a['pid']), k['ok'], k['n'], k['temps'], k['rms'], k['emax'], k['pertes'], str(k['pid'])), flush=True)
