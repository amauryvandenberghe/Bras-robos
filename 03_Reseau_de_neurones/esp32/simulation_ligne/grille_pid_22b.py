"""26/11 soir (2) : grille resserree, Kp faibles et lissage, ecart 22 mm, servo rapide et lent, 0,3 m/s. Affiche tout."""
import sys, itertools; sys.path.insert(0, '.')
from multiprocessing import Pool
import numpy as np
import sim_ligne2 as S
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)

def un(args):
    kp, ki, kd, tau = args
    P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=255.0, v255_estime=0.3)
    res = [S.simuler(dict(n=3, pas=22.0, h=5.0, methode='bary2', avant_seul=True, modele='etroite', **sv), P, 10, r, 0.3, tmax=90.0)
           for sv in ({}, LENT) for r in (230.0, 200.0)]
    return (kp, ki, kd, tau, sum(x['ok'] for x in res), len(res), np.mean([x['rms'] for x in res]), max(x['emax'] for x in res),
            np.mean([x['vit_servo'] for x in res]), np.mean([x['temps'] for x in res]))

if __name__ == '__main__':
    grille = list(itertools.product((0.5, 0.7, 1.0, 1.3), (1.0, 2.0, 3.5), (0.05, 0.1), (0.03, 0.1)))
    with Pool(5) as pool:
        out = pool.map(un, grille)
    out.sort(key=lambda r: (-r[4], r[8]))
    print('  Kp   Ki   Kd   tau  reussis  RMS mm  max mm  servo deg/s  temps s')
    for kp, ki, kd, tau, ok, n, rms, mx, vs, t in out:
        print('%4.1f %4.1f %5.2f %5.2f  %d/%d  %5.1f  %5.0f  %6.0f  %6.1f' % (kp, ki, kd, tau, ok, n, rms, mx, vs, t), flush=True)
