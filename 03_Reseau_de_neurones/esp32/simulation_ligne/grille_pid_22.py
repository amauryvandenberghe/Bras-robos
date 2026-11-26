"""26/11 soir : capteurs ecartes (~22 mm, a mesurer). Grille Kp / Ki / Kd a 0,3 m/s, barycentre comme le programme (bary2),
servo rapide ET servo lent (retard 40 ms, 80 ms, 250 deg/s), ecart 20, 22 et 25 mm. Classement : reussite partout, puis
ecart RMS + max/5 + oscillation du servo."""
import sys, itertools; sys.path.insert(0, '.')
from multiprocessing import Pool
import numpy as np
import sim_ligne2 as S
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)

def un(args):
    kp, ki, kd = args
    P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=0.03, vitesse_servo=257.0, v_max=255.0, v_min=255.0, v255_estime=0.3)
    res = []
    for pas in (20.0, 22.0, 25.0):
        for sv in ({}, LENT):
            for r in (230.0, 200.0):
                res.append(S.simuler(dict(n=3, pas=pas, h=5.0, methode='bary2', avant_seul=True, modele='etroite', **sv), P, 10, r, 0.3, tmax=90.0))
    ok = sum(x['ok'] for x in res)
    rms = np.mean([x['rms'] for x in res]); mx = max(x['emax'] for x in res); vs = np.mean([x['vit_servo'] for x in res])
    score = (len(res) - ok) * 100 + rms + mx / 5 + vs / 50
    return score, kp, ki, kd, ok, len(res), rms, mx, vs

if __name__ == '__main__':
    grille = list(itertools.product((0.4, 0.6, 0.8, 1.0, 1.3, 1.7, 2.2), (0.0, 0.5, 1.0, 2.0, 3.5), (0.0, 0.02, 0.05, 0.1)))
    out = []
    with Pool(5) as pool:
        for k, r in enumerate(pool.imap_unordered(un, grille)):
            out.append(r)
            if k % 20 == 0: print('%d/%d' % (k, len(grille)), flush=True)
    out.sort()
    print('score  Kp   Ki   Kd   reussis  RMS mm  max mm  servo deg/s')
    for s_, kp, ki, kd, ok, n, rms, mx, vs in out[:15]:
        print('%6.1f %4.1f %4.1f %5.2f  %2d/%2d  %5.1f  %5.0f  %5.0f' % (s_, kp, ki, kd, ok, n, rms, mx, vs))
    print('actuel (2,5 / 5,3 / 0,04) :', un((2.5, 5.3, 0.04)))
