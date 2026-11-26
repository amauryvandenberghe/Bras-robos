"""26/11 nuit : vitesse variable (rapide en ligne droite, lent en virage) avec le PID du 26/11 (Kp 1,3 Ki 2 Kd 0,05 tau 0,1),
capteurs a 22 mm, servo rapide et lent. Virage detecte par le braquage MOYEN (0,3 s) et l'ecart moyen (0,2 s)."""
import sys, itertools; sys.path.insert(0, '.')
from multiprocessing import Pool
import numpy as np
import sim_ligne2 as S
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)
V255 = 0.8
def pwm(v): return 60.0 + v / V255 * 195.0
BASE = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=1.3, ki=2.0, kd=0.05, tau=0.1, vitesse_servo=257.0, v255_estime=V255,
            v_perdu=pwm(0.12), accel=1.5 / V255 * 195.0, tenue_perte=0.15)

def un(args):
    nom, P = args
    res = [S.simuler(dict(n=3, pas=22.0, h=5.0, methode='bary2', avant_seul=True, modele='etroite', **sv), P, g, r, V255, tmax=90.0)
           for sv in ({}, LENT) for r in (230.0, 200.0) for g in (10, 11)]
    ok = sum(x['ok'] for x in res); t = np.mean([x['temps'] for x in res]); mx = max(x['emax'] for x in res)
    pe = np.mean([x['pertes'] for x in res]); rms = np.mean([x['rms'] for x in res])
    score = (len(res) - ok) * 100 + t + 0.5 * max(0.0, mx - 20) + 2 * pe
    return score, nom, ok, len(res), t, rms, mx, pe

if __name__ == '__main__':
    taches = [('actuel 0,30/0,20 (ancienne loi)', dict(BASE, v_max=pwm(0.30), v_min=pwm(0.20), braq_lent=63.5, ecart_lent=33.6))]
    for vmax, vmin, bl, el in itertools.product((0.45, 0.55, 0.65), (0.20, 0.28), (8.0, 14.0, 20.0), (15.0, 30.0)):
        taches.append(('v_max %.2f v_min %.2f braq_lent %2.0f ecart_lent %2.0f' % (vmax, vmin, bl, el),
                       dict(BASE, vitesse2=True, v_max=pwm(vmax), v_min=pwm(vmin), braq_lent=bl, ecart_lent=el)))
    with Pool(5) as pool:
        out = pool.map(un, taches)
    print('actuel :', out[0])
    out.sort()
    print('score  reglage                                        reussis  temps s  RMS mm  max mm  pertes')
    for sc, nom, ok, n, t, rms, mx, pe in out[:20]:
        print('%6.1f  %-46s %2d/%2d  %6.1f  %5.1f  %5.0f  %5.1f' % (sc, nom, ok, n, t, rms, mx, pe), flush=True)
