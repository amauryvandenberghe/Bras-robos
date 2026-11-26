"""26/11 nuit (2) : la version 0,55/0,28 + detection abaissee a echoue sur le robot. Retour a la detection qui marchait
(un capteur > 50 %, barycentre sans les 20 % du bas) ; seule la vitesse change : virage = braquage moyen 0,3 s, SANS frein moteur,
v_min >= 0,27 m/s (la vitesse qui marchait). Capteurs a 22 mm, servo rapide et lent."""
import sys, itertools; sys.path.insert(0, '.')
from multiprocessing import Pool
import numpy as np
import sim_ligne2 as S
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)
V255 = 0.8
def pwm(v): return 60.0 + v / V255 * 195.0
BASE = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=1.3, ki=2.0, kd=0.05, tau=0.1, vitesse_servo=257.0, v255_estime=V255,
            v_perdu=pwm(0.12), accel=1.5 / V255 * 195.0)

def un(args):
    nom, P = args
    res = [S.simuler(dict(n=3, pas=pas, h=5.0, methode='bary2', avant_seul=True, modele=mod, **sv), P, g, r, V255, tmax=90.0)
           for pas in (20.0, 22.0) for mod in ('etroite', 'large') for sv in ({}, LENT) for r in (230.0, 200.0) for g in (10,)]
    ok = sum(x['ok'] for x in res); t = np.mean([x['temps'] for x in res]); mx = max(x['emax'] for x in res)
    pe = np.mean([x['pertes'] for x in res]); rms = np.mean([x['rms'] for x in res])
    return (len(res) - ok) * 100 + t + 0.5 * max(0.0, mx - 20) + 0.2 * pe, nom, ok, len(res), t, rms, mx, pe

if __name__ == '__main__':
    taches = [('VERSION QUI MARCHAIT (0,30/0,20, ancienne loi, frein)', dict(BASE, v_max=pwm(0.30), v_min=pwm(0.20), braq_lent=63.5, ecart_lent=33.6))]
    for vmax, bl, el in itertools.product((0.35, 0.40, 0.45), (10.0, 14.0, 20.0), (15.0, 30.0)):
        taches.append(('v_max %.2f v_min 0,27 braq_lent %2.0f ecart_lent %2.0f sans frein' % (vmax, bl, el),
                       dict(BASE, vitesse2=True, frein=False, v_max=pwm(vmax), v_min=pwm(0.27), braq_lent=bl, ecart_lent=el)))
    with Pool(5) as pool:
        out = pool.map(un, taches)
    print('reference :', out[0][1:])
    out.sort()
    print('score  reglage                                                   reussis  temps s  RMS mm  max mm  pertes')
    for sc, nom, ok, n, t, rms, mx, pe in out:
        print('%6.1f  %-56s %2d/%2d  %6.1f  %5.1f  %5.0f  %5.1f' % (sc, nom, ok, n, t, rms, mx, pe), flush=True)
