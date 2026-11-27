"""27/11/2026 : COMBIEN DE CAPTEURS ? 1 a 6 capteurs sur le bloc de direction, chacun avec ses meilleurs ecartement, fleche et PID.
Capteurs a pas regulier p (positions laterales symetriques), fleche : les plus a l'exterieur sont `recul` mm plus en arriere.
1 capteur : il suit le BORD de la ligne (place a 9,5 mm du centre) : ecart = (n - 0,5) x 19 mm.
Essais a 0,45 et 0,60 m/s (la vitesse compte), 2 modeles de capteur, servo rapide et lent, rayons 230 et 200 mm, h 8 mm.
Attention : un pas < 15 mm demande des cartes en quinconce (cartes TCRT5000 de 14,1 mm de large).
Usage : python3 etude_nb_capteurs.py
"""
import itertools
import json
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, '.')
import sim_ligne2 as S

A0 = 39.8          # elements les plus en avant : 39,8 mm devant le pivot (comme la fleche actuelle)
LENT = dict(sv_retard=0.04, sv_tau=0.08, sv_vmax=250.0)
_appel = S.Estimateur.__call__


def _appel_un(self, n):
    if self.methode == 'un':
        x = float(n[0])
        if x < 0.08:
            return False, 0.0
        return True, (x - 0.5) * 19.0
    return _appel(self, n)


S.Estimateur.__call__ = _appel_un


def capteurs(N, p, recul):
    if N == 1:
        return [(A0, 9.5)]
    bs = [(i - (N - 1) / 2) * p for i in range(N)]
    bm = max(abs(b) for b in bs)
    return [(A0 - recul * abs(b) / bm, b) for b in reversed(bs)]


def evaluer(args):
    (N, p, recul), pid = args
    kp, ki, kd, tau = pid
    res = []
    for v in (0.45, 0.6):
        P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=255.0, v255_estime=v)
        for m in ('etroite', 'large'):
            for sv in ({}, LENT):
                for r in (230.0, 200.0):
                    conf = dict(n=N, pas=p, h=8.0, methode='un' if N == 1 else 'bary2', avant_seul=True, modele=m, capteurs=capteurs(N, p, recul), **sv)
                    x = S.simuler(conf, P, 10, r, v, tmax=60.0)
                    x['v'] = v
                    res.append(x)
    ok = sum(x['ok'] for x in res)
    ok06 = sum(x['ok'] for x in res if x['v'] == 0.6)
    d = dict(N=N, p=p, recul=recul, pid=pid, ok=ok, n=len(res), ok06=ok06, rms=float(np.mean([x['rms'] for x in res])),
             emax=float(max(x['emax'] for x in res)), servo=float(np.mean([x['vit_servo'] for x in res])),
             pertes=float(np.mean([x['pertes'] for x in res])), progres=float(np.mean([x['progres'] for x in res])))
    d['score'] = (1000 - 500 * d['progres'] - 10 * ok) if ok < len(res) else d['rms'] + d['servo'] / 50 + 0.5 * d['pertes'] + 0.05 * d['emax']
    return d


if __name__ == '__main__':
    geos = [(1, 0.0, 0.0)] + [(N, p, r) for N in range(2, 7) for p in (8.0, 11.0, 15.2, 19.0, 24.0) for r in (0.0, 8.0)]
    pids = list(itertools.product((1.0, 1.6, 2.5, 4.0), (1.0, 3.0), (0.05,), (0.06, 0.1)))
    taches = [(g, pid) for g in geos for pid in pids]
    print('%d dispositions x %d PID x 16 trajets (0,45 et 0,60 m/s)' % (len(geos), len(pids)), flush=True)
    with Pool(5) as pool:
        out = pool.map(evaluer, taches, chunksize=4)
    json.dump(out, open('etude_nb_capteurs.json', 'w'), indent=1)
    best = {}
    for d in out:
        k = (d['N'], d['p'], d['recul'])
        if k not in best or d['score'] < best[k]['score']:
            best[k] = d
    print('\n=== MEILLEURE DISPOSITION POUR CHAQUE NOMBRE DE CAPTEURS ===')
    print(' N  pas mm  recul | reussis (dont 0,6 m/s)  RMS mm  max mm  servo deg/s  pertes  PID  score')
    for N in range(1, 7):
        d = min((x for x in best.values() if x['N'] == N), key=lambda x: x['score'])
        print('%2d  %5.1f  %5.1f |  %2d/%d (%d/8)  %6.1f  %6.1f  %6.0f  %6.1f  %s  %.1f' % (N, d['p'], d['recul'], d['ok'], d['n'], d['ok06'],
              d['rms'], d['emax'], d['servo'], d['pertes'], d['pid'], d['score']))
    print('\n=== DETAIL : chaque disposition avec son meilleur PID ===')
    for k in sorted(best):
        d = best[k]
        print('N %d pas %5.1f recul %4.1f | %2d/%d (%d/8)  RMS %5.1f  max %5.1f  servo %4.0f  pertes %5.1f  PID %s  score %.1f' % (
            d['N'], d['p'], d['recul'], d['ok'], d['n'], d['ok06'], d['rms'], d['emax'], d['servo'], d['pertes'], d['pid'], d['score']))
