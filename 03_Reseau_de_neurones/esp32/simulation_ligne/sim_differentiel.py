"""27/11/2026 : comparaison de 2 directions pour la course (meme piste, memes capteurs, meme bruit, meme estimateur) :
  A. ACTUELLE : bloc moteur + roues qui pivote (servo MG996R), 4 capteurs en fleche sur le bloc (sim_ligne2.py, etude_fleche.py) ;
  B. DIFFERENTIELLE (proposition) : 2 moteurs independants (L298N : 2e voie libre), une bille porteuse, 4 capteurs en fleche
     FIXES sur le chassis, L mm devant l'essieu moteur. Plus de servo : la direction est la difference de vitesse des 2 roues.
Modele B : chaque roue suit sa consigne avec un 1er ordre (TAU_MOTEUR) et une acceleration bornee ; commande rafraichie a 200 Hz ;
  PID sur l'ecart e (mm) -> difference de vitesse dv (m/s) ; vitesse de base reduite quand |dv| est grand.
Usage : python3 sim_differentiel.py   (grille de PID pour chaque vitesse, puis tableau A / B)
"""
import itertools
import math
import random
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, '.')
import sim_ligne2 as S
import etude_fleche as EF

TAU_MOTEUR = 0.08     # s (motoreducteur jaune / N20 sous charge : 50-100 ms)
ACC_MAX = 3.0         # m/s2 par roue
VOIE = 150.0          # mm entre les 2 roues motrices
L_CAPT = 120.0        # mm de l'essieu aux elements interieurs
CAPT = [(L_CAPT - 8.0, 19.5), (L_CAPT, 7.6), (L_CAPT, -7.6), (L_CAPT - 8.0, -19.5)]   # meme fleche (a devant l'essieu, b + = gauche)


def simuler_diff(pid, v_base, graine, rayon, modele, tmax=60.0):
    kp, ki, kd, k_lent = pid
    rnd = random.Random(graine)
    pts, arbre = S.get_piste(rayon)
    npts = len(pts)
    sig = S.sigma_tache(8.0, modele); amp = S.amplitude(8.0)
    nrnd = np.random.default_rng(graine)
    gains = 1.0 + 0.05 * nrnd.standard_normal(4)
    bruit_n = 18.0 / (3000.0 * amp)
    est = S.Estimateur(CAPT, 'bary2')
    dt = 0.005
    e0 = rnd.uniform(-6, 6); psi = math.radians(rnd.uniform(-5, 5))
    x, y = float(pts[0][0]) + 100.0 - L_CAPT, float(pts[0][1]) + e0
    vl = vr = 0.0
    I = 0.0; e_prec = 0.0; D = 0.0; cote = 1; t_vue = 0.0; vu_prec = True; pertes = 0
    t = 0.0; idx = 0; hors = 0.0; errs = []; dvs = []; dv_prec = 0.0; ok = False; v_cmd = 0.0
    while t < tmax:
        c, s = math.cos(psi), math.sin(psi)
        q = np.array([(x + a * c - b * s, y + a * s + b * c) for a, b in CAPT])
        dist, _ = arbre.query(q)
        n = np.clip(gains * S.F(dist, sig) + bruit_n * nrnd.standard_normal(4), 0.0, 1.0)
        vu, e = est(n)            # e : + = ligne a GAUCHE
        if vu:
            t_vue = t
            if abs(e) > 4.0:
                cote = 1 if e > 0 else -1
            if not vu_prec:
                e_prec = e
            D += ((e - e_prec) / dt - D) * (1.0 - math.exp(-dt / 0.02)); e_prec = e
            brut = kp * e + ki * I + kd * D
            if abs(brut) < 1.0 or brut * e < 0:
                I += e * dt
            dv = kp * e + ki * I + kd * D          # m/s : + = tourner a gauche (roue droite plus vite)
        else:
            dv = 0.6 * cote
            if t - t_vue > 1.5:
                break
        if not vu and vu_prec:
            pertes += 1
        vu_prec = vu
        dv = max(-1.0, min(1.0, dv))
        vc = v_base / (1.0 + k_lent * abs(dv) / max(v_base, 1e-3))
        v_cmd = min(vc, v_cmd + 2.0 * dt) if vc > v_cmd else vc
        cl, cr = v_cmd - dv / 2, v_cmd + dv / 2
        for nom, cmd in (('l', cl), ('r', cr)):
            v = vl if nom == 'l' else vr
            dvw = (cmd - v) * (1.0 - math.exp(-dt / TAU_MOTEUR))
            dvw = max(-ACC_MAX * dt, min(ACC_MAX * dt, dvw))
            if nom == 'l':
                vl += dvw
            else:
                vr += dvw
        dvs.append((dv - dv_prec) / dt); dv_prec = dv
        v = (vl + vr) / 2 * 1000.0
        psi += (vr - vl) * 1000.0 / VOIE * dt
        x += v * math.cos(psi) * dt; y += v * math.sin(psi) * dt
        t += dt
        de, j = arbre.query((x, y))     # ecart du milieu de l'essieu
        if abs(j - idx) < 400:
            idx = max(idx, j)
        errs.append(de)
        if de > 45.0:
            hors += dt
            if hors > 0.3:
                break
        else:
            hors = 0.0
        if idx >= npts - 10:
            ok = True
            break
    e = np.array(errs) if errs else np.zeros(1)
    return dict(ok=ok, temps=t, progres=idx / npts, rms=float(np.sqrt((e ** 2).mean())), emax=float(e.max()), pertes=pertes,
                agit=float(np.sqrt((np.array(dvs) ** 2).mean())) if dvs else 0.0)


def eval_diff(args):
    pid, v = args
    res = [simuler_diff(pid, v, 10, r, m) for r in (230.0, 200.0) for m in ('etroite', 'large')]
    ok = sum(r['ok'] for r in res)
    return dict(pid=pid, v=v, ok=ok, n=len(res), temps=float(np.mean([r['temps'] for r in res])), rms=float(np.mean([r['rms'] for r in res])),
                emax=float(max(r['emax'] for r in res)), pertes=float(np.mean([r['pertes'] for r in res])), progres=float(np.mean([r['progres'] for r in res])))


def eval_actuel(args):
    pid, v = args
    kp, ki, kd, tau = pid
    P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=255.0, v255_estime=v)
    res = [S.simuler(dict(n=4, pas=0, h=8.0, methode='bary2', avant_seul=True, modele=m, capteurs=EF.capteurs(7.6, 19.5, 5.5, 8.0), **sv), P, 10, r, v, tmax=90.0)
           for r in (230.0, 200.0) for m in ('etroite', 'large') for sv in ({}, EF.LENT)]
    ok = sum(r['ok'] for r in res)
    return dict(pid=pid, v=v, ok=ok, n=len(res), temps=float(np.mean([r['temps'] for r in res])), rms=float(np.mean([r['rms'] for r in res])),
                emax=float(max(r['emax'] for r in res)), pertes=float(np.mean([r['pertes'] for r in res])), progres=float(np.mean([r['progres'] for r in res])))


def meilleur(rs):
    def sc(r):
        if r['ok'] < r['n']:
            return 1000 - 500 * r['progres']
        return r['rms'] + 0.5 * r['pertes'] + 0.05 * r['emax']
    return min(rs, key=sc)


if __name__ == '__main__':
    vitesses = (0.3, 0.45, 0.6, 0.8, 1.0)
    g_diff = list(itertools.product((0.004, 0.008, 0.015, 0.025), (0.0, 0.01, 0.03), (0.0002, 0.0005, 0.001), (0.0, 1.0)))
    g_act = list(itertools.product((1.0, 1.6, 2.5), (1.0, 2.0, 3.5), (0.03, 0.05), (0.04, 0.06, 0.1)))
    taches_d = [(p, v) for v in vitesses for p in g_diff]
    taches_a = [(p, v) for v in vitesses for p in g_act]
    print('direction differentielle : %d essais ; actuelle : %d essais' % (len(taches_d), len(taches_a)), flush=True)
    with Pool(5) as pool:
        rd = pool.map(eval_diff, taches_d)
        ra = pool.map(eval_actuel, taches_a)
    print('\nvitesse | ACTUELLE (bloc pivotant + servo, fleche sur le bloc)            | DIFFERENTIELLE (2 moteurs, fleche fixe a %d mm)' % L_CAPT)
    print('  m/s   | reussis  temps s  RMS mm  max mm  pertes  PID                    | reussis  temps s  RMS mm  max mm  pertes  PID')
    for v in vitesses:
        a = meilleur([r for r in ra if r['v'] == v]); d = meilleur([r for r in rd if r['v'] == v])
        print('  %.2f  |  %d/%d   %6.1f   %5.1f  %6.1f  %5.1f   %-22s |  %d/%d   %6.1f   %5.1f  %6.1f  %5.1f   %s' % (
            v, a['ok'], a['n'], a['temps'], a['rms'], a['emax'], a['pertes'], str(a['pid']), d['ok'], d['n'], d['temps'], d['rms'], d['emax'], d['pertes'], str(d['pid'])), flush=True)
