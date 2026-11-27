"""28/11/2026 : ROBUSTESSE des 2 robots aux defauts du vrai materiel, avec leurs meilleurs reglages (etude_complete.log).
Kart : desaccord des 2 moteurs (0 a 15 %), seuil de demarrage (0,10 a 0,30 m/s), retard de commande (5 a 40 ms),
       reponse des moteurs (constante de temps 50 a 150 ms).
Actuel : retard du servo (5 a 40 ms), vitesse du servo (350 / 200 deg/s), constante de temps du servo (40 / 80 ms).
Pour chaque defaut, les autres restent a leur valeur nominale. 18 trajets par cas (3 rayons x 2 modeles x 3 tirages).
Usage : python3 etude_robustesse.py <v> "<pid kart>" "<pid actuel>"   ex. 0.8 "0.025,0,0.001,1" "1.6,3.5,0.05,0.04,255"
"""
import math
import random
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, '.')
import sim_ligne2 as S
import etude_fleche as EF
import sim_differentiel as D
import etude_complete as E

L = 110.0
CAPT = E.geometrie('5 (15/27, 0/4)', L)          # retenue par etude_complete.log
NOMINAL = dict(desaccord=0.08, seuil=0.15, retard=1, tau=0.08)


def kart(pid, v, graine, rayon, modele, desaccord, seuil, retard, tau, voie=136.0, tmax=60.0):
    """simuler_kart d'etude_complete, avec les defauts en parametres."""
    kp, ki, kd, k_lent = pid
    rnd = random.Random(graine * 7919 + int(rayon))
    pts, arbre = S.get_piste(rayon); npts = len(pts)
    sig = S.sigma_tache(8.0, modele); amp = S.amplitude(8.0)
    nrnd = np.random.default_rng(graine)
    gains = 1.0 + 0.05 * nrnd.standard_normal(len(CAPT))
    sgn = 1 if rnd.random() < 0.5 else -1
    g_mot = (1.0 + sgn * desaccord / 2, 1.0 - sgn * desaccord / 2)        # desaccord total = ecart entre les 2 moteurs
    bruit_n = 18.0 / (3000.0 * amp)
    est = S.Estimateur(CAPT, 'bary2'); dt = 0.005
    e0 = rnd.uniform(-6, 6); psi = math.radians(rnd.uniform(-5, 5))
    x, y = float(pts[0][0]) + 100.0 - L, float(pts[0][1]) + e0
    vl = vr = 0.0; I = 0.0; e_prec = 0.0; Dd = 0.0; cote = 1; t_vue = 0.0; vu_prec = True; pertes = 0
    t = 0.0; idx = 0; hors = 0.0; errs = []; ok = False; v_cmd = 0.0; file_cmd = [(0.0, 0.0)] * max(1, retard)
    while t < tmax:
        c, s = math.cos(psi), math.sin(psi)
        q = np.array([(x + a * c - b * s, y + a * s + b * c) for a, b in CAPT])
        dist, _ = arbre.query(q)
        n = np.clip(gains * S.F(dist, sig) + bruit_n * nrnd.standard_normal(len(CAPT)), 0.0, 1.0)
        vu, e = est(n)
        if vu:
            t_vue = t
            if abs(e) > 4.0:
                cote = 1 if e > 0 else -1
            if not vu_prec:
                e_prec = e
            Dd += ((e - e_prec) / dt - Dd) * (1.0 - math.exp(-dt / 0.02)); e_prec = e
            brut = kp * e + ki * I + kd * Dd
            if abs(brut) < 1.0 or brut * e < 0:
                I += e * dt
            dv = kp * e + ki * I + kd * Dd
        else:
            dv = 0.6 * cote
            if t - t_vue > 1.5:
                break
        if not vu and vu_prec:
            pertes += 1
        vu_prec = vu
        dv = max(-1.0, min(1.0, dv))
        vc = v / (1.0 + k_lent * abs(dv) / max(v, 1e-3))
        v_cmd = min(vc, v_cmd + 2.0 * dt) if vc > v_cmd else vc
        file_cmd.append((v_cmd - dv / 2, v_cmd + dv / 2)); appl = file_cmd.pop(0)
        for k, cm in enumerate(appl):
            cm = 0.0 if abs(cm) < seuil else cm * g_mot[k]
            w = vl if k == 0 else vr
            dw = max(-D.ACC_MAX * dt, min(D.ACC_MAX * dt, (cm - w) * (1.0 - math.exp(-dt / tau))))
            if k == 0:
                vl += dw
            else:
                vr += dw
        vm = (vl + vr) / 2 * 1000.0
        psi += (vr - vl) * 1000.0 / voie * dt
        x += vm * math.cos(psi) * dt; y += vm * math.sin(psi) * dt; t += dt
        de, j = arbre.query((x, y))
        if abs(j - idx) < 400:
            idx = max(idx, j)
        errs.append(de)
        if de > 45.0:
            hors += dt
            if hors > 0.3:
                break
        else:
            hors = 0.0
        _, jc = arbre.query((x + L * math.cos(psi), y + L * math.sin(psi)))
        if idx >= npts - 10 or (jc >= npts - 10 and abs(jc - idx) < 400 + int(L)):
            ok = True; t += L / 1000.0 / max(vm / 1000.0, 0.05); break
    e = np.array(errs) if errs else np.zeros(1)
    return dict(ok=ok, temps=t, progres=idx / npts, rms=float(np.sqrt((e ** 2).mean())), emax=float(e.max()), pertes=pertes)


def t_kart(a):
    pid, v, p = a
    return E.bilan([kart(pid, v, g, r, m, **p) for g in E.GRAINES2 for r in E.RAYONS for m in E.MODELES])


def t_actuel(a):
    pid, v, sv = a
    kp, ki, kd, tau, v_min = pid
    P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=v_min, v255_estime=v)
    res = [S.simuler(dict(n=4, pas=0, h=8.0, methode='bary2', avant_seul=True, modele=m, capteurs=EF.capteurs(7.6, 19.5, 5.5, 8.0), **sv), P, g, r, v, tmax=90.0)
           for g in E.GRAINES2 for r in E.RAYONS for m in E.MODELES]
    return E.bilan(res)


if __name__ == '__main__':
    vs = [float(x) for x in sys.argv[1].split(',')]
    pk = tuple(float(x) for x in sys.argv[2].split(','))
    pa = tuple(float(x) for x in sys.argv[3].split(','))
    cas_k = [('nominal', {})] + [('desaccord moteurs %.0f %%' % (d * 100), dict(desaccord=d)) for d in (0.0, 0.05, 0.15)] + \
            [('seuil %.2f m/s' % s, dict(seuil=s)) for s in (0.10, 0.25, 0.30)] + [('retard %d ms' % (5 * r), dict(retard=r)) for r in (4, 8)] + \
            [('moteurs tau %.0f ms' % (t * 1000), dict(tau=t)) for t in (0.05, 0.15)]
    cas_a = [('nominal (servo normal)', {}), ('servo lent', EF.LENT), ('servo retard 20 ms', dict(sv_retard=0.02)),
             ('servo retard 40 ms', dict(sv_retard=0.04)), ('servo 200 deg/s', dict(sv_vmax=200.0)), ('servo tau 80 ms', dict(sv_tau=0.08))]
    tk = [(pk, v, dict(NOMINAL, **p)) for v in vs for _, p in cas_k]
    ta = [(pa, v, p) for v in vs for _, p in cas_a]
    with Pool(5) as pool:
        rk = pool.map(t_kart, tk); ra = pool.map(t_actuel, ta)
    i = j = 0
    for v in vs:
        print('\n=== %.2f m/s ===  KART (PID %s)' % (v, pk))
        for nom, _ in cas_k:
            r = rk[i]; i += 1
            print('  %-24s %2d/%2d  temps %5.1f s  RMS %5.1f  max %5.1f  pertes %4.1f' % (nom, r['ok'], r['n'], r['temps'], r['rms'], r['emax'], r['pertes']))
        print('  ACTUEL (PID %s)' % (pa,))
        for nom, _ in cas_a:
            r = ra[j]; j += 1
            print('  %-24s %2d/%2d  temps %5.1f s  RMS %5.1f  max %5.1f  pertes %4.1f' % (nom, r['ok'], r['n'], r['temps'], r['rms'], r['emax'], r['pertes']))
