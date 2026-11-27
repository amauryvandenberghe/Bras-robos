"""28/11/2026 : GRANDE ETUDE kart (2 moteurs) / robot ACTUEL (roue directrice + servo), dans des conditions plus realistes et identiques.
Kart (simuler_kart) = sim_differentiel.simuler_diff + realisme ajoute :
  - seuil de demarrage des moteurs : une roue commandee sous V_SEUIL (m/s) s'arrete (PWM trop faible) ;
  - moteurs differents : gain gauche/droite tire au hasard (+-8 %) a chaque trajet ;
  - 1 pas (5 ms) de retard entre la mesure et la commande.
Actuel = sim_ligne2.simuler (deja realiste : servo a 50 Hz, retard, 1er ordre, zone morte, moteur avec seuil, servo normal ET lent).
Trajets : 3 rayons de virage (170, 200, 230 mm) x 2 modeles de tache des capteurs x 3 tirages de bruit = 18 (kart), 36 (actuel,
  x 2 vitesses de servo).
Phase 1 : grille large (1 tirage : 6 trajets kart, 12 actuel). Phase 2 : les 3 meilleurs reglages de chaque cas revus sur tous les
  trajets. Classement : 1) tous les trajets reussis, 2) temps moyen, 3) ecart moyen (RMS) + pertes de ligne.
Usage : python3 etude_complete.py   (resultats : etude_complete.log)
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
import sim_differentiel as D

V_SEUIL = 0.15        # m/s : en dessous, la roue s'arrete (seuil PWM du motoreducteur, hypothese)
DESACCORD = 0.08      # ecart de gain max entre les 2 moteurs
RAYONS = (170.0, 200.0, 230.0)
MODELES = ('etroite', 'large')
GRAINES1, GRAINES2 = (10,), (10, 11, 12)
VITESSES = (0.3, 0.45, 0.6, 0.8, 1.0, 1.2)


def geometrie(nom, L):
    if nom == '4 actuels':
        return [(L - 8.0, 19.5), (L, 7.6), (L, -7.6), (L - 8.0, -19.5)]
    if nom == '5 (15/33, 4/8)':
        return [(L - 8.0, 33.2), (L - 4.0, 15.2), (L, 0.0), (L - 4.0, -15.2), (L - 8.0, -33.2)]
    if nom == '5 (15/27, 0/4)':
        return [(L - 4.0, 27.2), (L, 15.2), (L, 0.0), (L, -15.2), (L - 4.0, -27.2)]
    raise ValueError(nom)


def simuler_kart(pid, v_base, graine, rayon, modele, capt, L, voie=136.0, tmax=60.0):
    kp, ki, kd, k_lent = pid
    rnd = random.Random(graine * 7919 + int(rayon))
    pts, arbre = S.get_piste(rayon)
    npts = len(pts)
    sig = S.sigma_tache(8.0, modele); amp = S.amplitude(8.0)
    nrnd = np.random.default_rng(graine)
    gains = 1.0 + 0.05 * nrnd.standard_normal(len(capt))
    g_mot = (1.0 + rnd.uniform(-DESACCORD, DESACCORD), 1.0 + rnd.uniform(-DESACCORD, DESACCORD))
    bruit_n = 18.0 / (3000.0 * amp)
    est = S.Estimateur(capt, 'bary2')
    dt = 0.005
    e0 = rnd.uniform(-6, 6); psi = math.radians(rnd.uniform(-5, 5))
    x, y = float(pts[0][0]) + 100.0 - L, float(pts[0][1]) + e0
    vl = vr = 0.0
    I = 0.0; e_prec = 0.0; Dd = 0.0; cote = 1; t_vue = 0.0; vu_prec = True; pertes = 0
    t = 0.0; idx = 0; hors = 0.0; errs = []; ok = False; v_cmd = 0.0; cmd_prec = (0.0, 0.0)
    while t < tmax:
        c, s = math.cos(psi), math.sin(psi)
        q = np.array([(x + a * c - b * s, y + a * s + b * c) for a, b in capt])
        dist, _ = arbre.query(q)
        n = np.clip(gains * S.F(dist, sig) + bruit_n * nrnd.standard_normal(len(capt)), 0.0, 1.0)
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
        vc = v_base / (1.0 + k_lent * abs(dv) / max(v_base, 1e-3))
        v_cmd = min(vc, v_cmd + 2.0 * dt) if vc > v_cmd else vc
        cmd = (v_cmd - dv / 2, v_cmd + dv / 2)
        appl, cmd_prec = cmd_prec, cmd                       # 1 pas de retard
        for k, cm in enumerate(appl):
            cm = 0.0 if abs(cm) < V_SEUIL else cm * g_mot[k]  # seuil puis desaccord des moteurs
            v = vl if k == 0 else vr
            dvw = (cm - v) * (1.0 - math.exp(-dt / D.TAU_MOTEUR))
            dvw = max(-D.ACC_MAX * dt, min(D.ACC_MAX * dt, dvw))
            if k == 0:
                vl += dvw
            else:
                vr += dvw
        v = (vl + vr) / 2 * 1000.0
        psi += (vr - vl) * 1000.0 / voie * dt
        x += v * math.cos(psi) * dt; y += v * math.sin(psi) * dt
        t += dt
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
        # arrivee : quand les CAPTEURS atteignent le bout de la ligne (sinon, a basse vitesse, le robot cherche la ligne disparue
        # en tournant et l'essieu n'arrive jamais : artefact) ; on ajoute le temps que met l'essieu pour y arriver (equitable)
        _, jc = arbre.query((x + L * math.cos(psi), y + L * math.sin(psi)))
        if idx >= npts - 10 or (jc >= npts - 10 and abs(jc - idx) < 400 + int(L)):
            ok = True
            t += L / 1000.0 / max(v / 1000.0, 0.05)
            break
    e = np.array(errs) if errs else np.zeros(1)
    return dict(ok=ok, temps=t, progres=idx / npts, rms=float(np.sqrt((e ** 2).mean())), emax=float(e.max()), pertes=pertes)


def bilan(res):
    ok = sum(r['ok'] for r in res)
    return dict(ok=ok, n=len(res), temps=float(np.mean([r['temps'] for r in res])), rms=float(np.mean([r['rms'] for r in res])),
                emax=float(max(r['emax'] for r in res)), pertes=float(np.mean([r['pertes'] for r in res])),
                progres=float(np.mean([r['progres'] for r in res])))


def tache_kart(a):
    geo, L, v, pid, graines = a
    capt = geometrie(geo, L)
    r = bilan([simuler_kart(pid, v, g, ra, m, capt, L) for g in graines for ra in RAYONS for m in MODELES])
    r.update(robot='kart', geo=geo, L=L, v=v, pid=pid)
    return r


def tache_actuel(a):
    v, pid, graines = a
    kp, ki, kd, tau, v_min = pid
    P = dict(S.P_DEFAUT, loi='pid', avant_seul=True, kp=kp, ki=ki, kd=kd, tau=tau, vitesse_servo=257.0, v_max=255.0, v_min=v_min,
             v255_estime=v)
    res = []
    for g in graines:
        for ra in RAYONS:
            for m in MODELES:
                for sv in ({}, EF.LENT):
                    conf = dict(n=4, pas=0, h=8.0, methode='bary2', avant_seul=True, modele=m, capteurs=EF.capteurs(7.6, 19.5, 5.5, 8.0), **sv)
                    res.append(S.simuler(conf, P, g, ra, v, tmax=90.0))
    r = bilan(res)
    r.update(robot='actuel', geo='4 sur le bloc', L=0.0, v=v, pid=pid)
    return r


def cle(r):
    if r['ok'] < r['n']:
        return (1, -r['ok'] / r['n'], -r['progres'], 0)
    return (0, round(r['temps'], 1), r['rms'] + 0.5 * r['pertes'], 0)


def meilleurs(rs, k=3):
    return sorted(rs, key=cle)[:k]


def ligne(r):
    return '%-7s %-16s L %3.0f | v %.2f | %2d/%2d  temps %5.1f s  RMS %5.1f  max %5.1f  pertes %4.1f | %s' % (
        r['robot'], r['geo'], r['L'], r['v'], r['ok'], r['n'], r['temps'], r['rms'], r['emax'], r['pertes'], r['pid'])


if __name__ == '__main__':
    geos = ('4 actuels', '5 (15/33, 4/8)', '5 (15/27, 0/4)')
    Ls = (85.0, 95.0, 110.0, 125.0)
    g_kart = list(itertools.product((0.004, 0.008, 0.015, 0.025, 0.04), (0.0, 0.01), (0.0005, 0.001, 0.002), (0.0, 1.0)))
    g_act = list(itertools.product((1.0, 1.6, 2.5, 4.0), (1.0, 2.0, 3.5), (0.03, 0.05), (0.04, 0.06, 0.1), (255.0, 170.0)))
    t1 = [(g, L, v, p, GRAINES1) for g in geos for L in Ls for v in VITESSES for p in g_kart]
    t1a = [(v, p, GRAINES1) for v in VITESSES for p in g_act]
    print('PHASE 1 : kart %d reglages x 6 trajets ; actuel %d reglages x 12 trajets' % (len(t1), len(t1a)), flush=True)
    with Pool(5) as pool:
        rk = pool.map(tache_kart, t1, chunksize=8)
        print('  kart fait', flush=True)
        ra = pool.map(tache_actuel, t1a, chunksize=2)
        print('  actuel fait', flush=True)
        t2 = [(g, L, v, b['pid'], GRAINES2) for g in geos for L in Ls for v in VITESSES
              for b in meilleurs([r for r in rk if r['geo'] == g and r['L'] == L and r['v'] == v])]
        t2a = [(v, b['pid'], GRAINES2) for v in VITESSES for b in meilleurs([r for r in ra if r['v'] == v])]
        print('PHASE 2 : %d + %d reglages revus sur tous les trajets (kart 18, actuel 36)' % (len(t2), len(t2a)), flush=True)
        rk2 = pool.map(tache_kart, t2, chunksize=2)
        ra2 = pool.map(tache_actuel, t2a, chunksize=1)
    print('\n=== KART : meilleur reglage pour chaque disposition, distance L et vitesse (18 trajets) ===')
    for g in geos:
        for L in Ls:
            for v in VITESSES:
                print(ligne(meilleurs([r for r in rk2 if r['geo'] == g and r['L'] == L and r['v'] == v], 1)[0]))
        print()
    print('=== ACTUEL : meilleur reglage pour chaque vitesse (36 trajets, servo normal et lent) ===')
    for v in VITESSES:
        print(ligne(meilleurs([r for r in ra2 if r['v'] == v], 1)[0]))
    print('\n=== FACE A FACE (kart : la meilleure disposition possible mecaniquement = L 110 ; et la meilleure toutes L confondues) ===')
    for v in VITESSES:
        a = meilleurs([r for r in ra2 if r['v'] == v], 1)[0]
        k110 = meilleurs([r for r in rk2 if r['v'] == v and r['L'] == 110.0], 1)[0]
        kt = meilleurs([r for r in rk2 if r['v'] == v], 1)[0]
        print('v %.2f | actuel %2d/%2d %5.1f s RMS %5.1f | kart L110 %2d/%2d %5.1f s RMS %5.1f (%s) | kart meilleur %2d/%2d %5.1f s RMS %5.1f (%s, L %.0f)' % (
            v, a['ok'], a['n'], a['temps'], a['rms'], k110['ok'], k110['n'], k110['temps'], k110['rms'], k110['geo'],
            kt['ok'], kt['n'], kt['temps'], kt['rms'], kt['geo'], kt['L']))
    # meilleur compromis : la vitesse la plus elevee ou tous les trajets passent
    print('\n=== VITESSE MAXIMALE SURE (tous les trajets reussis) ===')
    for nom, rs in (('actuel', ra2), ('kart L110 5 (15/33, 4/8)', [r for r in rk2 if r['L'] == 110.0 and r['geo'] == '5 (15/33, 4/8)']),
                    ('kart L110 4 actuels', [r for r in rk2 if r['L'] == 110.0 and r['geo'] == '4 actuels'])):
        sures = [r for r in rs if r['ok'] == r['n']]
        if sures:
            b = min(sures, key=lambda r: r['temps'])
            print('%-28s : %.2f m/s, %.1f s par trajet, RMS %.1f mm, PID %s' % (nom, b['v'], b['temps'], b['rms'], b['pid']))
        else:
            print('%-28s : aucune vitesse sure' % nom)
