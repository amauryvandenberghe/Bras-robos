"""Simulation du suivi de ligne, version 2 (22/11/2026) : 6 capteurs (ou plus) sur le bloc de direction, loi de commande de bras_robot.ino.

Ce qui change par rapport a sim_ligne.py :
- geometrie relevee dans SolidWorks le 22/11 : essieu libre (x 177,25), pivot de direction (x 37,3), contact de la roue motrice
  (x 43,7) ; rangee de tete 31,8 / 40,3 mm devant le pivot, 2e rangee 74,7 / 83,2 mm derriere ; tout tourne avec la roue ;
- servo de direction realiste : consigne rafraichie a 50 Hz seulement, 1 trame de retard, 1er ordre 40 ms, 350 deg/s, zone morte 0,4 deg ;
- moteur avec seuil (PWM < 60 : il ne tourne pas, alimentation 5 V), freinage actif possible (L298N : IN1 = IN2) ;
- deux modeles de capteur (tache optique etroite ou large), plusieurs hauteurs et ecartements, 3 ou 5 capteurs par rangee ;
- deux estimateurs de position : barycentre (ancien) et ajustement d'un modele de reponse (nouveau, sans zone morte) ;
- loi de commande : la 2e rangee donne le CAP du bloc par rapport a la ligne (parallelisme), la rangee de tete l'ECART ;
  braquage vise = braquage actuel + k_cap * cap + atan(k_e * ecart / (v + v0)) (loi de type Stanley appliquee a la roue avant),
  lisse et rafraichi en continu (pas a-coups) ; vitesse variable selon braquage, cap et ecart ; freinage avant les virages.
Piste : celle du reglement (sim_ligne.construire_piste), rayon 230 mm, et une variante a 200 mm (rayon mini du reglement).

Usage : python3 sim_ligne2.py etude [essais] | final | tracer
"""
import json
import math
import os
import random
import sys
import warnings
from multiprocessing import Pool

warnings.filterwarnings('ignore')
import numpy as np
from scipy.spatial import cKDTree

import sim_ligne

ICI = os.path.dirname(os.path.abspath(__file__))
LARGEUR = 19.0


# ---------------------------------------------------------------- pistes
def piste(rayon):
    sim_ligne.R_PISTE = rayon
    p = sim_ligne.construire_piste()
    # densifier a 0,5 mm (distance au trace exacte a 0,02 mm pres)
    d = np.sqrt((np.diff(p, axis=0) ** 2).sum(1))
    s = np.concatenate([[0], np.cumsum(d)])
    st = np.arange(0, s[-1], 0.5)
    pts = np.stack([np.interp(st, s, p[:, 0]), np.interp(st, s, p[:, 1])], 1)
    return pts, cKDTree(pts)


PISTES = {}


def get_piste(rayon):
    if rayon not in PISTES:
        PISTES[rayon] = piste(rayon)
    return PISTES[rayon]


# ---------------------------------------------------------------- robot
L_EMP = 133.55      # essieu libre -> contact de la roue motrice
S_PIVOT = 139.95    # essieu libre -> pivot de direction
A_AV, A_AV_C = 31.8, 40.3     # rangee de tete : lateraux, central (devant le pivot)
A_AR, A_AR_C = -74.7, -83.2   # 2e rangee : lateraux, central (derriere le pivot)


def rangee(n, pas, a_lat, a_cen):
    """Capteurs (a le long du bloc, b lateral + a gauche), de gauche a droite."""
    if n == 3:
        return [(a_lat, pas), (a_cen, 0.0), (a_lat, -pas)]
    if n == 5:   # chevron : les cartes se decalent en x pour tenir a 10 mm d'ecart
        return [(a_lat - 6, 2 * pas), (a_lat + 2, pas), (a_cen, 0.0), (a_lat + 2, -pas), (a_lat - 6, -2 * pas)]
    raise ValueError(n)


def phi(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def sigma_tache(h, modele):
    """Demi-largeur (ecart-type) de la tache vue par un TCRT5000 a la hauteur h (mm). HYPOTHESES."""
    return 2.5 + 0.45 * h if modele == 'large' else 1.0 + 0.33 * h


def amplitude(h):
    return math.exp(-(((h - 3.0) / 6.0) ** 2))


def F(u, sig):
    """Part de la tache sur la ligne (vectorise), u = ecart lateral capteur - ligne (mm)."""
    from scipy.special import erf
    k = 1.0 / (sig * math.sqrt(2.0))
    return 0.5 * (erf((LARGEUR / 2 - u) * k) - erf((-LARGEUR / 2 - u) * k))


# ---------------------------------------------------------------- estimation de la position (portee telle quelle en C)
SIGMA_MODELE = 3.0
P_SEUILS = [0.0, 0.5]    # bary2 : ligne vue si somme >= [0] et un capteur >= [1] ; 26/11 nuit : [0.35, 0.2] essaye sur le robot, MOINS BON -> retour a [0, 0.5]
GRILLE = np.arange(-45.0, 45.01, 0.5)


class Estimateur:
    def __init__(self, capteurs, methode):
        self.b = np.array([c[1] for c in capteurs])
        self.methode = methode
        self.R = F(self.b[:, None] - GRILLE[None, :], SIGMA_MODELE)   # reponse attendue de chaque capteur pour chaque position

    def __call__(self, n):
        """n : valeurs normalisees 0 (blanc) .. 1 (noir). Retourne (vu, position mm, + = ligne a gauche)."""
        if self.methode == 'bary2':   # 26/11 soir, comme bras_robot.ino : vu si un capteur > 50 %, barycentre sans les 20 % du bas
            if n.sum() < P_SEUILS[0] or n.max() < P_SEUILS[1]:   # 26/11 nuit : somme (ligne entre 2 capteurs ecartes : chacun la voit a moitie)
                return False, 0.0
            w = np.clip((n - 0.2) / 0.8, 0.0, 1.0)
            return True, float((self.b * w).sum() / w.sum())
        if n.max() < 0.25:
            return False, 0.0
        if self.methode == 'barycentre':
            return True, float((self.b * n).sum() / n.sum())
        c = ((self.R - n[:, None]) ** 2).sum(0)
        i = int(c.argmin())
        e = GRILLE[i]
        if 0 < i < len(GRILLE) - 1:
            a, b_, d = c[i - 1], c[i], c[i + 1]
            den = a - 2 * b_ + d
            if den > 1e-9:
                e += 0.5 * (a - d) / den * 0.5
        return True, float(e)


# ---------------------------------------------------------------- loi de commande (portee telle quelle dans bras_robot.ino)
P_DEFAUT = dict(k_cap=0.8, k_e=6.0, v0=0.15, tau=0.04, vitesse_servo=400.0, braq_max=45.0, v_max=255.0, v_min=110.0, braq_lent=30.0,
                cap_lent=20.0, ecart_lent=25.0, v_perdu=110.0, accel=900.0, tau_servo=0.05, v255_estime=0.6, avant_seul=False)


class Suiveur:
    def __init__(self, P, est_av, est_ar, d_rangees):
        self.P = P; self.est_av = est_av; self.est_ar = est_ar; self.D = d_rangees
        self.braq = 0.0          # consigne lissee (deg, + = vers la gauche)
        self.braq_envoye = 0.0   # derniere consigne envoyee au servo (50 Hz)
        self.braq_est = 0.0      # position estimee du servo
        self.braq_f = 0.0        # braquage filtre (courbure)
        self.v = 0.0             # consigne moteur (PWM)
        self.v_est = 0.0         # vitesse estimee (m/s)
        self.cote = 1; self.t_vue = 0.0; self.t_servo = 0.0; self.arret = False; self.frein = False
        self.pertes = 0; self.vu_prec = True; self.demarre = False

    def pas(self, t, dt, n_av, n_ar):
        P = self.P
        vu_av, e_av = self.est_av(n_av)
        vu_ar, e_ar = self.est_ar(n_ar) if not P['avant_seul'] else (False, 0.0)
        if not self.demarre:
            if not vu_av:
                return 0.0, 0.0, False
            self.demarre = True; self.t_vue = t
        cap = 0.0
        if vu_av and P.get('loi') == 'pid':   # 26/11 : PID sur l'ecart (braquage absolu = Kp e + Ki integrale + Kd derivee filtree)
            self.t_vue = t
            if abs(e_av) > 4.0:
                self.cote = 1 if e_av > 0 else -1
            if not hasattr(self, 'I'):
                self.I = 0.0; self.e_prec = e_av; self.D_f = 0.0
            if not self.vu_prec:
                self.e_prec = e_av          # ligne retrouvee : pas de pic de derivee
            de =(e_av - self.e_prec) / dt; self.e_prec = e_av
            self.D_f += (de - self.D_f) * (1.0 - math.exp(-dt / 0.02))
            brut = P['kp'] * e_av + P['ki'] * self.I + P['kd'] * self.D_f
            if abs(brut) < P['braq_max'] or brut * e_av < 0:   # anti-emballement : on n'integre pas en butee
                self.I += e_av * dt
            if P['ki'] > 0:
                lim = P['braq_max'] / P['ki']; self.I = max(-lim, min(lim, self.I))
            cible = P['kp'] * e_av + P['ki'] * self.I + P['kd'] * self.D_f
        elif vu_av:
            self.t_vue = t
            if abs(e_av) > 4.0:
                self.cote = 1 if e_av > 0 else -1
            v_ms = max(0.0, self.v_est)
            corr = math.degrees(math.atan(P['k_e'] * e_av * 1e-3 / (v_ms + P['v0'])))
            if vu_ar:
                cap = math.degrees(math.atan2(e_av - e_ar, self.D))
                cible = self.braq_est + P['k_cap'] * cap + corr
            else:
                cible = self.braq_est + corr
        elif vu_ar:   # la tete a quitte la ligne, la 2e rangee la voit : on revient du cote ou la tete l'a perdue
            cible = P['braq_max'] * self.cote
        else:
            cible = P['braq_max'] * self.cote
            if t - self.t_vue < P.get('tenue_perte', 0.0):   # 26/11 nuit : perte breve -> la roue garde son angle
                cible = self.braq
            if t - self.t_vue > 1.5:
                self.arret = True
                return 0.0, 0.0, False
        if vu_av != self.vu_prec and not vu_av:
            self.pertes += 1
        self.vu_prec = vu_av
        cible = max(-P['braq_max'], min(P['braq_max'], cible))
        d = (cible - self.braq) * (1.0 - math.exp(-dt / P['tau']))
        lim = P['vitesse_servo'] * dt
        self.braq += max(-lim, min(lim, d))
        # le servo ne recoit une nouvelle consigne que toutes les 20 ms
        if t - self.t_servo >= 0.0199:
            self.t_servo = t; self.braq_envoye = self.braq
        de = (self.braq_envoye - self.braq_est) * (1.0 - math.exp(-dt / P['tau_servo']))
        self.braq_est += max(-350.0 * dt, min(350.0 * dt, de))
        self.braq_f += (self.braq_est - self.braq_f) * (1.0 - math.exp(-dt / 0.08))
        # vitesse : pleine en ligne droite et parallele, reduite selon braquage, cap et ecart ; freinage si besoin
        if P.get('vitesse2'):   # 26/11 nuit : virage = braquage MOYEN (signe, 0,3 s : l'oscillation s'annule) ; ecart moyen (0,2 s)
            if not hasattr(self, 'b_lp'):
                self.b_lp = 0.0; self.e_lp = 0.0
            self.b_lp += (self.braq_est - self.b_lp) * (1.0 - math.exp(-dt / P.get('t_virage', 0.3)))
            self.e_lp += ((abs(e_av) if vu_av else 30.0) - self.e_lp) * (1.0 - math.exp(-dt / 0.2))
        if vu_av and P.get('vitesse2'):
            gene = abs(self.b_lp) / P['braq_lent'] + self.e_lp / P['ecart_lent']
            gene = max(0.0, gene - P.get('gene0', 0.0))
            vc = P['v_max'] - (P['v_max'] - P['v_min']) * min(1.0, gene)
        elif vu_av:
            gene = abs(self.braq_f) / P['braq_lent'] + abs(e_av) / P['ecart_lent'] + (abs(cap) / P['cap_lent'] if vu_ar else 0.3)
            vc = P['v_max'] - (P['v_max'] - P['v_min']) * min(1.0, gene)
        else:
            vc = P['v_perdu']
        if vc > self.v:
            self.v = min(vc, self.v + P['accel'] * dt)
        else:
            self.v = vc
        v_vise = P['v255_estime'] * max(0.0, (self.v - 60.0) / 195.0)
        self.frein = self.v_est > v_vise + 0.08 and P.get('frein', True)
        self.v_est += (v_vise - self.v_est) * (1.0 - math.exp(-dt / (0.05 if self.frein else 0.15)))
        return self.braq_envoye, self.v, self.frein


# ---------------------------------------------------------------- simulation
def simuler(conf, P, graine=0, rayon=230.0, v255=0.6, tmax=40.0, trace=False):
    """conf : dict(n=3|5, pas, h, modele, methode). Retourne un bilan."""
    rnd = random.Random(graine)
    pts, arbre = get_piste(rayon)
    npts = len(pts)
    av = [tuple(c) for c in conf['capteurs']] if 'capteurs' in conf else rangee(conf['n'], conf['pas'], A_AV, A_AV_C)   # 27/11 : position libre (a devant le pivot, b + = gauche)
    ar = rangee(3, conf.get('pas_ar', conf['pas'] if conf['n'] == 3 else 13.0), A_AR, A_AR_C)
    caps = av + ar
    na = len(av)
    sig = sigma_tache(conf['h'], conf['modele'])
    amp = amplitude(conf['h'])
    nrnd = np.random.default_rng(graine)
    gains = 1.0 + 0.05 * nrnd.standard_normal(len(caps))      # etalonnage imparfait
    bruit_n = 18.0 / (3000.0 * amp)                           # bruit ADC rapporte a l'ecart noir-blanc
    D = (sum(c[0] for c in av) / na) - (sum(c[0] for c in ar) / 3)
    s = Suiveur(dict(P), Estimateur(av, conf['methode']), Estimateur(ar, conf['methode']), D)
    v255r = v255 * (1.0 + rnd.gauss(0, 0.05))
    dt = 0.005
    # depart : sur la ligne, petit ecart et petit cap
    e0 = rnd.uniform(-6, 6); c0 = math.radians(rnd.uniform(-5, 5))
    x, y = float(pts[0][0]), float(pts[0][1]) + e0
    psi = c0
    x += 100.0 - S_PIVOT; v = 0.0; servo = 0.0; servo_cmd = 0.0; file_cmd = [0.0] * max(1, int(round(conf.get('sv_retard', 0.005) / 0.005)))
    t = 0.0; idx = 0; hors = 0.0
    errs = []; rates = []; alat = 0.0; chemin = []
    prec_cmd = 0.0
    ok = False; cause = 'temps'; pertes_fin = None
    while t < tmax:
        c, sn = math.cos(psi), math.sin(psi)
        pxv, pyv = x + S_PIVOT * c, y + S_PIVOT * sn          # pivot
        pb = psi + math.radians(servo)
        cb, sb = math.cos(pb), math.sin(pb)
        q = np.array([(pxv + a * cb - b * sb, pyv + a * sb + b * cb) for a, b in caps])
        dist, _ = arbre.query(q)
        n = np.clip(gains * F(dist, sig) + bruit_n * nrnd.standard_normal(len(caps)), 0.0, 1.0)
        cmd, pwm, frein = s.pas(t, dt, n[:na], n[na:])
        if s.arret:
            cause = 'ligne perdue'
            break
        rates.append((cmd - prec_cmd) / dt); prec_cmd = cmd
        # servo : retard (1 pas par defaut), 1er ordre 40 ms, 350 deg/s, zone morte 0,4 deg (reglables dans conf : sv_retard s, sv_tau, sv_vmax)
        servo_cmd = file_cmd.pop(0); file_cmd.append(cmd)
        if abs(servo_cmd - servo) > 0.4:
            dsv = (servo_cmd - servo) * (1 - math.exp(-dt / conf.get('sv_tau', 0.04)))
            vm = conf.get('sv_vmax', 350.0)
            servo += max(-vm * dt, min(vm * dt, dsv))
        # moteur : seuil 60, freinage actif
        if not s.demarre:
            vcible = 0.0
        else:
            vcible = v255r * 1000.0 * max(0.0, (pwm - 60.0) / 195.0)
        tau = 0.05 if (frein and v > vcible) else 0.15
        v += (vcible - v) * (1 - math.exp(-dt / tau))
        dl = math.radians(servo)
        psi_p = v * math.sin(dl) / L_EMP
        psi += psi_p * dt
        x += v * math.cos(dl) * math.cos(psi) * dt; y += v * math.cos(dl) * math.sin(psi) * dt
        alat = max(alat, abs(v * psi_p) / 9810.0)
        t += dt
        # ecart du contact de la roue motrice
        fx, fy = x + L_EMP * math.cos(psi), y + L_EMP * math.sin(psi)
        de, j = arbre.query((fx, fy))
        if abs(j - idx) < 400:
            idx = max(idx, j)
        errs.append(de)
        if trace and len(errs) % 4 == 0:
            chemin.append((fx, fy, de, servo, v))
        if de > 45.0:
            hors += dt
            if hors > 0.3:
                cause = 'sortie'
                break
        else:
            hors = 0.0
        if pertes_fin is None and idx >= npts - 300:
            pertes_fin = s.pertes
        if idx >= npts - 10:
            ok = True; cause = 'fin'
            break
    e = np.array(errs) if errs else np.zeros(1)
    r = np.array(rates) if rates else np.zeros(1)
    return dict(ok=ok, cause=cause, temps=t, progres=idx / npts, rms=float(np.sqrt((e ** 2).mean())), emax=float(e.max()),
                pertes=s.pertes if pertes_fin is None else pertes_fin, vit_servo=float(np.sqrt((r ** 2).mean())), alat=alat, chemin=chemin)


GRAINES_OPT = (0, 1, 2)
RAYONS_OPT = (230.0, 200.0)


def score(conf, P, graines=GRAINES_OPT, rayons=RAYONS_OPT, v255=0.6):
    res = [simuler(conf, P, g, r, v255) for g in graines for r in rayons]
    nok = sum(r['ok'] for r in res)
    if nok < len(res):
        return 1000.0 - 500.0 * sum(r['progres'] for r in res) / len(res), res
    tm = sum(r['temps'] for r in res) / len(res)
    pen = sum(r['pertes'] for r in res) / len(res) * 1.0 + sum(r['vit_servo'] for r in res) / len(res) / 100.0
    pen += 5.0 * sum(max(0.0, r['alat'] - 0.35) for r in res) / len(res)
    return tm + pen, res


BORNES = (('k_cap', 0.0, 1.5), ('k_e', 1.0, 30.0), ('tau', 0.01, 0.2), ('v_max', 120.0, 255.0), ('v_min', 70.0, 255.0),
          ('braq_lent', 5.0, 80.0), ('cap_lent', 3.0, 60.0), ('ecart_lent', 5.0, 60.0), ('vitesse_servo', 100.0, 600.0))


def optimiser(args):
    nom, conf, essais, graine, v255 = args
    rnd = random.Random(graine)
    P = dict(P_DEFAUT, avant_seul=conf.get('avant_seul', False))
    best, _ = score(conf, P, v255=v255)
    for k in range(essais):
        Q = dict(P)
        ech = 0.35 if k < essais // 2 else 0.15
        for cle, lo, hi in BORNES:
            if cle == 'k_cap' and Q['avant_seul']:
                continue
            if rnd.random() < 0.5:
                Q[cle] = min(hi, max(lo, Q[cle] * math.exp(rnd.gauss(0, ech)) if Q[cle] > 0.02 else rnd.uniform(0, 0.3)))
        Q['v_min'] = min(Q['v_min'], Q['v_max'])
        sc, _ = score(conf, Q, v255=v255)
        if sc < best:
            best, P = sc, Q
    # validation sur d'autres graines
    val, res = score(conf, P, graines=(10, 11, 12, 13), v255=v255)
    return nom, conf, P, best, val, res


def resume(res):
    ok = sum(r['ok'] for r in res)
    return '%d/%d reussis, temps %.1f s, ecart RMS %.1f mm, max %.0f mm, pertes de vue %.1f, servo %.0f deg/s RMS, a lat %.2f g' % (
        ok, len(res), np.mean([r['temps'] for r in res]), np.mean([r['rms'] for r in res]), max(r['emax'] for r in res),
        np.mean([r['pertes'] for r in res]), np.mean([r['vit_servo'] for r in res]), max(r['alat'] for r in res))


def etude(essais):
    confs = {}
    for modele in ('etroite', 'large'):
        for h in (3.0, 5.0, 8.0):
            for n, pas in ((3, 20.0), (3, 13.0), (5, 10.0)):
                for methode in ('barycentre', 'modele'):
                    confs['%s h%.0f %dx%.0f %s' % (modele, h, n, pas, methode)] = dict(n=n, pas=pas, h=h, modele=modele, methode=methode)
    taches = [(k, c, essais, i, 0.6) for i, (k, c) in enumerate(confs.items())]
    out = {}
    with Pool(6) as pool:
        for nom, conf, P, best, val, res in pool.imap_unordered(optimiser, taches):
            print('%-32s score %.2f  validation %.2f  %s' % (nom, best, val, resume(res)), flush=True)
            out[nom] = dict(conf=conf, P=P, score=best, validation=val, resume=resume(res))
    json.dump(out, open(os.path.join(ICI, 'etude_capteurs.json'), 'w'), indent=1)


# ---------------------------------------------------------------- 2e passe : configuration retenue, robot plus rapide, servo doux
POIDS_SERVO = 3.0   # secondes de penalite par 100 deg/s RMS de consigne (la douceur compte autant que la vitesse)


def score_robuste(conf, P, graines=(0, 1), v255=1.0):
    res = []
    for modele in ('etroite', 'large'):
        c = dict(conf, modele=modele)
        res += [simuler(c, P, g, r, v255) for g in graines for r in RAYONS_OPT]
    nok = sum(r['ok'] for r in res)
    if nok < len(res):
        return 1000.0 - 500.0 * sum(r['progres'] for r in res) / len(res), res
    tm = sum(r['temps'] for r in res) / len(res)
    pen = sum(r['pertes'] for r in res) / len(res) * 1.0 + POIDS_SERVO * sum(r['vit_servo'] for r in res) / len(res) / 100.0
    pen += 5.0 * sum(max(0.0, r['alat'] - 0.35) for r in res) / len(res)
    return tm + pen, res


def optimiser2(args):
    nom, conf, essais, graine, v255 = args
    rnd = random.Random(graine)
    P = dict(P_DEFAUT, avant_seul=conf.get('avant_seul', False), v255_estime=v255)
    best, _ = score_robuste(conf, P, v255=v255)
    for k in range(essais):
        Q = dict(P)
        ech = 0.35 if k < essais // 2 else 0.15
        for cle, lo, hi in BORNES:
            if cle == 'k_cap' and Q['avant_seul']:
                continue
            if rnd.random() < 0.5:
                Q[cle] = min(hi, max(lo, Q[cle] * math.exp(rnd.gauss(0, ech)) if Q[cle] > 0.02 else rnd.uniform(0, 0.3)))
        Q['v_min'] = min(Q['v_min'], Q['v_max'])
        sc, _ = score_robuste(conf, Q, v255=v255)
        if sc < best:
            best, P = sc, Q
    bilan = {}
    for v in (0.6, 1.0):
        for modele in ('etroite', 'large'):
            Pv = dict(P, v255_estime=v)
            res = [simuler(dict(conf, modele=modele), Pv, g, r, v) for g in (10, 11, 12) for r in RAYONS_OPT]
            bilan['%.1f m/s %s' % (v, modele)] = resume(res)
    return nom, conf, P, best, bilan


BORNES_PID = (('kp', 0.2, 8.0), ('ki', 0.0, 40.0), ('kd', 0.0, 0.6), ('tau', 0.01, 0.2), ('v_min', 70.0, 255.0),
              ('braq_lent', 5.0, 80.0), ('ecart_lent', 5.0, 60.0), ('vitesse_servo', 100.0, 600.0))


def optimiser_pid(args):
    nom, conf, essais, graine, v255 = args
    rnd = random.Random(graine)
    P = dict(P_DEFAUT, loi='pid', kp=3.0, ki=5.0, kd=0.05, tau=0.02, vitesse_servo=400.0, v_min=150.0, braq_lent=60.0,
             ecart_lent=30.0, avant_seul=True, v255_estime=v255)
    best, _ = score_robuste(conf, P, v255=v255)
    for k in range(essais):
        Q = dict(P)
        ech = 0.35 if k < essais // 2 else 0.15
        for cle, lo, hi in BORNES_PID:
            if rnd.random() < 0.5:
                Q[cle] = min(hi, max(lo, Q[cle] * math.exp(rnd.gauss(0, ech)) if Q[cle] > 0.02 else rnd.uniform(0, 0.3 * hi)))
        Q['v_min'] = min(Q['v_min'], Q['v_max'])
        sc, _ = score_robuste(conf, Q, v255=v255)
        if sc < best:
            best, P = sc, Q
    bilan = {}
    for v in (0.6, 1.0):
        for modele in ('etroite', 'large'):
            Pv = dict(P, v255_estime=v)
            res = [simuler(dict(conf, modele=modele), Pv, g, r, v) for g in (10, 11, 12) for r in RAYONS_OPT]
            bilan['%.1f m/s %s' % (v, modele)] = resume(res)
    return nom, conf, P, best, bilan


def final(essais):
    confs = {'3x13 h5 (tete + cap)': dict(n=3, pas=13.0, h=5.0, methode='modele'),
             '3x13 h5 (tete seule)': dict(n=3, pas=13.0, h=5.0, methode='modele', avant_seul=True),
             '3x20 h5 (tete + cap, ecart actuel)': dict(n=3, pas=20.0, h=5.0, methode='modele'),
             '3x13 h3 (tete + cap)': dict(n=3, pas=13.0, h=3.0, methode='modele'),
             '3x13 h8 (tete + cap)': dict(n=3, pas=13.0, h=8.0, methode='modele'),
             '5x10 h5 (tete + cap)': dict(n=5, pas=10.0, h=5.0, methode='modele')}
    taches = [(k, c, essais, 7 + i, 1.0) for i, (k, c) in enumerate(confs.items())]
    out = {}
    with Pool(6) as pool:
        for nom, conf, P, best, bilan in pool.imap_unordered(optimiser2, taches):
            print('=== %s : score %.2f' % (nom, best), flush=True)
            print('    reglages', {k: round(v, 3) for k, v in P.items() if isinstance(v, float)}, flush=True)
            for k, v in bilan.items():
                print('    %-16s %s' % (k, v), flush=True)
            out[nom] = dict(conf=conf, P=P, score=best, bilan=bilan)
    json.dump(out, open(os.path.join(ICI, 'reglages_finaux.json'), 'w'), indent=1)


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'etude'
    if mode == 'etude':
        etude(int(sys.argv[2]) if len(sys.argv) > 2 else 60)
    elif mode == 'final':
        final(int(sys.argv[2]) if len(sys.argv) > 2 else 60)
    elif mode == 'lent':   # reglages pour le robot actuel (L298N en 5 V : PWM 255 ~ 0,6 m/s, a mesurer)
        nom, conf, P, best, bilan = optimiser2(('3x13 h5 (tete + cap) a 0,6 m/s', dict(n=3, pas=13.0, h=5.0, methode='modele'), int(sys.argv[2]), 21, 0.6))
        print('=== %s : score %.2f' % (nom, best)); print('    reglages', {k: round(v, 3) for k, v in P.items() if isinstance(v, float)})
        for k, v in bilan.items():
            print('    %-16s %s' % (k, v))
        json.dump(dict(conf=conf, P=P, score=best, bilan=bilan), open(os.path.join(ICI, 'reglages_0.6.json'), 'w'), indent=1)
    elif mode == 'bary':   # 26/11 : barycentre, 3 capteurs de tete seuls (2e rangee non branchee), 0,6 m/s au plus
        conf = dict(n=3, pas=13.0, h=5.0, methode='barycentre', avant_seul=True)
        essais = int(sys.argv[2]) if len(sys.argv) > 2 else 40
        taches = [('3x13 h5 barycentre, tete seule, graine %d' % g, conf, essais, 30 + g, 0.6) for g in range(4)]
        out = []
        with Pool(4) as pool:
            for nom, conf_, P, best, bilan in pool.imap_unordered(optimiser2, taches):
                print('=== %s : score %.2f' % (nom, best), flush=True)
                print('    reglages', {k: round(v, 3) for k, v in P.items() if isinstance(v, float)}, flush=True)
                for k, v in bilan.items():
                    print('    %-16s %s' % (k, v), flush=True)
                out.append(dict(nom=nom, P=P, score=best, bilan=bilan))
        out.sort(key=lambda d: d['score'])
        print('MEILLEUR :', out[0]['nom'], round(out[0]['score'], 2), flush=True)
        json.dump(dict(conf=conf, resultats=out), open(os.path.join(ICI, 'reglages_barycentre.json'), 'w'), indent=1)
    elif mode == 'pid':   # 26/11 : PID (Kp, Ki, Kd) sur le barycentre des 3 capteurs de tete, 0,6 m/s au plus
        conf = dict(n=3, pas=13.0, h=5.0, methode='barycentre', avant_seul=True)
        essais = int(sys.argv[2]) if len(sys.argv) > 2 else 40
        taches = [('PID barycentre, graine %d' % g, conf, essais, 50 + g, 0.6) for g in range(4)]
        out = []
        with Pool(4) as pool:
            for nom, conf_, P, best, bilan in pool.imap_unordered(optimiser_pid, taches):
                print('=== %s : score %.2f' % (nom, best), flush=True)
                print('    reglages', {k: round(v, 3) for k, v in P.items() if isinstance(v, float)}, flush=True)
                for k, v in bilan.items():
                    print('    %-16s %s' % (k, v), flush=True)
                out.append(dict(nom=nom, P=P, score=best, bilan=bilan))
        out.sort(key=lambda d: d['score'])
        print('MEILLEUR :', out[0]['nom'], round(out[0]['score'], 2), flush=True)
        json.dump(dict(conf=conf, resultats=out), open(os.path.join(ICI, 'reglages_pid.json'), 'w'), indent=1)
    elif mode == 'un':
        conf = dict(n=3, pas=13.0, h=5.0, modele='etroite', methode='modele')
        import time
        t0 = time.time()
        sc, res = score(conf, P_DEFAUT)
        print('score %.2f' % sc, resume(res), '%.1f s' % (time.time() - t0))
        for r in res:
            print(r['cause'], round(r['temps'], 1), round(r['progres'], 2), round(r['rms'], 1))
