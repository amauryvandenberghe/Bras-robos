"""Simulation du suivi de ligne du robot (19/11/2026) : piste du reglement, capteurs analogiques TCRT5000, modele cinematique a 2 essieux.

Piste : dessinee d'apres le schema du reglement (Challenge 2026-2027, p. 5) : depart en bas a gauche, droite, virage a gauche, remontee,
virage, ondulation, demi-tour a gauche, ligne droite, 2 virages a gauche, trait de fin. Rayon mini 200 mm, ligne de 19 mm.
L'echelle exacte du schema n'est pas donnee : rayon de 210 a 250 mm (valeurs du reglement), longueurs lues sur le dessin.

Robot : essieu fixe + roue motrice et directrice a 133 mm (empattement mesure dans SolidWorks). Deux configurations :
  A = ancienne (roue directrice a l'ARRIERE, capteurs devant), B = inversee du 19/11 (roue directrice devant, capteurs derriere),
  C = hypothese : B avec les 2 rangees de capteurs DEVANT la roue directrice (a construire).
Le regulateur est un portage fidele de pas_suivi() de bras_robot.ino (position, derivee, cap avant-arriere, rampe, autoregulation).

Modele des capteurs (HYPOTHESES a recaler sur des mesures) : reponse = recouvrement d'une tache gaussienne (sigma = 2,5 + 0,45 h mm)
avec la bande de 19 mm ; amplitude maximale vers 3 mm de hauteur, decroissante ensuite : a(h) = exp(-((h-3)/6)^2) ; bruit 15 pas.
Vitesse : PWM 255 = 0,60 m/s (inconnu : alimentation 5 V), retard 0,12 s. Servo de direction : 400 deg/s, retard 0,05 s.

Usage : python3 sim_ligne.py [comparer | optimiser | tracer]
"""
import math
import random
import sys

import numpy as np

# ---------------------------------------------------------------- piste
R_PISTE = 230.0


def construire_piste():
    """Liste de points (mm) tous les 2 mm, de (0, 0) cap est, par segments droits et arcs exacts."""
    seg = [('S', 1900), ('A', +90, R_PISTE), ('S', 330), ('A', +90, R_PISTE), ('S', 380),
           ('A', +45, R_PISTE), ('A', -90, R_PISTE), ('A', +45, R_PISTE), ('S', 260),
           ('A', -180, 250.0), ('S', 1050), ('A', +90, R_PISTE), ('S', 160), ('A', +90, R_PISTE), ('S', 420)]
    x = y = 0.0
    th = 0.0
    pts = [(x, y)]
    for s in seg:
        if s[0] == 'S':
            n = int(s[1] / 2)
            for _ in range(n):
                x += 2.0 * math.cos(th); y += 2.0 * math.sin(th); pts.append((x, y))
        else:
            ang = math.radians(s[1]); r = s[2]
            n = max(2, int(abs(ang) * r / 2.0))
            dth = ang / n; ds = abs(ang) * r / n
            for _ in range(n):
                th += dth / 2; x += ds * math.cos(th); y += ds * math.sin(th); th += dth / 2; pts.append((x, y))
    return np.array(pts)


PISTE = construire_piste()
LONGUEUR = 2.0 * (len(PISTE) - 1)

# ---------------------------------------------------------------- capteurs
L_EMP = 133.0   # empattement (essieu fixe - roue directrice)


def phi(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def reponse(d, h, largeur=19.0):
    """Part de la tache vue sur la ligne, pour un ecart lateral d (mm) et une hauteur h (mm), entre 0 et 1."""
    sig = 2.5 + 0.45 * h
    a = math.exp(-(((h - 3.0) / 6.0) ** 2))
    f = phi((largeur / 2 - d) / sig) - phi((-largeur / 2 - d) / sig)
    return a * f


CONFIGS = {
    # nom : (sens de rotation sigma, s essieu fixe -> roue directrice, rangee 'avant' s, rangee 'arriere' s, signe lateral des indices)
    'A': dict(sigma=+1, s_dir=-L_EMP, s_av=+28.0, s_ar=-58.0, lat=+1),
    'B': dict(sigma=-1, s_dir=+L_EMP, s_av=+58.0, s_ar=-28.0, lat=-1),
    'C': dict(sigma=-1, s_dir=+L_EMP, s_av=+L_EMP + 30.0, s_ar=+L_EMP - 56.0, lat=-1),
    # D = B + 3 capteurs de tete FIXES AU BLOC DE DIRECTION (tournent avec la roue), 38 et 47 mm devant l'axe de la roue
    'D': dict(sigma=-1, s_dir=+L_EMP, s_av=None, s_ar=+58.0, lat=-1, roue=True, d_lat=38.2, d_cen=46.7),
    # E (20/11) = 6 capteurs sur le bloc de direction : 3 devant la roue, 3 derriere le moteur (68 et 77 mm derriere l'axe de la roue)
    'E': dict(sigma=-1, s_dir=+L_EMP, s_av=None, s_ar=None, lat=-1, roue=True, d_lat=38.2, d_cen=46.7, d_lat_ar=-68.3, d_cen_ar=-76.8),
}

# ---------------------------------------------------------------- regulateur (portage de bras_robot.ino)
PARAM_DEFAUT = dict(kp=30.0, kd=60.0, kh=15.0, v_droite=210.0, v_virage=125.0, v_perdu=120.0, v_demarrage=230.0, accel=700.0,
                    braquage_max=32.0, contraste_min=120, perte_max_ms=1500, auto=True, lissage=0.4, rate=300.0)


class Regulateur:
    def __init__(self, P):
        self.P = P
        self.cmin = [[0] * 3, [0] * 3]; self.cmax = [[0] * 3, [0] * 3]
        self.etat = 'attente'; self.pos_prec = 0.0; self.deriv = 0.0; self.v = 0.0; self.braq = 0.0
        self.dernier_cote = 1; self.derniere_vue = 0.0; self.facteur = 1.0; self.en_perte = False; self.dernier_ajust = 0.0
        self.init = False; self.arret = False

    def _mesurer(self, r, b):
        P = self.P
        cmin, cmax = self.cmin[r], self.cmax[r]
        if not self.init:
            for i in range(3):
                cmin[i] = cmax[i] = b[i]
        gmin = 4095; gmax = 0
        for i in range(3):
            cmin[i] = min(cmin[i], b[i]); cmax[i] = max(cmax[i], b[i])
            gmin = min(gmin, cmin[i]); gmax = max(gmax, cmax[i])
        if gmax - gmin < P['contraste_min']:
            return False, 0.0
        n = [0, 0, 0]
        for i in range(3):
            propre = cmax[i] - cmin[i] >= P['contraste_min']
            bas, haut = (cmin[i], cmax[i]) if propre else (gmin, gmax)
            x = min(1.0, max(0.0, (b[i] - bas) / (haut - bas)))
            n[i] = x
        t = n[0] + n[1] + n[2]
        if t >= 0.35:
            return True, (n[2] - n[0]) / t
        return False, 0.0

    def pas(self, t, dt, b_av, b_ar):
        """b_av, b_ar : 3 valeurs brutes. Retourne (braquage en degres, vitesse PWM 0..255)."""
        P = self.P
        vu_av, p_av = self._mesurer(0, b_av)
        vu_ar, p_ar = self._mesurer(1, b_ar)
        self.init = True
        vu = vu_av or vu_ar
        p = p_av; cap = 0.0
        if vu_av and vu_ar:
            cap = p_av - p_ar
        elif (not vu_av) and vu_ar:
            p = 1.3 * p_ar
        if self.etat == 'attente':
            if not vu_av:
                return 0.0, 0.0
            self.etat = 'roule'; self.derniere_vue = t; self.pos_prec = p; self.v = P['v_demarrage']
        if vu:
            self.derniere_vue = t; self.en_perte = False
            if abs(p) > 0.15:
                self.dernier_cote = 1 if p > 0 else -1
            if P['auto'] and abs(p) < 0.6 and t - self.dernier_ajust > 2.0:
                self.facteur = min(1.0, self.facteur + 0.04); self.dernier_ajust = t
        else:
            if not self.en_perte:
                self.en_perte = True
                if P['auto']:
                    self.facteur = max(0.55, self.facteur - 0.12)
                self.dernier_ajust = t
            if (t - self.derniere_vue) * 1000 > P['perte_max_ms']:
                self.arret = True
                return 0.0, 0.0
            p = 1.5 * self.dernier_cote
        if P.get('roue'):
            self.p_ar_roue = p_ar if vu_ar else None
            return self._pas_roue(t, dt, vu_av, p_av, vu)
        derive = (p - self.pos_prec) / dt
        self.pos_prec = p
        self.deriv += 0.5 * (derive - self.deriv)
        p_eff = 0.0 if abs(p) < 0.08 else p * (0.5 + 0.5 * abs(p))
        but = -(P['kp'] * p_eff + P['kd'] * 0.01 * self.deriv + P['kh'] * cap)
        but = max(-P['braquage_max'], min(P['braquage_max'], but))
        pas = P['rate'] * dt
        self.braq += max(-pas, min(pas, P['lissage'] * (but - self.braq)))
        gene = min(1.0, 1.6 * abs(p) + 0.08 * abs(self.deriv) + 1.0 * abs(cap))
        vc = (P['v_droite'] - (P['v_droite'] - P['v_virage']) * gene) * self.facteur if vu else P['v_perdu']
        if vc > self.v:
            self.v = min(vc, self.v + P['accel'] * dt)
        else:
            self.v = vc
        return self.braq, self.v


def _pas_roue(self, t, dt, vu_av, p, vu):
    """Capteurs sur la roue : on oriente la roue vers le point de ligne vu (poursuite), par petits pas."""
    P = self.P
    if vu_av:
        alpha = math.degrees(math.atan2(p * 20.0, P.get('d_vise', 42.0)))
        derive = (p - self.pos_prec) / dt; self.pos_prec = p
        self.deriv += 0.5 * (derive - self.deriv)
        cible = self.braq - P['g'] * alpha - P['kd'] * 0.01 * self.deriv
    elif getattr(self, 'p_ar_roue', None) is not None and P.get('arriere', True):
        alpha = math.degrees(math.atan2(self.p_ar_roue * 20.0, 40.0))    # l'avant a depasse : on suit la rangee arriere
        cible = self.braq - P['g'] * 1.5 * alpha
    else:
        cible = -P['braquage_max'] * self.dernier_cote
    cible = max(-P['braquage_max'], min(P['braquage_max'], cible))
    self.braq += (cible - self.braq) * min(1.0, dt / P['tau'])
    pr = getattr(self, 'p_ar_roue', None)
    gene = min(1.0, 1.2 * abs(p) + abs(self.braq) / P.get('braq_lent', 35.0) + (P.get('k_courbe', 0.0) * abs(p - pr) if (pr is not None and vu_av) else 0.0)) if vu_av else 1.0
    vc = (P['v_droite'] - (P['v_droite'] - P['v_virage']) * gene) * self.facteur if vu else P['v_perdu']
    self.v = min(vc, self.v + P['accel'] * dt) if vc > self.v else vc
    return self.braq, self.v


Regulateur._pas_roue = _pas_roue
PARAM_ROUE = dict(PARAM_DEFAUT, roue=True, g=1.0, tau=0.06, kd=0.0, braquage_max=45.0, braq_lent=35.0)

# ---------------------------------------------------------------- simulation
def simuler(cfg, P, hauteur=5.0, pitch=20.0, v255=0.60, tau_v=0.12, tau_s=0.05, rate_s=400.0, bruit=15.0, seed=0, tmax=60.0, trace=False,
            decalage0=0.0):
    rnd = random.Random(seed)
    c = CONFIGS[cfg]
    dt = 0.005
    reg = Regulateur(P)
    x, y = float(PISTE[0][0]), float(PISTE[0][1] + decalage0)
    phi_ = 0.0
    v = 0.0
    servo = 0.0; servo_cmd = 0.0; file_cmd = [0.0, 0.0]
    idx = 0
    n = len(PISTE)
    t = 0.0
    nperdu = 0
    maxerr = 0.0
    chemin = []
    blanc, noir = 600.0, 3600.0
    ok = False; cause = 'temps'
    while t < tmax:
        # position des 6 capteurs et lecture analogique (4 echantillons moyennes, bruit)
        fx, fy = math.cos(phi_), math.sin(phi_)
        nx, ny = -fy, fx
        lec = []
        for rang, s in enumerate((c['s_av'], c['s_ar'])):
            raw = []
            for i in range(3):
                lat = c['lat'] * (1 - i) * pitch
                if (rang == 0 and c.get('roue')) or (rang == 1 and c.get('d_lat_ar') is not None):
                    pw = phi_ - math.radians(servo)          # cap de la roue (le servo + fait tourner a droite)
                    d = (c['d_cen'] if i == 1 else c['d_lat']) if rang == 0 else (c['d_cen_ar'] if i == 1 else c['d_lat_ar'])
                    ax_ = x + c['s_dir'] * fx; ay_ = y + c['s_dir'] * fy
                    px = ax_ + d * math.cos(pw) - lat * math.sin(pw); py = ay_ + d * math.sin(pw) + lat * math.cos(pw)
                else:
                    px = x + s * fx + lat * nx; py = y + s * fy + lat * ny
                i0 = max(0, idx - 40); i1 = min(n, idx + 120)
                seg = PISTE[i0:i1]
                d = np.sqrt((seg[:, 0] - px) ** 2 + (seg[:, 1] - py) ** 2).min()
                a = reponse(d, hauteur)
                val = blanc * (1.0 - 0.0) + (noir - blanc) * a + rnd.gauss(0.0, bruit)
                raw.append(int(min(4095, max(0, val))))
            lec.append(raw)
        br, pwm = reg.pas(t, dt, lec[0], lec[1])
        if reg.arret:
            cause = 'ligne perdue (arret)'
            break
        # servo : retard d'un pas, 1er ordre, vitesse limitee
        servo_cmd = file_cmd.pop(0); file_cmd.append(br)
        cible = max(-45.0, min(45.0, servo_cmd))
        dservo = (cible - servo) * (1 - math.exp(-dt / tau_s))
        dservo = max(-rate_s * dt, min(rate_s * dt, dservo))
        servo += dservo
        # moteur
        vcible = pwm / 255.0 * v255 * 1000.0   # mm/s
        v += (vcible - v) * (1 - math.exp(-dt / tau_v))
        if reg.etat == 'attente':
            v = 0.0
        # cinematique (essieu fixe de reference)
        phi_ += c['sigma'] * v * math.tan(math.radians(servo)) / L_EMP * dt
        x += v * math.cos(phi_) * dt; y += v * math.sin(phi_) * dt
        t += dt
        # avancement sur la piste
        i0 = max(0, idx - 10); i1 = min(n, idx + 80)
        seg = PISTE[i0:i1]
        dd = np.sqrt((seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2)
        k = int(dd.argmin()); idx = i0 + k
        err = float(dd[k]); maxerr = max(maxerr, err)
        if trace and int(t / dt) % 4 == 0:
            chemin.append((x, y))
        if err > 90.0:
            nperdu += 1
            if nperdu > 80:
                cause = 'sortie de piste'
                break
        else:
            nperdu = 0
        if idx >= n - 3:
            ok = True; cause = 'fin'
            break
    return dict(ok=ok, cause=cause, temps=t, vmoy=(idx * 2.0) / max(t, 1e-6) / 1000.0, maxerr=maxerr, progres=idx / n, chemin=chemin)


def evaluer(cfg, P, graines=(0, 1, 2), **kw):
    res = [simuler(cfg, P, seed=s, **kw) for s in graines]
    nok = sum(r['ok'] for r in res)
    tmoy = sum(r['temps'] for r in res) / len(res)
    return nok, tmoy, res


def score(cfg, P, graines=(0, 1), **kw):
    nok, tmoy, res = evaluer(cfg, P, graines, **kw)
    prog = sum(r['progres'] for r in res) / len(res)
    if nok < len(res):
        return 1000.0 - 500.0 * prog, nok, tmoy
    return tmoy, nok, tmoy


def optimiser(cfg, essais=120, graine=1, **kw):
    rnd = random.Random(graine)
    roue = CONFIGS[cfg].get('roue')
    meilleur = dict(PARAM_ROUE if roue else PARAM_DEFAUT); bs = score(cfg, meilleur, **kw)[0]
    bornes = ((('g', 0.3, 2.5), ('tau', 0.02, 0.3), ('kd', 0, 60), ('braq_lent', 15, 80)) if roue else
              (('kp', 5, 90), ('kd', 0, 120), ('kh', 0, 60), ('braquage_max', 20, 45))) + (('v_droite', 120, 255), ('v_virage', 60, 255))
    for _ in range(essais):
        P = dict(meilleur)
        for k, lo, hi in bornes:
            if rnd.random() < 0.5:
                P[k] = min(hi, max(lo, P[k] * math.exp(rnd.gauss(0, 0.25)) if P[k] > 0 else rnd.uniform(0, 10)))
        P['v_virage'] = min(P['v_virage'], P['v_droite'])
        s = score(cfg, P, **kw)[0]
        if s < bs:
            bs, meilleur = s, P
            print('   ', cfg, 'mieux : %.2f s' % s, {k: round(v, 2) for k, v in P.items() if isinstance(v, float)}, flush=True)
    return meilleur, bs


if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'comparer'
    if mode == 'six':
        import json
        P0 = json.load(open('resultats_optimisation.json'))['D_0.6']['P']
        for v255 in (0.6, 0.9):
            for cfg, extra in (('D', {}), ('E', {}), ('E', dict(k_courbe=0.5))):
                P = dict(P0, **extra)
                nok, tm, res = evaluer(cfg, P, graines=(10, 11, 12, 13, 14), v255=v255)
                print(cfg, extra, 'PWM255=%.1f m/s' % v255, 'reussis %d/5' % nok, 'temps %.1f s' % tm, 'ecart max %.0f mm' % max(r['maxerr'] for r in res), flush=True)
    if mode == 'optimiser':
        import json
        out = {}
        for v255 in (0.6, 0.9):
            for cfg in (sys.argv[3].split(',') if len(sys.argv) > 3 else ('A', 'B', 'D')):
                print('=== config', cfg, 'PWM 255 =', v255, 'm/s', flush=True)
                P, s = optimiser(cfg, essais=int(sys.argv[2]) if len(sys.argv) > 2 else 80, v255=v255)
                nok, tm, res = evaluer(cfg, P, graines=(10, 11, 12, 13, 14), v255=v255)
                print('RESULTAT', cfg, v255, 'reussis %d/5' % nok, 'temps %.1f s' % tm, 'vmoy %.2f m/s' % (sum(r['vmoy'] for r in res) / 5),
                      'ecart max %.0f mm' % max(r['maxerr'] for r in res), flush=True)
                out['%s_%.1f' % (cfg, v255)] = dict(P=P, reussis=nok, temps=tm)
        json.dump(out, open('resultats_optimisation.json', 'w'), indent=1)
    print('piste : %.2f m, %d points' % (LONGUEUR / 1000, len(PISTE)))
    if mode == 'comparer':
        for cfg in ('A', 'B', 'C', 'D'):
            for h, pitch in ((12.0, 13.0), (5.0, 20.0)):
                nok, tm, res = evaluer(cfg, PARAM_ROUE if cfg == 'D' else PARAM_DEFAUT, hauteur=h, pitch=pitch)
                print('config %s hauteur %4.1f pitch %4.1f : %d/3 reussis, temps moyen %5.1f s, causes %s, vmoy %.2f m/s, ecart max %.0f mm' % (
                    cfg, h, pitch, nok, tm, [r['cause'] for r in res], res[0]['vmoy'], max(r['maxerr'] for r in res)))
