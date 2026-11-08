"""Quel ecartement de capteurs pour une ligne de 19 mm ? (modele, 08/11/2026)
1. reponse optique d'un TCRT5000 a la hauteur h, ligne noire de 19 mm
2. estimation de la position par 3 capteurs analogiques (barycentre) : pente, zone morte, portee, ambiguite des traits
3. robot simule : essieu fixe a l'avant (x 177,26), roue directrice a l'arriere (x 37,26), virages de rayon 200
"""
import sys
import numpy as np

LIGNE = 19.0
DEMI = LIGNE / 2


def m_lambert(phi):
    return np.log(0.5) / np.log(np.cos(np.radians(phi)))


ME, MD = m_lambert(16.0), m_lambert(30.0)   # TCRT5000 (fiche constructeur) : emetteur +-16 deg, recepteur +-30 deg
PAS_LED = 3.6                                # entre emetteur et recepteur, mesure sur votre modele
NOIR = 0.08                                  # reflectance relative du scotch noir (a mesurer)


def noyau(h, axe_leds_en_travers, n=241, demi=30.0):
    """Poids de chaque point du sol dans le signal (x = en travers de la ligne, y = le long)."""
    xs = np.linspace(-demi, demi, n)
    X, Y = np.meshgrid(xs, xs, indexing='ij')
    ex, ey = (-PAS_LED / 2, 0.0) if axe_leds_en_travers else (0.0, -PAS_LED / 2)
    re2 = (X - ex) ** 2 + (Y - ey) ** 2 + h * h
    rd2 = (X + ex) ** 2 + (Y + ey) ** 2 + h * h
    ce, cd = h / np.sqrt(re2), h / np.sqrt(rd2)
    w = ce ** (ME + 1) / re2 * cd ** (MD + 1) / rd2
    return xs, w.sum(axis=1) / w.sum()


def reponse(h, axe_leds_en_travers):
    """Noirceur vue (0 = blanc, 1 = noir) en fonction de la distance capteur - centre de la ligne."""
    xs, p = noyau(h, axe_leds_en_travers)
    dx = xs[1] - xs[0]
    d = np.arange(-60, 60.01, 0.1)
    c = np.cumsum(p)
    # noirceur = poids de la tache sur la ligne (integrale cumulee interpolee, sinon la courbe fait des marches)
    v = np.interp(d + DEMI, xs + dx / 2, c, left=0, right=1) - np.interp(d - DEMI, xs + dx / 2, c, left=0, right=1)
    i10, i90 = np.searchsorted(c, 0.1), np.searchsorted(c, 0.9)
    return d, v, xs[i90] - xs[i10], dx


def analyse(s, d, v, bruit=0.01):
    e = np.arange(-45, 45.01, 0.1)
    n = [np.interp(e - p, d, v) for p in (-s, 0, s)]
    tot = n[0] + n[1] + n[2]
    est = s * (n[2] - n[0]) / np.maximum(tot, 1e-6)
    pente = np.gradient(est, e)
    c = np.abs(e) <= 2.0
    pente0 = pente[np.abs(e) < 0.05].mean()
    # portee : tant que le robot voit encore la ligne et que l'estimation croit
    ok = (tot > 0.15) & (pente > 0.02)
    i0 = len(e) // 2
    j = i0
    while j + 1 < len(e) and ok[j + 1]:
        j += 1
    portee = e[j]
    # regularite : ecart entre la pente la plus faible et la plus forte sur +-(portee-2)
    z = np.abs(e) <= max(portee - 2, 1)
    pmin, pmax = pente[z].min(), pente[z].max()
    z2 = np.abs(e) <= 12
    erreur_bruit = bruit * s * np.sqrt(2) / max(abs(pente0), 1e-6) / max(tot[i0], 1e-6)
    return dict(pente0=pente0, portee=portee, pmin=pmin, pmax=pmax, regul=pmin / pmax, bruit=erreur_bruit,
                cotes_centre=float(n[2][i0]), mini12=float(pente[z2].min()), trait=1.0 - max(n[0][i] if n[0][i] > n[2][i] else n[2][i] for i in range(i0, i0 + 1)))


# ---------------- parcours : droites et arcs de rayon 200 ----------------
def parcours():
    R, pts, x, y, th = 200.0, [], 0.0, 0.0, 0.0
    morceaux = [(0, 500), (1, np.pi / 2), (0, 150), (-1, np.pi / 2), (1, np.pi), (0, 200), (-1, np.pi * 0.75), (1, np.pi / 3), (0, 600)]
    for sens, q in morceaux:
        L = q if sens == 0 else R * q
        n = int(L / 0.5)
        for _ in range(n):
            pts.append((x, y))
            x += 0.5 * np.cos(th); y += 0.5 * np.sin(th)
            th += sens * 0.5 / R
    return np.array(pts)


LIG = parcours()


def ecart(px, py, i):
    a, b = max(i - 200, 0), min(i + 200, len(LIG) - 1)
    d = (LIG[a:b, 0] - px) ** 2 + (LIG[a:b, 1] - py) ** 2
    k = int(np.argmin(d)) + a
    k2 = min(k + 1, len(LIG) - 1); k1 = max(k2 - 1, 0)
    tx, ty = LIG[k2] - LIG[k1]
    return ((px - LIG[k, 0]) * (-ty) + (py - LIG[k, 1]) * tx) / max(np.hypot(tx, ty), 1e-9), k


def simule(s, d, v, a, V, Kp, Kd, graine=0, bruit=0.01):
    """a = distance capteurs - essieu fixe (mm) ; V en mm/s ; sortie : erreur, activite de la direction, ligne perdue."""
    rng = np.random.default_rng(graine)
    E = 140.0                           # empattement essieu - roue directrice
    dt_ctl, dt = 0.01, 0.002
    x, y, th = 0.0, 0.0, 0.0
    delta, delta_c = 0.0, 0.0           # angle de la roue directrice
    i = 0
    ee_f = 0.0; ee_d = 0.0; dernier = 0.0
    err, cmd, perdu, t, k_ctl = [], [], 0, 0.0, 0
    vmax = np.radians(250)              # servo en charge
    dmax = np.radians(45)
    nmax = int(30.0 / dt)
    for it in range(nmax):
        if it % int(dt_ctl / dt) == 0:
            px, py = x + a * np.cos(th), y + a * np.sin(th)
            e, i = ecart(px, py, i)
            if i >= len(LIG) - 40:
                break
            n = [np.interp(e - p, d, v) + rng.normal(0, bruit) for p in (-s, 0, s)]
            n = np.clip(n, 0, 1)
            tot = n.sum()
            if tot > 0.25:
                mes = s * (n[2] - n[0]) / tot
                dernier = mes
            else:
                perdu += 1
                mes = np.sign(dernier) * (2 * s)
            ee_d += 0.25 * (((mes - ee_f) / dt_ctl) - ee_d)
            ee_f = mes
            # e > 0 : capteur a gauche de la ligne (normale) -> tourner a droite ; courbure > 0 = gauche
            kap = -(Kp * mes + Kd * ee_d)
            delta_c = np.clip(np.arctan(kap * E), -dmax, dmax)
            err.append(e); cmd.append(delta_c)
        dd = np.clip(delta_c - delta, -vmax * dt, vmax * dt)
        delta += dd
        kap_r = np.tan(delta) / E
        x += V * dt * np.cos(th); y += V * dt * np.sin(th); th += V * dt * kap_r
        t += dt
    arrive = i >= len(LIG) - 45
    err, cmd = np.array(err), np.array(cmd)
    return dict(rms=float(np.sqrt((err ** 2).mean())), max=float(np.abs(err).max()), acoups=float(np.degrees(np.sqrt((np.diff(cmd) ** 2).mean()) / dt_ctl)),
                inversions=int(((np.diff(np.sign(np.diff(cmd))) != 0) & (np.abs(np.diff(cmd))[1:] > np.radians(0.5))).sum()), perdu=perdu, ok=arrive, t=t)


def meilleur(s, d, v, a, V):
    best = None
    for Kp in np.geomspace(1e-4, 3e-3, 11):
        for Kd in (0.0, 1e-5, 3e-5, 6e-5, 1e-4, 2e-4, 4e-4):
            rr = [simule(s, d, v, a, V, Kp, Kd, g) for g in range(1)]
            if not all(r['ok'] and r['perdu'] == 0 for r in rr):
                continue
            cout = np.mean([r['rms'] / 3.0 + r['acoups'] / 60.0 + r['max'] / 12.0 for r in rr])
            if best is None or cout < best[0]:
                best = (cout, Kp, Kd, rr[0])
    return best


if __name__ == '__main__':
    hs = [float(a) for a in sys.argv[1:]] or [12.9]
    for h in hs:
        for axe, nom in ((True, 'LEDs en travers de la ligne'), (False, 'LEDs le long de la ligne')):
            d, v, f, _ = reponse(h, axe)
            print(f'\n=== h = {h} mm, {nom} : largeur de tache 10-90 % = {f:.1f} mm, noirceur maxi {v.max():.2f}')
            print(' s  | pente au centre | portee | pente min/max | bruit 1 % -> erreur | cote au centre')
            for s in (10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 25, 30, 33):
                r = analyse(s, d, v)
                print(f'{s:3d} | {r["pente0"]:6.2f}          | +-{r["portee"]:4.1f} | {r["pmin"]:5.2f}/{r["pmax"]:4.2f}   | {r["bruit"]:5.2f} mm           | {r["cotes_centre"]:.2f}')
    h = hs[0]
    d, v, f, _ = reponse(h, True)
    print('\n=== robot simule, reglage optimise pour chaque ecartement, rangee avant a 22 mm devant l essieu')
    print('V mm/s | s  | Kp      Kd      | RMS mm | max mm | a-coups deg/s | perdu')
    for V in (300.0, 600.0):
        for s in (12, 15, 17, 19, 22, 26, 33):
            b = meilleur(s, d, v, 22.0, V)
            if b is None:
                print(f'{V:5.0f}  | {s:2d} | aucun reglage ne suit la ligne'); continue
            _, Kp, Kd, r = b
            print(f'{V:5.0f}  | {s:2d} | {Kp:.1e} {Kd:.1e} | {r["rms"]:5.2f}  | {r["max"]:5.1f}  | {r["acoups"]:7.1f}       | {r["perdu"]}', flush=True)
