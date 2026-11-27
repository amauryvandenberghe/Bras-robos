"""PCB du robot MAVE (27/11/2026) : carte d'accueil de l'ESP32 DevKit V1 (30 broches) qui regroupe TOUS les branchements :
4 capteurs de ligne TCRT5000 (36, 39, 34, 35), 6 servos (base 21, epaule 4, coude 13, poignet 22, pince 16, direction 17),
L298N (ENA 25, IN1 33, IN2 32), alimentation 5 V des servos (sortie de l'abaisseur) + condensateur 1000 uF, cavalier 5 V -> VIN
(ESP32 alimente par l'abaisseur), connecteur d'extension (27, 14, 3V3, GND).
Tout est traversant (soudure au fer), 2 couches, 100 x 72 mm, pistes >= 0,3 mm, ecarts >= 0,3 mm (JLCPCB : 0,127 mm mini).
Face du dessous : plan de masse. L'ESP32 se branche sur 2 barrettes femelles 1 x 15 : 3e rangee de trous pour les modules
dont les rangees sont a 22,86 mm au lieu de 25,4 mm (mesurer le sien : souder la barrette droite dans la bonne rangee).

Sorties (dossier donne en argument) : Gerber (GTL GBL GTS GBS GTO GBO GKO), percage (DRL PTH + NPTH), zip pour la commande,
apercus PNG, nomenclature, liste des liaisons. Le script verifie : ecarts entre equipotentielles differentes sur chaque couche,
continuite de chaque equipotentielle (pistes, plan de masse, trous metallises), ecart au bord.
Usage : python pcb_mave.py <dossier de sortie>
"""
import math
import os
import sys
import zipfile
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

W, H = 100.0, 72.0
R_COIN = 3.0
P = 2.54
PISTE, PISTE_FINE, PISTE_5V = 0.4, 0.25, 2.0
ECART = 0.3                 # ecart minimal verifie entre cuivres differents
ECART_PLAN = 0.4            # ecart du plan de masse
PAD_BARRETTE, TROU_BARRETTE = 1.6, 1.0
PAD_BORNIER, TROU_BORNIER = 2.6, 1.3
PAD_VIA, TROU_VIA = 1.0, 0.5

# ------------------------------------------------------------------ composants
pads = []      # dict(x, y, d, trou, net, ref, carre)
pistes = []    # dict(couche 'T'|'B', pts, l, net)
vias = []      # dict(x, y, net)
seri = []      # (texte, x, y, hauteur, rotation, ancrage)  serigraphie du dessus
seri_traits = []  # polylignes de serigraphie (x, y)...
trous_fix = [(4.0, 4.0), (96.0, 4.0), (4.0, 68.0), (96.0, 68.0)]


def pad(x, y, net, ref, d=PAD_BARRETTE, trou=TROU_BARRETTE, carre=False):
    pads.append(dict(x=x, y=y, d=d, trou=trou, net=net, ref=ref, carre=carre))


def piste(couche, net, *pts, l=PISTE):
    pistes.append(dict(couche=couche, pts=list(pts), l=l, net=net))


def via(x, y, net):
    vias.append(dict(x=x, y=y, net=net))


def texte(t, x, y, h=1.2, rot=0, ancre='c'):
    seri.append((t, x, y, h, rot, ancre))


# ESP32 DevKit V1, 30 broches, USB en bas, vu de dessus
XL, XR, XR2 = 34.0, 34.0 + 25.4, 34.0 + 22.86
YK = lambda k: 64.0 - P * (k - 1)
GAUCHE = ['EN', 'VP36', 'VN39', 'D34', 'D35', 'D32', 'D33', 'D25', 'D26', 'D27', 'D14', 'D12', 'D13', 'GND', 'VIN']
DROITE = ['D23', 'D22', 'TX0', 'RX0', 'D21', 'D19', 'D18', 'D5', 'D17', 'D16', 'D4', 'D2', 'D15', 'GND', '3V3']
NET = {'GND': 'GND', 'VIN': 'VIN', '3V3': '3V3'}
for k, n in enumerate(GAUCHE, 1):
    pad(XL, YK(k), NET.get(n, n), 'U1', carre=(k == 1))
for k, n in enumerate(DROITE, 1):
    pad(XR, YK(k), NET.get(n, n), 'U1')
    pad(XR2, YK(k), NET.get(n, n), 'U1b')
    piste('T', NET.get(n, n), (XR2, YK(k)), (XR, YK(k)), l=0.5)          # 2 rangees de la barrette droite reliees
# contour du module et reperes
seri_traits.append([(XL - 2.4, YK(15) - 9.5), (XR + 2.4, YK(15) - 9.5), (XR + 2.4, YK(1) + 7.0), (XL - 2.4, YK(1) + 7.0), (XL - 2.4, YK(15) - 9.5)])
texte('ESP32 DevKit V1', (XL + XR) / 2 - 2.5, 25.0, 1.3)
texte('30 broches - USB en bas', (XL + XR) / 2, 21.6, 1.0)
texte('antenne', (XL + XR) / 2, YK(1) + 4.0, 1.2)
texte('22,86', XR2 - 0.6, YK(15) - 2.0, 0.8)
texte('25,4', XR + 0.9, YK(15) - 3.4, 0.8)
for k, n in enumerate(GAUCHE, 1):
    texte(n.replace('VP36', '36').replace('VN39', '39').replace('D', ''), XL + 2.1, YK(k), 0.9, ancre='g')
for k, n in enumerate(DROITE, 1):
    texte(n.replace('D', ''), XR2 - 2.0, YK(k), 0.9, ancre='d')

# --- 4 capteurs de ligne : rangees "S + -" (AO, 3V3, GND) alignees sur 36 / 39 / 34 / 35
XG, X3, XS = 5.0, 5.0 + P, 5.0 + 2 * P
CAPT = [('C1 ext G', 'VP36', 2), ('C2 int G', 'VN39', 3), ('C3 int D', 'D34', 4), ('C4 ext D', 'D35', 5)]
for nom, net, k in CAPT:
    y = YK(k)
    pad(XG, y, 'GND', 'J1'); pad(X3, y, '3V3', 'J1'); pad(XS, y, net, 'J1', carre=True)
    piste('T', net, (XS, y), (XL, y))
    texte(nom, 22.0, y + 1.0, 0.9)
texte('CAPTEURS', 15.0, YK(1) + 3.6, 1.3)
texte('- GND  + 3V3  S AO', 15.0, YK(1) + 1.9, 0.8)
texte('-  +  S', X3, YK(1) + 0.3, 1.0)

# --- L298N : IN2 (32), IN1 (33), ENA (25), GND
for net, k, lib in (('D32', 6, 'IN2 32'), ('D33', 7, 'IN1 33'), ('D25', 8, 'ENA 25')):
    pad(XS, YK(k), net, 'J2', carre=(k == 6)); piste('T', net, (XS, YK(k)), (XL, YK(k)))
    texte(lib, XS - 1.6, YK(k), 0.9, ancre='d')
pad(XS, YK(9), 'GND', 'J2'); texte('GND', XS - 1.6, YK(9), 0.9, ancre='d')
texte('L298N', 22.0, YK(6) + 1.1, 1.1)
texte('(enlever le cavalier ENA)', 22.0, (YK(8) + YK(9)) / 2, 0.8)

# --- extension : 27, 14, 3V3, GND
for net, k, lib in (('D27', 10, '27'), ('D14', 11, '14')):
    pad(XS, YK(k), net, 'J3', carre=(k == 10)); piste('T', net, (XS, YK(k)), (XL, YK(k)))
    texte(lib, XS - 1.6, YK(k), 0.9, ancre='d')
pad(XS, YK(12), '3V3', 'J3'); texte('3V3', XS - 1.6, YK(12), 0.9, ancre='d')
pad(XS, YK(13), 'GND', 'J3'); texte('GND', XS - 1.6, YK(13), 0.9, ancre='d')
texte('EXTENSION', 22.0, YK(10) + 1.1, 1.0)

# 3V3 : face du dessous, de la broche 3V3 aux capteurs et a l'extension
Y3 = 22.0
piste('B', '3V3', (XR, YK(15)), (XR, Y3), (X3, Y3), (X3, YK(2)), l=0.6)
piste('B', '3V3', (X3, YK(12)), (XS, YK(12)), l=0.6)

# --- 6 servos : rangees "S + -" alignees sur leurs broches
XSV, XP, XM = 70.0, 70.0 + P, 70.0 + 2 * P
SERVOS = [('POIGNET 22', 'D22', 2), ('BASE 21', 'D21', 5), ('DIRECTION 17', 'D17', 9), ('PINCE 16', 'D16', 10), ('EPAULE 4', 'D4', 11)]
for nom, net, k in SERVOS:
    y = YK(k)
    pad(XSV, y, net, 'J4', carre=True); pad(XP, y, '5V', 'J4'); pad(XM, y, 'GND', 'J4')
    piste('T', net, (XR, y), (XSV, y))
    texte(nom, XM + 2.2, y, 1.0, ancre='g')
# coude (13, cote gauche) : passe sous le module, entre les broches 12 et 13 de droite
yc = YK(12)
pad(XSV, yc, 'D13', 'J4', carre=True); pad(XP, yc, '5V', 'J4'); pad(XM, yc, 'GND', 'J4')
ym = (YK(12) + YK(13)) / 2
piste('T', 'D13', (XL, YK(13)), (XL + 1.6, YK(13)), (XL + 1.6 + (ym - YK(13)), ym), (XSV - 3.2, ym), (XSV - 3.2 + (yc - ym), yc), (XSV, yc), l=PISTE_FINE)
texte('COUDE 13', XM + 2.2, yc, 1.0, ancre='g')
texte('SERVOS', XP, YK(1) + 2.6, 1.3)
texte('S  +  -', XP, YK(1) + 0.4, 1.0)
# rails + et - des servos (dessus), du coude au poignet
piste('T', '5V', (XP, yc), (XP, YK(2)), l=1.6)
piste('T', 'GND', (XM, yc), (XM, YK(2)), l=1.6)

# --- alimentation 5 V des servos (sortie de l'abaisseur) : bornier + condensateurs
XB, YB5, YBG = 93.0, 26.0, 26.0 - 5.08
pad(XB, YB5, '5V', 'J5', PAD_BORNIER, TROU_BORNIER, carre=True); pad(XB, YBG, 'GND', 'J5', PAD_BORNIER, TROU_BORNIER)
texte('+5V', XB + 1.8, YB5, 1.0, ancre='g'); texte('GND', XB + 1.8, YBG, 1.0, ancre='g')
texte('ALIM SERVOS', XB - 2.0, YB5 + 7.4, 1.1)
texte('(abaisseur 5-6 V)', XB - 2.0, YB5 + 5.7, 0.8)
XC1, XC2 = 85.0, 78.5
pad(XC1, YB5, '5V', 'C1', 1.8, 1.0, carre=True); pad(XC1, YBG, 'GND', 'C1', 1.8, 1.0)
texte('C1 1000uF 16V', XC1, YBG - 6.6, 0.9); texte('+', XC1 - 2.0, YB5 + 1.7, 1.2)
seri_traits.append([(XC1 + 5.0 * math.cos(a), (YB5 + YBG) / 2 + 5.0 * math.sin(a)) for a in [i * 2 * math.pi / 48 for i in range(49)]])
pad(XC2, YB5, '5V', 'C2', 1.6, 0.9); pad(XC2, YBG, 'GND', 'C2', 1.6, 0.9)
texte('C2 100nF', XC2, YBG - 2.4, 0.8)
piste('T', '5V', (XB, YB5), (XP, YB5), l=PISTE_5V)

# --- cavalier 5V -> VIN (ESP32 alimente par l'abaisseur ; l'enlever pour le televersement si on veut)
YJ = 16.0
pad(XL, YJ, 'VIN', 'JP1', carre=True); pad(XL + P, YJ, '5V', 'JP1')
piste('T', 'VIN', (XL, YK(15)), (XL, YJ), l=0.6)
piste('T', '5V', (XL + P, YJ), (XL + P + 2.0, YJ - 2.0), (XP, YJ - 2.0), (XP, yc), l=1.0)
texte('JP1 5V->VIN', XL + P / 2, YJ - 3.4, 0.9)

texte('MAVE - robot suiveur + bras', 50.0, 6.4, 1.6)
texte('TIPe ICAM 2026-2027  v1  27/11/2026', 50.0, 3.8, 1.1)

# ------------------------------------------------------------------ geometrie
def contour():
    return box(R_COIN, 0, W - R_COIN, H).union(box(0, R_COIN, W, H - R_COIN)).union(unary_union(
        [Point(x, y).buffer(R_COIN, 64) for x in (R_COIN, W - R_COIN) for y in (R_COIN, H - R_COIN)]))


def forme_pad(p):
    if p['carre']:
        r = p['d'] / 2
        return box(p['x'] - r, p['y'] - r, p['x'] + r, p['y'] + r)
    return Point(p['x'], p['y']).buffer(p['d'] / 2, 32)


def forme_piste(t):
    return LineString(t['pts']).buffer(t['l'] / 2, 16)


def forme_via(v):
    return Point(v['x'], v['y']).buffer(PAD_VIA / 2, 32)


def cuivre(couche):
    """{net: [formes]} sur une couche (les pastilles traversantes sont sur les 2)."""
    d = defaultdict(list)
    for p in pads:
        d[p['net']].append(forme_pad(p))
    for v in vias:
        d[v['net']].append(forme_via(v))
    for t in pistes:
        if t['couche'] == couche:
            d[t['net']].append(forme_piste(t))
    return d


def plan_de_masse():
    bord = contour().buffer(-0.6)
    autres = []
    for net, fs in cuivre('B').items():
        if net != 'GND':
            autres += [f.buffer(ECART_PLAN) for f in fs]
    autres += [Point(x, y).buffer(3.2 / 2 + 1.2) for x, y in trous_fix]
    plan = bord.difference(unary_union(autres))
    # liaison des pastilles de masse : degagement + 4 rayons de 0,6 mm (soudure plus facile)
    gnd = [p for p in pads if p['net'] == 'GND']
    for p in gnd:
        r = p['d'] / 2
        plan = plan.difference(Point(p['x'], p['y']).buffer(r + ECART_PLAN, 32))
    rayons = []
    for p in gnd:
        r = p['d'] / 2 + ECART_PLAN + 0.6
        rayons += [box(p['x'] - r, p['y'] - 0.3, p['x'] + r, p['y'] + 0.3), box(p['x'] - 0.3, p['y'] - r, p['x'] + 0.3, p['y'] + r)]
    autres_u = unary_union(autres)
    plan = plan.union(unary_union(rayons).difference(autres_u).intersection(bord))
    plan = plan.buffer(-0.15).buffer(0.15)       # pas de lamelles fines
    morceaux = list(plan.geoms) if isinstance(plan, MultiPolygon) else [plan]
    gnd_u = unary_union([forme_pad(p) for p in gnd])
    return unary_union([m for m in morceaux if m.intersects(gnd_u) and m.area > 2.0])


def polygones_texte():
    fp = FontProperties(family='DejaVu Sans', weight='bold')
    out = []
    for t, x, y, h, rot, ancre in seri:
        tp = TextPath((0, 0), t, size=h * 1.37, prop=fp)
        polys = []
        for c in tp.to_polygons(closed_only=True):
            if len(c) >= 3:
                pg = Polygon(c).buffer(0)
                if not pg.is_empty:
                    polys.append(pg)
        # anneaux exterieurs / trous : pair-impair
        g = Polygon()
        for pg in sorted(polys, key=lambda q: -q.area):
            g = g.symmetric_difference(pg)
        bx = g.bounds
        dx = {'c': -(bx[0] + bx[2]) / 2, 'g': -bx[0], 'd': -bx[2]}[ancre]
        g = affinity.translate(g, dx, -(bx[1] + bx[3]) / 2)
        g = affinity.rotate(g, rot, origin=(0, 0))
        out.append(affinity.translate(g, x, y))
    return unary_union(out)


# ------------------------------------------------------------------ verifications
def verifier():
    erreurs = []
    plan = plan_de_masse()
    for couche in ('T', 'B'):
        c = cuivre(couche)
        if couche == 'B':
            c['GND'].append(plan)
        nets = {n: unary_union(f) for n, f in c.items()}
        noms = sorted(nets)
        for i, a in enumerate(noms):
            for b in noms[i + 1:]:
                d = nets[a].distance(nets[b])
                if d < ECART - 1e-6:
                    erreurs.append('%s : %s / %s a %.3f mm' % (couche, a, b, d))
        bord = contour().exterior
        for n, g in nets.items():
            if g.distance(bord) < 0.5:
                erreurs.append('%s : %s a %.2f mm du bord' % (couche, n, g.distance(bord)))
        for x, y in trous_fix:
            for n, g in nets.items():
                if g.distance(Point(x, y)) < 1.6 + 0.8:
                    erreurs.append('%s : %s trop pres du trou de fixation (%g, %g)' % (couche, n, x, y))
    # continuite : chaque equipotentielle doit former un seul morceau (dessus + dessous relies par les trous)
    for net in sorted({p['net'] for p in pads}):
        morceaux = []
        for couche in ('T', 'B'):
            fs = cuivre(couche).get(net, [])
            if couche == 'B' and net == 'GND':
                fs = fs + [plan]
            u = unary_union(fs)
            morceaux += [(couche, g) for g in (u.geoms if hasattr(u, 'geoms') else [u])]
        parent = list(range(len(morceaux)))

        def f(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        trous = [Point(p['x'], p['y']) for p in pads if p['net'] == net] + [Point(v['x'], v['y']) for v in vias if v['net'] == net]
        for h in trous:
            idx = [i for i, (_, g) in enumerate(morceaux) if g.contains(h)]
            for i in idx[1:]:
                parent[f(i)] = f(idx[0])
        groupes = {f(i) for i in range(len(morceaux))}
        nb_pads = sum(1 for p in pads if p['net'] == net)
        if len(groupes) > 1 and nb_pads > 1:
            # un groupe sans pastille ni via (isole) ne compte pas comme une coupure si c'est un bout de piste ? non : erreur
            erreurs.append('%s : %d morceaux non relies' % (net, len(groupes)))
    return erreurs, plan


# ------------------------------------------------------------------ Gerber
def g_entete(f, nom):
    f.write('G04 MAVE robot - %s*\n%%FSLAX46Y46*%%\n%%MOMM*%%\n%%LPD*%%\n' % nom)


def c6(v):
    return '%d' % round(v * 1e6)


def g_region(f, pg):
    for anneau, sombre in [(pg.exterior, True)] + [(i, False) for i in pg.interiors]:
        f.write('%LPD*%\n' if sombre else '%LPC*%\n')
        pts = list(anneau.coords)
        f.write('G36*\nX%sY%sD02*\n' % (c6(pts[0][0]), c6(pts[0][1])))
        for x, y in pts[1:]:
            f.write('G01X%sY%sD01*\n' % (c6(x), c6(y)))
        f.write('G37*\n')
    f.write('%LPD*%\n')


def g_formes(f, geom):   # les plus grandes d'abord : un ilot dans le trou d'une autre forme n'est pas efface
    for pg in sorted((geom.geoms if hasattr(geom, 'geoms') else [geom]), key=lambda q: -q.area):
        if not pg.is_empty:
            g_region(f, pg)


class Ouvertures:
    def __init__(self, f):
        self.f = f; self.d = {}; self.n = 10

    def __call__(self, cle):
        if cle not in self.d:
            forme, a = cle
            self.f.write('%%ADD%d%s,%.4f%s*%%\n' % (self.n, forme, a, 'X%.4f' % a if forme == 'R' else ''))
            self.d[cle] = self.n; self.n += 1
        return self.d[cle]


def g_cuivre(chemin, couche, plan=None):
    with open(chemin, 'w') as f:
        g_entete(f, 'cuivre ' + ('dessus' if couche == 'T' else 'dessous'))
        if plan is not None:
            g_formes(f, plan)
        ap = Ouvertures(f)
        for p in pads:
            f.write('D%d*\nX%sY%sD03*\n' % (ap(('R' if p['carre'] else 'C', p['d'])), c6(p['x']), c6(p['y'])))
        for v in vias:
            f.write('D%d*\nX%sY%sD03*\n' % (ap(('C', PAD_VIA)), c6(v['x']), c6(v['y'])))
        for t in pistes:
            if t['couche'] != couche:
                continue
            f.write('D%d*\n' % ap(('C', t['l'])))
            x, y = t['pts'][0]
            f.write('X%sY%sD02*\n' % (c6(x), c6(y)))
            for x, y in t['pts'][1:]:
                f.write('G01X%sY%sD01*\n' % (c6(x), c6(y)))
        f.write('M02*\n')


def g_masque(chemin):
    with open(chemin, 'w') as f:
        g_entete(f, 'vernis epargne (ouvertures)')
        ap = Ouvertures(f)
        for p in pads:
            f.write('D%d*\nX%sY%sD03*\n' % (ap(('R' if p['carre'] else 'C', p['d'] + 0.1)), c6(p['x']), c6(p['y'])))
        for x, y in trous_fix:
            f.write('D%d*\nX%sY%sD03*\n' % (ap(('C', 6.0)), c6(x), c6(y)))
        f.write('M02*\n')


def g_seri(chemin, polys, traits):
    with open(chemin, 'w') as f:
        g_entete(f, 'serigraphie dessus')
        g_formes(f, polys)
        ap = Ouvertures(f)
        f.write('D%d*\n' % ap(('C', 0.2)))
        for tr in traits:
            f.write('X%sY%sD02*\n' % (c6(tr[0][0]), c6(tr[0][1])))
            for x, y in tr[1:]:
                f.write('G01X%sY%sD01*\n' % (c6(x), c6(y)))
        f.write('M02*\n')


def g_vide(chemin, nom):
    with open(chemin, 'w') as f:
        g_entete(f, nom); f.write('M02*\n')


def g_contour(chemin):
    with open(chemin, 'w') as f:
        g_entete(f, 'contour')
        f.write('%ADD10C,0.1000*%\nD10*\n')
        pts = list(contour().exterior.coords)
        f.write('X%sY%sD02*\n' % (c6(pts[0][0]), c6(pts[0][1])))
        for x, y in pts[1:]:
            f.write('G01X%sY%sD01*\n' % (c6(x), c6(y)))
        f.write('M02*\n')


def percage(chemin, trous, plaque):
    outils = sorted({d for _, _, d in trous})
    with open(chemin, 'w') as f:
        f.write('M48\n; MAVE robot - trous %s\nFMAT,2\nMETRIC,TZ\n' % ('metallises' if plaque else 'NON metallises'))
        f.write('; #@! TF.FileFunction,%s,1,2,%s\n' % ('Plated' if plaque else 'NonPlated', 'PTH' if plaque else 'NPTH'))
        for i, d in enumerate(outils, 1):
            f.write('T%dC%.3f\n' % (i, d))
        f.write('%\nG90\nG05\n')
        for i, d in enumerate(outils, 1):
            f.write('T%d\n' % i)
            for x, y, dd in trous:
                if dd == d:
                    f.write('X%.3fY%.3f\n' % (x, y))
        f.write('T0\nM30\n')


# ------------------------------------------------------------------ apercu
def apercu(chemin, plan, polys, face):
    fig, ax = plt.subplots(figsize=(11, 8), dpi=130)
    ax.set_facecolor('white')
    def tracer(g, **kw):
        for pg in (g.geoms if hasattr(g, 'geoms') else [g]):
            if pg.is_empty:
                continue
            xs, ys = pg.exterior.xy; ax.fill(xs, ys, **kw)
            for i in pg.interiors:
                xs, ys = i.xy; ax.fill(xs, ys, color='#1b5e20' if face == 'T' else '#0d47a1')
    tracer(contour(), color='#1b5e20' if face == 'T' else '#0d47a1')
    if face == 'B':
        tracer(plan, color='#d4a017', alpha=0.55)
    for t in pistes:
        if t['couche'] == face:
            tracer(forme_piste(t), color='#ffcc33' if face == 'T' else '#ff8f00')
    for p in pads:
        tracer(forme_pad(p), color='#e0e0e0')
        ax.add_patch(plt.Circle((p['x'], p['y']), p['trou'] / 2, color='black'))
    for v in vias:
        tracer(forme_via(v), color='#e0e0e0'); ax.add_patch(plt.Circle((v['x'], v['y']), TROU_VIA / 2, color='black'))
    for x, y in trous_fix:
        ax.add_patch(plt.Circle((x, y), 1.6, color='black'))
    if face == 'T':
        tracer(polys, color='white')
        for tr in seri_traits:
            ax.plot([a for a, _ in tr], [b for _, b in tr], color='white', lw=0.8)
    ax.set_aspect('equal'); ax.set_xlim(-2, W + 2); ax.set_ylim(-2, H + 2)
    ax.set_title('PCB MAVE - face %s (%g x %g mm)' % ('du dessus (composants)' if face == 'T' else 'du dessous (plan de masse), vue de dessus', W, H))
    if face == 'B':
        ax.invert_xaxis()
        ax.set_title(ax.get_title().replace('vue de dessus', 'vue de dessous'))
    fig.savefig(chemin, bbox_inches='tight'); plt.close(fig)


# ------------------------------------------------------------------ programme
if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fabrication')
    gerb = os.path.join(out, 'gerber')
    os.makedirs(gerb, exist_ok=True)
    erreurs, plan = verifier()
    polys = polygones_texte()
    nom = 'MAVE_robot'
    g_cuivre(os.path.join(gerb, nom + '.GTL'), 'T')
    g_cuivre(os.path.join(gerb, nom + '.GBL'), 'B', plan)
    g_masque(os.path.join(gerb, nom + '.GTS'))
    g_masque(os.path.join(gerb, nom + '.GBS'))
    g_seri(os.path.join(gerb, nom + '.GTO'), polys, seri_traits)
    g_vide(os.path.join(gerb, nom + '.GBO'), 'serigraphie dessous (vide)')
    g_contour(os.path.join(gerb, nom + '.GKO'))
    percage(os.path.join(gerb, nom + '-PTH.DRL'), [(p['x'], p['y'], p['trou']) for p in pads] + [(v['x'], v['y'], TROU_VIA) for v in vias], True)
    percage(os.path.join(gerb, nom + '-NPTH.DRL'), [(x, y, 3.2) for x, y in trous_fix], False)
    with zipfile.ZipFile(os.path.join(out, nom + '_gerber_JLCPCB.zip'), 'w', zipfile.ZIP_DEFLATED) as z:
        for fn in sorted(os.listdir(gerb)):
            z.write(os.path.join(gerb, fn), fn)
    apercu(os.path.join(out, 'apercu_dessus.png'), plan, polys, 'T')
    apercu(os.path.join(out, 'apercu_dessous.png'), plan, polys, 'B')
    # liaisons (pour verifier le cablage)
    with open(os.path.join(out, 'liaisons.txt'), 'w', encoding='utf-8') as f:
        par_net = defaultdict(list)
        for p in pads:
            par_net[p['net']].append('%s(%.2f;%.2f)' % (p['ref'], p['x'], p['y']))
        for n in sorted(par_net):
            f.write('%-6s %d pastilles : %s\n' % (n, len(par_net[n]), ' '.join(par_net[n])))
    print('pastilles %d, pistes %d, vias %d, trous de fixation %d' % (len(pads), len(pistes), len(vias), len(trous_fix)))
    print('plan de masse : %.0f mm2 (%.0f %% de la carte)' % (plan.area, 100 * plan.area / contour().area))
    print('VERIFICATIONS :', 'aucune erreur' if not erreurs else '%d ERREURS' % len(erreurs))
    for e in erreurs:
        print('  ', e)
    sys.exit(1 if erreurs else 0)
