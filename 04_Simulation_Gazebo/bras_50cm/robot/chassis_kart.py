"""PROPOSITION 2 (28/11/2026, a part) : ROBOT "KART" COMPACT, version 2 (demandes de l'utilisateur le 28/11 apres-midi) :
- plus de billes avant ni de patins arriere : UNE roulette a bille au centre, a l'avant ;
- 5 CAPTEURS EN FLECHE (etude_kart5.py, etude_kart5.log) : milieu en pointe a L_CAPT devant l'essieu, paire 1 a +-15,2 mm en
  retrait de 4 mm, paire 2 a +-33,2 mm en retrait de 8 mm ;
- PCB pas modelise pour le moment (place libre devant la tour) ;
- un petit trou par capteur pour les fils (au-dessus du connecteur de la carte) au lieu des grandes fenetres ;
- TOUTE la visserie dans l'assemblage, montage facile (ecrous captifs ou glisses par le cote, ordre de montage dans le LISEZMOI).
Roulette : PAS de modele dans le projet ni de reference dans la liste du materiel -> roulette a bille standard dessinee ici
(cotes R_* et H_ROUL a remplacer par celles de la vraie roulette ; le bloc du nez s'adapte).
Equilibre (stabilite_kart.py) : la roulette est DEVANT les capteurs (triangle d'appui long) ; derriere eux, le robot basculerait
dans 29 % des positions du bras.
Repere : celui d'Assemblage1v2 (mm ; x vers l'avant, y vers le haut, z en travers ; sol y 449,6 ; axe du bras x 164,25 z 512,72).
Usage : ~/venv_cao/bin/python chassis_kart.py <dossier>
"""
import json
import math
import os
import sys
import cadquery as cq
from chassis_differentiel import boite, cyl_y, cyl_z, cyl_x, B_MOT, A_MOT, TROUS_MOT, BOSSAGE, C_MOT, R_ROUE, L_ROUE, L_MOYEU

XA, ZM, YS = 164.25, 512.72, 449.6
YT = 529.69                                  # dessus de la tour = socle du bras (80,1 mm du sol, comme aujourd'hui)
XE = XA - 10.0                               # essieu
YE = YS + R_ROUE
W_ROUE_EXT = 75.0
W_ROUE = W_ROUE_EXT - L_ROUE / 2             # 68,25
W_FACE_MOT = W_ROUE_EXT - L_MOYEU - 1.0      # 61,2
W_MOT = W_FACE_MOT - C_MOT                   # 51,7
W_MOT_INT = W_MOT - C_MOT                    # 42,2
M3, M3_ECROU, H_ECROU = 3.4, 6.4, 2.6        # trou, logement d'ecrou (5,5 sur plats + jeu), profondeur
# ---------------------------------------------------------------- platine
EP = 4.0
YP0 = YE + B_MOT + 0.25                      # dessous (pose sur les moteurs) : 41,4 mm du sol
YP1 = YP0 + EP
W_PL = 59.0
# ---------------------------------------------------------------- tour du bras
T_MUR, T_EXT, T_HAUT, EP_HAUT = 3.0, 38.0, 48.0, 4.0
PATTES_TOUR = ((-43.0, -43.0), (-43.0, 43.0), (43.0, -22.0), (43.0, 22.0))
X_L298N, H_PLOT = XA - 12.0, 3.0             # L298N (43 x 43, trous a +-18,75) sur 4 plots de 3 mm ; abaisseur devant lui
X_ABAISSEUR = XA + 22.5
# ---------------------------------------------------------------- capteurs (5 en fleche)
L_CAPT = 110.0                               # essieu -> element du milieu
B1, R1, B2, R2 = 15.2, 4.0, 33.2, 8.0        # paire 1 (cartes le long du robot), paire 2 (cartes en travers)
H_ELEM = 8.0                                 # face des elements a 8 mm du sol
Y_PL = (YS + H_ELEM + 10.0 - 4.0, YS + H_ELEM + 10.0)        # plaque des cartes (la carte repose dessus)
CARTE_L, CARTE_AV, CARTE_AR, TROU_CARTE, PIQUES = 14.1, 3.8, 27.4, 20.3, (23.4, 26.2)
JEU = 1.0
X_C = XE + L_CAPT
ELEMENTS = [(X_C - R2, B2), (X_C - R1, B1), (X_C, 0.0), (X_C - R1, -B1), (X_C - R2, -B2)]     # (x, w)
# ---------------------------------------------------------------- roulette a bille (cotes a remplacer par la vraie)
R_BILLE, R_BOITIER, H_ROUL = 7.5, 11.0, 20.0                # bille 15 mm, boitier 22 mm, sol -> dessus de la platine de la roulette
R_PLAT, R_PLAT_EP, R_ENTRAXE = (20.0, 38.0), 2.5, 30.0      # platine de la roulette (x, z), epaisseur, entraxe des 2 trous (en z)
X_ROUL = X_C + CARTE_AV + JEU + R_BOITIER + 3.0
Y_ROUL = YS + H_ROUL                          # dessus de la platine de la roulette = dessous du bloc du nez
# ---------------------------------------------------------------- batteries
L_BAT, D_SIEGE, W_BAT = 113.0, 20.8, 11.0
X_AR_PLAQUE = X_C - R1 - CARTE_AR - JEU      # arriere de la plaque des capteurs
XC_BAT = X_AR_PLAQUE - 1.0 - L_BAT / 2 - 1.5
Y_FOND, EP_FOND = YS + 9.0, 2.5
Y_BAT = Y_FOND + EP_FOND + 9.0
COL_BERCEAU = ((XE - 22.0, 28.5), (XC_BAT + 50.0, 28.5))     # arriere : sous la tour, a cote des plots du L298N
XP0, XP1 = XE - 40.0, X_ROUL + 14.0
# ---------------------------------------------------------------- fixations du nez
NEZ_AV = (X_ROUL, 0.0)                       # 1 vis dans le bloc de la roulette
X_POTEAU = X_C - R2 - CARTE_L / 2 - JEU - 4.0                # 2 poteaux derriere les cartes de la paire 2
NEZ_AR = (X_POTEAU, 36.0)
Y_ECROU_NEZ = YP0 - 7.2                      # bas de l'ecrou des vis du nez (fentes laterales)
# ---------------------------------------------------------------- ESP32 DevKit V1 30 broches (sans PCB), RETOURNE : broches en haut
# carte 51,8 x 28,2 (mesures publiees, a verifier), 4 trous ~2,8 dans les coins : 43,9 x 23,4 (a verifier) -> vis M2 x 8
X_ESP = XE + 109.0                           # centre ; la longueur de la carte est en travers, port USB vers -z (cote droit)
ESP_TROUS = (11.7, 21.95)                    # demi-entraxes (x, z)
H_SUP, H_PLOT_ESP = 2.5, 5.0                 # semelle du support, plots (les composants de la carte pendent entre les plots)
W_VIS_ESP = 30.0                             # 2 vis M3 x 8 du support dans la platine (ecrous captifs sous la platine)


def wz(sg, a):
    return ZM + sg * a


def ecrou_y(x, z, y_face, sens):
    """Logement d'ecrou hexagonal d'axe y, de profondeur H_ECROU, depuis la face y_face vers le haut (sens +1) ou le bas (-1)."""
    e = cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(-H_ECROU)       # de 0 a +H_ECROU en y
    return e.translate((x, y_face if sens > 0 else y_face - H_ECROU, z))


def fente(x, z, y_bas, dx0, dx1, dz0, dz1):
    """Fente pour glisser un ecrou M3 (axe y) : y_bas..y_bas+2,6, ouverte du cote des grands debords."""
    return boite(x + dx0, x + dx1, y_bas - 0.1, y_bas + 2.5, z + dz0, z + dz1)


def trous_platine():
    t = []
    for dx in (-18.75, 18.75):
        for dw in (-18.75, 18.75):
            t.append((X_L298N + dx, dw, 'L298N'))
    for dw in (-15.0, 15.0):
        t.append((X_ABAISSEUR, dw, 'abaisseur'))
    for dx, dw in PATTES_TOUR:
        t.append((XA + dx, dw, 'tour'))
    for sg in (-1, 1):
        for x, w in COL_BERCEAU:
            t.append((x, sg * w, 'berceau'))
        t.append((NEZ_AR[0], sg * NEZ_AR[1], 'nez'))
    t.append((NEZ_AV[0], NEZ_AV[1], 'nez'))
    for sg in (-1, 1):
        t.append((X_ESP, sg * W_VIS_ESP, 'ESP32'))
    return t


def connecteurs():
    """(x, w, orientation) du connecteur de chaque carte : 'x' = carte le long du robot, 'z' = carte en travers."""
    c = []
    for x, w in ELEMENTS:
        if abs(w) > B1 + 1:
            c.append((x, math.copysign(abs(w) + (PIQUES[0] + PIQUES[1]) / 2, w), 'z'))
        else:
            c.append((x - (PIQUES[0] + PIQUES[1]) / 2, w, 'x'))
    return c


# ------------------------------------------------------------------------------------------------ platine (+ flasques moteur)
def platine():
    p = boite(XP0, XP1, YP0, YP1, ZM - W_PL, ZM + W_PL).edges('|Y').fillet(10.0)
    for sg in (-1, 1):                                                       # oreilles au-dessus des connecteurs de la paire 2
        x = X_C - R2
        p = p.union(boite(x - 9, x + 9, YP0, YP1, wz(sg, W_PL - 6), wz(sg, B2 + 25.0 + 7.0)).edges('|Y').fillet(4.0))
    # flasques moteur : 2 vis M3 x 30 a travers le moteur (tete cote roue, ecrou captif cote interieur)
    w_in1 = W_MOT_INT - BOSSAGE - 0.2
    w_in0 = w_in1 - 4.0
    xa, xb = XE - 10.0, XE + A_MOT[1] + 1.0
    for sg in (-1, 1):
        f = boite(xa, xb, YE - B_MOT - 2.0, YP0 + 0.1, wz(sg, w_in0), wz(sg, w_in1))
        for x0 in (xa, xb - 6.0):                                                                  # goussets aux 2 bouts
            f = f.union(boite(x0, x0 + 6.0, YP0 - 8.0, YP0 + 0.1, wz(sg, w_in0 - 8.0), wz(sg, w_in1)))
        f = f.cut(cyl_z(XE, YE, wz(sg, w_in0 - 9), wz(sg, w_in1 + 1), 4.0))                       # bout d'arbre interieur
        for a, b in TROUS_MOT:
            f = f.cut(cyl_z(XE + a, YE + b, wz(sg, w_in0 - 9), wz(sg, w_in1 + 1), M3 / 2))
            z_e = ZM + w_in0 if sg > 0 else ZM - w_in0 - H_ECROU
            f = f.cut(cq.Workplane('XY').polygon(6, M3_ECROU).extrude(H_ECROU).translate((XE + a, YE + b, z_e)))
        p = p.union(f)
    # plots du L298N (3 mm) sur le dessus ; bossages sous la platine pour les ecrous captifs (L298N, abaisseur, tour)
    for x, w, r in trous_platine():
        if r == 'L298N':
            p = p.union(cyl_y(x, ZM + w, YP1 - 0.1, YP1 + H_PLOT, 3.5))
        if r in ('L298N', 'abaisseur', 'tour'):
            p = p.union(cyl_y(x, ZM + w, YP0 - 3.0, YP0 + 0.1, 5.0))
    for x, w, r in trous_platine():
        p = p.cut(cyl_y(x, ZM + w, YP0 - 10, YP1 + H_PLOT + 1, M3 / 2))
        if r in ('L298N', 'abaisseur', 'tour'):
            p = p.cut(ecrou_y(x, ZM + w, YP0 - 3.0, +1))
    # passages de fils : 1 fente par capteur (connecteur Dupont 4 broches : 12 x 4), 2 trous moteurs, 1 trou batteries
    for x, w, o in connecteurs():
        dx, dw = (2.0, 6.0) if o == 'x' else (6.0, 2.0)
        p = p.cut(boite(x - dx, x + dx, YP0 - 6, YP1 + 1, ZM + w - dw, ZM + w + dw).edges('|Y').fillet(1.9))
    for sg in (-1, 1):
        p = p.cut(cyl_y(XE - 24.0, wz(sg, 50.0), YP0 - 6, YP1 + 1, 4.0))
    p = p.cut(cyl_y(XA, ZM - 28.0, YP0 - 6, YP1 + 1, 5.0))
    for x, w, r in trous_platine():
        if r == 'ESP32':
            p = p.cut(ecrou_y(x, ZM + w, YP0 - 0.01, +1))
    return p.val().clean()


# ------------------------------------------------------------------------------------------------------------ tour du bras
def tour():
    y0, y1 = YP1, YT
    t = boite(XA - T_EXT, XA + T_EXT, y0, y1 - EP_HAUT + 0.1, ZM - T_EXT, ZM + T_EXT)
    t = t.cut(boite(XA - T_EXT + T_MUR, XA + T_EXT - T_MUR, y0 - 1, y1, ZM - T_EXT + T_MUR, ZM + T_EXT - T_MUR))
    h0, h1 = y0 + 5.0, y1 - EP_HAUT - 4.0                                     # grandes fenetres : air du L298N, fils, montage
    t = t.cut(boite(XA - 26, XA + 26, h0, h1, ZM - T_EXT - 1, ZM + T_EXT + 1))
    t = t.cut(boite(XA - T_EXT - 1, XA + T_EXT + 1, h0, h1, ZM - 26, ZM + 26))
    t = t.union(boite(XA - T_HAUT, XA + T_HAUT, y1 - EP_HAUT, y1, ZM - T_HAUT, ZM + T_HAUT).edges('|Y').fillet(8.0))
    for dx, dw in PATTES_TOUR:
        t = t.union(boite(XA + dx - 6, XA + dx + 6, y0, y0 + 4.0, ZM + dw - 6, ZM + dw + 6).edges('|Y').fillet(3.0))
        t = t.cut(cyl_y(XA + dx, ZM + dw, y0 - 1, y0 + 5, M3 / 2))
    for dx in (-33.0, 33.0):
        for dw in (-33.0, 33.0):
            t = t.cut(cyl_y(XA + dx, ZM + dw, y1 - EP_HAUT - 1, y1 + 1, M3 / 2))
            t = t.cut(ecrou_y(XA + dx, ZM + dw, y1 - EP_HAUT, +1))
    t = t.cut(cyl_y(XA + 10.2, ZM, y1 - EP_HAUT - 1, y1 + 1, 17.0))              # passage des cables du bras
    return t.val().clean()


# ------------------------------------------------------------------------------- nez : 5 cartes capteurs + bloc de la roulette
def cartes_volumes():
    y1 = Y_PL[1]
    c = []
    for x, w in ELEMENTS:
        if abs(w) > B1 + 1:
            sg = 1 if w > 0 else -1
            c.append(boite(x - CARTE_L / 2, x + CARTE_L / 2, y1, y1 + 1.6, ZM + w - sg * CARTE_AV, ZM + w + sg * CARTE_AR))
        else:
            c.append(boite(x - CARTE_AR, x + CARTE_AV, y1, y1 + 1.6, ZM + w - CARTE_L / 2, ZM + w + CARTE_L / 2))
    return c


def nez():
    y0, y1 = Y_PL
    zi = B1 + CARTE_L / 2 + JEU
    s = boite(X_AR_PLAQUE, X_C + CARTE_AV + JEU, y0, y1, ZM - zi, ZM + zi)               # 3 cartes le long du robot
    xa1 = X_C - R2 + CARTE_L / 2 + JEU
    ze = B2 + CARTE_AR + JEU
    s = s.union(boite(X_POTEAU - 4.0, xa1, y0, y1, ZM - ze, ZM + ze))                       # ailes (paire 2) + pieds des poteaux
    for sg in (-1, 1):
        for xx in (xa1 - 2.0, X_POTEAU - 4.0):                                                 # nervures sous les ailes
            s = s.union(boite(xx, xx + 2.0, y0 - 3.0, y0 + 0.1, wz(sg, zi - 2), wz(sg, ze)))
        xp, wp = NEZ_AR                                                                        # poteaux vers la platine
        s = s.union(boite(xp - 4.0, xp + 4.0, y1 - 0.1, YP0, wz(sg, wp - 5.0), wz(sg, wp + 5.0)))
    # bloc de la roulette, relie a la pointe par une ame verticale
    s = s.union(boite(X_ROUL - R_PLAT[0] / 2 - 1, X_ROUL + R_PLAT[0] / 2 + 1, Y_ROUL, YP0, ZM - R_PLAT[1] / 2, ZM + R_PLAT[1] / 2))
    s = s.union(boite(X_C + CARTE_AV + JEU - 0.1, X_ROUL - R_PLAT[0] / 2 + 0.1, y0, YP0, ZM - 8.0, ZM + 8.0))
    s = s.cut(cyl_y(X_ROUL, ZM, y0 - 4, Y_ROUL, R_BOITIER + 0.6))                            # place du boitier de la roulette
    # cartes : fenetre de l'element, trou de la vis M3 (ecrou dessous), encoche des picots
    for x, w in ELEMENTS:
        z = ZM + w
        if abs(w) > B1 + 1:
            sg = 1 if w > 0 else -1
            s = s.cut(boite(x - 5.5, x + 5.5, y0 - 4, y1 + 1, z - 3.9, z + 3.9))
            s = s.cut(cyl_y(x, z + sg * TROU_CARTE, y0 - 4, y1 + 1, M3 / 2))
            s = s.cut(boite(x - 6.5, x + 6.5, y1 - 1.3, y1 + 1, z + sg * PIQUES[0], z + sg * PIQUES[1]))
        else:
            s = s.cut(boite(x - 3.9, x + 3.9, y0 - 4, y1 + 1, z - 5.5, z + 5.5))
            s = s.cut(cyl_y(x - TROU_CARTE, z, y0 - 4, y1 + 1, M3 / 2))
            s = s.cut(boite(x - PIQUES[1], x - PIQUES[0], y1 - 1.3, y1 + 1, z - 6.5, z + 6.5))
    # vis du nez depuis le dessus de la platine ; ecrous glisses par le cote
    for sg in (-1, 1):
        x, z = NEZ_AR[0], wz(sg, NEZ_AR[1])
        s = s.cut(cyl_y(x, z, YP0 - 12, YP0 + 1, M3 / 2))
        s = s.cut(fente(x, z, Y_ECROU_NEZ, -3.2, 3.2, *((-3.2, 9.0) if sg > 0 else (-9.0, 3.2))))
    x, z = NEZ_AV[0], ZM + NEZ_AV[1]
    s = s.cut(cyl_y(x, z, YP0 - 12, YP0 + 1, M3 / 2))
    s = s.cut(fente(x, z, Y_ECROU_NEZ, -3.2, 15.0, -3.2, 3.2))
    # roulette : 2 vis M3 par dessous, ecrous glisses par l'arriere du bloc
    for sg in (-1, 1):
        z = wz(sg, R_ENTRAXE / 2)
        s = s.cut(cyl_y(X_ROUL, z, Y_ROUL - 1, Y_ROUL + 12, M3 / 2))
        s = s.cut(fente(X_ROUL, z, Y_ROUL + 5.7, -20.0, 3.2, -3.2, 3.2))
    return s.val().clean()


def support_esp32():
    """Semelle + 4 plots (trous de 1,8 pour vis M2 x 8 autotaraudeuses) ; l'ESP32 est pose retourne sur les plots."""
    y0 = YP1
    s = boite(X_ESP - 16.0, X_ESP + 16.0, y0, y0 + H_SUP, ZM - 33.5, ZM + 33.5).edges('|Y').fillet(4.0)
    s = s.cut(boite(X_ESP - 8.0, X_ESP + 8.0, y0 - 1, y0 + H_SUP + 1, ZM - 15.0, ZM + 15.0).edges('|Y').fillet(3.0))   # allegement
    for sx in (-1, 1):
        for sz in (-1, 1):
            x, z = X_ESP + sx * ESP_TROUS[0], ZM + sz * ESP_TROUS[1]
            s = s.union(cyl_y(x, z, y0 + H_SUP - 0.1, y0 + H_SUP + H_PLOT_ESP, 2.5))
            s = s.cut(cyl_y(x, z, y0 + 1.0, y0 + H_SUP + H_PLOT_ESP + 1, 0.9))
    for sg in (-1, 1):
        s = s.cut(cyl_y(X_ESP, wz(sg, W_VIS_ESP), y0 - 1, y0 + H_SUP + 1, M3 / 2))
    return s.val().clean()


def roulette():
    """Roulette a bille standard (encombrement, cotes a remplacer par celles de la vraie)."""
    yb = YS + R_BILLE
    r = boite(X_ROUL - R_PLAT[0] / 2, X_ROUL + R_PLAT[0] / 2, Y_ROUL - R_PLAT_EP, Y_ROUL, ZM - R_PLAT[1] / 2, ZM + R_PLAT[1] / 2).edges('|Y').fillet(4.0)
    r = r.union(cyl_y(X_ROUL, ZM, yb, Y_ROUL - R_PLAT_EP + 0.1, R_BOITIER))
    r = r.cut(cq.Workplane('XY').sphere(R_BILLE + 0.3).translate((X_ROUL, yb, ZM)))
    for sg in (-1, 1):
        r = r.cut(cyl_y(X_ROUL, wz(sg, R_ENTRAXE / 2), Y_ROUL - 5, Y_ROUL + 1, 1.6))
    r = r.union(cq.Workplane('XY').sphere(R_BILLE).translate((X_ROUL, yb, ZM)))
    return r.val().clean()


# --------------------------------------------------------------------------------------------- berceau (batteries)
def berceau():
    xb0, xb1 = XC_BAT - L_BAT / 2 - 1.5, XC_BAT + L_BAT / 2 + 1.5
    yf0, yf1 = Y_FOND, Y_FOND + EP_FOND
    ys = yf1 + D_SIEGE / 2
    b = boite(xb0, xb1, yf0, ys + 1.0, ZM - 23.5, ZM + 23.5)
    b = b.cut(boite(xb0 + 1.5, xb1 - 1.5, ys, ys + 2, ZM - 22, ZM + 22))
    for x in (xb0, xb1 - 1.5):
        b = b.union(boite(x, x + 1.5, yf0, Y_BAT + 6.0, ZM - 23.5, ZM + 23.5))
    b = b.cut(boite(xb0 - 1, xb1 + 1, Y_BAT - 4, Y_BAT + 4, ZM - 4, ZM + 4))
    for xs in (XC_BAT - 24.0, XC_BAT + 32.0):                         # fentes des 2 sangles velcro de 20 mm
        for sg in (-1, 1):
            b = b.union(boite(xs - 13, xs + 13, yf0, yf1 + 2.0, wz(sg, 23.0), wz(sg, 31.0)))
            b = b.cut(boite(xs - 10.5, xs + 10.5, yf0 - 1, yf1 + 3, wz(sg, 25.5), wz(sg, 28.5)))
    for xc, wc in COL_BERCEAU:
        b = b.union(boite(xc - 4, xc + 4, yf0, yf1 + 2.0, ZM - wc - 4, ZM + wc + 4))
        for sg in (-1, 1):
            w = wz(sg, wc)
            b = b.union(boite(xc - 4, xc + 4, yf0, YP0, w - 4, w + 4))
            b = b.cut(cyl_y(xc, w, YP0 - 14, YP0 + 1, M3 / 2))
            vers = -1 if xc < XE else 1                                  # fente d'ecrou ouverte vers l'arriere / l'avant (les flasques
            b = b.cut(boite(xc - 3.2 if vers > 0 else xc - 6, xc + 6 if vers > 0 else xc + 3.2, YP0 - 11, YP0 - 8.4, w - 3.2, w + 3.2))   # bouchent les cotes)
    for sg in (-1, 1):
        b = b.cut(cyl_x(ys, wz(sg, W_BAT), xb0 + 1.5, xb1 - 1.5, D_SIEGE / 2))
    return b.val().clean()


# ------------------------------------------------------------------------------------------------------------ visserie
def vis_modele(L):
    """Vis CHC M3 x L : dessous de la tete en y = 0, tete vers +y, tige vers -y."""
    v = cq.Workplane('XZ').circle(2.75).extrude(-3.0)
    v = v.cut(cq.Workplane('XZ').polygon(6, 2.5 * 1.1547).extrude(-2.0).translate((0, 1.0, 0)))
    v = v.union(cq.Workplane('XZ').circle(1.5).extrude(L))
    return v.val().clean()


def vis_m2():
    """Vis M2 x 8 (tete 3,8 x 2)."""
    v = cq.Workplane('XZ').circle(1.9).extrude(-2.0)
    v = v.union(cq.Workplane('XZ').circle(1.0).extrude(8.0))
    return v.val().clean()


def ecrou_modele():
    """Ecrou M3 (5,5 sur plats, 2,4 d'epaisseur) : y 0..2,4."""
    e = cq.Workplane('XZ').polygon(6, 5.5 * 1.1547).extrude(-2.4)
    return e.cut(cq.Workplane('XZ').circle(1.55).extrude(-2.4)).val().clean()     # 1,55 : pas d'interference affichee avec la tige


# rotations (lignes ; assemblage = R . local) : la tige (local -y) part dans la direction voulue
R_BAS = (1, 0, 0, 0, 1, 0, 0, 0, 1)
R_HAUT = (1, 0, 0, 0, -1, 0, 0, 0, -1)
R_VERS_MZ = (1, 0, 0, 0, 0, -1, 0, 1, 0)
R_VERS_PZ = (1, 0, 0, 0, 0, 1, 0, -1, 0)


def visserie():
    """(fichier, R, point, role) : point = dessous de la tete (vis) ou face de l'ecrou (local y = 0)."""
    v = []
    def vis(L, x, y, z, role, R=R_BAS):
        v.append(('kart vis M3x%d' % L, R, (x, y, z), role))
    def ecr(x, y, z, role, R=R_BAS):
        v.append(('kart ecrou M3', R, (x, y, z), role))
    w_in0 = W_MOT_INT - BOSSAGE - 0.2 - 4.0
    for sg in (-1, 1):
        R = R_VERS_MZ if sg > 0 else R_VERS_PZ
        for a, b in TROUS_MOT:
            vis(30, XE + a, YE + b, wz(sg, W_FACE_MOT), 'moteur', R)
            ecr(XE + a, YE + b, wz(sg, w_in0), 'moteur', R)
    for x, w, r in trous_platine():
        z = ZM + w
        if r == 'tour':
            vis(12, x, YP1 + 4.0, z, r); ecr(x, YP0 - 2.9, z, r)
        elif r == 'L298N':
            vis(12, x, YP1 + H_PLOT + 1.6, z, r); ecr(x, YP0 - 2.9, z, r)
        elif r == 'berceau':
            vis(16, x, YP1, z, r); ecr(x, YP0 - 10.9, z, r)
        elif r == 'nez':
            vis(12, x, YP1, z, r); ecr(x, Y_ECROU_NEZ, z, r)
    for dx in (-33.0, 33.0):                     # bras : ecrous captifs sous le plateau de la tour (vis du socle : reprises du robot)
        for dw in (-33.0, 33.0):
            ecr(XA + dx, YT - EP_HAUT + 0.1, ZM + dw, 'bras')
    for x, w in ELEMENTS:
        p = (x, ZM + math.copysign(abs(w) + TROU_CARTE, w)) if abs(w) > B1 + 1 else (x - TROU_CARTE, ZM + w)
        vis(8, p[0], Y_PL[1] + 1.6, p[1], 'capteur'); ecr(p[0], Y_PL[0] - 2.4, p[1], 'capteur')
    for sg in (-1, 1):                           # support de l'ESP32 : M3 x 8, ecrou captif sous la platine
        vis(8, X_ESP, YP1 + H_SUP, wz(sg, W_VIS_ESP), 'ESP32'); ecr(X_ESP, YP0 + 0.05, wz(sg, W_VIS_ESP), 'ESP32')
    for sx in (-1, 1):                           # ESP32 : 4 vis M2 x 8 dans les plots (carte 1,6 mm)
        for sz in (-1, 1):
            v.append(('kart vis M2x8', R_BAS, (X_ESP + sx * ESP_TROUS[0], YP1 + H_SUP + H_PLOT_ESP + 1.6, ZM + sz * ESP_TROUS[1]), 'ESP32'))
    for sg in (-1, 1):
        vis(10, X_ROUL, Y_ROUL - R_PLAT_EP, wz(sg, R_ENTRAXE / 2), 'roulette', R_HAUT)
        ecr(X_ROUL, Y_ROUL + 5.8, wz(sg, R_ENTRAXE / 2), 'roulette')
    return v


def pieces():
    return [('platine kart', platine()), ('tour du bras', tour()), ('nez 5 capteurs et roulette', nez()),
            ('berceau batteries kart', berceau()), ('roulette a bille (encombrement)', roulette()), ('support ESP32', support_esp32())]


def positions():
    return dict(XA=XA, ZM=ZM, YS=YS, YT=YT, XE=XE, YE=YE, W_ROUE=W_ROUE, W_MOT=W_MOT, A_MOT=A_MOT, Y_BAT=Y_BAT, W_BAT=W_BAT,
                XC_BAT=XC_BAT, YP0=YP0, YP1=YP1, L_CAPT=L_CAPT, X_ROUL=X_ROUL, Y_PL=list(Y_PL),
                cartes=[[x, w, 'travers' if abs(w) > B1 + 1 else 'long'] for x, w in ELEMENTS],
                appui=[[XE, ZM + W_ROUE], [X_ROUL, ZM], [XE, ZM - W_ROUE]],
                L298N=[X_L298N, YP1 + H_PLOT, ZM], esp32=[X_ESP, YP1 + H_SUP + H_PLOT_ESP, ZM], abaisseur=[X_ABAISSEUR, YP1, ZM], trous_platine=trous_platine(),
                visserie=[[f, list(R), list(p), r] for f, R, p, r in visserie()], longueur=[XP0, XP1])


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for nom, s in pieces() + [('kart vis M3x%d' % L, vis_modele(L)) for L in (8, 10, 12, 16, 30)] + [('kart vis M2x8', vis_m2())] + [('kart ecrou M3', ecrou_modele())]:
        b = s.BoundingBox()
        print('%-34s valide %s solides %d  volume %6.1f cm3  x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (
            nom, s.isValid(), len(s.Solids()), s.Volume() / 1000, b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.step'))
        if not nom.startswith('kart '):
            cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
    cs = cartes_volumes()
    for i in range(len(cs)):
        for j in range(i + 1, len(cs)):
            k = cs[i].intersect(cs[j])
            if k.vals() and k.val().Volume() > 1e-6:
                print('ATTENTION cartes %d et %d se chevauchent' % (i, j))
    P = positions()
    json.dump(P, open(os.path.join(out, 'chassis_kart.json'), 'w'), indent=1)
    print('longueur %.1f mm (essieu %+.1f..%+.1f), roulette essieu %+.1f, batteries %+.1f..%+.1f, plaque des capteurs des %+.1f' % (
        XP1 - XP0, XP0 - XE, XP1 - XE, X_ROUL - XE, XC_BAT - L_BAT / 2 - XE, XC_BAT + L_BAT / 2 - XE, X_AR_PLAQUE - XE))
    from collections import Counter
    print('visserie :', dict(Counter(f for f, _, _, _ in P['visserie'])), dict(Counter(r for _, _, _, r in P['visserie'])))
