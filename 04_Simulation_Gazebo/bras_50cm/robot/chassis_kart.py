"""PROPOSITION 2 (28/11/2026, a part) : ROBOT "KART" COMPACT. Remplace la proposition differentielle du 27/11 (jugee trop grande
et trop haute par l'utilisateur). Modele demande : robot suiveur de ligne de competition (photo) : 2 moteurs a l'ARRIERE avec
les roues juste a cote, une platine minimale, la barre de capteurs loin devant ; plus une place pour le bras.

Choix :
- 2 moteurs jaunes (TT) SOUS une platine basse (dessous a 41,4 mm du sol, juste sur les moteurs), roues collees aux moteurs,
  voie 136,5 mm (largeur hors tout 150 mm) ;
- le bras garde son socle a 80,1 mm du sol (le reseau a ete entraine avec cette hauteur) : il est pose sur une TOUR ajouree de
  35 mm, 10 mm devant l'essieu ; le L298N est DANS la tour (la place sous le bras ne sert a rien d'autre) ;
- 2 batteries couchees le long du robot, ENTRE les 2 moteurs, a 9 mm du sol (berceau visse sous la platine) ;
- PCB + ESP32 sur la platine, devant la tour ;
- nez visse sous la platine : les 4 capteurs EN FLECHE du robot actuel (8 mm du sol), elements interieurs L_CAPT devant l'essieu,
  et 2 billes d'acier de 16 mm ; 2 patins arriere a 1 mm du sol (le robot ne s'y appuie que si le bras part en arriere).
Repere : celui d'Assemblage1v2 (mm ; x vers l'avant, y vers le haut, z en travers ; sol y 449,6 ; axe du bras x 164,25 z 512,72).
Usage : ~/venv_cao/bin/python chassis_kart.py <dossier>
"""
import json
import math
import os
import sys
import cadquery as cq
import support_capteurs_fleche as SC
from chassis_differentiel import boite, cyl_y, cyl_z, cyl_x, B_MOT, A_MOT, TROUS_MOT, BOSSAGE, C_MOT, R_ROUE, L_ROUE, L_MOYEU, M3, M3_ECROU

XA, ZM, YS = 164.25, 512.72, 449.6
YT = 529.69                                  # dessus de la tour = socle du bras (80,1 mm du sol, comme aujourd'hui)
XE = XA - 10.0                               # essieu
YE = YS + R_ROUE
W_ROUE_EXT = 75.0
W_ROUE = W_ROUE_EXT - L_ROUE / 2             # 68,25
W_FACE_MOT = W_ROUE_EXT - L_MOYEU - 1.0      # 61,2
W_MOT = W_FACE_MOT - C_MOT                   # 51,7
W_MOT_INT = W_MOT - C_MOT                    # 42,2
# platine
EP = 4.0
YP0 = YE + B_MOT + 0.25                      # dessous (pose sur les moteurs) : 41,4 mm du sol
YP1 = YP0 + EP
XP0, XP1, W_PL = XE - 40.0, XE + 130.0, 59.0
# tour du bras
T_MUR, T_EXT, T_HAUT = 3.0, 38.0, 48.0       # murs a +-38, plateau du haut 96 x 96 (socle du bras +-47)
EP_HAUT = 4.0
PATTES_TOUR = ((-43.0, -43.0), (-43.0, 43.0), (43.0, -30.0), (43.0, 30.0))   # avant : a +-30 (entretoises du PCB a +-46)
# capteurs (fleche de support_capteurs_fleche.py tournee de 180 deg) et billes
L_CAPT = 110.0
X_RETOURNE = XE + L_CAPT + SC.X_IN
DY_SC = YS - SC.Y_SOL
X_BILLE, W_BILLE, R_BILLE = X_RETOURNE - SC.X_OUT, 60.0, 8.0
Y_BILLE = YS + R_BILLE
# batteries
L_BAT, D_SIEGE, W_BAT = 113.0, 20.8, 11.0
XC_BAT = XE + 20.5
Y_FOND, EP_FOND = YS + 9.0, 2.5
Y_BAT = Y_FOND + EP_FOND + 9.0
X_L298N = XA - 12.0                          # L298N (43 x 43) a l'arriere de la tour, abaisseur (43 x 21) devant lui
X_ABAISSEUR = XA + 22.5
COL_BERCEAU = ((XE - 22.0, 27.0), (XE + 72.0, 27.0))
X_PATIN, W_PATIN, JEU_PATIN = XE - 36.0, 25.0, 1.0      # 2 patins (1 seul au milieu : marge 15,7 mm au pire)
# PCB (100 x 72), en travers, sur 4 entretoises de 8 mm
PCB_X0, PCB_H = XE + 56.0, 8.0
PCB_Y = YP1 + PCB_H
PCB_W, PCB_L, PCB_EP = 100.0, 72.0, 1.6
TROUS_PCB = [(PCB_X0 + a, w) for a in (4.0, 68.0) for w in (-46.0, 46.0)]


def wz(sg, a):
    return ZM + sg * a


def trous_platine():
    t = []
    for dx in (-18.75, 18.75):
        for dw in (-18.75, 18.75):
            t.append((X_L298N + dx, dw, M3, 'L298N (dans la tour, a l arriere)'))
    for dw in (-15.0, 15.0):
        t.append((X_ABAISSEUR, dw, M3, 'abaisseur (dans la tour, a l avant)'))
    for dx, dw in PATTES_TOUR:
        t.append((XA + dx, dw, M3, 'tour du bras'))
    for sg in (-1, 1):
        for x, w in COL_BERCEAU:
            t.append((x, sg * w, M3, 'berceau'))
        for x in (X_BILLE - 8.0, X_BILLE + 8.0):
            t.append((x, sg * W_BILLE, M3, 'nez'))
    for x, w in TROUS_PCB:
        t.append((x, w, M3, 'PCB'))
    return t


# --------------------------------------------------------------------------------------------- platine (+ flasques moteur)
def platine():
    p = boite(XP0, XP1, YP0, YP1, ZM - W_PL, ZM + W_PL).edges('|Y').fillet(10.0)
    for sg in (-1, 1):                                                       # oreilles des vis du nez
        p = p.union(boite(X_BILLE - 13, X_BILLE + 13, YP0, YP1, wz(sg, W_PL - 5), wz(sg, W_BILLE + 8)).edges('|Y').fillet(4.0))
    # flasques moteur : 2 vis M3 x 30 a travers le moteur (tete cote roue, ecrou cote interieur), bout d'arbre interieur libre
    w_in1 = W_MOT_INT - BOSSAGE - 0.2
    w_in0 = w_in1 - 4.0
    xa, xb = XE - 10.0, XE + A_MOT[1] + 1.0
    for sg in (-1, 1):
        f = boite(xa, xb, YE - B_MOT - 2.0, YP0 + 0.1, wz(sg, w_in0), wz(sg, w_in1))
        for x0 in (xa, xb - 6.0):                                                                  # goussets aux 2 bouts
            f = f.union(boite(x0, x0 + 6.0, YP0 - 8.0, YP0 + 0.1, wz(sg, w_in0 - 8.0), wz(sg, w_in1)))
        f = f.cut(cyl_z(XE, YE, wz(sg, w_in0 - 9), wz(sg, w_in1 + 1), 4.0))
        for a, b in TROUS_MOT:
            f = f.cut(cyl_z(XE + a, YE + b, wz(sg, w_in0 - 9), wz(sg, w_in1 + 1), M3 / 2))
            z_ecrou = ZM + w_in0 if sg > 0 else ZM - w_in0 - 2.6                                     # ecrou cote interieur
            f = f.cut(cq.Workplane('XY').polygon(6, M3_ECROU).extrude(2.6).translate((XE + a, YE + b, z_ecrou)))
        p = p.union(f)
    # entretoises du PCB (ecrou par dessous)
    for x, w in TROUS_PCB:
        p = p.union(cyl_y(x, ZM + w, YP1 - 0.1, PCB_Y, 3.8))
    # trous
    for x, w, d, _ in trous_platine():
        p = p.cut(cyl_y(x, ZM + w, YP0 - 6, PCB_Y + 1, d / 2))
        if _ == 'PCB':
            p = p.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(-2.6).translate((x, YP0 - 0.01, ZM + w)))
    # allegement et passages de fils
    def fen(x0, x1, w0, w1, r=4.0):
        return boite(x0, x1, YP0 - 6, YP1 + 1, ZM + w0, ZM + w1).edges('|Y').fillet(r)
    p = p.cut(fen(X_L298N - 12, X_L298N + 12, -12, 12))                                  # sous le L298N : fils des batteries
    for sg in (-1, 1):
        a, b = sorted((sg * 45.0, sg * 54.0))
        p = p.cut(fen(XE - 14, XE + 38, a, b, 3.0))                            # a cote de la tour (au-dessus du moteur)
        a, b = sorted((sg * 8.0, sg * 21.0))
        p = p.cut(fen(XE + 60, XE + 86, a, b))
        a, b = sorted((sg * 33.0, sg * 52.0))
        p = p.cut(fen(XE + 60, XE + 86, a, b))
        p = p.cut(cyl_y(XE - 24.0, wz(sg, 50.0), YP0 - 6, YP1 + 1, 4.0))       # fils des moteurs
    p = p.cut(fen(XE + 94, XE + 113, -44, 44))                                 # cables des capteurs + allegement
    return p.val().clean()


# ------------------------------------------------------------------------------------------------------------ tour du bras
def tour():
    y0, y1 = YP1, YT
    t = boite(XA - T_EXT, XA + T_EXT, y0, y1 - EP_HAUT + 0.1, ZM - T_EXT, ZM + T_EXT)
    t = t.cut(boite(XA - T_EXT + T_MUR, XA + T_EXT - T_MUR, y0 - 1, y1, ZM - T_EXT + T_MUR, ZM + T_EXT - T_MUR))
    # grandes fenetres sur les 4 faces (air du L298N, fils) : il reste 4 piliers et 2 cadres
    h0, h1 = y0 + 5.0, y1 - EP_HAUT - 4.0
    t = t.cut(boite(XA - 26, XA + 26, h0, h1, ZM - T_EXT - 1, ZM + T_EXT + 1))
    t = t.cut(boite(XA - T_EXT - 1, XA + T_EXT + 1, h0, h1, ZM - 26, ZM + 26))
    # plateau du haut (socle du bras) et pattes du bas
    t = t.union(boite(XA - T_HAUT, XA + T_HAUT, y1 - EP_HAUT, y1, ZM - T_HAUT, ZM + T_HAUT).edges('|Y').fillet(8.0))
    for dx, dw in PATTES_TOUR:
        t = t.union(boite(XA + dx - 6, XA + dx + 6, y0, y0 + 4.0, ZM + dw - 6, ZM + dw + 6).edges('|Y').fillet(3.0))
        t = t.cut(cyl_y(XA + dx, ZM + dw, y0 - 1, y0 + 5, M3 / 2))
    for dx in (-33.0, 33.0):
        for dw in (-33.0, 33.0):
            t = t.cut(cyl_y(XA + dx, ZM + dw, y1 - EP_HAUT - 1, y1 + 1, M3 / 2))
            t = t.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(-2.6).translate((XA + dx, y1 - EP_HAUT - 0.01, ZM + dw)))
    t = t.cut(cyl_y(XA + 10.2, ZM, y1 - EP_HAUT - 1, y1 + 1, 17.0))              # passage des cables du bras
    return t.val().clean()


# ------------------------------------------------------------------------------------------------- nez (capteurs + billes)
def nez():
    s = cq.Workplane().add(SC.support()).translate((0, DY_SC, 0))
    s = s.rotate((X_RETOURNE / 2, 0, ZM), (X_RETOURNE / 2, 1, ZM), 180)
    y_pl0, y_pl1 = SC.Y_PL[0] + DY_SC, SC.Y_PL[1] + DY_SC
    s = s.cut(boite(150, 320, y_pl1 + 0.01, 560, 400, 620))
    xo0, xo1 = X_BILLE - SC.LARG / 2 - SC.JEU, X_BILLE + SC.LARG / 2 + SC.JEU
    for sg in (-1, 1):
        wc = wz(sg, W_BILLE)
        s = s.union(boite(xo0, xo1, y_pl0, y_pl1, wz(sg, 45.0), wc))
        s = s.union(boite(xo0, xo1, y_pl0 - 3.0, y_pl0 + 0.1, wz(sg, 45.0), wc))
        s = s.union(boite(X_BILLE - 7, X_BILLE + 7, Y_BILLE + 4.0, YP0, wc - 7, wc + 7))
        s = s.union(boite(X_BILLE - 12, X_BILLE + 12, YP0 - 5.0, YP0, wc - 7, wc + 7))
        s = s.union(cyl_y(X_BILLE, wc, YS + 5.4, Y_BILLE + 6.0, R_BILLE + 2.6))
        s = s.cut(cq.Workplane('XY').sphere(R_BILLE + 0.15).translate((X_BILLE, Y_BILLE, wc)))
        for k in range(3):
            t = math.radians(90 + 120 * k)
            s = s.cut(boite(-0.6, 0.6, YS + 5.0, YS + 10.0, 0, R_BILLE + 4).rotate((0, 0, 0), (0, 1, 0), math.degrees(t))
                      .translate((X_BILLE, 0, wc)))
        for x in (X_BILLE - 8.0, X_BILLE + 8.0):
            s = s.cut(cyl_y(x, wc, YP0 - 6, YP0 + 1, M3 / 2))
            s = s.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(2.6).translate((x, YP0 - 5.0 + 2.6, wc)))
    return s.val().clean()


def bille(sg):
    return cq.Workplane('XY').sphere(R_BILLE).translate((X_BILLE, Y_BILLE, wz(sg, W_BILLE))).val()


# --------------------------------------------------------------------------------------- berceau (batteries + patin arriere)
def berceau():
    xb0, xb1 = XC_BAT - L_BAT / 2 - 1.5, XC_BAT + L_BAT / 2 + 1.5
    yf0, yf1 = Y_FOND, Y_FOND + EP_FOND
    ys = yf1 + D_SIEGE / 2
    b = boite(xb0, xb1, yf0, ys + 1.0, ZM - 23.5, ZM + 23.5)
    b = b.cut(boite(xb0 + 1.5, xb1 - 1.5, ys, ys + 2, ZM - 22, ZM + 22))
    for x in (xb0, xb1 - 1.5):
        b = b.union(boite(x, x + 1.5, yf0, Y_BAT + 6.0, ZM - 23.5, ZM + 23.5))
    b = b.cut(boite(xb0 - 1, xb1 + 1, Y_BAT - 4, Y_BAT + 4, ZM - 4, ZM + 4))
    for xs in (XC_BAT - 32.0, XC_BAT + 32.0):                         # fentes des 2 sangles velcro de 20 mm
        for sg in (-1, 1):
            b = b.union(boite(xs - 13, xs + 13, yf0, yf1 + 2.0, wz(sg, 23.0), wz(sg, 31.0)))
            b = b.cut(boite(xs - 10.5, xs + 10.5, yf0 - 1, yf1 + 3, wz(sg, 25.5), wz(sg, 28.5)))
    for xc, wc in COL_BERCEAU:
        b = b.union(boite(xc - 4, xc + 4, yf0, yf1 + 2.0, ZM - wc - 4, ZM + wc + 4))
        for sg in (-1, 1):
            w = wz(sg, wc)
            b = b.union(boite(xc - 4, xc + 4, yf0, YP0, w - 4, w + 4))
            b = b.cut(cyl_y(xc, w, YP0 - 14, YP0 + 1, M3 / 2))
            b = b.cut(boite(xc - 3.2, xc + 3.2, YP0 - 11, YP0 - 8.4, w - 3.2 if sg > 0 else w - 5, w + 5 if sg > 0 else w + 3.2))
    # patin arriere (1 mm du sol)
    b = b.union(boite(X_PATIN - 6, xb0 + 1, yf0, yf1, ZM - W_PATIN - 6, ZM + W_PATIN + 6))
    for sg in (-1, 1):
        b = b.union(cq.Workplane('XY').sphere(6.0).translate((X_PATIN, YS + JEU_PATIN + 6.0, wz(sg, W_PATIN))))
        b = b.union(cyl_y(X_PATIN, wz(sg, W_PATIN), YS + JEU_PATIN + 6.0, yf1, 6.0))
    for sg in (-1, 1):
        b = b.cut(cyl_x(ys, wz(sg, W_BAT), xb0 + 1.5, xb1 - 1.5, D_SIEGE / 2))
    return b.val().clean()


# ------------------------------------------------------------------------------------------------------------------ PCB
def pcb():
    def P(X, Y):                       # Y du PCB vers l'avant, X du PCB en travers
        return PCB_X0 + Y, ZM + (X - 50.0)
    y0, y1 = PCB_Y, PCB_Y + PCB_EP
    c = boite(PCB_X0, PCB_X0 + PCB_L, y0, y1, ZM - PCB_W / 2, ZM + PCB_W / 2).edges('|Y').fillet(3.0)
    for X, Y in ((4, 4), (96, 4), (4, 68), (96, 68)):
        x, z = P(X, Y); c = c.cut(cyl_y(x, z, y0 - 1, y1 + 1, 1.6))
    for X in (34.0, 59.4):
        x, z = P(X, 46.22); c = c.union(boite(x - 19.05, x + 19.05, y1, y1 + 8.5, z - 1.27, z + 1.27))
    x, z = P(85.0, 23.46); c = c.union(cyl_y(x, z, y1, y1 + 20.0, 5.0))
    x, z = P(93.0, 23.46); c = c.union(boite(x - 5, x + 5, y1, y1 + 10, z - 4, z + 4))
    return c.val().clean()


def pieces():
    return [('platine kart', platine()), ('tour du bras', tour()), ('nez capteurs et billes kart', nez()),
            ('berceau batteries kart', berceau()), ('bille 16 gauche', bille(1)), ('bille 16 droite', bille(-1)),
            ('PCB MAVE (encombrement)', pcb())]


def positions():
    return dict(XA=XA, ZM=ZM, YS=YS, YT=YT, XE=XE, YE=YE, W_ROUE=W_ROUE, W_MOT=W_MOT, A_MOT=A_MOT, Y_BAT=Y_BAT, W_BAT=W_BAT,
                XC_BAT=XC_BAT, X_RETOURNE=X_RETOURNE, DY_SC=DY_SC, X_BILLE=X_BILLE, W_BILLE=W_BILLE, Y_BILLE=Y_BILLE,
                PCB_X0=PCB_X0, PCB_Y=PCB_Y, YP0=YP0, YP1=YP1, patins=[[X_PATIN, W_PATIN]], jeu_patin=JEU_PATIN, L_CAPT=L_CAPT,
                L298N=[X_L298N, YP1, ZM], abaisseur=[X_ABAISSEUR, YP1, ZM], elements=[[X_RETOURNE - x, 2 * ZM - z] for x, z in SC.elements()], trous_platine=trous_platine())


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for nom, s in pieces():
        b = s.BoundingBox()
        print('%-30s valide %s solides %d  volume %6.1f cm3  x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (
            nom, s.isValid(), len(s.Solids()), s.Volume() / 1000, b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.step'))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
    json.dump(positions(), open(os.path.join(out, 'chassis_kart.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in positions().items() if k != 'trous_platine'}))
