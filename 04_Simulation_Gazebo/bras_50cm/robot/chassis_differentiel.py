"""PROPOSITION (27/11/2026, a part) : ROBOT DIFFERENTIEL COMPACT. Demande de l'utilisateur : 2 moteurs MCC (un a gauche, un a
droite), roues sur les cotes, capteurs a l'avant, place pour la batterie et le bras, robot bas, inertie maitrisee, bras en equilibre,
pas trop grand, bati pas plus haut qu'aujourd'hui.

Choix (chiffres : etude_chassis_diff.log, stabilite_chassis_diff.py) :
- carre de 200 mm (reglement) centre sur l'axe du bras ; dessus du bati a 80,1 mm du sol, comme aujourd'hui (le reseau du bras a ete
  entraine avec cette hauteur) ;
- 2 moteurs jaunes (TT, ceux du robot) chacun avec UNE roue, roues a l'exterieur du bati (face exterieure a 100 mm de l'axe), voie
  186,5 mm ; essieu 10 mm derriere l'axe du bras : le bras est presque au-dessus des roues, le robot pivote sur place sous le bras ;
- 4 capteurs EN FLECHE (meme disposition que sur le robot actuel), elements interieurs 102 mm devant l'essieu, a 8 mm du sol, sur un
  NEZ fixe qui porte aussi les 2 BILLES avant (les capteurs restent toujours a la meme hauteur, les billes sont a cote d'eux) ;
- tout ce qui est lourd en BAS et pres de l'axe de rotation : 2 batteries couchees dans l'axe du robot, centrees sur l'essieu, a
  9 mm du sol ; PCB + ESP32 juste au-dessus des batteries, sous le bati ; moteurs au ras des roues ;
- 2 patins arriere a 1 mm du sol (le robot ne s'appuie dessus que si le bras part en arriere ou en forte acceleration).
Repere : celui de l'assemblage Assemblage1v2 (mm ; x vers l'avant du bras, y vers le haut, z en travers ; sol y 449,6 ; axe du bras
x 164,25 z 512,72 ; dessus du bati y 529,69). Pieces dessinees dans ce repere, inserees a l'origine dans SolidWorks.
Usage : ~/venv_cao/bin/python chassis_differentiel.py <dossier>
"""
import json
import math
import os
import sys
import cadquery as cq
import support_capteurs_fleche as SC

XA, ZM, YS = 164.25, 512.72, 449.6           # axe du bras, milieu, sol
YT, EP = 529.69, 5.0                         # dessus du bati, epaisseur
YB = YT - EP                                 # dessous du bati
X0, X1, W_BATI, R_COIN = XA - 98.0, XA + 98.0, 98.0, 16.0
# roues et moteurs
XE = XA - 10.0                               # essieu
R_ROUE, L_ROUE, L_MOYEU = 30.0, 13.5, 12.8
YE = YS + R_ROUE
W_ROUE_EXT = 100.0
W_ROUE = W_ROUE_EXT - L_ROUE / 2             # 93,25 : milieu du pneu
W_FACE_MOT = W_ROUE_EXT - L_MOYEU - 1.0      # 86,2 : face exterieure du corps du moteur (1 mm du moyeu)
C_MOT = 9.5                                  # demi-epaisseur du corps (axe de l'arbre) ; bossages des trous : +1,5 cote interieur,
                                             # clip (a 23..34) : +2,3 cote roue (mesure sur mcc.SLDPRT exporte en STL)
BOSSAGE = 1.5
W_MOT = W_FACE_MOT - C_MOT                   # 76,8 : milieu du corps
W_MOT_INT = W_MOT - C_MOT                    # 67,4 : face interieure
A_MOT = (-10.92, 54.73)                      # corps le long de x depuis l'arbre (vers l'avant)
B_MOT = 11.17                                # demi-hauteur
TROUS_MOT = ((20.25, 8.71), (20.25, -8.71))  # trous de fixation (a, b) : traversent le corps parallelement a l'arbre (vis M3 x 30)
A_CLIP = 34.5                                # au-dela : plus de clip sur la face cote roue
# capteurs : la fleche de support_capteurs_fleche.py, tournee de 180 deg autour de la verticale, montee de 7 mm (sol 442,6 -> 449,6)
L_CAPT = 102.0                               # essieu -> elements interieurs
X_RETOURNE = XE + L_CAPT + SC.X_IN           # x_nouveau = X_RETOURNE - x_ancien
DY_SC = YS - SC.Y_SOL
X_BILLE, W_BILLE, R_BILLE = X_RETOURNE - SC.X_OUT, 62.0, 8.0
Y_BILLE = YS + R_BILLE
# batteries (2 batons de 110 x 18, jusqu'a 20,5 de diametre) couchees le long de x, centrees sur l'essieu
L_BAT, D_SIEGE, W_BAT = 113.0, 20.8, 11.0      # batterie modelisee : 111 mm (1 mm de jeu a chaque bout)
Y_FOND = YS + 9.0                            # dessous du berceau
EP_FOND = 2.5
Y_BAT = Y_FOND + EP_FOND + 9.0               # centre d'un baton de 18
# PCB (06_PCB/pcb_mave.py : 100 x 72, trous a 4 mm des coins) sous le bati, au-dessus des batteries, USB vers l'arriere
PCB_X0, PCB_Y = XA - 94.25, 488.0            # bord arriere du PCB, dessous du PCB
PCB_W, PCB_L, PCB_EP = 100.0, 72.0, 1.6
# colonnes du berceau (vis M3 depuis le dessus du bati, hors du socle du bras qui couvre |w| < 47)
COLONNES = ((PCB_X0 + 4.0, 56.0), (PCB_X0 + 68.0, 56.0))
JEU_PATIN = 1.0
M3, M3_ECROU = 3.4, 6.4


def boite(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, y0, y1, r):
    y0, y1 = sorted((y0, y1))
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_z(x, y, z0, z1, r):
    z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').circle(r).extrude(z1 - z0).translate((x, y, z0))


def cyl_x(y, z, x0, x1, r):
    x0, x1 = sorted((x0, x1))
    return cq.Workplane('YZ').circle(r).extrude(x1 - x0).translate((x0, y, z))


def wz(sg, a):
    return ZM + sg * a


# ------------------------------------------------------------------------------------------------------------------ bati
def trous_bati():
    """(x, w, diametre, role) de tous les trous du bati."""
    t = []
    for dx in (-33.0, 33.0):
        for dw in (-33.0, 33.0):
            t.append((XA + dx, dw, M3, 'bras'))
    for x in (81.25, 118.75):
        for w in (-74.97, -37.47):
            t.append((x, w, M3, 'L298N'))
    for sg in (-1, 1):
        for x in (XE - 4.0, XE + 50.0):
            t.append((x, sg * 78.0, M3, 'etrier moteur'))
        for x in (X_BILLE - 8.0, X_BILLE + 8.0):
            t.append((x, sg * W_BILLE, M3, 'nez'))
        for x, w in COLONNES:
            t.append((x, sg * w, M3, 'berceau'))
    return t


def bati():
    p = (cq.Workplane('XZ').center(XA, ZM).rect(X1 - X0, 2 * W_BATI).extrude(-EP).edges('|Y').fillet(R_COIN)
         .translate((0, YB, 0)))
    for x, w, d, _ in trous_bati():
        p = p.cut(cyl_y(x, ZM + w, YB - 1, YT + 1, d / 2))
    p = p.cut(cyl_y(XA + 10.2, ZM, YB - 1, YT + 1, 17.0))                        # passage des cables du bras (trou du socle)
    p = p.cut(boite(XE + 74.0, XE + 82.0, YB - 1, YT + 1, ZM - 10, ZM + 10))      # cables des capteurs
    p = p.cut(cyl_y(100.0, ZM - 24.0, YB - 1, YT + 1, 6.0))                       # batteries -> L298N / interrupteur
    for sg in (-1, 1):
        p = p.cut(cyl_y(XE - 16.0, wz(sg, 62.0), YB - 1, YT + 1, 5.0))           # fils des moteurs
        p = p.cut(boite(XA + 52.0, XA + 72.0, YB - 1, YT + 1, wz(sg, 14.0), wz(sg, 44.0)).edges('|Y').fillet(4.0))   # allegement
        p = p.cut(boite(X0 + 14.0, X0 + 50.0, YB - 1, YT + 1, wz(sg, 14.0), wz(sg, 26.0)).edges('|Y').fillet(3.0))   # allegement
    # nervures sous les bords lateraux (raideur), au-dessus des roues (haut des roues y 509,6)
    for sg in (-1, 1):
        p = p.union(boite(X0 + 12.0, X1 - 12.0, YB - 6.0, YB + 0.1, wz(sg, W_BATI - 3.0), wz(sg, W_BATI)))
    return p.val().clean()


# --------------------------------------------------------------------------------------------------------- etriers moteur
def etrier(sg):
    """Tient un moteur : flasque interieure (2 vis M3 x 30 a travers le moteur, tete cote roue dans le creux entre moyeu et voile,
    ecrou cote interieur), collier ferme derriere la roue, semelle vissee sous le bati (2 M3)."""
    w_in1 = W_MOT_INT - BOSSAGE - 0.2
    w_in0 = w_in1 - 3.8
    w_ext0, w_ext1 = W_FACE_MOT + 0.3, W_FACE_MOT + 4.3
    xa, xb = XE - 10.0, XE + A_MOT[1] + 1.5
    xc = XE + A_CLIP                                                    # collier derriere la roue et le clip
    y_bas = YE - B_MOT - 0.2
    e = boite(xa, xb, YE - B_MOT - 2.0, YB, wz(sg, w_in0), wz(sg, w_in1))          # flasque interieure
    e = e.union(boite(xa, xb, YB - 4.0, YB, wz(sg, w_in0), wz(sg, w_ext1)))         # semelle
    e = e.union(boite(xc, xb, y_bas - 4.0, YB, wz(sg, w_ext0), wz(sg, w_ext1)))     # joue exterieure
    e = e.union(boite(xc, xb, y_bas - 4.0, y_bas, wz(sg, w_in0), wz(sg, w_ext1)))   # sangle sous le moteur
    e = e.union(boite(xa, xa + 4.0, YE + B_MOT + 3.0, YB, wz(sg, w_in0), wz(sg, 82.0)))
    e = e.union(boite(xb - 4.0, xb, YE + B_MOT + 0.4, YB, wz(sg, w_in0), wz(sg, w_ext1)))
    # trous
    e = e.cut(cyl_z(XE, YE, wz(sg, w_in0 - 1), wz(sg, w_in1 + 1), 4.0))              # bout d'arbre interieur
    for a, b in TROUS_MOT:
        e = e.cut(cyl_z(XE + a, YE + b, wz(sg, w_in0 - 1), wz(sg, w_in1 + 1), M3 / 2))
    for x in (XE - 4.0, XE + 50.0):
        e = e.cut(cyl_y(x, wz(sg, 78.0), YB - 6, YB + 1, M3 / 2))
        e = e.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(2.6).translate((x, YB - 4.0 + 2.6, wz(sg, 78.0))))
    return e.val().clean()


# ------------------------------------------------------------------------------------------------------ nez (capteurs + billes)
def nez():
    s = cq.Workplane().add(SC.support()).translate((0, DY_SC, 0))
    s = s.rotate((X_RETOURNE / 2, 0, ZM), (X_RETOURNE / 2, 1, ZM), 180)
    y_pl0, y_pl1 = SC.Y_PL[0] + DY_SC, SC.Y_PL[1] + DY_SC
    s = s.cut(boite(150, 300, y_pl1 + 0.01, 560, 400, 620))        # plus de plaque verticale (elle etait collee au boitier moteur)
    xo0, xo1 = X_BILLE - SC.LARG / 2 - SC.JEU, X_BILLE + SC.LARG / 2 + SC.JEU
    for sg in (-1, 1):
        wc = wz(sg, W_BILLE)
        s = s.union(boite(xo0, xo1, y_pl0, y_pl1, wz(sg, 45.0), wc))                                 # pont aile -> colonne
        s = s.union(boite(xo0, xo1, y_pl0 - 3.0, y_pl0 + 0.1, wz(sg, 45.0), wc))                    # nervure sous le pont
        s = s.union(boite(X_BILLE - 7, X_BILLE + 7, Y_BILLE + 4.0, YB, wc - 7, wc + 7))             # colonne
        s = s.union(boite(X_BILLE - 12, X_BILLE + 12, YB - 5.0, YB, wc - 7, wc + 7))                # semelle sous le bati
        s = s.union(cyl_y(X_BILLE, wc, YS + 5.4, Y_BILLE + 6.0, R_BILLE + 2.6))                     # coupelle
        s = s.cut(cq.Workplane('XY').sphere(R_BILLE + 0.15).translate((X_BILLE, Y_BILLE, wc)))
        for k in range(3):                                                                            # 3 fentes : la bille s'emboite
            t = math.radians(90 + 120 * k)
            s = s.cut(boite(-0.6, 0.6, YS + 5.0, YS + 10.0, 0, R_BILLE + 4).rotate((0, 0, 0), (0, 1, 0), math.degrees(t))
                      .translate((X_BILLE, 0, wc)))
        for x in (X_BILLE - 8.0, X_BILLE + 8.0):
            s = s.cut(cyl_y(x, wc, YB - 6, YB + 1, M3 / 2))
            s = s.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(2.6).translate((x, YB - 5.0 + 2.6, wc)))
    return s.val().clean()


def bille(sg):
    return cq.Workplane('XY').sphere(R_BILLE).translate((X_BILLE, Y_BILLE, wz(sg, W_BILLE))).val()


# ------------------------------------------------------------------------------------------- berceau (batteries, PCB, patins)
def berceau():
    xb0, xb1 = XE - L_BAT / 2 - 1.5, XE + L_BAT / 2 + 1.5
    yf0, yf1 = Y_FOND, Y_FOND + EP_FOND
    ys = yf1 + D_SIEGE / 2                                           # centre des sieges
    b = boite(xb0, xb1, yf0, ys + 1.0, ZM - 23.5, ZM + 23.5)
    for sg in (-1, 1):
        b = b.cut(cyl_x(ys, wz(sg, W_BAT), xb0 + 1.5, xb1 - 1.5, D_SIEGE / 2))
    b = b.cut(boite(xb0 + 1.5, xb1 - 1.5, ys, ys + 2, ZM - 22, ZM + 22))
    for x in (xb0, xb1 - 1.5):                                       # butees en bout
        b = b.union(boite(x, x + 1.5, yf0, Y_BAT + 6.0, ZM - 23.5, ZM + 23.5))
    b = b.cut(boite(xb0 - 1, xb1 + 1, Y_BAT - 4, Y_BAT + 4, ZM - 4, ZM + 4))            # passage des fils des batteries
    # oreilles a fente pour 2 sangles (velcro 20 mm) qui entourent les 2 batteries
    for xs in (XE - 32.0, XE + 32.0):
        for sg in (-1, 1):
            b = b.union(boite(xs - 13, xs + 13, yf0, yf1 + 2.0, wz(sg, 23.0), wz(sg, 31.0)))
            b = b.cut(boite(xs - 10.5, xs + 10.5, yf0 - 1, yf1 + 3, wz(sg, 25.5), wz(sg, 28.5)))
    # echelle : 2 longerons jusqu'aux traverses des colonnes
    xc0, xc1 = COLONNES[0][0], COLONNES[1][0]
    for sg in (-1, 1):
        b = b.union(boite(xc0 - 4, xb0 + 1, yf0, yf1 + 2.0, wz(sg, 17.0), wz(sg, 23.5)))
    for xc, wc in COLONNES:
        b = b.union(boite(xc - 4, xc + 4, yf0, yf1 + 2.0, ZM - wc - 4, ZM + wc + 4))     # traverse
        for sg in (-1, 1):
            w = wz(sg, wc)
            y_bas = YS + JEU_PATIN + 6.0 if xc == xc0 else yf0
            b = b.union(boite(xc - 4, xc + 4, y_bas, YB, w - 4, w + 4))                   # colonne
            b = b.union(boite(xc - 4, xc + 4, PCB_Y - 4.0, PCB_Y, wz(sg, 42.0), w))       # patte du PCB
            b = b.cut(cyl_y(xc, wz(sg, 46.0), PCB_Y - 5, PCB_Y + 1, M3 / 2))
            b = b.cut(cq.Workplane('XZ').polygon(6, M3_ECROU).extrude(2.6).translate((xc, PCB_Y - 4.0 + 2.6, wz(sg, 46.0))))
            b = b.cut(cyl_y(xc, w, YB - 14, YB + 1, M3 / 2))                               # vis depuis le dessus du bati
            b = b.cut(boite(xc - 3.2, xc + 3.2, YB - 11, YB - 8.4, w - 3.2 if sg > 0 else w - 5, w + 5 if sg > 0 else w + 3.2))
            if xc == xc0:                                                                     # patin anti-bascule (1 mm du sol)
                b = b.union(cq.Workplane('XY').sphere(6.0).translate((xc, YS + JEU_PATIN + 6.0, w)))
                b = b.union(cyl_y(xc, w, YS + JEU_PATIN + 6.0, YS + JEU_PATIN + 14.0, 6.0))
    for sg in (-1, 1):                                               # la traverse avant passe sous les batteries : sieges recreuses
        b = b.cut(cyl_x(ys, wz(sg, W_BAT), xb0 + 1.5, xb1 - 1.5, D_SIEGE / 2))
    return b.val().clean()


# ------------------------------------------------------------------------------------------------------------------ PCB
def pcb():
    """Encombrement du PCB 06_PCB (100 x 72), avec les 2 barrettes femelles de l'ESP32, C1 et le bornier 5 V (pour l'assemblage)."""
    def P(X, Y):                       # coordonnees du PCB -> assemblage (Y du PCB vers l'avant, X du PCB vers +z)
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


# -------------------------------------------------------------------------------------------------------------- export
def pieces():
    return [('bati differentiel', bati()), ('etrier moteur gauche', etrier(1)), ('etrier moteur droit', etrier(-1)),
            ('nez capteurs et billes', nez()), ('bille 16 gauche', bille(1)), ('bille 16 droite', bille(-1)),
            ('berceau batteries PCB et patins', berceau()), ('PCB MAVE (encombrement)', pcb())]


def positions():
    return dict(XA=XA, ZM=ZM, YS=YS, YT=YT, XE=XE, YE=YE, W_ROUE=W_ROUE, W_MOT=W_MOT, A_MOT=A_MOT, Y_BAT=Y_BAT, W_BAT=W_BAT,
                X_RETOURNE=X_RETOURNE, DY_SC=DY_SC, X_BILLE=X_BILLE, W_BILLE=W_BILLE, Y_BILLE=Y_BILLE, PCB_X0=PCB_X0, PCB_Y=PCB_Y,
                patins=[[c[0], c[1]] for c in COLONNES[:1]], jeu_patin=JEU_PATIN, L_CAPT=L_CAPT,
                elements=[[X_RETOURNE - x, 2 * ZM - z] for x, z in SC.elements()], trous_bati=trous_bati())


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for nom, s in pieces():
        b = s.BoundingBox()
        print('%-34s valide %s solides %d  volume %6.1f cm3  x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (
            nom, s.isValid(), len(s.Solids()), s.Volume() / 1000, b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.step'))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
    json.dump(positions(), open(os.path.join(out, 'chassis_differentiel.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in positions().items() if k != 'trous_bati'}))
