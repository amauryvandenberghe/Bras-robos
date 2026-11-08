"""Direction du robot (08/11/2026) : MG996R a la place du MG90S et 2e roulement sous le bati.

Repere : celui de Assemblage1v2 (mm), x vers l'avant, y vers le haut, z en travers. Pieces inserees a l'origine.
- cuve_direction : 2e roulement (8 x 22 x 7) sous le bati, le plus bas possible. Il guide la tige et, par l'entretoise qui
  appuie sur sa bague INTERIEURE, reprend la poussee verticale de la roue motrice (avant : seule la goupille et le servo
  la retenaient). Roulement entre par le dessous, levre au-dessus.
- support_mg996r : semelle identique a l'ancien support (memes 2 trous du bati, memes vis du maintien du 1er roulement),
  caisson + tablette : le servo entre par le dessus, ses pattes se posent sur la tablette, 4 vis M3 + ecrous glisses.
- liaison_mg996r : arm01 dans une empreinte, 4 vis de 2 mm ; manchon de 8,2 mm sur la tige (9 mm avant : du jeu),
  goupille M3 dans le VRAI trou de la tige (y = 549,48 ; le modele d'origine le ratait de 3,6 mm).
Usage : ~/venv_cao/bin/python direction.py <dossier de sortie>
"""
import json
import os
import sys
import numpy as np
import cadquery as cq
from scipy.spatial import ConvexHull

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..'))
import cao_bras as C  # noqa: E402

AX, AZ = 37.26, 512.72            # axe de la tige
Y_BATI_BAS, Y_BATI_HAUT = 524.69, 529.7
Y_MOTEUR = 504.6                  # dessus du bloc moteur (tourne avec la tige)
Y_TIGE_HAUT, Y_GOUPILLE = 554.6, 549.48
TROUS_BATI = [(54.76, 477.22), (54.76, 548.22)]   # trous existants (ancien support)
TROU_NEUF = (18.26, 512.72)       # seul trou a ajouter au bati
ROUL_BAS = 514.7                  # bas du 2e roulement : la tige n'a son diametre de 8 qu'au-dessus de y = 514,6 (carre + evasement dessous)
Y_PAL = 560.0                     # dessus du palonnier arm01 (face tournee vers le bas)
Z_TABLETTE_EP = 5.0
# servo : model x -> +x, model y -> +z, model z -> -y
def servo_vers_monde(p):
    return np.array([AX + p[0] - C.MG_REF[0], Y_PAL + C.MG_REF[2] - p[2], AZ + p[1] - C.MG_REF[1]], float)


def T_servo():
    T = np.eye(4)
    T[:3, :3] = np.column_stack([(1, 0, 0), (0, 0, 1), (0, -1, 0)])
    T[:3, 3] = servo_vers_monde((0, 0, 0))
    return T


def T_arm():
    """Repere propre de arm01 -> monde : z local vers le bas (cote oppose au servo), bras le long de z monde."""
    T = np.eye(4)
    T[:3, :3] = np.column_stack([(0, 0, 1), (-1, 0, 0), (0, -1, 0)])
    T[:3, 3] = np.array((AX, Y_PAL, AZ)) - T[:3, :3] @ np.array((0, 0, C.PALONNIER['arm01']['ref_z']))
    return T


def boite(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(x, z, y0, y1, r):
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_z(x, y, z0, z1, r):
    return cq.Workplane('XY').circle(r).extrude(z1 - z0).translate((x, y, z0))


def cyl_x(y, z, x0, x1, r):
    return cq.Workplane('YZ').circle(r).extrude(x1 - x0).translate((x0, y, z))


def prisme(pts, y0, y1):
    return cq.Workplane('XZ', origin=(0, y1, 0)).polyline([tuple(p) for p in pts]).close().extrude(y1 - y0)


def enveloppe(cercles, n=64):
    pts = np.array([(x + r * np.cos(a), z + r * np.sin(a)) for x, z, r in cercles for a in np.linspace(0, 2 * np.pi, n, endpoint=False)])
    return pts[ConvexHull(pts).vertices]


def ecrou_y(x, z, y0, y1, fente_x=None):
    h = cq.Workplane('XZ', origin=(x, y1, z)).polygon(6, C.M3_ECROU / 0.866).extrude(y1 - y0)
    if fente_x is not None:
        h = h.union(boite(min(x, fente_x), max(x, fente_x), y0, y1, z - C.M3_ECROU / 2, z + C.M3_ECROU / 2))
    return h


# ---- cote du servo dans le monde
def bornes(xs, ys, zs, jeu=0.0):
    c = np.array([servo_vers_monde((x, y, z)) for x in xs for y in ys for z in zs])
    return c.min(0) - jeu, c.max(0) + jeu


def support():
    a, b = bornes(C.MG_X, C.MG_Y, (C.MG_FOND, C.MG_DESSUS), C.JEU)    # boitier
    y_appui = servo_vers_monde((0, 0, C.MG_PATTE_HAUT))[1]            # face des pattes cote sortie
    y0 = y_appui - Z_TABLETTE_EP
    x_av, x_ar = 20.0, 76.7                                           # tablette
    z0, z1 = 497.0, 528.0
    semelle = boite(48.27, 61.26, Y_BATI_HAUT, 536.69, 470.22, 555.22).edges('|Y').fillet(4.0)
    caisson = boite(60.5, x_ar, Y_BATI_HAUT, y_appui, z0, z1)
    tablette = boite(x_av, x_ar, y0, y_appui, z0, z1)
    rails = [boite(x_av, x_ar, 566.5, y_appui, z0, a[2]), boite(x_av, x_ar, 566.5, y_appui, b[2], z1)]
    s = semelle.union(caisson).union(tablette).union(rails[0]).union(rails[1])
    creux = boite(64.5, x_ar - 4.0, Y_BATI_HAUT - 1, y0 + 0.01, z0 + 4.0, z1 - 4.0)
    fenetre = boite(a[0], b[0], 566.0, y_appui + 1, a[2], b[2])
    nervures = [boite(xa, xb, y_appui - 1.7, y_appui + 1, AZ - 0.8, AZ + 0.8) for xa, xb in ((x_av - 1, a[0] + 0.1), (b[0] - 0.1, x_ar + 1))]   # nervure centrale sous chaque patte
    passage = boite(x_av - 1, 60.5, 500.0, y0, a[2], b[2])            # sous la tablette : le palonnier tourne (rayon 21)
    s = s.cut(creux).cut(fenetre).cut(passage).cut(nervures[0]).cut(nervures[1])
    for x, z in TROUS_BATI:
        s = s.cut(cyl(x, z, Y_BATI_HAUT - 1, 537.7, C.M3 / 2))
    for z in (492.72, 532.72):                                         # vis du maintien du 1er roulement
        s = s.cut(cyl_x(533.2, z, 47.0, 62.5, C.M3 / 2))
    y_e = y0 + 1.4
    for xm, ym in C.MG_TROUS:
        p = servo_vers_monde((xm, ym, 0))
        s = s.cut(cyl(p[0], p[2], y0 - 6, y_appui + 1, C.M3 / 2))
        s = s.cut(ecrou_y(p[0], p[2], y_e - 1.4, y_e + 1.4, x_av - 1 if p[0] < AX else x_ar + 1))
    return s.val().clean()


def cuve():
    r_siege, r_levre = C.SIEGE_R, C.LEVRE_R
    y_levre = ROUL_BAS + C.ROUL_B
    tube = cyl(AX, AZ, ROUL_BAS, Y_BATI_BAS, 14.0)
    cercles = [(TROUS_BATI[0][0], TROUS_BATI[0][1], 7.0), (TROUS_BATI[1][0], TROUS_BATI[1][1], 7.0), (TROU_NEUF[0], TROU_NEUF[1], 5.5), (AX, AZ, 14.0)]
    bride = prisme(enveloppe(cercles), Y_BATI_BAS - 4.0, Y_BATI_BAS)
    s = tube.union(bride)
    for x, z in TROUS_BATI + [TROU_NEUF]:                              # nervures entre le tube et chaque vis
        d = np.hypot(x - AX, z - AZ)
        ang = np.degrees(np.arctan2(z - AZ, x - AX))
        n = (cq.Workplane('XY').polyline([(13.0, Y_BATI_BAS - 3.9), (d - 5.5, Y_BATI_BAS - 3.9), (d - 5.5, Y_BATI_BAS - 6.5), (13.0, ROUL_BAS + 3.0)])
             .close().extrude(2.5, both=True).rotate((0, 0, 0), (0, 1, 0), -ang).translate((AX, 0, AZ)))
        s = s.union(n)
    s = s.cut(cyl(AX, AZ, ROUL_BAS - 1, y_levre, r_siege)).cut(cyl(AX, AZ, y_levre - 0.1, Y_BATI_BAS + 1, r_levre))
    for x, z in TROUS_BATI + [TROU_NEUF]:
        s = s.cut(cyl(x, z, Y_BATI_BAS - 9, Y_BATI_BAS + 1, C.M3 / 2))
    return s.val().clean()


def entretoise():
    # large sur le bloc moteur, col de 5,75 mm qui n'appuie que sur la bague interieure ; alesage autour de l'evasement de la tige
    y_col = ROUL_BAS - 1.5
    s = cyl(AX, AZ, Y_MOTEUR + 0.02, y_col, 7.0).union(cyl(AX, AZ, y_col - 0.1, ROUL_BAS - 0.01, 5.75))
    return s.cut(cyl(AX, AZ, Y_MOTEUR - 1, y_col, 4.9)).cut(cyl(AX, AZ, y_col - 0.1, ROUL_BAS + 1, 4.4)).val().clean()


def _contour_monde(jeu):
    d = C.PALONNIER['arm01']
    return [(AX - p[1], AZ + p[0]) for p in C._contour(d, jeu)]           # x local -> z monde, y local -> -x monde


def liaison():
    y_bas = Y_TIGE_HAUT + 0.6
    y_haut = Y_PAL + 2.0
    disque = prisme(_contour_monde(C.JEU_EMPREINTE + 2.0), y_bas, y_haut)
    manchon = cyl(AX, AZ, 541.0, y_bas + 0.1, 7.0)
    s = disque.union(manchon)
    s = s.cut(prisme(_contour_monde(C.JEU_EMPREINTE), Y_PAL, y_haut + 1))
    s = s.cut(cyl(AX, AZ, 540.0, y_haut + 1, 4.1))                         # tige + passage du tournevis de la vis centrale
    s = s.cut(cyl_z(AX, Y_GOUPILLE, AZ - 9, AZ + 9, C.M3 / 2))
    for r in (10.5, 16.5, -10.5, -16.5):
        s = s.cut(cyl(AX, AZ + r, y_bas - 1, Y_PAL + 0.1, C.VIS_PAL / 2))
    return s.val().clean()


def placements():
    """Tableaux SolidWorks (9 rotation : images de x, y, z ; 3 translation en m ; 1 ; 0 0 0)."""
    def tab(T):
        return [float(v) for v in T[:3, 0]] + [float(v) for v in T[:3, 1]] + [float(v) for v in T[:3, 2]] + [float(v) / 1000 for v in T[:3, 3]] + [1.0, 0.0, 0.0, 0.0]
    M = C.recalage('mg996r_corps_vers_sborka')
    trous_servo = [[float(v) for v in servo_vers_monde((xm, ym, 0))[[0, 2]]] for xm, ym in C.MG_TROUS]
    y_appui = float(servo_vers_monde((0, 0, C.MG_PATTE_HAUT))[1])
    y_tete = float(servo_vers_monde((0, 0, C.MG_PATTE_BAS))[1])
    return dict(servo=tab(T_servo() @ M), arm=tab(T_arm()), trous_servo=trous_servo, y_appui=y_appui, y_tete=y_tete,
                vis_pal_z=[AZ + r for r in (10.5, 16.5, -10.5, -16.5)], y_bas_liaison=Y_TIGE_HAUT + 0.6)


if __name__ == '__main__':
    sortie = sys.argv[1] if len(sys.argv) > 1 else ICI
    pieces = {'support_mg996r_direction': support, 'cuve_direction': cuve, 'entretoise_direction': entretoise, 'liaison_mg996r_tige': liaison}
    formes = {}
    for nom, f in pieces.items():
        s = f(); formes[nom] = s; bb = s.BoundingBox()
        print(f'{nom:26s} valide {s.isValid()} solides {len(s.Solids())} masse {s.Volume() * C.DENSITE_PLA * C.TAUX_REMPLI:5.1f} g  x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}')
        cq.exporters.export(cq.Workplane().add(s), os.path.join(sortie, nom + '.step'))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(sortie, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
    json.dump(placements(), open(os.path.join(sortie, 'direction_positions.json'), 'w'), indent=1)
    T = T_servo()
    servo = C.modele_servo('mg996r').transformGeometry(cq.Matrix([list(T[0]), list(T[1]), list(T[2])]))
    Ta = T_arm()
    d = C.PALONNIER['arm01']
    arm = [s for s in cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, d['fichier'])).val().Solids() if s.Volume() > 0.5]
    arm = cq.Compound.makeCompound(arm).transformGeometry(cq.Matrix([list(Ta[0]), list(Ta[1]), list(Ta[2])]))
    bs, ba = servo.BoundingBox(), arm.BoundingBox()
    print('servo x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (bs.xmin, bs.xmax, bs.ymin, bs.ymax, bs.zmin, bs.zmax))
    print('arm01 x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (ba.xmin, ba.xmax, ba.ymin, ba.ymax, ba.zmin, ba.zmax))
    print('servo/support %.3f mm3, servo/liaison %.3f, arm/liaison %.3f, arm/support %.3f, cuve/support %.3f' % (
        servo.intersect(formes['support_mg996r_direction']).Volume(), servo.intersect(formes['liaison_mg996r_tige']).Volume(),
        arm.intersect(formes['liaison_mg996r_tige']).Volume(), arm.intersect(formes['support_mg996r_direction']).Volume(),
        formes['cuve_direction'].intersect(formes['support_mg996r_direction']).Volume()))
    # balayage : la liaison et l'arm tournent de +-90 degres autour de la tige
    for ang in range(-90, 91, 15):
        for nom, s0 in (('liaison', formes['liaison_mg996r_tige']), ('arm', arm)):
            s = s0.rotate(cq.Vector(AX, 0, AZ), cq.Vector(AX, 1, AZ), ang)
            v = s.intersect(formes['support_mg996r_direction']).Volume()
            if v > 0.01:
                print('  COLLISION', nom, ang, 'deg', round(v, 2))
    print('balayage +-90 deg termine')
