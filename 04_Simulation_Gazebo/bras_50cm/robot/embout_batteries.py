"""Embout de batteries (20/11/2026) : 2 pieces identiques qui tiennent les 2 batons colles par leurs BOUTS, comme le
"maitient roulement a bille" : un godet a 2 alveoles + une colonne vissee sous le bati par UNE vis M3 x 20 (ecrou captif).
Remplace le berceau + 2 brides (14 vis). Repere de l'assemblage Assemblage1v2 (mm), dessine pour le bout gauche (z faible) ;
l'autre est le meme, tourne de 180 degres autour de la verticale passant par x = 136, z = 512,72.
Montage : visser un embout, engager les batons, poser le 2e embout sur les autres bouts, visser : les batons sont serres entre les deux.
Usage : ~/venv_cao/bin/python embout_batteries.py <dossier>
"""
import os
import sys
import cadquery as cq

Y_BATI = 524.69
Z_M = 512.72
X_C = 136.0                       # milieu des 2 batons (sous le trou de 3,4 mm deja perce dans le bati, x 136, z 449,72)
# 27/11 : les vraies batteries sont plus grosses que les 18 mm du modele (retour de l'utilisateur, diametre non mesure).
# Le logement devient une GORGE OBLONGUE : 2 batons colles de diametre D_BAT_MAX au plus y entrent (plus petits : un peu de jeu,
# a caler avec un tour de ruban adhesif). Pour l'ajuster au diametre mesure : changer D_BAT_MAX et relancer.
D_BAT_MAX = 20.5                  # diametre maximal d'un baton (mm)
JEU_BAT = 0.3
R_LOG = D_BAT_MAX / 2 + JEU_BAT   # rayon des bouts de la gorge (10,55 ; 9,4 avant)
X_A, X_B = X_C - D_BAT_MAX / 2, X_C + D_BAT_MAX / 2   # centres des bouts de la gorge (axes des batons s'ils font D_BAT_MAX)
EQ = 504.39                       # axe des batons
Z_BAT0 = Z_M - 55.0               # bout du baton
FOND = Z_BAT0 - 1.0               # fond de l'alveole (27/11 : 1 mm de jeu en longueur au lieu de 0,5)
PROF = 9.0                        # profondeur de l'alveole
PAROI = 3.0
Z_VIS = 449.72
Y_HAUT = Y_BATI - 6.0             # dessus du godet (broches de l'ESP32 a 519,2)


def boite(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, y0, y1, r):
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def cyl_z(x, y, z0, z1, r):
    return cq.Workplane('XY').circle(r).extrude(z1 - z0).translate((x, y, z0))


def embout():
    z0, z1 = FOND - PAROI, FOND + PROF
    # godet : 2 alveoles entourees de 3 mm
    g = cyl_z(X_A, EQ, z0, z1, R_LOG + PAROI).union(cyl_z(X_B, EQ, z0, z1, R_LOG + PAROI))
    g = g.union(boite(X_A, X_B, EQ - R_LOG - PAROI, EQ + R_LOG + PAROI, z0, z1))
    # pont vers la colonne : de l'arriere du godet a la vis, jusqu'au dessus
    g = g.union(boite(X_C - 6.0, X_C + 6.0, EQ, Y_HAUT, Z_VIS - 5.5, z0 + 1.0))
    g = g.union(boite(X_A - 4, X_B + 4, EQ + 6, Y_HAUT, z0, z0 + PAROI).edges('|Z').fillet(2.0))
    g = g.union(cyl_y(X_C, Z_VIS, Y_HAUT - 0.01, Y_BATI, 4.5))
    # alveoles + passage des fils + vis + ecrou
    for x in (X_A, X_B):
        g = g.cut(cyl_z(x, EQ, FOND, z1 + 1, R_LOG))
        g = g.cut(cyl_z(x, EQ, z0 - 1, FOND + 1, 3.5))
    g = g.cut(boite(X_A, X_B, EQ - R_LOG, EQ + R_LOG, FOND, z1 + 1))      # gorge oblongue entre les 2 bouts
    g = g.cut(cyl_y(X_C, Z_VIS, 508.5, Y_BATI + 1, 1.7))
    g = g.cut(boite(X_C - 2.9, X_C + 2.9, 510.9, 513.5, Z_VIS - 40, Z_VIS + 3.4))
    return g.val().clean()


if __name__ == '__main__':
    sortie = sys.argv[1]
    e = embout()
    b = e.BoundingBox()
    print('embout valide', e.isValid(), 'solides', len(e.Solids()), 'masse %.1f g' % (e.Volume() * 1.24e-3 * 0.8),
          'x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
    w = cq.Workplane().add(e)
    cq.exporters.export(w, os.path.join(sortie, 'embout_batteries.step'))
    cq.exporters.export(w, os.path.join(sortie, 'embout_batteries.stl'), tolerance=0.05, angularTolerance=0.2)
