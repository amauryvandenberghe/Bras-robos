"""Support de 6 capteurs de ligne (2 rangees de 3) sous le bati du robot (07/11/2026, resserre le 08/11/2026).

Repere : celui de l'assemblage du robot complet Assemblage1v2 (mm) : x vers l'avant, y vers le haut, z en travers.
La piece est dessinee directement dans ce repere : on l'insere a l'origine de l'assemblage.

Ecartement : ligne de 19 mm (reglement). Le meilleur ecart est ~ demi-ligne + demi-tache du capteur : 13 mm dans les 2 rangees
(etude : bras_50cm/robot/etude_ecartement_capteurs.py). A 33 mm, +-6 mm autour du centre ne donnaient aucune information.
Des cartes de 14,1 mm cote a cote ne peuvent pas descendre a 13 mm : les 2 capteurs lateraux ont leur carte vers l'exterieur (en travers).
- Rangee avant : lateraux en x 205,05, carte en travers au bord du bati ; central en x 190,55 (14,5 mm derriere), carte en travers aussi.
  Les 2 rangs de cartes font 28,6 mm pour 27,5 disponibles entre poutre et bord du bati : encoche de 1,46 mm dans la poutre (fonction SolidWorks).
  Le lateral reste a 27,8 mm devant l'essieu : la simulation montre qu'il faut garder cette avance (roue directrice a l'arriere).
- Rangee arriere : les 3 elements en x 118,8, central carte le long de x, lateraux en travers. Descendue de 3 mm : meme
  hauteur au sol que l'avant (dans le modele, les roues avant touchent 5,4 mm plus haut que les roues arriere).
- Fixation inchangee : 2 vis M3 x 20 a travers la poutre + colonne sous le bati.

Usage : ~/venv_cao/bin/python support_capteurs.py <dossier de sortie>  -> support_capteurs.step / .stl + positions.json
"""
import json
import os
import sys
import cadquery as cq

# ---- cotes du robot (mesurees dans Assemblage1v2) ----
Y_PLAQUE = (467.7, 470.69)       # plaque avant, 3 mm : son dessus touche le dessous de la poutre (470,696)
BAISSE_AR = 3.0                  # rangee arriere plus basse
Y_JUPE = 457.7                   # bas des jupes a l'avant
Y_BATI = 524.69                  # dessous du bati (524,696)
Z_MILIEU = 512.72
ECART = 13.0                     # ecart lateral entre capteurs
X_ARRIERE = 118.80               # centre des elements, rangee arriere
X_LAT = 205.05                   # centre des elements lateraux, rangee avant (bord de carte a 212,10 ; bati a 212,26)
X_CENTRE = 190.55                # centre de l'element central, rangee avant
Z_POUTRE_VIS = [492.72, 532.72]
X_POUTRE_VIS = 177.26
COLONNE = ((103.26, 111.26), (502.72, 522.72))   # face arriere a 66 mm de l'axe de direction (bloc moteur : 64 mm)
VIS_COLONNE = (107.26, 512.72)
TROU_CARTE = 20.3                # du centre de l'element au trou de la carte
PIQUES = (23.4, 26.2)            # broches du connecteur (1 mm sous la carte), distances au centre de l'element
L_AV, L_AR = 3.8, 27.4           # carte : longueur devant / derriere le centre de l'element
TROU = 3.5
X_FIN_PLAQUE = 212.0


def boite(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, y0, y1, r):
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def support():
    y0, y1 = Y_PLAQUE
    yr0, yr1 = y0 - BAISSE_AR, y1 - BAISSE_AR
    m, e = Z_MILIEU, ECART
    zb = e + L_AR + 1.0           # demi-largeur des ailes qui portent les cartes en travers
    # plaques : arriere (basse), marche, centrale, aile avant
    s = boite(COLONNE[0][0], 160.0, yr0, yr1, m - 13.0, m + 13.0)
    s = s.union(boite(X_ARRIERE - 8.6, X_ARRIERE + 8.6, yr0, yr1, m - zb, m + zb))              # aile arriere
    s = s.union(boite(156.0, 162.0, yr0, y1, m - 13.0, m + 13.0))                                # marche de 3 mm
    s = s.union(boite(160.0, X_CENTRE + 8.6, y0, y1, m - 26.0, m + 26.0))                        # sous la poutre
    s = s.union(boite(X_LAT - 8.0, X_FIN_PLAQUE, y0, y1, m - zb, m + zb))                        # aile avant
    a_oter = []

    def cellule(xa, xb, za, zb2, yb, yh):
        return boite(xa, xb, yb, yh + 0.01, za, zb2)

    # jupes arriere : une jupe commune, 3 cellules
    xr, yj = X_ARRIERE, Y_JUPE - BAISSE_AR
    s = s.union(boite(xr - 8.6, xr + 8.6, yj, yr0 + 0.01, m - e - 7.5, m + e + 7.5).edges('|Y').fillet(2.0))
    a_oter.append(cellule(xr - 4.5, xr + 4.5, m - 6.0, m + 6.0, yj - 1, yr0))                      # central (element 7 x 10,2)
    a_oter.append(boite(xr - 3.9, xr + 3.9, yr0 - 1, yr1 + 1, m - 5.5, m + 5.5))
    for sg in (-1, 1):                                                                            # lateraux (element 10,2 x 7)
        zc = m + sg * e
        a_oter.append(cellule(xr - 6.1, xr + 6.1, zc - 5.0, zc + 5.0, yj - 1, yr0))
        a_oter.append(boite(xr - 5.5, xr + 5.5, yr0 - 1, yr1 + 1, zc - 3.9, zc + 3.9))
    # jupes avant
    s = s.union(boite(X_LAT - 8.0, X_FIN_PLAQUE, Y_JUPE, y0 + 0.01, m - e - 7.5, m + e + 7.5).edges('|Y').fillet(1.0))
    s = s.union(boite(X_CENTRE - 8.0, X_CENTRE + 8.6, Y_JUPE, y0 + 0.01, m - 8.0, m + 8.0).edges('|Y').fillet(1.0))
    for sg in (-1, 1):
        zc = m + sg * e
        a_oter.append(cellule(X_LAT - 6.0, X_LAT + 5.85, zc - 5.0, zc + 5.0, Y_JUPE - 1, y0))
        a_oter.append(boite(X_LAT - 5.5, X_LAT + 5.5, y0 - 1, y1 + 1, zc - 3.9, zc + 3.9))
    a_oter.append(cellule(X_CENTRE - 6.0, X_CENTRE + 6.0, m - 5.0, m + 5.0, Y_JUPE - 1, y0))
    a_oter.append(boite(X_CENTRE - 5.5, X_CENTRE + 5.5, y0 - 1, y1 + 1, m - 3.9, m + 3.9))
    # colonne vers le bati, fente pour l'ecrou + la rondelle (ouverte de part en part en x)
    (cx0, cx1), (cz0, cz1) = COLONNE
    s = s.union(boite(cx0, cx1, yr1 - 0.01, Y_BATI, cz0, cz1))
    a_oter.append(cyl_y(*VIS_COLONNE, 505.0, Y_BATI + 1, TROU / 2))
    a_oter.append(boite(cx0 - 1, cx1 + 1, 516.2, 520.0, VIS_COLONNE[1] - 4.8, VIS_COLONNE[1] + 4.8))
    # trous des cartes (vis M2) + degagement des broches du connecteur
    p0, p1 = PIQUES
    for ya, yb, x, zs in ((yr0, yr1, X_ARRIERE, (-1, 1)), (y0, y1, X_LAT, (-1, 1))):
        for sg in zs:
            z = m + sg * e
            a_oter.append(cyl_y(x, z + sg * TROU_CARTE, ya - 1, yb + 1, TROU / 2))
            za, zb2 = sorted((z + sg * p0, z + sg * p1))
            a_oter.append(boite(x - 6.5, x + 6.5, yb - 1.3, yb + 1, za, zb2))
    a_oter.append(cyl_y(X_ARRIERE + TROU_CARTE, m, yr0 - 1, yr1 + 1, TROU / 2))                     # central arriere : le long de x
    a_oter.append(boite(X_ARRIERE + p0, X_ARRIERE + p1, yr1 - 1.3, yr1 + 1, m - 6.5, m + 6.5))
    a_oter.append(cyl_y(X_CENTRE, m + TROU_CARTE, y0 - 1, y1 + 1, TROU / 2))                        # central avant : carte vers +z
    a_oter.append(boite(X_CENTRE - 6.5, X_CENTRE + 6.5, y1 - 1.3, y1 + 1, m + p0, m + p1))
    # vis de la poutre
    for z in Z_POUTRE_VIS:
        a_oter.append(cyl_y(X_POUTRE_VIS, z, y0 - 1, y1 + 1, TROU / 2))
    for o in a_oter:
        s = s.cut(o)
    return s.val().clean()


def positions():
    return dict(ecart=ECART, x_arriere=X_ARRIERE, x_lateraux_avant=X_LAT, x_central_avant=X_CENTRE, baisse_arriere=BAISSE_AR,
                z_lateraux=[Z_MILIEU - ECART, Z_MILIEU + ECART], z_milieu=Z_MILIEU, ecart_rangees=X_LAT - X_ARRIERE)


if __name__ == '__main__':
    sortie = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    s = support()
    print('valide', s.isValid(), 'volume cm3', round(s.Volume() / 1000, 2), 'masse g (PLA, 80 %)', round(s.Volume() * 1.24e-3 * 0.8, 1), 'solides', len(s.Solids()))
    bb = s.BoundingBox()
    print('boite x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax))
    cq.exporters.export(cq.Workplane().add(s), os.path.join(sortie, 'support_capteurs.step'))
    cq.exporters.export(cq.Workplane().add(s), os.path.join(sortie, 'support_capteurs.stl'), tolerance=0.05, angularTolerance=0.2)
    json.dump(positions(), open(os.path.join(sortie, 'support_capteurs_positions.json'), 'w'), indent=1)
    print(positions())
