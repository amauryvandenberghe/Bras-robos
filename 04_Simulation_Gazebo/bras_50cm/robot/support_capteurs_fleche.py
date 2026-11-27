"""Support de 4 capteurs de ligne TCRT5000 EN FLECHE sur le bloc moteur de direction (27/11/2026). Remplace le support a 6 capteurs
(3 devant + 3 derriere, support_capteurs_direction.py) : plus de rangee arriere, elements releves (8 mm du sol au lieu de 5).

Disposition choisie par simulation (03_Reseau_de_neurones/esp32/simulation_ligne/etude_fleche.py, etude_fleche_geometrie.log) :
- paire INTERIEURE : 2 cartes cote a cote, le long du robot, entre les 2 roues motrices ; elements a +-B_IN du milieu
  (15,2 mm d'ecart : la ligne de 19 mm est toujours vue par au moins un des deux, pas de zone morte au centre) ;
- paire EXTERIEURE : cartes en travers vers l'exterieur, devant les roues, elements a +-B_OUT ; RECUL mm en arriere des
  interieurs (pointe de la fleche vers l'avant). Les 4 elements forment un V ouvert vers l'arriere.
Repere : celui de support_capteurs_direction.py (assemblage Assemblage1v2 avant le rabaissement du 22/11 : sol y 442,6 ; le bloc et
le sol ont ete montes ensemble de 7 mm, la geometrie relative ne change pas). x vers l'ancien avant : le robot roule vers les x
decroissants ; pivot de direction x 37,3, z 512,72.
Cartes TCRT5000 (mesurees le 19/11) : 14,1 mm de large, element a 3,8 mm d'un bout et 27,4 de l'autre, trou de fixation a 20,3 mm
de l'element (M3 + ecrou sous la plaque), broches du connecteur de 23,4 a 26,2 mm. Fenetre de l'element : 11 x 7,8 mm.
Usage : ~/venv_cao/bin/python support_capteurs_fleche.py <dossier>   (fusion avec le boitier : fusion_moteur_capteurs.py fleche)
"""
import json
import os
import sys
import cadquery as cq

Z_M = 512.72
Y_SOL = 442.6
H_ELEM = 8.0                       # 27/11 : face de l'element a 8 mm du sol (5 avant : frottements)
Y_ELEM = Y_SOL + H_ELEM
EP_PLAQUE = 4.0                    # 3 avant ; la carte repose sur le dessus de la plaque (element 10 mm sous la carte)
Y_PL = (Y_ELEM + 10.0 - EP_PLAQUE, Y_ELEM + 10.0)
B_IN, B_OUT = 7.6, 19.5            # demi-ecarts lateraux des elements (mm)
X_OUT = 5.5                        # x des elements exterieurs (devant les roues)
RECUL = 8.0                        # les interieurs sont RECUL mm plus en avant (x plus petit)
X_IN = X_OUT - RECUL
TROU_CARTE, PIQUES, L_AV, L_AR, LARG = 20.3, (23.4, 26.2), 3.8, 27.4, 14.1
TROU_M3 = 3.4
X_FACE = 27.1                      # face avant du boitier (27,9 dans la fusion : plaque enfoncee pour souder)
JEU = 1.0


def boite(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, y0, y1, r):
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def elements():
    """Centres des 4 elements (x, z), de gauche a droite dans le sens de marche (z croissant = gauche ? voir positions())."""
    return [(X_OUT, Z_M - B_OUT), (X_IN, Z_M - B_IN), (X_IN, Z_M + B_IN), (X_OUT, Z_M + B_OUT)]


def support():
    y0, y1 = Y_PL
    zi = B_IN + LARG / 2 + JEU                     # demi-largeur de la partie centrale (cartes interieures)
    ze = B_OUT + L_AR + JEU                        # bout des ailes (cartes exterieures)
    xa0, xa1 = X_OUT - LARG / 2 - JEU, X_OUT + LARG / 2 + JEU
    # partie centrale : de la pointe jusqu'a la face du boitier
    s = boite(X_IN - L_AV - JEU, X_FACE, y0, y1, Z_M - zi, Z_M + zi)
    # ailes des cartes exterieures
    s = s.union(boite(xa0, xa1, y0, y1, Z_M - ze, Z_M + ze))
    # nervures sous les ailes (raideur), 3 mm sous la plaque, bord arriere et bord avant
    for zz0, zz1 in ((Z_M - ze, Z_M - zi + 2), (Z_M + zi - 2, Z_M + ze)):
        s = s.union(boite(xa1 - 2.0, xa1, y0 - 3.0, y0 + 0.1, zz0, zz1))
        s = s.union(boite(xa0, xa0 + 2.0, y0 - 3.0, y0 + 0.1, zz0, zz1))
    # plaque collee sur la face avant du boitier (fusionnee) + nervures verticales de part et d'autre
    s = s.union(boite(X_FACE - 2.5, X_FACE, y0, 488.0, Z_M - 9.5, Z_M + 9.5))
    a_oter = []
    # cartes interieures : le long de x, element vers l'avant, carte vers le boitier
    for z in (Z_M - B_IN, Z_M + B_IN):
        a_oter.append(boite(X_IN - 3.9, X_IN + 3.9, y0 - 1, y1 + 1, z - 5.5, z + 5.5))
        a_oter.append(cyl_y(X_IN + TROU_CARTE, z, y0 - 1, y1 + 1, TROU_M3 / 2))
        a_oter.append(boite(X_IN + PIQUES[0], X_IN + PIQUES[1], y1 - 1.3, y1 + 1, z - 6.5, z + 6.5))
    # cartes exterieures : en travers vers l'exterieur
    for sg, z in ((-1, Z_M - B_OUT), (1, Z_M + B_OUT)):
        a_oter.append(boite(X_OUT - 5.5, X_OUT + 5.5, y0 - 1, y1 + 1, z - 3.9, z + 3.9))
        a_oter.append(cyl_y(X_OUT, z + sg * TROU_CARTE, y0 - 4, y1 + 1, TROU_M3 / 2))
        a_oter.append(boite(X_OUT - 6.5, X_OUT + 6.5, y1 - 1.3, y1 + 1, z + sg * PIQUES[0], z + sg * PIQUES[1]))
    for o in a_oter:
        s = s.cut(o)
    return s.val().clean()


def cartes():
    """Volumes des 4 cartes (pour verifier qu'elles ne se touchent pas et ne touchent pas les roues)."""
    y1 = Y_PL[1]
    c = []
    for z in (Z_M - B_IN, Z_M + B_IN):
        c.append(boite(X_IN - L_AV, X_IN + L_AR, y1, y1 + 1.6, z - LARG / 2, z + LARG / 2))
    for sg, z in ((-1, Z_M - B_OUT), (1, Z_M + B_OUT)):
        c.append(boite(X_OUT - LARG / 2, X_OUT + LARG / 2, y1, y1 + 1.6, z - sg * L_AV, z + sg * L_AR))
    return c


def positions():
    return dict(elements=[list(e) for e in elements()], y_bas_element=Y_ELEM, plaque=list(Y_PL), b_in=B_IN, b_out=B_OUT, x_in=X_IN,
                x_out=X_OUT, recul=RECUL, hauteur=H_ELEM)


if __name__ == '__main__':
    sortie = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    os.makedirs(sortie, exist_ok=True)
    s = support()
    bb = s.BoundingBox()
    print('support_capteurs_fleche valide', s.isValid(), 'solides', len(s.Solids()), 'masse %.1f g' % (s.Volume() * 1.24e-3 * 0.8),
          'x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax))
    cs = cartes()
    for i in range(4):
        for j in range(i + 1, 4):
            v = cs[i].intersect(cs[j]).val().Volume() if cs[i].intersect(cs[j]).vals() else 0.0
            if v > 1e-6:
                print('ATTENTION cartes %d et %d se chevauchent (%.1f mm3)' % (i, j, v))
    w = cq.Workplane().add(s)
    cq.exporters.export(w, os.path.join(sortie, 'support_capteurs_fleche.step'))
    cq.exporters.export(w, os.path.join(sortie, 'support_capteurs_fleche.stl'), tolerance=0.05, angularTolerance=0.2)
    json.dump(positions(), open(os.path.join(sortie, 'support_capteurs_fleche.json'), 'w'), indent=1)
    print(positions())
