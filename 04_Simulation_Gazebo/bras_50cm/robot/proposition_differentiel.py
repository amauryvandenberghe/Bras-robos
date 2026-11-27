"""PROPOSITION (27/11/2026, a part de l'assemblage du robot) : direction DIFFERENTIELLE pour la course.
- 2 motoreducteurs (le meme modele "mcc" qu'aujourd'hui), chacun avec UNE roue (roue arriere motrice), essieu a x 150, voie 150 mm ;
  la 2e voie du L298N (ENB / IN3 / IN4) commande le 2e moteur. Plus de servo de direction, plus de bloc pivotant.
- a l'avant : une BILLE PORTEUSE (bille acier 16 mm) sous une colonne vissee sous le bati ;
- les 4 capteurs EN FLECHE (meme disposition qu'aujourd'hui) sur une plaque FIXE au bas de la colonne, ~150 mm devant l'essieu.
Simulation (03_Reseau_de_neurones/esp32/simulation_ligne/sim_differentiel.py) : a 1 m/s, 9,0 s par tour, ecart max 20 mm ;
la direction actuelle a 1 m/s : 1 sortie sur 8, 13,1 s, ecart max 54 mm. Jusqu'a 0,6 m/s, l'actuelle suffit.
Repere : celui de l'assemblage Assemblage1v2 APRES le rabaissement du 22/11 (sol y 449,6, dessous du bati y 524,69).
Pieces de cette proposition dessinees dans ce repere : on les insere a l'origine. Usage : ~/venv_cao/bin/python proposition_differentiel.py <dossier>
"""
import os
import sys
import cadquery as cq
import support_capteurs_fleche as SC

Y_SOL, Y_BATI = 449.6, 524.69
Z_M = 512.72
X_ESSIEU, VOIE = 150.0, 150.0
X_COL, R_BILLE = 36.0, 8.0


def boite(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_y(x, z, y0, y1, r):
    return cq.Workplane('XZ').circle(r).extrude(-(y1 - y0)).translate((x, y0, z))


def colonne_capteurs():
    """Colonne de la bille porteuse + plaque des 4 capteurs (la meme que sur le bloc de direction, decalee de +7 mm en y)."""
    yb = Y_SOL + R_BILLE                                    # centre de la bille
    col = boite(X_COL - 8, X_COL + 8, yb + 3, Y_BATI, Z_M - 10, Z_M + 10)
    col = col.union(boite(X_COL - 14, X_COL + 14, Y_BATI - 4, Y_BATI, Z_M - 16, Z_M + 16))          # semelle sous le bati
    col = col.union(cq.Workplane('XZ').circle(R_BILLE + 2.5).extrude(-(R_BILLE + 3)).translate((X_COL, yb, Z_M)))   # coupelle
    col = col.cut(cq.Workplane('XY').sphere(R_BILLE + 0.3).translate((X_COL, yb, Z_M)))
    for dx, dz in ((-10, -12), (10, -12), (-10, 12), (10, 12)):
        col = col.cut(cyl_y(X_COL + dx, Z_M + dz, Y_BATI - 10, Y_BATI + 1, 1.7))                       # 4 vis M3 dans le bati
    SC.X_FACE = X_COL - 7.5                                  # la plaque verticale du support se soude a la colonne
    plaque = cq.Workplane().add(SC.support()).translate((0, 7.0, 0))
    return col.union(plaque).val().clean()


def bille():
    return cq.Workplane('XY').sphere(R_BILLE).translate((X_COL, Y_SOL + R_BILLE, Z_M)).val()


def support_moteur(sg):
    """Etrier qui tient un moteur sous le bati (dessus du moteur a y 491), sg = -1 : cote z faible, +1 : cote z fort."""
    zc = Z_M + sg * (VOIE / 2 - 19.4)                      # centre du moteur (roue a 19,4 mm du centre, cote exterieur)
    # derriere la roue (x > essieu + 31) : l'etrier tient le bout du reducteur, levre sous le moteur, joues de chaque cote
    e = boite(X_ESSIEU + 33, X_ESSIEU + 54, 466.0, Y_BATI, zc - 22, zc + 22)
    e = e.cut(boite(X_ESSIEU + 31, X_ESSIEU + 56, 468.6, 491.0, zc - 18.6, zc + 18.6))
    e = e.cut(boite(X_ESSIEU + 31, X_ESSIEU + 56, 491.0 - 0.1, Y_BATI - 4, zc - 14, zc + 14))
    for dx in (37, 50):
        for dz in (-18, 18):
            e = e.cut(cyl_y(X_ESSIEU + dx, zc + dz, Y_BATI - 6, Y_BATI + 1, 1.7))
    return e.val().clean()


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for nom, s in (('colonne bille et capteurs', colonne_capteurs()), ('bille 16', bille()),
                   ('etrier moteur gauche', support_moteur(1)), ('etrier moteur droit', support_moteur(-1))):
        b = s.BoundingBox()
        print('%-26s valide %s solides %d  %.1f g  x %.1f..%.1f y %.1f..%.1f z %.1f..%.1f' % (nom, s.isValid(), len(s.Solids()), s.Volume() * 1.24e-3 * 0.8,
              b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.step'))
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
