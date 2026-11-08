"""Mesure fine (pas 0,05 mm) des faces des pattes et du fond des modeles de servos, et ou se touchent servo et pieces."""
import os
import sys
import numpy as np
import cadquery as cq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402


def largeur(forme, z):
    inter = forme.intersect(cq.Solid.makeBox(400, 400, 0.01, cq.Vector(-200, -200, z)))
    if inter.Volume() < 1e-7:
        return None
    b = inter.BoundingBox()
    return b.xmax - b.xmin


for modele, zones in (('mg996r', [(38.5, 44.5), (10.5, 13.0)]), ('mg90s', [(-15.5, -10.5), (-32.5, -30.5), (-8, -2.5)])):
    s = C.modele_servo(modele)
    print('==', modele)
    for z0, z1 in zones:
        prec = None
        for z in np.arange(z0, z1, 0.05):
            w = largeur(s, z)
            w = None if w is None else round(w, 1)
            if w != prec:
                print(f'  z={z:7.2f} largeur x = {w}')
            prec = w

for n, p in (('servo_lacet', 'socle'), ('servo_coude', 'bride_coude')):
    piece = cq.importers.importStep(os.path.join(C.SORTIE, p + '.step')).val()
    inter = C.servo_place(n).intersect(piece)
    for k, so in enumerate(inter.Solids()):
        b = so.BoundingBox()
        print(f'{n}/{p} contact {k}: x[{b.xmin:.2f} {b.xmax:.2f}] y[{b.ymin:.2f} {b.ymax:.2f}] z[{b.zmin:.2f} {b.zmax:.2f}] {so.Volume():.2f} mm3')
