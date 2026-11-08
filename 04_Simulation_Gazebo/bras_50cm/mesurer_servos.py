"""Mesure les modeles de servos fournis (STEP exportes depuis SolidWorks) : solides, puis coupes horizontales
(selon z) pour reperer le fond du boitier, les pattes et la sortie de l'axe."""
import sys
import numpy as np
import cadquery as cq

for chemin in sys.argv[1:]:
    forme = cq.importers.importStep(chemin)
    solides = forme.solids().vals()
    print(f'=== {chemin.split("/")[-1]} : {len(solides)} solides')
    for k, s in enumerate(solides):
        b = s.BoundingBox()
        c = s.Center()
        print(f'  solide {k}: x[{b.xmin:7.2f} {b.xmax:7.2f}] y[{b.ymin:7.2f} {b.ymax:7.2f}] z[{b.zmin:7.2f} {b.zmax:7.2f}] '
              f'vol {s.Volume():8.0f} centre ({c.x:.2f} {c.y:.2f} {c.z:.2f})')
    tout = cq.Compound.makeCompound(solides)
    b = tout.BoundingBox()
    prec = None
    for t in np.arange(b.zmin + 0.1, b.zmax, 0.4):
        plan = cq.Solid.makeBox(400, 400, 0.05, cq.Vector(-200, -200, t))
        inter = tout.intersect(plan)
        if inter.Volume() < 1e-6:
            cle = 'vide'
        else:
            bb = inter.BoundingBox()
            cle = f'x[{bb.xmin:6.1f} {bb.xmax:6.1f}] y[{bb.ymin:6.1f} {bb.ymax:6.1f}]'
        aire = inter.Volume() / 0.05
        if cle != prec:
            print(f'    z={t:6.2f}: {cle} aire {aire:6.0f}')
        prec = cle
