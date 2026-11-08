"""Detail des palonniers : corps de la "broche" (SG90), trous et epaisseurs de arm01 (MG996R).
A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import cadquery as cq

EXP = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cao_bras as C  # noqa: E402


def cylindres(forme, nom):
    vus = set()
    for f in forme.Faces():
        if f.geomType() == 'CYLINDER':
            cy = f._geomAdaptor().Cylinder()
            l, d = cy.Location(), cy.Axis().Direction()
            bb = f.BoundingBox()
            cle = (round(cy.Radius(), 2), round(l.X(), 1), round(l.Y(), 1))
            if cle in vus:
                continue
            vus.add(cle)
            print(f'   {nom} cyl r={cy.Radius():.2f} axe ({d.X():.1f},{d.Y():.1f},{d.Z():.1f}) centre ({l.X():.2f}, {l.Y():.2f}) '
                  f'z {bb.zmin:.2f}..{bb.zmax:.2f}')


b = cq.importers.importStep(EXP + 'broche.step').val()
for i, s in enumerate(b.Solids()):
    bb = s.BoundingBox()
    print(f'broche corps {i} : volume {s.Volume():.1f} mm3, x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} '
          f'z {bb.zmin:.2f}..{bb.zmax:.2f}')
    cylindres(s, f'  corps {i}')
a = cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, 'mg996r_origine.step')).val().Solids()[6]
print('arm01 :')
cylindres(a, 'arm01')
mg = cq.importers.importStep(os.path.join(C.DOSSIER_SERVOS, 'mg996r_origine.step')).val().Solids()[5]
bb = mg.BoundingBox()
print(f'arbre MG996R (solide 5) : x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} z {bb.zmin:.2f}..{bb.zmax:.2f}')
