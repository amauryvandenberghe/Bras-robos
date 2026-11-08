"""La pince d'origine a-t-elle une empreinte pour la broche du SG90 de poignet ? Faces planes en creux et cylindres.
A lancer avec ~/venv_cao/bin/python."""
import cadquery as cq
import numpy as np

EXP = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/'
p = cq.importers.importStep(EXP + 'pince.step').val()
bb = p.BoundingBox()
print(f'pince : {len(p.Solids())} solide(s), x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} z {bb.zmin:.2f}..{bb.zmax:.2f}')
vus = set()
for f in p.Faces():
    if f.geomType() == 'CYLINDER':
        cy = f._geomAdaptor().Cylinder()
        l, d = cy.Location(), cy.Axis().Direction()
        fb = f.BoundingBox()
        cle = (round(cy.Radius(), 2), round(l.X(), 1), round(l.Y(), 1), round(l.Z(), 1))
        if cle in vus:
            continue
        vus.add(cle)
        print(f'  cyl r={cy.Radius():.2f} axe ({d.X():.2f},{d.Y():.2f},{d.Z():.2f}) passe par ({l.X():.2f},{l.Y():.2f},{l.Z():.2f}) '
              f'boite x {fb.xmin:.1f}..{fb.xmax:.1f} y {fb.ymin:.1f}..{fb.ymax:.1f} z {fb.zmin:.1f}..{fb.zmax:.1f}')
for f in p.Faces():
    if f.geomType() == 'PLANE':
        n, c, fb = f.normalAt(), f.Center(), f.BoundingBox()
        if f.Area() > 15:
            print(f'  plan n=({n.x:.2f},{n.y:.2f},{n.z:.2f}) centre ({c.x:.1f},{c.y:.1f},{c.z:.1f}) aire {f.Area():.0f} '
                  f'x {fb.xmin:.1f}..{fb.xmax:.1f} y {fb.ymin:.1f}..{fb.ymax:.1f} z {fb.zmin:.1f}..{fb.zmax:.1f}')
