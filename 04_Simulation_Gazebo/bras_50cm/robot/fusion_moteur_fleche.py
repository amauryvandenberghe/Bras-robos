"""Piece unique (20/11/2026) : "piece moteur direction" d'origine + support des 4 capteurs EN FLECHE (27/11/2026), fusionnes pour etre imprimes d'un seul tenant.
La piece est sortie dans le repere de la PIECE moteur (celui de SolidWorks) : elle remplace le fichier d'origine sans toucher a
son placement dans l'assemblage. Transformation piece -> assemblage relevee dans Assemblage1v2 (x_asm = R^T v + t).
Usage : ~/venv_cao/bin/python fusion_moteur_capteurs.py <dossier contenant piece_moteur_direction.step>
"""
import os
import sys
import numpy as np
import cadquery as cq
import support_capteurs_fleche as SC

R = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])   # (rotation de 0,1 degre du releve negligee)
T = np.array([58.4, 458.6, 512.6])


def mat(M, t):
    return (M, t)


def appliquer(shape, mt):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    M, v = mt
    tr = gp_Trsf()
    tr.SetValues(float(M[0, 0]), float(M[0, 1]), float(M[0, 2]), float(v[0]), float(M[1, 0]), float(M[1, 1]), float(M[1, 2]), float(v[1]),
                 float(M[2, 0]), float(M[2, 1]), float(M[2, 2]), float(v[2]))
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, tr, True).Shape())


if __name__ == '__main__':
    d = sys.argv[1]
    moteur = cq.importers.importStep(os.path.join(d, 'piece_moteur_direction.step')).val()
    b = moteur.BoundingBox(); print('moteur (repere piece)', round(b.xmin, 1), round(b.xmax, 1), round(b.ymin, 1), round(b.ymax, 1), round(b.zmin, 1), round(b.zmax, 1))
    ma = appliquer(moteur, mat(R.T, T)); b = ma.BoundingBox()
    print('moteur (repere assemblage)', round(b.xmin, 1), round(b.xmax, 1), round(b.ymin, 1), round(b.ymax, 1), round(b.zmin, 1), round(b.zmax, 1))
    # support : plaque et poutre enfoncees de 0,5 mm dans le boitier pour souder les deux
    SC.X_FACE = 27.9
    sup = SC.support()
    inv = R
    sup_p = appliquer(sup, mat(inv, -inv @ T))
    tout = moteur.fuse(sup_p).clean()
    print('fusion valide', tout.isValid(), 'solides', len(tout.Solids()), 'volume %.1f cm3, masse %.1f g' % (tout.Volume() / 1000, tout.Volume() * 1.24e-3 * 0.8))
    w = cq.Workplane().add(tout)
    cq.exporters.export(w, os.path.join(d, 'piece_moteur_direction_capteurs.step'))
    cq.exporters.export(w, os.path.join(d, 'piece_moteur_direction_capteurs.stl'), tolerance=0.05, angularTolerance=0.2)
    wa = cq.Workplane().add(appliquer(tout, mat(R.T, T)))
    b = wa.val().BoundingBox(); print('fusion (repere assemblage)', round(b.xmin, 1), round(b.xmax, 1), round(b.ymin, 1), round(b.ymax, 1), round(b.zmin, 1), round(b.zmax, 1))
