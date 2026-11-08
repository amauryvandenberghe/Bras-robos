"""Le maillage du corps de pince (meshes/bras_quatre.stl, repere du lien "bras quatre") et pince.step (piece
SolidWorks d'origine) sont-ils dans le meme repere ? Ou est l'axe du poignet ? A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import cadquery as cq
import numpy as np
import trimesh

sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/mon_bras_robot/ia_50cm'))
from cinematique import Bras  # noqa: E402

m = trimesh.load(os.path.expanduser('~/ros2_ws/src/mon_bras_robot/meshes/bras_quatre.stl'))
print('bras_quatre.stl (m) :', np.round(m.bounds, 4).tolist(), ' corps :', len(m.split()))
for i, c in enumerate(sorted(m.split(), key=lambda c: -c.area)[:6]):
    print(f'  morceau {i} : {len(c.faces)} facettes, boite mm {np.round(c.bounds * 1000, 1).tolist()}')
p = cq.importers.importStep('/mnt/c/Users/amaur/AppData/Local/Temp/bras50/sw_export/pince.step').val()
for i, s in enumerate(p.Solids()):
    bb = s.BoundingBox()
    print(f'pince.step solide {i} (mm) : x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}')
b = Bras()
j4 = [v for v in b.joints.values() if v['nom'] == 'joint_base_bras_4'][0]
print('joint 4 : axe (repere enfant) =', j4['axe'], ' origine dans le parent (mm) =', np.round(j4['T'][:3, 3] * 1000, 2).tolist())
for v in b.joints.values():
    if v['parent'] == 'bras quatre':
        print(f"  enfant de 'bras quatre' : {v['nom']} en {np.round(v['T'][:3, 3] * 1000, 2).tolist()} mm, axe {v['axe']}")
