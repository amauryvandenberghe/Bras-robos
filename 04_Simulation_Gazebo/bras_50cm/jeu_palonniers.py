"""Pour chaque servo : position le long de son axe (dans le repere de la piece entrainee) du bout de la cannelure, de la
face de sortie du servo et de la face de la piece entrainee qui lui fait face. A lancer avec ~/venv_cao/bin/python."""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/mon_bras_robot/ia_50cm'))
import cao_bras as C  # noqa: E402
from cinematique import Bras  # noqa: E402

bras = Bras()
J = {v['nom']: v for v in bras.joints.values()}


def T_joint(nom):
    return J[nom]['T']


def maillage(forme):
    v, f = forme.tessellate(0.1, 0.3)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in v]), np.array(f))


# (servo, joint entraine, piece entrainee (cao_bras) ou None = plaque de la pince, sortie du modele, bout de cannelure)
CAS = [('servo_lacet', 'joint_base_bras_1', 'plateau', C.MG_DESSUS, C.MG_REF_Z - 2.3),
       ('servo_epaule', 'joint_base_bras_2', 'bras', C.MG_DESSUS, C.MG_REF_Z - 2.3),
       ('servo_coude', 'joint_base_bras_3', 'avant_bras', C.SG_DESSUS, C.SG_REF_Z),
       ('servo_poignet', 'joint_base_bras_4', None, C.SG_DESSUS, C.SG_REF_Z)]
for servo, joint, piece, z_sortie, z_cannelure in CAS:
    modele, lien, T = C.SERVOS[servo]
    # repere du lien porteur -> repere de la piece entrainee (q = 0) : inverse de la transformation du joint
    Tj = T_joint(joint).copy()
    Tj[:3, 3] *= 1000
    Tinv = np.linalg.inv(Tj)
    M = Tinv @ T                                     # modele du servo -> repere de la piece entrainee
    ax, ay = (C.MG_AXE_XY if modele == 'mg996r' else (0, 0))
    p_c = M @ [ax, ay, z_cannelure, 1]
    p_s = M @ [ax, ay, z_sortie, 1]
    d = M[:3, 2]                                     # direction de sortie (z du modele)
    print(f'{servo} dans le repere de "{piece or "pince (bras quatre)"}" : sortie {np.round(d, 3).tolist()}, '
          f'face de sortie en {np.round(p_s[:3], 2).tolist()}, haut de l\'arbre / cannelure en {np.round(p_c[:3], 2).tolist()}')
    if piece:
        m = maillage(C.PIECES[piece]())
    else:
        m = sorted(trimesh.load(os.path.expanduser('~/ros2_ws/src/mon_bras_robot/meshes/bras_quatre.stl')).split(),
                   key=lambda c: -c.area)[2]
        m.apply_scale(1000)
    # premiere matiere de la piece rencontree en partant de la face de sortie le long de l'axe, a differents rayons
    u = np.cross(d, [1, 0, 0] if abs(d[0]) < 0.9 else [0, 1, 0]); u /= np.linalg.norm(u)
    w = np.cross(d, u)
    for r in (0, 4, 8, 12, 17):
        vals = []
        for a in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            o = p_s[:3] + r * (np.cos(a) * u + np.sin(a) * w)
            loc, _, _ = m.ray.intersects_location([o - d * 0.01], [d])
            if len(loc):
                vals.append(min(np.dot(l - p_s[:3], d) for l in loc))
        print(f'   rayon {r:2d} mm : premiere matiere a {min(vals):.2f} mm de la face de sortie' if vals else
              f'   rayon {r:2d} mm : rien')
