"""Couple de gravite sur chaque servo, calcule avec les masses et centres de gravite de l'URDF (ce que Gazebo simule).
Usage : python3 couples_urdf.py [q1 q2 q3 q4]"""
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ia_50cm'))
from cinematique import Bras, BRAS, URDF, _rpy  # noqa: E402

G = 9.81
b = Bras()
racine = ET.parse(URDF).getroot()
inertiels = {}
for l in racine.findall('link'):
    i = l.find('inertial')
    if i is not None:
        o = i.find('origin')
        inertiels[l.get('name')] = (float(i.find('mass').get('value')), np.array(list(map(float, o.get('xyz').split()))))


def couples(q, balle=0.002):
    qd = dict(zip(BRAS, q))
    res = []
    for j_nom in BRAS:
        enfant = next(k for k, v in b.joints.items() if v['nom'] == j_nom)
        T = b.pose(enfant, qd)
        axe = T[:3, :3] @ b.joints[enfant]['axe']
        o = T[:3, 3]
        c = 0.0
        for lien, (m, cdg) in inertiels.items():
            # lien en aval du joint ?
            n, aval = lien, False
            while n in b.joints:
                if n == enfant:
                    aval = True
                    break
                n = b.joints[n]['parent']
            if not aval:
                continue
            Tl = b.pose(lien, qd)
            p = Tl[:3, :3] @ cdg + Tl[:3, 3]
            c += np.cross(p - o, [0, 0, -m * G]) @ axe
        p = b.outil(q)[0]
        c += np.cross(p - o, [0, 0, -balle * G]) @ axe
        res.append(c)
    return np.array(res)


if __name__ == '__main__':
    print({k: round(v[0] * 1000, 1) for k, v in inertiels.items()}, 'g')
    poses = [np.array(list(map(float, sys.argv[1:5])))] if len(sys.argv) > 4 else \
        [np.array(p) for p in ([0, 0, 1.0, 0], [0, 0, 1.57, 0], [0, 0, 0.914, 0], [0, 1.4, 1.0, 0.6], [0, 0.8, 1.2, 0.8])]
    for q in poses:
        c = couples(q)
        print(f'q {q} : couples {np.round(c, 4)} N.m = {np.round(np.abs(c) / 0.0981, 2)} kg.cm')
