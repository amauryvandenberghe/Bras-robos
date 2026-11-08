"""Quelle paire de pieces fait planter l'intersection du balayage du coude ? Usage : python diag_balayage.py i j"""
import os
import sys

import cadquery as cq
import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
import cao_bras as C  # noqa: E402
from cinematique import Bras  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

i, j = int(sys.argv[1]), int(sys.argv[2])
pieces = {n: cq.importers.importStep(os.path.join(C.SORTIE, n + '.step')).val() for n in ('bras', 'bride_coude', 'avant_bras', 'bride_poignet')}
A = [pieces['bras'], pieces['bride_coude'], C.enveloppe_servo('servo_coude')]
B = [pieces['avant_bras'], pieces['bride_poignet'], C.enveloppe_servo('servo_poignet'),
     max(C.palonnier_place('broche_coude').Solids(), key=lambda s: s.Volume())]
b = Bras()
j3 = [v for v in b.joints.values() if v['nom'] == 'joint_base_bras_3'][0]
M = j3['T'].copy()
M[:3, 3] *= 1000
D = np.eye(4)
D[:3, :3] = Rotation.from_rotvec(np.array(j3['axe']) * b.bas[2]).as_matrix()
M = M @ D
m = cq.Matrix([list(r) for r in M[:3]])
print(i, j, A[i].isValid(), B[j].isValid(), flush=True)
print('volume commun', A[i].intersect(B[j].transformGeometry(m)).Volume(), flush=True)
