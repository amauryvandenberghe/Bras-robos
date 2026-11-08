"""Un angle d'un balayage de verifier_logements.py, dans un programme a part (OpenCascade plante parfois apres une
longue suite d'operations). Usage : python balayage_un_angle.py coude|poignet <angle en rad>
Affiche "volume_pieces volume_broche" (mm3)."""
import os
import sys

import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
import cao_bras as C  # noqa: E402
from cinematique import Bras  # noqa: E402

joint, q = sys.argv[1], float(sys.argv[2])
nom = 'joint_base_bras_3' if joint == 'coude' else 'joint_base_bras_4'
j = [v for v in Bras().joints.values() if v['nom'] == nom][0]
M = j['T'].copy()
M[:3, 3] *= 1000
D = np.eye(4)
D[:3, :3] = Rotation.from_rotvec(np.array(j['axe']) * q).as_matrix()
M = M @ D
m = cq.Matrix([list(r) for r in M[:3]])


def piece(n):
    return cq.importers.importStep(os.path.join(C.SORTIE, n + '.step')).val()


def commun(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return float('nan')


broche = {p: max(C.palonnier_place(p).Solids(), key=lambda s: s.Volume()) for p in ('broche_coude', 'broche_poignet')}
if joint == 'coude':
    fixe = [piece('bras'), piece('bride_coude'), C.enveloppe_servo('servo_coude')]
    mobile = [piece('avant_bras'), piece('bride_poignet'), C.enveloppe_servo('servo_poignet')]
    v = sum(commun(a, g.transformGeometry(m)) for a in fixe for g in mobile)
    # la broche tourne avec l'avant-bras autour de la cannelure du servo (contact voulu) : contre le bras seulement
    vb = sum(commun(a, broche['broche_coude'].transformGeometry(m)) for a in fixe[:2])
else:
    fixe = [piece('avant_bras'), piece('bride_poignet'), C.servo_place('servo_poignet')]
    v = sum(commun(a, piece('corps_pince').transformGeometry(m)) for a in fixe)
    vb = sum(commun(a, broche['broche_poignet'].transformGeometry(m)) for a in fixe[:2])
print(f'{v:.4f} {vb:.4f}')
