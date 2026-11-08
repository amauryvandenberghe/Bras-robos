"""Essai de prise seul dans Gazebo : reglette devant le robot, l'expert prend la balle 1 et la souleve de 8 cm.
Affiche la hauteur de la balle, l'angle reel des machoires et la consigne, pour regler la prise.
Usage (Gazebo bras_50cm lance) : python3 essai_prise.py [n]"""
import sys

import numpy as np
import rclpy

from cinematique import PINCE, SERVO_CONTACT, SERVO_OUVERT
from expert import Expert
from pilote_gazebo import Pilote
from politique import PERIODE, securite, servo_suivant
from scene_gazebo import MONDE, deplacer, ranger, recreer
from zone import TYPES, Z_BALLE

rclpy.init()
p = Pilote(MONDE)
machoires = {}


def lire_machoires(msg):
    pos = dict(zip(msg.name, msg.position))
    machoires['val'] = [pos.get(n, np.nan) for n in PINCE]


from sensor_msgs.msg import JointState  # noqa: E402
p.create_subscription(JointState, '/joint_states', lire_machoires, 10)
ex = Expert(p.cine)
n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
for essai in range(n):
    ranger()
    p.aller(ex.q_repos, SERVO_OUVERT, 3.0)
    angle = [0.0, 0.5, -0.6][essai % 3]
    u = np.array([np.cos(angle), np.sin(angle)])
    centre = 0.27 * u
    deplacer('reglette', centre[0], centre[1], 0.0, angle)
    for i, k in enumerate((1, 2, 3)):
        b = centre + (k - 2) * 0.05 * u
        recreer(f'balle_{i + 1}', b[0], b[1], Z_BALLE)
    p.attendre_sim(1.5)
    balle0 = p.suivi.positions['balle_2'].copy()
    a1 = angle - 0.9 if angle > -0.3 else angle + 0.9
    scene = dict(receptacles=[np.array([0.27 * np.cos(a1), 0.27 * np.sin(a1)]), np.array([5.0, -5.0]), np.array([5.0, 0.0])],
                 types=list(TYPES), modeles=[1, 2, 3],
                 balles=[balle0, p.suivi.positions['balle_1'], p.suivi.positions['balle_3']])
    if not ex.preparer(scene, 0, ex.q_repos):
        print('plan impossible', ex.raison)
        continue
    q, servo = ex.q_repos.copy(), SERVO_OUVERT
    prochain, leve_a = p.t_sim, None
    for pas in range(400):
        balle = p.suivi.positions['balle_2'].copy()
        outil = p.cine.outil(q)[0]
        tenue = servo <= SERVO_CONTACT and (balle[2] > Z_BALLE + 0.008 or np.linalg.norm(balle - outil) < 0.008)
        a = ex.action(q, servo, balle, tenue)
        q, pince = securite(p.cine, q, a, servo)
        servo = servo_suivant(servo, pince)
        p.commander(q, servo, PERIODE)
        prochain += PERIODE
        p.attendre_jusqua(prochain)
        if pas % 25 == 0 or (servo < 0.2 and pas % 3 == 0):
            print(f'  pas {pas:3d} pince {np.round(outil, 3)} consigne servo {servo:+.3f} machoires reelles '
                  f'{np.round(machoires.get("val", []), 3)} balle {np.round(p.suivi.positions["balle_2"], 4)}', flush=True)
        if outil[2] > Z_BALLE + 0.06 and servo < 0:
            leve_a = pas
            break
    b = p.suivi.positions['balle_2']
    print(f'essai {essai + 1}: balle a z = {b[2]:.3f} -> {"TENUE" if b[2] > Z_BALLE + 0.03 else "RATEE"}', flush=True)
p.suivi.arreter()
rclpy.try_shutdown()
