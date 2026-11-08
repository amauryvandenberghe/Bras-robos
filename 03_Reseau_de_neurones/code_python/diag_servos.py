"""Diagnostic dans Gazebo : on commande des postures et on releve ou les servos arrivent vraiment.
Usage (Gazebo bras_50cm lance) : python3 diag_servos.py"""
import numpy as np
import rclpy

from cinematique import SERVO_OUVERT
from expert import Expert
from pilote_gazebo import Pilote
from scene_gazebo import MONDE

rclpy.init()
p = Pilote(MONDE)
ex = Expert(p.cine)
# postures les plus dures pour les servos (couple de gravite maxi, voir bras_50cm/couples_urdf.py)
postures = {
    'repos': ex.q_repos,
    'avant-bras horizontal': np.array([0, 0.3, 1.27, 0.0]),
    'loin au sol': np.array([0, 1.4, 1.0, 0.6]),
    'au-dessus du grand': np.array([0, 0.0, 0.7, 1.0]),
    'bras horizontal': np.array([0, 1.5, 0.1, 0.0]),
}
print('mesure au depart', np.round(p.q_mesure, 3))
for nom, q in postures.items():
    p.commander(q, SERVO_OUVERT, 3.0)
    p.attendre_sim(5.0)
    print(f'{nom:22s} consigne {np.round(q, 2)}  mesure {np.round(p.q_mesure, 3)}  '
          f'ecart {np.round(np.degrees(p.q_mesure - q), 1)} deg', flush=True)
p.suivi.arreter()
rclpy.try_shutdown()
