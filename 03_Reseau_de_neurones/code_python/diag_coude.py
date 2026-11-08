"""Diagnostic : jusqu'ou le coude monte-t-il dans Gazebo (couple de gravite croissant avec l'angle) ?
Usage (Gazebo lance) : python3 diag_coude.py"""
import numpy as np
import rclpy

from cinematique import SERVO_OUVERT
from pilote_gazebo import Pilote
from scene_gazebo import MONDE

rclpy.init()
p = Pilote(MONDE)
for q3 in (0.6, 0.9, 1.2, 1.57):
    q = np.array([0, 0, q3, 0])
    p.commander(q, SERVO_OUVERT, 2.0)
    p.attendre_sim(4.0)
    print(f'coude consigne {q3:.2f} -> mesure {p.q_mesure[2]:.3f} (ecart {np.degrees(p.q_mesure[2] - q3):5.1f} deg)', flush=True)
p.suivi.arreter()
rclpy.try_shutdown()
