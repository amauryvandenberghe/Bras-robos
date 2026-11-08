"""Liaison avec Gazebo : envoi des consignes aux servos, lecture des angles, de l'horloge simulee et des positions
des objets (flux dynamic_pose/info, plus tard : camera)."""
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cinematique import Bras, BRAS, PINCE, SERVO_OUVERT, train_engrenages


class SuiviObjets(threading.Thread):
    """Lit en continu la position des objets mobiles publiee par Gazebo. `positions[nom]` : (x, y, z)."""
    def __init__(self, monde):
        super().__init__(daemon=True)
        self.positions = {}
        self.proc = subprocess.Popen(['ign', 'topic', '-e', '-t', f'/world/{monde}/dynamic_pose/info'],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

    def run(self):
        profondeur, bloc, nom, xyz = 0, None, None, [0.0, 0.0, 0.0]
        for ligne in self.proc.stdout:
            l = ligne.strip()
            if l.endswith('{'):
                profondeur += 1
                if profondeur == 1:
                    nom, xyz = None, [0.0, 0.0, 0.0]
                elif profondeur == 2:
                    bloc = l.split()[0]
            elif l == '}':
                if profondeur == 1 and nom is not None:
                    self.positions[nom] = np.array(xyz)
                profondeur -= 1
            elif profondeur == 1 and l.startswith('name:'):
                nom = l.split('"')[1]
            elif profondeur == 2 and bloc == 'position' and l[:2] in ('x:', 'y:', 'z:'):
                xyz['xyz'.index(l[0])] = float(l.split()[1])

    def arreter(self):
        self.proc.terminate()


class Pilote(Node):
    def __init__(self, monde):
        super().__init__('ia_bras_50cm')
        self.cine = Bras()
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.q_mesure, self.t_sim, self.effort = None, 0.0, None
        self.create_subscription(JointState, '/joint_states', self._etat, 10)
        self.suivi = SuiviObjets(monde)
        self.suivi.start()
        fin = time.time() + 20
        while (self.q_mesure is None or not self.suivi.positions) and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _etat(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in BRAS):
            self.q_mesure = np.array([pos[n] for n in BRAS])
        self.t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def attendre_sim(self, duree, limite=60.0):
        """Attend `duree` secondes de temps SIMULE (Gazebo charge tourne moins vite que le temps reel)."""
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.0)
        t_cible, fin = self.t_sim + duree, time.time() + limite
        while self.t_sim < t_cible and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.01)
        return self.t_sim >= t_cible

    def attendre_jusqua(self, t_cible):
        limite = time.time() + 2.0
        while self.t_sim < t_cible and time.time() < limite:
            rclpy.spin_once(self, timeout_sec=0.005)
        return self.t_sim >= t_cible

    def commander(self, q, servo, duree):
        msg = JointTrajectory(joint_names=BRAS + PINCE)
        p = JointTrajectoryPoint(positions=[float(v) for v in q] + train_engrenages(servo))
        p.time_from_start.sec, p.time_from_start.nanosec = int(duree), int((duree % 1) * 1e9)
        msg.points = [p]
        self.pub.publish(msg)

    def aller(self, q, servo=SERVO_OUVERT, duree=3.0, precision=0.03):
        """Va a la posture q et attend d'y etre (ou 20 s)."""
        self.commander(q, servo, duree)
        self.attendre_sim(duree + 0.5)
        fin = time.time() + 20
        while np.abs(self.q_mesure - q).max() > precision and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.01)
        return np.abs(self.q_mesure - q).max()
