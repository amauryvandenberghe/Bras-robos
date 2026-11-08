"""Position home dans Gazebo : envoie le bras dans la posture de repos (replie au-dessus du robot), pince ouverte.
Gazebo doit etre lance (terminal 1). Usage : python3 aller_repos_gazebo.py
Sur le vrai bras, c'est l'ordre serie "R" (fonction aller_repos() du programme ESP32)."""
import numpy as np
import rclpy

from cinematique import Bras, SERVO_OUVERT
from expert import Expert
from pilote_gazebo import Pilote
from scene_gazebo import MONDE


def main():
    rclpy.init()
    pilote = Pilote(MONDE)
    q_repos = Expert(Bras()).q_repos
    ecart = pilote.aller(q_repos, SERVO_OUVERT, 4.0)
    print('posture de repos (deg) :', np.round(np.degrees(q_repos), 1), f'; ecart maxi des servos : {np.degrees(ecart):.1f} deg')
    pilote.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
