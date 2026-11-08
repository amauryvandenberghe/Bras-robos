"""Le reseau de neurones pilote le bras dans Gazebo (vraie physique).

  python3 evaluer_gazebo.py 20        : 20 essais, balle (avec son socle) et bac places au hasard a chaque fois
  python3 evaluer_gazebo.py --scene   : un seul essai sur la scene telle qu'elle est (balle et bac ou ils sont)
Options : --modele chemin.pt (defaut : modeles/politique_bras.pt), --graine N

Seule la position de la balle et du bac est lue dans Gazebo (plus tard : la camera).
Les decisions viennent uniquement du reseau, filtrees par la couche de securite.
"""
import argparse
import json
import os
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cinematique import Bras, BRAS, PINCE, SERVO_CONTACT, SERVO_OUVERT, train_engrenages
from competences import MONDE, pose_gazebo, signal_pince
from expert import Expert
from politique import PERIODE, Politique, observation, securite, servo_suivant
from zone import Z_BALLE, dans_bac, tirer

ICI = os.path.dirname(os.path.abspath(__file__))
PAS_MAX = 400           # 40 s par essai (comme dans le simulateur rapide)


class SuiviBalle(threading.Thread):
    """Lit en continu la position des objets mobiles publiee par Gazebo (flux dynamic_pose/info).
    `pos` : position de l'objet suivi (`nom`, modifiable) ; `positions` : tous les objets mobiles."""
    def __init__(self, nom='balle_rouge', monde='scene_balle'):
        super().__init__(daemon=True)
        self.nom, self.positions = nom, {}
        self.proc = subprocess.Popen(['ign', 'topic', '-e', '-t', f'/world/{monde}/dynamic_pose/info'],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)

    @property
    def pos(self):
        return self.positions.get(self.nom)

    def run(self):
        profondeur, bloc, nom, xyz = 0, None, None, [0.0, 0.0, 0.0]
        for ligne in self.proc.stdout:
            l = ligne.strip()
            if l.endswith('{'):
                profondeur += 1
                if profondeur == 1:
                    nom, xyz = None, [0.0, 0.0, 0.0]      # les champs a zero ne sont pas ecrits
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


def placer(nom, x, y, z):
    subprocess.run(['ign', 'service', '-s', '/world/scene_balle/set_pose', '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                    '--req', f'name: "{nom}", position: {{x: {x}, y: {y}, z: {z}}}'], capture_output=True, timeout=20)


def _sdf_balle():
    """Ecrit la balle telle que definie dans le monde (seule source de verite) dans un fichier SDF a part."""
    racine = ET.parse(MONDE).getroot()
    modele = next(m for m in racine.iter('model') if m.get('name') == 'balle_rouge')
    chemin = os.path.join(ICI, 'modeles', 'balle_rouge.sdf')
    with open(chemin, 'w') as f:
        f.write('<?xml version="1.0" ?>\n<sdf version="1.6">\n' + ET.tostring(modele, encoding='unicode') + '\n</sdf>\n')
    return chemin


def recreer_balle(x, y, z):
    """Supprime la balle et en cree une neuve, immobile, a (x, y, z).
    (set_pose deplace la balle mais lui laisse sa vitesse : une balle ejectee repartirait aussitot.)"""
    service = ['ign', 'service', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000']
    subprocess.run(service + ['-s', '/world/scene_balle/remove', '--reqtype', 'ignition.msgs.Entity',
                              '--req', 'name: "balle_rouge", type: MODEL'], capture_output=True, timeout=20)
    time.sleep(0.5)
    subprocess.run(service + ['-s', '/world/scene_balle/create', '--reqtype', 'ignition.msgs.EntityFactory',
                              '--req', f'sdf_filename: "{_sdf_balle()}", name: "balle_rouge", '
                                       f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'],
                   capture_output=True, timeout=20)


class Pilote(Node):
    def __init__(self, politique, suivi=None):
        super().__init__('ia_reseau')
        self.cine, self.politique = Bras(), politique
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.q_mesure, self.t_sim = None, 0.0
        self.create_subscription(JointState, '/joint_states', self._etat, 10)
        self.suivi = suivi or SuiviBalle()
        self.suivi.start()
        fin = time.time() + 15
        while (self.q_mesure is None or self.suivi.pos is None) and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _etat(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in BRAS):
            self.q_mesure = np.array([pos[n] for n in BRAS])
        self.pince_mesure = [pos.get(n, float('nan')) for n in PINCE]
        self.t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # horloge de la simulation

    def _attendre(self, duree):
        fin = time.time() + duree
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _attendre_sim(self, t_cible):
        """Attend que la SIMULATION atteigne t_cible : si Gazebo tourne moins vite que le temps reel
        (fenetre 3D, PC charge), le reseau decide quand meme toutes les 0,1 s de temps simule."""
        limite = time.time() + 2.0
        while self.t_sim < t_cible and time.time() < limite:
            rclpy.spin_once(self, timeout_sec=0.005)
        return self.t_sim >= t_cible          # False : la simulation n'avance plus (bloquee)

    def commander(self, q, servo, duree):
        msg = JointTrajectory(joint_names=BRAS + PINCE)
        p = JointTrajectoryPoint(positions=[float(v) for v in q] + train_engrenages(servo))
        p.time_from_start.sec, p.time_from_start.nanosec = int(duree), int((duree % 1) * 1e9)
        msg.points = [p]
        self.pub.publish(msg)

    def repos(self):
        signal_pince('liberer')
        self.commander(self.q_mesure, SERVO_OUVERT, 1.0)
        self._attendre(1.2)
        self.commander(np.zeros(4), SERVO_OUVERT, 3.0)
        self._attendre(3.5)

    def essai(self, bac, trace=False, expert=None, collecte=None, part_expert=0.0, rng=None):
        """Le reseau pilote jusqu'a la reussite, un echec ou 40 s.
        Avec `expert` seul : c'est l'expert qui pilote (pour verifier qu'il reussit avec la vraie physique).
        Avec `collecte` (une liste) : le reseau pilote (et l'expert une part `part_expert` des pas) ;
        a chaque pas, l'expert note ce qu'il aurait fait -> corrections DAgger issues de la vraie physique."""
        q, servo = np.zeros(4), SERVO_OUVERT
        depart, levee, pose_depuis = self.suivi.pos.copy(), False, None
        prochain = self.t_sim
        for pas in range(1, PAS_MAX + 1):
            balle = self.suivi.pos.copy()
            outil = self.cine.outil(q)[0]
            obs = observation(q, servo, balle, bac, outil)
            a_expert = None
            if expert is not None:
                # balle tenue : pince fermee et balle soulevee du sol (ou encore tout pres de la pince)
                tenue = servo <= SERVO_CONTACT and (balle[2] > Z_BALLE + 0.008 or np.linalg.norm(balle - outil) < 0.008)
                a_expert = expert.action(q, servo, balle, tenue)
            if collecte is not None:
                collecte.append((obs, a_expert.astype(np.float32)))
                action = a_expert if rng.random() < part_expert else self.politique(obs)
            elif expert is not None:
                action = a_expert
            else:
                action = self.politique(obs)
            q, pince = securite(self.cine, q, action, servo)
            servo = servo_suivant(servo, pince)
            self.commander(q, servo, PERIODE)
            prochain += PERIODE
            self._attendre_sim(prochain)
            balle = self.suivi.pos
            if trace:
                reel = self.cine.outil(self.q_mesure)[0]
                print(f'  pas {pas:3d} pince commandee {np.round(outil, 3)} reelle {np.round(reel, 3)} '
                      f'servo {servo:+.2f} machoires reelles {np.round(self.pince_mesure, 3)} '
                      f'balle {np.round(balle, 3)} ecart {np.linalg.norm(reel - balle) * 1000:5.1f} mm', flush=True)
            if balle[2] > Z_BALLE + 0.025:
                levee, pose_depuis = True, None
            elif levee:                                  # la balle est retombee quelque part
                pose_depuis = pose_depuis or time.time()
                if time.time() - pose_depuis > 1.0:
                    return ('reussite' if dans_bac(balle, bac) else 'tombee'), pas
            elif balle[2] < Z_BALLE + 0.003 and np.linalg.norm(balle[:2] - depart[:2]) > 0.03:
                # poussee = deplacee de plus de 3 cm en restant au sol (une balle tenue qui monte en biais
                # se deplace aussi a l'horizontale : ce n'est pas un echec)
                return 'poussee', pas
        return ('reussite' if dans_bac(self.suivi.pos, bac) else 'temps'), PAS_MAX


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('n', nargs='?', type=int, default=20)
    ap.add_argument('--scene', action='store_true')
    ap.add_argument('--modele', default=os.path.join(ICI, 'modeles', 'politique_bras.pt'))
    ap.add_argument('--graine', type=int, default=20_000_000)
    ap.add_argument('--trace', action='store_true', help='affiche la pince et la balle a chaque pas')
    ap.add_argument('--expert', action='store_true', help="fait piloter l'expert au lieu du reseau (verification)")
    ap.add_argument('--collecter', action='store_true',
                    help="le reseau pilote et l'expert note ses corrections -> modeles/donnees_gazebo.npz")
    ap.add_argument('--part-expert', type=float, default=0.3, help='avec --collecter : part des pas pilotes par l expert')
    args = ap.parse_args()

    rclpy.init()
    pilote = Pilote(None if args.expert else Politique(args.modele))
    collecte = [] if args.collecter else None
    expert = Expert(pilote.cine)          # sert seulement a ecarter les tirages irrealisables
    rng = np.random.default_rng(args.graine)
    bilans = []
    try:
        for k in range(1 if args.scene else args.n):
            pilote.repos()
            if args.scene:
                balle, bac = pose_gazebo('balle_rouge'), pose_gazebo('bac')[:2]
            else:
                while True:
                    balle, bac = tirer(rng)
                    if expert.preparer(balle, bac):
                        break
                placer('bac', bac[0], bac[1], 0)
                placer('socle_balle', balle[0], balle[1], 0)
                recreer_balle(balle[0], balle[1], 0.0130)
                pilote._attendre(1.5)
                if np.linalg.norm(pilote.suivi.pos[:2] - balle[:2]) > 0.01:
                    print(f'essai {k + 1}: balle mal placee, essai ignore', flush=True)
                    continue
            t = time.time()
            if (args.expert or args.collecter) and not expert.preparer(balle, bac):
                print(f'essai {k + 1}: chemin impossible pour l expert')
                continue
            resultat, pas = pilote.essai(np.asarray(bac), trace=args.trace,
                                         expert=expert if (args.expert or args.collecter) else None,
                                         collecte=collecte, part_expert=args.part_expert, rng=rng)
            bilans.append(dict(balle=np.round(balle, 3).tolist(), bac=np.round(bac, 3).tolist(),
                               resultat=resultat, duree_s=round(time.time() - t, 1)))
            print(f'essai {k + 1}: {resultat:9s} en {pas / 10:.1f} s  balle {np.round(balle[:2], 3)}  bac {np.round(bac, 3)}',
                  flush=True)
        pilote.repos()
    finally:
        pilote.suivi.arreter()
        rclpy.shutdown()
    reussis = sum(b['resultat'] == 'reussite' for b in bilans)
    print(f'BILAN : {reussis}/{len(bilans)} balles dans le bac')
    if collecte:
        chemin = os.path.join(ICI, 'modeles', 'donnees_gazebo.npz')
        X, Y = np.array([o for o, _ in collecte]), np.array([a for _, a in collecte])
        if os.path.exists(chemin):                  # on ajoute aux corrections deja collectees
            d = np.load(chemin)
            X, Y = np.concatenate([d['X'], X]), np.concatenate([d['Y'], Y])
        np.savez_compressed(chemin, X=X, Y=Y)
        print(f'{len(collecte)} corrections ajoutees ({len(X)} au total) -> {chemin}')
    elif not args.scene:
        with open(os.path.join(ICI, 'modeles', 'eval_gazebo.json'), 'w') as f:
            json.dump(dict(modele=args.modele, reussite=reussis / len(bilans), essais=bilans), f, indent=2)


if __name__ == '__main__':
    main()
