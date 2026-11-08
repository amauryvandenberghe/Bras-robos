"""Cinematique du bras 50 cm, calculee directement depuis son URDF (urdf/bras_50cm.urdf, genere par bras_50cm/).

Geometrie :
  joint_base_bras_1 : lacet (MG996R vertical dans le socle), debattement +-90 degres
  joint_base_bras_2 : epaule (MG996R), a 0 le bras de 25 cm est vertical, > 0 il penche vers l'avant
  joint_base_bras_3 : coude (SG90), a 0 l'avant-bras prolonge le bras, > 0 il se replie vers le bas
  joint_base_bras_4 : poignet (SG90)
  joint_base_bras_44/45/46 : pignon et machoires de la pince (servo MG90S), inchanges
Le point de prise (OUTIL) et la direction d'approche sont dans le repere de la pince, la meme que sur le bras actuel.
"""
import os
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares

URDF = os.path.expanduser('~/ros2_ws/src/mon_bras_robot/urdf/bras_50cm.urdf')
BRAS = ['joint_base_bras_1', 'joint_base_bras_2', 'joint_base_bras_3', 'joint_base_bras_4']
PINCE = ['joint_base_bras_44', 'joint_base_bras_45', 'joint_base_bras_46']

RAPPORT_PIGNON_A = -1.0
RAPPORT_A_B = -1.0
SERVO_OUVERT = 0.25
SERVO_CONTACT = -0.05
SERVO_FERME = -0.13

OUTIL = np.array([-0.0430, 0.0881, -0.01102])
APPROCHE = np.array([-0.251, 0.966, -0.065])


def train_engrenages(angle_servo):
    pignon = angle_servo
    machoire_a = RAPPORT_PIGNON_A * pignon
    machoire_b = RAPPORT_A_B * machoire_a
    return [pignon, machoire_a, machoire_b]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _rot(axe, t):
    a = axe / np.linalg.norm(axe)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t)*K + (1 - np.cos(t))*K@K


class Bras:
    def __init__(self, urdf=URDF):
        self.joints = {}
        limites = {}
        for j in ET.parse(urdf).getroot().findall('joint'):
            o, ax = j.find('origin'), j.find('axis')
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = _rpy(*map(float, o.get('rpy', '0 0 0').split()))
                T[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
            self.joints[j.find('child').get('link')] = dict(
                nom=j.get('name'), parent=j.find('parent').get('link'), T=T,
                axe=np.array(list(map(float, ax.get('xyz').split()))) if ax is not None else None)
            lim = j.find('limit')
            if lim is not None:
                limites[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
        # butees des 4 servos du bras, lues dans l'URDF (fenetre de 180 degres de chaque servo)
        self.bas = np.array([limites[n][0] for n in BRAS])
        self.haut = np.array([limites[n][1] for n in BRAS])

    def pose(self, lien, q):
        if lien == 'world':
            return np.eye(4)
        j = self.joints[lien]
        T = self.pose(j['parent'], q) @ j['T']
        if j['axe'] is not None:
            R = np.eye(4)
            R[:3, :3] = _rot(j['axe'], q.get(j['nom'], 0.0))
            T = T @ R
        return T

    def outil(self, q4):
        """Position du point de prise et direction d'approche pour les 4 angles du bras."""
        T = self.pose('bras quatre', dict(zip(BRAS, q4)))
        return T[:3, :3] @ OUTIL + T[:3, 3], T[:3, :3] @ APPROCHE

    def points_bras(self, q4):
        """Coude, poignet et point de prise (pour verifier que le bras ne heurte rien)."""
        q = dict(zip(BRAS, q4))
        return self.pose('bras trois', q)[:3, 3], self.pose('bras quatre', q)[:3, 3], self.outil(q4)[0]

    def _residu(self, cible, approche_z, poids_approche):
        """Ecart de position + ecart d'inclinaison de la pince (composante verticale de la direction d'approche :
        -1 = pince vers le bas, 0 = horizontale). Seule l'inclinaison est imposee : la pince est construite
        legerement de biais (4 degres), une direction complete imposee fausserait la position."""
        def residu(q):
            p, a = self.outil(q)
            return np.r_[p - cible, poids_approche * (a[2] - approche_z)]
        return residu

    def ik_locale(self, cible, graine, approche_z=-1.0, poids_approche=0.05):
        """Une seule optimisation partant de `graine` : rapide, trouve la solution la plus proche de la graine."""
        cible = np.asarray(cible, float)
        r = least_squares(self._residu(cible, approche_z, poids_approche), np.clip(graine, self.bas, self.haut),
                          bounds=(self.bas, self.haut))
        return r.x, float(np.linalg.norm(r.fun[:3]))

    def ik(self, cible, q_depart, approche_z=-1.0, poids_approche=0.05):
        """Angles du bras pour amener le point de prise sur `cible`, pince inclinee selon `approche_z`.
        Parmi les solutions, la plus proche de la posture actuelle (pas de geste brusque)."""
        cible = np.asarray(cible, float)
        q_depart = np.clip(q_depart, self.bas, self.haut)
        residu = self._residu(cible, approche_z, poids_approche)
        r = least_squares(residu, q_depart, bounds=(self.bas, self.haut))
        if np.linalg.norm(r.fun[:3]) < 1e-3 and np.abs(r.x - q_depart).max() < 0.6:
            return r.x, float(np.linalg.norm(r.fun[:3]))
        lacet = np.arctan2(cible[1], cible[0])
        meilleur = None
        departs = [q_depart] + [np.array([lacet, s, c, w]) for s in (-0.2, 0.4, 1.0, 1.4) for c in (0.2, 1.0, 1.8, 2.5)
                                for w in (-1.2, -0.4, 0.4, 1.2)]
        for q0 in departs:
            r = least_squares(residu, np.clip(q0, self.bas, self.haut), bounds=(self.bas, self.haut))
            err = float(np.linalg.norm(r.fun[:3]))
            cout = (0.0 if err < 1e-3 else 100.0 + err) + np.abs(r.x - q_depart).sum()
            if meilleur is None or cout < meilleur[0]:
                meilleur = (cout, r.x, err)
        return meilleur[1], meilleur[2]
