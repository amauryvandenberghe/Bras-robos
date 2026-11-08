"""Simulateur rapide pour l'apprentissage : la vraie geometrie du bras 50 cm (URDF), sans la physique de Gazebo.

Meme modele de prise que pour le bras actuel (cale sur Gazebo) :
  - servos en retard (a chaque pas, ils ne font qu'une partie du chemin vers la consigne) ;
  - balle attrapee si, quand les machoires arrivent au contact, le centre reel de la pince est a moins de 6 mm ;
  - balle tenue decalee de 1,5 a 7 mm ; soulever avant la fin du serrage la fait glisser ;
  - position de la balle vue par le reseau legerement bruitee (camera) ;
  - pince rouverte : la balle tombe a la verticale, dans le receptacle ou par terre ;
  - une pince qui arrive de travers pousse la balle.
En plus : bras, pince ou balle tenue qui heurte un receptacle, le robot ou une balle voisine = echec ('collision').
"""
import numpy as np

from cinematique import Bras, SERVO_CONTACT, SERVO_FERME, SERVO_OUVERT
from obstacles import heurte
from politique import observation, securite, servo_suivant
from zone import GRAND, R_BALLE, cible_lacher

TOLERANCE_PRISE = 0.006
DECALAGE_TENUE = (0.0015, 0.007)
BRUIT_VUE = 0.0005
SUIVI_SERVO = 0.5
ENTRE_MACHOIRES = 0.010
PORTEE_MACHOIRES = 0.030
HAUT_MACHOIRES = 0.0175
PAS_MAX = 700            # 70 s par balle


class SimReceptacles:
    pas_max = PAS_MAX

    def __init__(self, cine=None):
        self.cine = cine or Bras()
        self.rng = np.random.default_rng()

    def reset(self, scene, k, q, servo=SERVO_OUVERT, graine=None):
        """Balle k (0, 1, 2) a mettre dans le receptacle k ; les balles d'indice > k sont encore sur la reglette."""
        if graine is not None:
            self.rng = np.random.default_rng(graine)
        self.scene, self.k = scene, k
        self.balle = np.array(scene['balles'][k], float)
        self.autres = [np.array(b, float) for j, b in enumerate(scene['balles']) if j > k]
        self.cible, self.grand = cible_lacher(scene, k), scene['types'][k] is GRAND
        self.q = np.array(q, float)
        self.q_reel = self.q.copy()
        self.servo, self.tenue, self.pas = servo, False, 0
        self.decalage = np.zeros(3)
        self.resultat = None
        return self.obs()

    def outil(self):
        return self.cine.outil(self.q)[0]

    def outil_reel(self):
        return self.cine.outil(self.q_reel)[0]

    def obs(self):
        vue = self.balle + self.rng.normal(0, BRUIT_VUE, 3)
        return observation(self.q, self.servo, vue, self.cible, self.outil(), self.grand)

    def _pousser(self, outil):
        if outil[2] > self.balle[2] + HAUT_MACHOIRES:
            return
        d = self.balle[:2] - outil[:2]
        n = np.linalg.norm(d)
        if n < ENTRE_MACHOIRES or n >= PORTEE_MACHOIRES:
            return
        direction = d / n if n > 1e-6 else np.array([1.0, 0.0])
        self.balle[:2] = outil[:2] + direction * PORTEE_MACHOIRES

    def step(self, action):
        self.pas += 1
        q, pince = securite(self.cine, self.q, action, self.servo)
        servo_avant = self.servo
        self.q, self.servo = q, servo_suivant(self.servo, pince)
        self.q_reel += SUIVI_SERVO * (self.q - self.q_reel)
        outil = self.outil_reel()
        if not self.tenue:
            self._pousser(outil)
        au_contact = servo_avant > SERVO_CONTACT >= self.servo
        if not self.tenue and au_contact and np.linalg.norm(outil - self.balle) < TOLERANCE_PRISE:
            self.tenue = True
            self.z_prise, self.z_sol_balle = outil[2], self.balle[2]
            d = self.rng.normal(size=3)
            self.decalage = d / np.linalg.norm(d) * self.rng.uniform(*DECALAGE_TENUE)
        serrage_en_cours = SERVO_FERME + 0.005 < self.servo <= servo_avant
        if self.tenue and serrage_en_cours and self.z_prise + 0.003 < outil[2] < self.z_prise + 0.03:
            self.tenue = False
            self.balle = np.r_[self.balle[:2], self.z_sol_balle]
        if self.tenue:
            self.balle = outil + self.decalage
            if self.servo > SERVO_CONTACT:
                self.tenue = False
                self.resultat = self._lacher()
        if self.resultat is None:
            self.choc = heurte(self.cine, self.q_reel, self.scene, self.k, self.tenue, self.autres)
            if self.choc:
                self.resultat = 'collision'
        if self.resultat is None and self.pas >= self.pas_max:
            self.resultat = 'temps'
        return self.obs(), self.resultat is not None

    def _lacher(self):
        """La balle tombe a la verticale : dans le receptacle vise (ouverture, au-dessus du bord) ou par terre."""
        c, t = np.asarray(self.scene['receptacles'][self.k], float), self.scene['types'][self.k]
        if np.linalg.norm(self.balle[:2] - c) < t['r_int'] - 0.005 and self.balle[2] - R_BALLE > t['bord']:
            self.balle[2] = t['bord'] - t['r_int'] + R_BALLE
            return 'reussite'
        self.balle[2] = R_BALLE
        return 'tombee'
