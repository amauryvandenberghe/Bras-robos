"""Le reseau de neurones qui pilote le bras 50 cm, et la couche de securite autour de lui.

Tout ce qui est ici doit pouvoir tourner sur la carte du robot : observation calculee a partir des consignes
des servos, de la position de la balle et du point de lacher (plus tard : camera), une petite cinematique directe,
puis un reseau de 2 couches cachees.

Rythme : 10 pas par seconde. A chaque pas le reseau choisit :
  - de combien tourner chacun des 4 servos du bras (au plus PAS_SERVO rad),
  - s'il faut fermer (< 0) ou ouvrir (> 0) la pince.
"""
import numpy as np
import torch
from torch import nn

from cinematique import SERVO_CONTACT, SERVO_OUVERT, SERVO_FERME
from zone import AXE, DEMI_ROBOT, HAUT_ROBOT

PERIODE = 0.1            # s entre deux decisions
PAS_SERVO = 0.06         # rad max par pas et par servo du bras (0,6 rad/s)
PAS_PINCE = 0.04         # rad max par pas pour le servo de pince (fermeture progressive)
PAS_SERRAGE = 0.01       # rad par pas pour les derniers degres du serrage, contre la balle (~0,6 s)
Z_MIN_OUTIL = 0.009      # le point de prise ne descend jamais plus bas (sol + marge)
N_OBS, N_ACT = 27, 5


def observation(q, servo, balle, cible, outil, grand):
    """Vecteur d'entree du reseau, a peu pres a l'echelle [-1, 1].
    cible = point de lacher (x, y, z) ; grand = 1 si c'est le grand receptacle (lacher pince horizontale)."""
    b = np.asarray(balle) - np.r_[AXE, 0]
    c = np.asarray(cible) - np.r_[AXE, 0]
    o = np.asarray(outil) - np.r_[AXE, 0]
    return np.concatenate([
        np.asarray(q) / 3.0,
        [(servo - SERVO_FERME) / (SERVO_OUVERT - SERVO_FERME) * 2 - 1],
        b * 4, c * 4, o * 4,
        (np.asarray(balle) - outil) * 10,
        (np.asarray(cible) - outil) * 4,
        # vue "zoomee" : precision au millimetre pres de la balle et du point de lacher (sature a +-2 cm)
        np.clip((np.asarray(balle) - outil) * 50, -1, 1),
        np.clip((np.asarray(cible) - outil) * 50, -1, 1),
        [1.0 if grand else -1.0],
    ]).astype(np.float32)


# indices utiles dans l'observation
I_SERVO, I_BALLE_OUTIL, I_ZOOM_BALLE, I_GRAND = 4, slice(14, 17), slice(20, 23), 26


class Reseau(nn.Module):
    def __init__(self, cache=512):
        super().__init__()
        self.couches = nn.Sequential(
            nn.Linear(N_OBS, cache), nn.Tanh(),
            nn.Linear(cache, cache), nn.Tanh(),
            nn.Linear(cache, N_ACT))

    def brut(self, x):
        """Sorties avant le tanh final (la 5e sert de score "ouvrir la pince" pour l'apprentissage)."""
        return self.couches(x)

    def forward(self, x):
        return torch.tanh(self.couches(x))


def dans_robot(p, marge=0.012):
    """Point (x, y, z) dans le volume du robot roulant (marge comprise)."""
    return bool(np.all(np.abs(np.asarray(p[:2]) - AXE) < DEMI_ROBOT + marge) and p[2] < HAUT_ROBOT + marge)


def securite(cine, q, action, servo=None):
    """Garde-fou : borne la vitesse, respecte les butees des servos, interdit a la pince de toucher le sol ou le
    robot, et bloque le bras pendant le serrage (soulever avant la fin du serrage fait glisser la balle)."""
    a = np.clip(np.asarray(action, dtype=float), -1, 1)
    if servo is not None and a[4] < 0 and SERVO_FERME + 0.005 < servo <= SERVO_CONTACT + 0.03:
        return np.asarray(q, float).copy(), a[4]       # la pince serre : le bras attend la fin du serrage
    q_suivant = np.clip(q + a[:4] * PAS_SERVO, cine.bas, cine.haut)
    coude, poignet, outil = cine.points_bras(q_suivant)
    if outil[2] < Z_MIN_OUTIL or dans_robot(outil) or dans_robot(poignet, 0.0):
        q_suivant = np.asarray(q, float).copy()    # on refuse le mouvement qui heurterait le sol ou le robot
    return q_suivant, a[4]


def servo_suivant(servo, commande_pince):
    """Angle du servo de pince au pas suivant. Pres de la balle, on serre lentement."""
    cible = SERVO_FERME if commande_pince < 0 else SERVO_OUVERT
    pas = PAS_SERRAGE if cible == SERVO_FERME and servo <= SERVO_CONTACT + 0.03 else PAS_PINCE
    return servo + np.clip(cible - servo, -pas, pas)


class Politique:
    """Charge le reseau entraine et donne l'action pour un etat."""
    def __init__(self, chemin):
        poids = torch.load(chemin, map_location='cpu')
        self.reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
        self.reseau.load_state_dict(poids)
        self.reseau.eval()

    @torch.no_grad()
    def __call__(self, obs):
        return self.reseau(torch.from_numpy(obs)[None])[0].numpy()
