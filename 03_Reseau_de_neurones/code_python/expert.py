"""L'expert (le professeur) du bras 50 cm : pour l'etat courant, la bonne action. Sert a l'imitation et a DAgger.

Pour une balle et son receptacle, il prepare deux chemins de la pince (positions + inclinaison) :
  approche  : depart -> passage interieur -> au-dessus de la balle (7 cm) ; puis descente verticale, prise
  transport : levee verticale (8 cm) -> passage interieur -> au-dessus du point de lacher -> lacher
Passage interieur : on rentre la pince a 8 cm de l'axe du bras, 26 cm de haut, et on tourne a ce rayon. La colonne
du grand receptacle (20 cm de diametre, 50 cm de haut, face au robot) commence a 11,5 cm de l'axe au plus pres :
en tournant plus pres de l'axe, le bras ne peut pas la heurter. Pour le grand receptacle, on monte a 54 cm a ce
rayon, puis on avance au-dessus de l'ouverture en mettant la pince a l'horizontale (pince verticale, le poignet
devrait etre a 64 cm : hors de portee).
Chaque pose des chemins est verifiee (portee, butees des servos, chocs avec receptacles, robot, balles voisines).
"""
import numpy as np

from cinematique import SERVO_CONTACT, SERVO_FERME, SERVO_OUVERT
from obstacles import heurte
from politique import PAS_SERVO
from zone import GRAND, cible_lacher

R_INT, Z_INT = 0.08, 0.26        # passage interieur
Z_LEVEE = 0.08                   # levee verticale apres la prise
H_APPROCHE = 0.07                # pince au-dessus de la balle avant la descente verticale
Z_PASSAGE_PETIT = 0.21           # au-dessus d'un petit receptacle, avant de descendre a la hauteur de lacher
PAS_CHEMIN = 0.012               # m entre deux points d'un chemin
SAUT_MAX = 0.5                   # rad : deux points voisins du chemin ne doivent pas demander un geste brusque
ERREUR_MAX = 0.002
RAYON_LACHER = 0.012
HAUTEUR_DECOLLAGE = 0.03
VITESSE_DECOLLAGE = 0.25
HAUTEUR_DOUTE = 0.03
# Hauteur de prise au-dessus du centre de la balle. Essais Gazebo : 0 (centre) avec la balle de 50 g de la
# simulation = prise sure ; +3 mm = la balle glisse. (Avec une balle de 2 g, aucun reglage ne tenait dans Gazebo.)
DZ_PRISE = 0.0
MARGE_PLAN = 0.008              # marge de securite des chemins (le bras reel suit ses consignes avec retard)
# (inclinaison voulue de la pince, poids) pour l'IK : -1 = vers le bas. Au-dessus du grand receptacle, la pince
# pointe un peu vers le haut (+0,4 : 24 degres) : a l'horizontale, le bras serait tendu a 99 %.
VERTICALE, GRAND_HAUT = (-1.0, 0.05), (0.4, 0.3)


def interieur(angle, z):
    return np.array([R_INT * np.cos(angle), R_INT * np.sin(angle), z])


def angle(p):
    return float(np.arctan2(p[1], p[0]))


class Expert:
    def __init__(self, cine):
        self.cine = cine
        self.plan = None
        self.raison = ''
        q, _ = cine.ik(interieur(0.0, Z_INT), np.array([0.0, 0.0, 2.2, 1.1]), *VERTICALE)
        self.q_repos = q            # posture de repos : pince a 8 cm de l'axe, 26 cm de haut, vers le bas

    # ------------------------------------------------------------------ construction des chemins
    @staticmethod
    def _segment(a, b, o_a, o_b):
        """Points de a vers b (a exclu), avec l'inclinaison voulue qui passe de o_a a o_b."""
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / PAS_CHEMIN)))
        return [(a + (b - a) * s, (o_a[0] + (o_b[0] - o_a[0]) * s, o_a[1] + (o_b[1] - o_a[1]) * s))
                for s in np.linspace(0, 1, n + 1)[1:]]

    def _route(self, depart, o_depart, arrivee, o_arrivee, z_arrivee_int):
        """depart -> passage interieur -> arrivee. z_arrivee_int : hauteur au rayon interieur avant de ressortir."""
        pts, p, o = [], np.asarray(depart, float), o_depart
        a0, a1 = angle(depart), angle(arrivee)
        if np.hypot(*p[:2]) > R_INT + 0.005 and p[2] > Z_INT + 0.01:
            # en haut (au-dessus du grand receptacle) : on rentre d'abord a la meme hauteur
            pts += self._segment(p, interieur(a0, p[2]), o, o)
            p = interieur(a0, p[2])
        pts += self._segment(p, interieur(a0, Z_INT), o, VERTICALE)
        n = max(1, int(np.ceil(abs(a1 - a0) / 0.08)))
        for s in np.linspace(0, 1, n + 1)[1:]:
            pts.append((interieur(a0 + (a1 - a0) * s, Z_INT), VERTICALE))
        p, o = interieur(a1, Z_INT), VERTICALE
        if abs(z_arrivee_int - Z_INT) > 1e-6:
            pts += self._segment(p, interieur(a1, z_arrivee_int), VERTICALE, o_arrivee)
            p, o = interieur(a1, z_arrivee_int), o_arrivee
        pts += self._segment(p, np.asarray(arrivee, float), o, o_arrivee)
        # sans les points en double (segments de longueur nulle, arc nul quand on part dans la bonne direction) :
        # sinon le "point suivant" serait le point actuel et le bras resterait sur place
        sans_doublons = [pts[0]]
        for pt in pts[1:]:
            if np.linalg.norm(pt[0] - sans_doublons[-1][0]) > 1e-4:
                sans_doublons.append(pt)
        return sans_doublons

    def _verifier(self, chemin, q, scene, tenue, autres, k):
        """Suit le chemin avec l'IK depuis la posture q. Renvoie la liste des postures, ou None (+ self.raison)."""
        postures = []
        for i, (p, (az, poids)) in enumerate(chemin):
            q2, err = self.cine.ik_locale(p, q, az, poids)
            if err > ERREUR_MAX or np.abs(q2 - q).max() > SAUT_MAX:
                q2, err = self.cine.ik(p, q, az, poids)
                if err > ERREUR_MAX or np.abs(q2 - q).max() > SAUT_MAX:
                    self.raison = f'pose {i} hors de portee ({np.round(p, 3)})'
                    return None
            choc = heurte(self.cine, q2, scene, k, tenue, autres, MARGE_PLAN)
            if choc:
                self.raison = f'choc {choc} en {np.round(p, 3)}'
                return None
            postures.append(q2)
            q = q2
        return postures

    def preparer(self, scene, k, q0, balle=None):
        """Prepare les chemins pour amener la balle k (0, 1, 2) dans le receptacle k, depuis la posture q0.
        `balle` : position actuelle de la balle (par defaut, celle de la disposition). Renvoie False si impossible."""
        balle = np.asarray(scene['balles'][k] if balle is None else balle, float)
        autres = [b for j, b in enumerate(scene['balles']) if j > k]        # encore sur la reglette
        grand = scene['types'][k] is GRAND
        cible = cible_lacher(scene, k)
        q0 = np.asarray(q0, float)
        dessus = balle + [0, 0, H_APPROCHE]
        outil0, direction0 = self.cine.outil(q0)
        approche = self._route(outil0, (direction0[2], 0.05), dessus, VERTICALE, Z_INT)   # depart : inclinaison actuelle
        q_approche = self._verifier(approche, q0, scene, False, autres, k)
        if q_approche is None:
            self.raison = 'approche : ' + self.raison
            return False
        # descente (verifiee comme le reste), prise, puis transport
        descente = self._segment(dessus, balle + [0, 0, DZ_PRISE], VERTICALE, VERTICALE)
        q_desc = self._verifier(descente, q_approche[-1], scene, False, autres, k)
        if q_desc is None:
            self.raison = 'descente : ' + self.raison
            return False
        haut = np.r_[balle[:2], balle[2] + Z_LEVEE]
        transport = self._segment(balle, haut, VERTICALE, VERTICALE)
        if grand:
            transport += self._route(haut, VERTICALE, cible, GRAND_HAUT, cible[2])
        else:
            transport += self._route(haut, VERTICALE, np.r_[cible[:2], Z_PASSAGE_PETIT], VERTICALE, Z_INT)
            transport += self._segment(np.r_[cible[:2], Z_PASSAGE_PETIT], cible, VERTICALE, VERTICALE)
        q_tr = self._verifier(transport, q_desc[-1], scene, True, autres, k)
        if q_tr is None:
            self.raison = 'transport : ' + self.raison
            return False
        self.plan = dict(k=k, grand=grand, balle0=balle.copy(), dessus=dessus, cible=cible, scene=scene,
                         approche=[(p, o) for p, o in approche], transport=transport,
                         approche_xyz=np.array([p for p, _ in approche]),
                         transport_xyz=np.array([p for p, _ in transport]))
        return True

    # ------------------------------------------------------------------ actions
    @staticmethod
    def _vers(q, cible):
        """Pas vers `cible`, reduit dans la meme proportion pour tous les servos (le geste garde sa direction)."""
        d = (cible - q) / PAS_SERVO
        return d / max(1.0, np.abs(d).max())

    def _vers_point(self, q, point, orientation=VERTICALE):
        q_cible, _ = self.cine.ik_locale(point, q, *orientation)
        return self._vers(q, q_cible)

    def _suivre(self, q, outil, chemin, xyz):
        """Pas vers le point du chemin qui suit le point le plus proche de la pince."""
        i = min(int(np.argmin(np.linalg.norm(xyz - outil, axis=1))) + 1, len(chemin) - 1)
        p, o = chemin[i]
        return self._vers_point(q, p, o)

    def action(self, q, servo, balle, tenue):
        """Bonne action [dq1..dq4, pince] pour l'etat courant (pince : -1 fermer, +1 ouvrir)."""
        p = self.plan
        outil = self.cine.outil(q)[0]
        if tenue:
            if np.linalg.norm(outil[:2] - p['cible'][:2]) < RAYON_LACHER and outil[2] > p['cible'][2] - 0.007:
                return np.r_[np.zeros(4), 1.0]                # au-dessus de l'ouverture : on lache
            if servo > SERVO_FERME + 0.005 and outil[2] < p['balle0'][2] + 0.01:
                return np.r_[np.zeros(4), -1.0]               # serrage pas fini : on ne souleve pas encore
            a = self._suivre(q, outil, p['transport'], p['transport_xyz'])
            if outil[2] < p['balle0'][2] + HAUTEUR_DECOLLAGE:
                a = a * VITESSE_DECOLLAGE                     # on decolle doucement
            return np.r_[a, -1.0]
        if np.linalg.norm(balle[:2] - p['balle0'][:2]) > 0.015:
            # balle poussee : nouveau chemin depuis sa nouvelle position (sinon on vise la balle la ou elle est)
            if self.preparer(p['scene'], p['k'], q, balle):
                p = self.plan
            else:
                p['balle0'] = np.asarray(balle, float).copy()
                p['dessus'] = p['balle0'] + [0, 0, H_APPROCHE]
        xy = np.linalg.norm(outil[:2] - balle[:2])
        prise = balle + [0, 0, DZ_PRISE]
        ecart = np.linalg.norm(outil - prise)
        en_fermeture = servo < SERVO_OUVERT - 0.02
        if servo <= SERVO_CONTACT and xy < 0.015 and outil[2] < balle[2] + HAUTEUR_DOUTE:
            # machoires fermees sur la balle, pince encore basse : on fait comme si elle etait tenue (fin du
            # serrage, levee lente) ; ce n'est qu'a 3 cm de haut, balle restee au sol, que la prise est ratee
            if servo > SERVO_FERME + 0.005:
                return np.r_[np.zeros(4), -1.0]
            return np.r_[self._vers_point(q, outil + [0, 0, 0.01]) * VITESSE_DECOLLAGE, -1.0]
        if servo > SERVO_CONTACT and (ecart < 0.003 or (en_fermeture and ecart < 0.008)):
            return np.r_[self._vers_point(q, prise), -1.0]    # pince autour de la balle : on serre
        if en_fermeture and xy < 0.03 and outil[2] < balle[2] + H_APPROCHE:
            # pince fermee a cote de la balle (prise ratee) : on rouvre en se recentrant au-dessus
            # (loin de la balle, par exemple juste apres un lacher, la pince finit de s'ouvrir en chemin)
            return np.r_[self._vers_point(q, np.r_[balle[:2], max(outil[2], balle[2] + 0.01)]), 1.0]
        if xy < 0.01 and outil[2] < p['dessus'][2] + 0.01:
            # a la verticale de la balle, pince ouverte : descente verticale
            return np.r_[self._vers_point(q, np.r_[balle[:2], max(prise[2], outil[2] - 0.01)]), 1.0]
        if xy >= 0.01 and outil[2] < balle[2] + H_APPROCHE - 0.01:
            # pince basse mais decalee : on remonte d'abord a la verticale (sinon on balaierait la reglette)
            return np.r_[self._vers_point(q, outil + [0, 0, 0.015]), 1.0]
        if np.linalg.norm(outil - p['dessus']) < 0.02:
            return np.r_[self._vers_point(q, p['dessus']), 1.0]
        return np.r_[self._suivre(q, outil, p['approche'], p['approche_xyz']), 1.0]
