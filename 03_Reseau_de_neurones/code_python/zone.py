"""Scene du challenge (simplifiee) : le robot (200 x 200 mm, bras au centre de son dessus), une reglette de 5 trous
au pas de 50 mm portant 3 balles, deux petits receptacles (demi-sphere de 100 mm, bord a 150 mm) et le grand
receptacle (demi-sphere de 200 mm, bord a 500 mm). Chaque receptacle est pose sur une colonne de son diametre.

Tache : une balle par receptacle, du receptacle le plus proche des balles au plus eloigne (scene['types'] dit
lequel est le grand). Sur le parcours, le robot se gare devant un receptacle a la fois : seul le receptacle vise
est un obstacle, le bras peut traverser les deux autres (dans Gazebo, ils sont affiches "fantomes", sans collision).
Tout est tire au hasard a chaque essai, dans ce que le bras peut atteindre (mesure : bras_50cm/verifier_tache.py) :
  - balles a 19-35 cm de l'axe du bras (reglette dirigee vers le robot, a +-15 degres pres :
    les machoires s'ouvrent sur le cote et ne touchent pas les balles voisines) ;
  - petits receptacles a 21-34 cm ; grand receptacle face au robot (+-15 degres), gare a 1,5-3,5 cm de lui ;
  - le servo de lacet a 180 degres de course : tout est devant le robot, entre -85 et +85 degres.
"""
import numpy as np

AXE = np.zeros(2)                     # axe du bras = centre du robot
DEMI_ROBOT, HAUT_ROBOT = 0.10, 0.08   # robot : carre de 200 mm, dessus a 80 mm
Z_BALLE = 0.0148                      # centre d'une balle posee sur la reglette
R_BALLE = 0.0115
PAS_REGLETTE, N_TROUS = 0.05, 5
LONG_REGLETTE, LARG_REGLETTE = 0.27, 0.03
LACET_MAX = np.radians(85)

# receptacles : rayon exterieur, rayon interieur, hauteur du bord
PETIT = dict(r_ext=0.05, r_int=0.04, bord=0.15)
GRAND = dict(r_ext=0.10, r_int=0.09, bord=0.50)
TYPES = [PETIT, PETIT, GRAND]          # modeles Gazebo receptacle_1, _2, _3 (l'ordre de depot depend de la scene)

R_BALLES = (0.19, 0.35)
R_PETITS = (0.21, 0.34)
D_GRAND = (0.215, 0.235)
ANGLE_GRAND = np.radians(15)
RENTRE_GRAND = 0.06                   # on lache a 6 cm du centre du grand receptacle, du cote du robot
Z_LACHER = {0.15: 0.18, 0.50: 0.56}   # hauteur du point de prise au moment du lacher (selon le bord)


def point(r, angle):
    return AXE + r * np.array([np.cos(angle), np.sin(angle)])


def distance_robot(p):
    """Distance horizontale d'un point au carre du robot (0 si dedans)."""
    d = np.maximum(np.abs(np.asarray(p)[:2] - AXE) - DEMI_ROBOT, 0)
    return float(np.linalg.norm(d))


def segment_points(a, b, n=12):
    return [a + (b - a) * s for s in np.linspace(0, 1, n)]


def tirer_scene(rng):
    """Disposition aleatoire. Renvoie dict(receptacles=[3 x (x, y)], reglette=(centre, angle), trous=[3 indices],
    balles=[3 x (x, y, z)])."""
    while True:
        a3 = rng.uniform(-ANGLE_GRAND, ANGLE_GRAND)
        grand = point(rng.uniform(*D_GRAND), a3)
        if distance_robot(grand) < GRAND['r_ext'] + 0.008:
            continue
        # reglette : dirigee vers le robot ; 3 trous occupes, tous a portee
        a_r = rng.uniform(-LACET_MAX + 0.1, LACET_MAX - 0.1)
        if abs(a_r - a3) < np.radians(40):
            continue     # pour prendre une balle dans la direction du grand receptacle, le coude entrerait dans sa colonne
        psi = a_r + rng.uniform(-np.radians(15), np.radians(15))
        u = np.array([np.cos(psi), np.sin(psi)])
        trous = np.sort(rng.choice(N_TROUS, 3, replace=False))
        centre = point(rng.uniform(0.18, 0.36), a_r)
        pos_trous = [centre + (k - 2) * PAS_REGLETTE * u for k in range(N_TROUS)]
        balles = [pos_trous[k] for k in trous]
        r_b = [np.linalg.norm(b - AXE) for b in balles]
        if min(r_b) < R_BALLES[0] or max(r_b) > R_BALLES[1]:
            continue
        if any(abs(np.arctan2(b[1], b[0])) > LACET_MAX for b in balles):
            continue
        bord_regl = segment_points(centre - u * LONG_REGLETTE / 2, centre + u * LONG_REGLETTE / 2, 20)
        if min(distance_robot(p) for p in bord_regl) < 0.012 + LARG_REGLETTE / 2:
            continue
        if min(np.linalg.norm(p - grand) for p in bord_regl) < GRAND['r_ext'] + 0.06:
            continue
        # sur le parcours, un seul receptacle est devant le robot a la fois : les receptacles n'ont pas a s'ecarter
        # les uns des autres, seulement du robot et de la reglette
        petits, essais = [], 0
        while len(petits) < 2 and essais < 200:
            essais += 1
            c = point(rng.uniform(*R_PETITS), rng.uniform(-LACET_MAX + 0.15, LACET_MAX - 0.15))
            if distance_robot(c) < PETIT['r_ext'] + 0.01:
                continue
            if min(np.linalg.norm(p - c) for p in bord_regl) < PETIT['r_ext'] + 0.045:
                continue
            # les colonnes ne se chevauchent pas (sinon une balle finirait au-dessus de la cible suivante)
            if np.linalg.norm(c - grand) < GRAND['r_ext'] + PETIT['r_ext'] + 0.01:
                continue
            if any(np.linalg.norm(c - p) < 2 * PETIT['r_ext'] + 0.01 for p in petits):
                continue
            petits.append(c)
        if len(petits) < 2:
            continue
        ordre = rng.permutation(3)                       # quelle balle de la reglette est la balle 1, 2, 3
        # ordre de depot : du receptacle le plus proche des balles (centre de la reglette) au plus eloigne
        recs = [(petits[0], 1), (petits[1], 2), (grand, 3)]
        recs.sort(key=lambda r: np.linalg.norm(r[0] - centre))
        return dict(receptacles=[r[0] for r in recs], modeles=[r[1] for r in recs],
                    types=[TYPES[r[1] - 1] for r in recs], reglette=(centre, psi),
                    trous=[int(trous[i]) for i in ordre], balles=[np.r_[balles[i], Z_BALLE] for i in ordre])


def cible_lacher(scene, k):
    """Point ou la pince lache la balle k (0, 1, 2) : (x, y, z)."""
    c, t = np.asarray(scene['receptacles'][k], float), scene['types'][k]
    if t is GRAND:
        c = c - RENTRE_GRAND * c / np.linalg.norm(c)
    return np.r_[c, Z_LACHER[t['bord']]]


def dans_receptacle(balle, centre, t):
    """Balle posee au fond du receptacle (et pas sur le bord ni par terre)."""
    return (np.linalg.norm(np.asarray(balle[:2]) - centre) < t['r_int']
            and t['bord'] - t['r_ext'] < balle[2] < t['bord'])
