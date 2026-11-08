"""Dimensionnement du nouveau bras : longueurs des segments et couples sur chaque servomoteur.

Hypotheses (toutes modifiables ci-dessous) :
  - le bras est fixe au centre du dessus du robot roulant (200 x 200 mm, dessus a HAUTEUR_CHASSIS du sol) ;
  - balles de 23 mm / 2 g posees sur la reglette (centre de la balle a Z_BALLE du sol) ;
  - petits receptacles : demi-sphere de 100 mm, bord a 150 mm ; grand receptacle : demi-sphere de 200 mm, bord a 500 mm.
    Chaque receptacle est pose sur une colonne du meme diametre : le robot ne peut pas passer dessous ;
  - servos (fiches constructeur, a 4,8 V) : MG996R 9,4 kg.cm / 55 g, SG90 1,8 kg.cm / 9 g, MG90S 1,8 kg.cm / 13,4 g.
    Regle de conception : couple statique <= 50 % du couple de blocage (au-dela un servo chauffe, tremble et s'use).

Compare deux architectures :
  5 axes : lacet (MG996R) + epaule (MG996R) + coude + poignet (tangage) + pince
  6 axes : la meme chose + rotation du poignet (roulis, un SG90 de plus entre poignet et pince)
Usage : python3 dimensionnement.py
"""
import numpy as np

G = 9.81
KGCM = 0.0981          # 1 kg.cm en N.m

HAUTEUR_CHASSIS = 0.080   # dessus du robot roulant
H_EPAULE = 0.093          # axe d'epaule au-dessus du dessus du robot (servo de lacet vertical + plateau + berceau)
Z_EPAULE = HAUTEUR_CHASSIS + H_EPAULE
Z_BALLE = 0.0137          # balle de 23 mm posee sur un anneau de 15 mm, dessus de la reglette a 5 mm
R_BALLE = 0.0115
MARGE_BORD = 0.015        # dessous de la balle au-dessus du bord du receptacle au moment du lacher

PETIT = dict(rayon=0.05, bord=0.150)
GRAND = dict(rayon=0.10, bord=0.500)
DEMI_ROBOT = 0.10 * np.sqrt(2)   # coin du robot, vu depuis l'axe du bras
JEU = 0.01                       # entre le robot et une colonne

SERVOS = {  # couple de blocage a 4,8 V (N.m), masse (kg)
    'MG996R': (9.4 * KGCM, 0.055),
    'MG90S': (1.8 * KGCM, 0.0134),
    'SG90': (1.8 * KGCM, 0.009),
}

# Masses des pieces imprimees (PLA, 3 perimetres, 20 % de remplissage) : masse lineique + embouts (logements de servo).
# Recalees ensuite sur les volumes reels des pieces CAO (voir cao_bras.py).
BRAS_G_PAR_M = 0.11       # kg/m, poutre creuse 22 x 14 mm
FAVANT_G_PAR_M = 0.08     # kg/m, poutre creuse 18 x 12 mm
EMBOUT = 0.008            # kg par extremite avec logement de servo
PINCE = 0.045             # pince actuelle : corps + 3 engrenages + MG90S
C_PINCE = 0.045           # centre de gravite de la pince, depuis l'axe du poignet
L_OUTIL = 0.098           # axe du poignet -> centre de la balle tenue (pince actuelle)
ROULIS_L = 0.032          # longueur ajoutee par l'etage de roulis (6 axes)
ROULIS_M = 0.009 + 0.006  # SG90 + etrier


def ik_plan(r, z, phi, L1, L2, Lt):
    """Angles (epaule depuis l'horizontale, coude relatif, poignet relatif) pour mettre la balle en (r, z),
    pince orientee a phi (0 = horizontale vers l'exterieur, -pi/2 = verticale vers le bas). Coude en haut."""
    wx, wz = r - Lt * np.cos(phi), z - Lt * np.sin(phi) - Z_EPAULE
    d = np.hypot(wx, wz)
    if d > L1 + L2 - 1e-3 or d < abs(L1 - L2) + 1e-3:
        return None
    c2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    t2 = -np.arccos(np.clip(c2, -1, 1))             # coude en haut : coude replie vers le bas
    t1 = np.arctan2(wz, wx) - np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
    t3 = phi - t1 - t2
    return np.array([t1, t2, t3])


def butees_ok(t):
    """Servos a 180 degres, montes pour couvrir ce dont la tache a besoin."""
    t1, t2, t3 = t
    return (np.radians(-10) <= t1 <= np.radians(190) and np.radians(-160) <= t2 <= np.radians(20)
            and np.radians(-175) <= t3 <= np.radians(5))


def couples(t, L1, L2, Lt, six, balle=True):
    """Couple statique (N.m) sur epaule, coude, poignet pour les angles t."""
    t1, t2, t3 = t
    a1, a2, a3 = t1, t1 + t2, t1 + t2 + t3
    e = np.array([L1 * np.cos(a1), 0])
    w = e + [L2 * np.cos(a2), 0]
    u3 = np.array([np.cos(a3), 0])
    masses = [  # (masse, abscisse horizontale depuis l'epaule)
        (BRAS_G_PAR_M * L1 + 2 * EMBOUT, L1 / 2 * np.cos(a1)),
        (SERVOS[COUDE][1], e[0]),
        (FAVANT_G_PAR_M * L2 + 2 * EMBOUT, e[0] + L2 / 2 * np.cos(a2)),
        (SERVOS['SG90'][1], w[0]),                                   # servo de tangage du poignet
        (PINCE, w[0] + (C_PINCE + (ROULIS_L if six else 0)) * u3[0]),
    ]
    if six:
        masses.append((ROULIS_M, w[0] + 0.5 * ROULIS_L * u3[0]))
    if balle:
        masses.append((0.002, w[0] + Lt * u3[0]))
    m = np.array(masses)
    c_epaule = G * np.abs((m[:, 0] * m[:, 1]).sum())
    c_coude = G * np.abs((m[2:, 0] * (m[2:, 1] - e[0])).sum())
    c_poignet = G * np.abs((m[4:, 0] * (m[4:, 1] - w[0])).sum())
    return np.array([c_epaule, c_coude, c_poignet])


def poses_tache(rng, n=400):
    """Poses dont la tache a besoin : prise au sol (pince verticale), lacher dans les petits et le grand receptacle."""
    poses = []
    for _ in range(n):
        r = rng.uniform(R_SOL[0], R_SOL[1])
        poses.append(('sol', r, Z_BALLE, [-np.pi / 2]))
        poses.append(('sol_haut', r, Z_BALLE + 0.06, [-np.pi / 2]))      # balle levee de 6 cm
        d = rng.uniform(*D_PETIT)
        poses.append(('petit', d, PETIT['bord'] + R_BALLE + MARGE_BORD, [-np.pi / 2, -1.2, -1.0]))
        d = rng.uniform(*D_GRAND)
        lacher = d - 0.06            # 6 cm vers l'interieur depuis le centre : 3 cm de marge sur la paroi
        # pince a peu pres horizontale (verticale, le poignet serait a 60 cm) ; balle 4 cm au-dessus du bord :
        # le dessous des machoires (1 cm sous la balle) passe au-dessus du bord
        poses.append(('grand', lacher, GRAND['bord'] + 0.04, [0.0, 0.2, 0.35, -0.2]))
    return poses


def evaluer(L1, L2, six, rng):
    Lt = L_OUTIL + (ROULIS_L if six else 0)
    pires, rates, confs = np.zeros(3), 0, []
    for nom, r, z, phis in poses_tache(rng):
        ok = None
        for phi in phis:
            t = ik_plan(r, z, phi, L1, L2, Lt)
            if t is not None and butees_ok(t):
                ok = t
                break
        if ok is None:
            rates += 1
            continue
        confs.append(ok)
        pires = np.maximum(pires, couples(ok, L1, L2, Lt, six))
    # trajets entre poses (interpolation articulaire) : le pire couple peut etre au milieu d'un mouvement
    for i in range(0, len(confs) - 1, 3):
        for s in np.linspace(0, 1, 6):
            pires = np.maximum(pires, couples((1 - s) * confs[i] + s * confs[i + 1], L1, L2, Lt, six))
    return rates, pires


COUDE = 'SG90'
R_SOL = (0.24, 0.34)                                         # reglette de balles : distance a l'axe du bras
D_PETIT = (PETIT['rayon'] + DEMI_ROBOT + JEU, 0.34)          # centre des petits receptacles
D_GRAND = (0.10 + GRAND['rayon'] + JEU, 0.235)               # robot gare face a la colonne (cote du carre, 1 a 3,5 cm)

if __name__ == '__main__':
    lim = {k: v[0] for k, v in SERVOS.items()}
    print(f'epaule a {Z_EPAULE * 100:.1f} cm du sol ; balles a {R_SOL[0] * 100:.0f}-{R_SOL[1] * 100:.0f} cm de l axe, '
          f'petits receptacles a {D_PETIT[0] * 100:.0f}-{D_PETIT[1] * 100:.0f} cm, grand a {D_GRAND[0] * 100:.0f}-{D_GRAND[1] * 100:.0f} cm')
    for COUDE in ('SG90', 'MG90S'):
        for six in (False, True):
            meilleur = None
            for L1 in np.arange(0.22, 0.40, 0.01):
                for L2 in np.arange(0.06, 0.20, 0.01):
                    rates, c = evaluer(L1, L2, six, np.random.default_rng(0))
                    if rates:
                        continue
                    taux = c / np.array([lim['MG996R'], lim[COUDE], lim['SG90']])
                    if meilleur is None or taux.max() < meilleur[0].max():
                        meilleur = (taux, L1, L2, c)
            nom = '6 axes' if six else '5 axes'
            if meilleur is None:
                print(f'{nom}, coude {COUDE} : aucune longueur ne couvre toute la tache')
                continue
            taux, L1, L2, c = meilleur
            print(f'{nom}, coude {COUDE} : bras {L1 * 100:.0f} cm, avant-bras {L2 * 100:.0f} cm -> couple max '
                  f'epaule {c[0] / KGCM:.2f} kg.cm ({taux[0]:.0%}), coude {c[1] / KGCM:.2f} kg.cm ({taux[1]:.0%}), '
                  f'poignet {c[2] / KGCM:.2f} kg.cm ({taux[2]:.0%})')
