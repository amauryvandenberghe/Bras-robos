"""Le bras heurte-t-il le receptacle vise, le robot ou une balle voisine ? (expert, simulateur rapide, Gazebo)

Le bras est represente par des points de controle : coude, avant-bras, poignet, pince, pointes des machoires
(6 mm sous le point de prise) et, si elle est tenue, la balle. Un receptacle est un cylindre plein de son diametre,
du sol jusqu'a son bord (la colonne + la demi-sphere) : un point dedans = choc.
"""
import numpy as np

from politique import dans_robot
from zone import R_BALLE, Z_BALLE

MARGE_LIEN = 0.018       # demi-epaisseur des pieces (avant-bras, pince)
MARGE_POINTES = 0.004


# Bras de 25 cm (repere "bras deux") : poutre centree 2,8 cm a cote du plan du bras, bout du coude et servo de coude
POINTS_BRAS = np.array([[0.08, 0, 0.028], [0.13, 0, 0.028], [0.18, 0, 0.028], [0.22, 0, 0.028],
                        [0.245, 0, 0.034], [0.245, 0, 0.047]])
NOMS_POINTS = ['coude', 'avant-bras 1', 'avant-bras 2', 'poignet', 'pince', 'bras 1', 'bras 2', 'bras 3', 'bras 4',
               'bout du bras', 'servo du coude']


def points_controle(cine, q):
    from cinematique import BRAS
    coude, poignet, outil = cine.points_bras(q)
    T = cine.pose('bras deux', dict(zip(BRAS, q)))
    bras = list(POINTS_BRAS @ T[:3, :3].T + T[:3, 3])
    liens = [coude + (poignet - coude) * s for s in (0.0, 0.33, 0.66)] + \
            [poignet + (outil - poignet) * s for s in (0.0, 0.5)] + bras
    return liens, outil


def heurte(cine, q, scene, cible, tenue=False, autres_balles=(), marge=0.0):
    """Nom de l'obstacle touche, ou None. `cible` : indice du receptacle vise ; c'est le seul present pres du bras
    (sur le parcours, le robot se gare devant un receptacle a la fois : les autres sont hors de portee).
    `autres_balles` : balles encore sur la reglette (sauf celle visee).
    `marge` : distance de securite en plus (l'expert planifie avec une marge : le vrai bras, en retard sur ses
    consignes, coupe un peu les trajectoires)."""
    liens, outil = points_controle(cine, q)
    pointes = outil - [0, 0, 0.006]
    for i, (c, t) in enumerate(zip(scene['receptacles'], scene['types'])):
        if i != cible:
            continue
        for j, p in enumerate(liens):
            if np.linalg.norm(p[:2] - c) < t['r_ext'] + MARGE_LIEN + marge and p[2] < t['bord'] + MARGE_LIEN + marge / 2:
                return f'receptacle {i + 1} (point {NOMS_POINTS[j]} en {np.round(p, 3)})'
        for p in [pointes] + ([outil - [0, 0, R_BALLE]] if tenue else []):
            if np.linalg.norm(p[:2] - c) < t['r_ext'] + MARGE_POINTES + marge and p[2] < t['bord'] + 0.002 + marge / 2:
                return f'receptacle {i + 1}'
    for p in liens[1:]:
        if dans_robot(p, 0.0):
            return 'robot'
    for b in autres_balles:
        if np.linalg.norm(pointes[:2] - b[:2]) < 0.028 and pointes[2] < Z_BALLE + R_BALLE + 0.003:
            return 'balle voisine'
    return None
