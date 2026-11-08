"""Le retour a la posture de repos ("home") est-il sur ? Le reseau joue des balles dans le simulateur rapide ; a chaque
lacher (et a un instant pris au hasard, comme un ordre R en pleine prise), on ramene le bras vers le repos pas a pas
(0,06 rad au plus par servo et par pas, comme sur l'ESP32), depuis la position REELLE des servos, et on cherche une
collision avec le receptacle vise, le robot ou le sol.

Strategies comparees :
  ensemble    : les 4 servos vont ensemble vers le repos ;
  lacet_fin   : epaule, coude, poignet d'abord, puis le lacet ;
  par_le_haut : comme lacet_fin, mais si la pince est haute (grand receptacle), on passe d'abord par le point haut
                du passage interieur (8 cm de l'axe, 56 cm de haut), comme l'expert pour y entrer.
Usage : python3 essai_retour_repos.py [n essais]
"""
import multiprocessing
import os
import sys
import numpy as np
import torch

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
from cinematique import Bras  # noqa: E402
from expert import Expert, GRAND_HAUT, interieur  # noqa: E402
from obstacles import heurte  # noqa: E402
from politique import PAS_SERVO, Reseau, dans_robot, Z_MIN_OUTIL  # noqa: E402
from sim_cinematique import SimReceptacles, SUIVI_SERVO  # noqa: E402
from entrainer_imitation import depart  # noqa: E402

STRATEGIES = {'ensemble': [[0, 1, 2, 3]], 'lacet_fin': [[1, 2, 3], [0]], 'par_le_haut': [[1, 2, 3], [0]],
              'a_rebours': [[1, 2, 3], [0]]}
R_RENTRE, Z_RENTRE = 0.10, 0.31   # a_rebours : on rejoue les consignes a l'envers jusqu'au passage interieur bas
N_HISTO = 300          # consignes gardees en memoire (30 s, 4,8 Ko sur l'ESP32)


def a_rebours(histo, cine):
    """Consignes deja envoyees, rejouees a l'envers, jusqu'a ce que la pince soit revenue au passage interieur
    (pres de l'axe et basse) : le chemin aller n'a rien touche, le meme chemin a l'envers non plus."""
    for q in reversed(histo[-N_HISTO:]):
        yield np.array(q, float)
        o = cine.points_bras(q)[2]
        if np.hypot(*o[:2]) < R_RENTRE and o[2] < Z_RENTRE:
            return
Z_HAUT = 0.40          # pince plus haute que ca : on est au grand receptacle


def q_haut_interieur(cine):
    """Posture (lacet 0) ou la pince est au point haut du passage interieur, inclinee comme pour entrer dans le
    grand receptacle : c'est par la que l'expert y entre."""
    q, _ = cine.ik(interieur(0.0, 0.56), np.array([0.0, -0.1, 1.0, 0.45]), *GRAND_HAUT)
    return q


def chemin(q, q_repos, strategie, cine, q_haut, histo=()):
    q = np.array(q, float)
    if strategie == 'a_rebours':
        for q in a_rebours(histo, cine):
            yield q.copy()
    etapes = [(j, q_repos) for j in STRATEGIES[strategie]]
    if strategie == 'par_le_haut' and cine.points_bras(q)[2][2] > Z_HAUT:
        etapes = [([1, 2, 3], q_haut)] + etapes
    for joints, but in etapes:
        while True:
            d = but[joints] - q[joints]
            if np.all(np.abs(d) < 1e-9):
                break
            q[joints] += np.clip(d, -PAS_SERVO, PAS_SERVO)
            yield q.copy()


def probleme(cine, q, scene, k):
    coude, poignet, outil = cine.points_bras(q)
    if outil[2] < Z_MIN_OUTIL or dans_robot(outil) or dans_robot(poignet, 0.0):
        return 'robot/sol'
    if heurte(cine, q, scene, k):
        return 'receptacle'
    return None


def _init(poids):
    global _sim, _expert, _reseau, _q_haut
    torch.set_num_threads(1)
    _sim = SimReceptacles(Bras())
    _expert = Expert(_sim.cine)
    _q_haut = q_haut_interieur(_sim.cine)
    _reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    _reseau.load_state_dict(poids)
    _reseau.eval()


def _essai(graine):
    rng = np.random.default_rng(graine)
    scene, k, q0, servo = depart(rng, _sim, _expert)
    obs = _sim.reset(scene, k, q0, servo, graine=graine)
    instant_r = int(rng.integers(5, 150))
    departs, fini, histo = [], False, [_sim.q.copy()]
    while not fini:
        with torch.no_grad():
            a = _reseau(torch.from_numpy(obs)[None])[0].numpy()
        obs, fini = _sim.step(a)
        histo.append(_sim.q.copy())
        if _sim.pas == instant_r and not fini:
            departs.append(('ordre R en cours', _sim.q_reel.copy(), list(histo)))
    if _sim.resultat == 'reussite':
        departs.append(('apres lacher', _sim.q_reel.copy(), list(histo)))
    res = []
    for quand, q, h in departs:
        for strat in STRATEGIES:
            p = 'deja en contact au depart' if probleme(_sim.cine, q, scene, k) else None
            q_reel = q.copy()                  # les servos suivent les consignes avec retard (comme le simulateur)
            consignes = [] if p else list(chemin(q, _expert.q_repos, strat, _sim.cine, _q_haut, h))
            for qq in consignes + consignes[-1:] * 10:
                q_reel += SUIVI_SERVO * (qq - q_reel)
                p = probleme(_sim.cine, q_reel, scene, k)
                if p:
                    break
            res.append((quand, strat, p))
    return res


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    poids = torch.load(os.path.join(ICI, 'modeles', 'politique_finale.pt'))
    with multiprocessing.get_context('spawn').Pool(4, initializer=_init, initargs=(poids,)) as pool:
        tout = [r for rs in pool.map(_essai, range(30_000_000, 30_000_000 + n)) for r in rs]
    print('q_haut_interieur =', np.round(q_haut_interieur(Bras()), 4).tolist())
    for quand in ('apres lacher', 'ordre R en cours'):
        for strat in STRATEGIES:
            r = [p for qd, s, p in tout if qd == quand and s == strat]
            pb = {x: r.count(x) for x in set(r) if x}
            print(f'{quand:17s} {strat:12s} : {len(r) - sum(pb.values())}/{len(r)} retours sans choc {pb}')
