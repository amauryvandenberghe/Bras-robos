"""Verifie l'expert seul dans le simulateur rapide : sequences completes (3 balles) sur des dispositions au hasard.
Usage : python3 essai_expert.py [n] [--graine G] [--detail]"""
import argparse
import time
from collections import Counter

import numpy as np

from cinematique import SERVO_OUVERT
from expert import Expert
from sim_cinematique import SimReceptacles
from zone import tirer_scene


def sequence(sim, expert, scene, graine, detail=False):
    q, servo, res = expert.q_repos.copy(), SERVO_OUVERT, []
    for k in range(3):
        if not expert.preparer(scene, k, q):
            res.append(('plan impossible', expert.raison))
            break
        sim.reset(scene, k, q, servo, graine=graine + k)
        fini = False
        while not fini:
            _, fini = sim.step(expert.action(sim.q, sim.servo, sim.balle, sim.tenue))
        res.append((sim.resultat, sim.pas))
        if detail:
            choc = f' ({sim.choc}, pince {np.round(sim.outil_reel(), 3)}, q {np.round(sim.q_reel, 2)})' \
                if sim.resultat == 'collision' else ''
            print(f'   balle {k + 1}: {sim.resultat} en {sim.pas / 10:.1f} s{choc}')
        if sim.resultat != 'reussite':
            break
        q, servo = expert.q_repos.copy(), SERVO_OUVERT     # le robot roule jusqu'au receptacle suivant, bras replie
    return res


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('n', nargs='?', type=int, default=30)
    ap.add_argument('--graine', type=int, default=0)
    ap.add_argument('--detail', action='store_true')
    args = ap.parse_args()
    sim = SimReceptacles()
    expert = Expert(sim.cine)
    rng = np.random.default_rng(args.graine)
    bilan, t = Counter(), time.time()
    ok = 0
    for i in range(args.n):
        scene = tirer_scene(rng)
        res = sequence(sim, expert, scene, args.graine * 1000 + i * 10, args.detail)
        reussie = len(res) == 3 and all(r == 'reussite' for r, _ in res)
        ok += reussie
        for k, (r, info) in enumerate(res):
            bilan[f'balle {k + 1}: {r}'] += 1
        if not reussie:
            print(f'sequence {i}: {res}', flush=True)
    print(f'{ok}/{args.n} sequences reussies ({(time.time() - t) / args.n:.1f} s par sequence)')
    for k, v in sorted(bilan.items()):
        print(f'  {k}: {v}')
