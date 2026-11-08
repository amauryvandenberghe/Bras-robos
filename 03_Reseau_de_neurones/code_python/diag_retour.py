"""Detail des chocs au retour vers le repos apres un lacher au grand receptacle (voir essai_retour_repos.py)."""
import os
import sys
import numpy as np
import torch
ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
import essai_retour_repos as E  # noqa: E402
from obstacles import heurte  # noqa: E402

poids = torch.load(os.path.join(ICI, 'modeles', 'politique_finale.pt'))
E._init(poids)
cine = E._sim.cine
for g in (30_000_003, 30_000_004):
    rng = np.random.default_rng(g)
    scene, k, q0, servo = E.depart(rng, E._sim, E._expert)
    obs = E._sim.reset(scene, k, q0, servo, graine=g)
    fini, histo, reels = False, [], []
    while not fini:
        with torch.no_grad():
            a = E._reseau(torch.from_numpy(obs)[None])[0].numpy()
        obs, fini = E._sim.step(a)
        histo.append(E._sim.q.copy())
        reels.append(E._sim.q_reel.copy())
    c = np.asarray(scene['receptacles'][k])
    print(f'== graine {g} : {E._sim.resultat}, receptacle centre {np.round(c, 3)}')
    for i in range(-6, 0):
        o = cine.points_bras(reels[i])[2]
        print(f'  pas {i}: consigne {np.round(histo[i], 3)} reel {np.round(reels[i], 3)} outil {np.round(o, 3)} '
              f'choc consigne: {heurte(cine, histo[i], scene, k, True)} | choc reel: {heurte(cine, reels[i], scene, k, True)}'
              f' | reel pince ouverte: {heurte(cine, reels[i], scene, k, False)}')
