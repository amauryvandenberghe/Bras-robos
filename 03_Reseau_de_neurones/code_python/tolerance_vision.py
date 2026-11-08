"""Quelle precision la camera doit-elle donner ? Le reseau joue dans le simulateur rapide en voyant la balle avec
une erreur constante (comme une camera mal etalonnee) de E mm, dans une direction horizontale tiree au hasard.

Usage : python3 tolerance_vision.py [modele] [n essais par niveau]   (3 ouvriers, nice)
"""
import multiprocessing
import os
import sys
import numpy as np
import torch

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
from cinematique import Bras  # noqa: E402
from expert import Expert  # noqa: E402
from politique import Reseau, observation  # noqa: E402
import sim_cinematique  # noqa: E402
from entrainer_imitation import depart  # noqa: E402

ERREURS_MM = (0, 3, 6, 9)


class SimCamera(sim_cinematique.SimReceptacles):
    biais = np.zeros(3)

    def obs(self):
        vue = self.balle + self.biais + self.rng.normal(0, sim_cinematique.BRUIT_VUE, 3)
        return observation(self.q, self.servo, vue, self.cible, self.outil(), self.grand)


def _init(poids):
    global _sim, _expert, _reseau
    torch.set_num_threads(1)
    _sim = SimCamera(Bras())
    _expert = Expert(_sim.cine)
    _reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    _reseau.load_state_dict(poids)
    _reseau.eval()


def _essai(args):
    graine, erreur = args
    rng = np.random.default_rng(graine)
    scene, k, q0, servo = depart(rng, _sim, _expert)
    a = rng.uniform(0, 2 * np.pi)
    _sim.biais = erreur / 1000 * np.array([np.cos(a), np.sin(a), 0.0])
    obs = _sim.reset(scene, k, q0, servo, graine=graine)
    fini = False
    while not fini:
        with torch.no_grad():
            act = _reseau(torch.from_numpy(obs)[None])[0].numpy()
        obs, fini = _sim.step(act)
    return erreur, _sim.resultat


if __name__ == '__main__':
    modele = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ICI, 'modeles', 'politique_bras.pt')
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    poids = torch.load(modele)
    taches = [(20_000_000 + i, e) for e in ERREURS_MM for i in range(n)]
    with multiprocessing.get_context('spawn').Pool(3, initializer=_init, initargs=(poids,)) as pool:
        res = pool.map(_essai, taches, chunksize=2)
    lignes = []
    for e in ERREURS_MM:
        r = [x for ee, x in res if ee == e]
        echecs = {k: r.count(k) for k in set(r) if k != 'reussite'}
        lignes.append(f'erreur de vision {e} mm : {100 * r.count("reussite") / len(r):.0f} % de balles deposees ({len(r)} essais) {echecs}')
        print(lignes[-1], flush=True)
    open(os.path.join(ICI, 'modeles', 'tolerance_vision.txt'), 'w').write(os.path.basename(modele) + '\n' + '\n'.join(lignes) + '\n')
