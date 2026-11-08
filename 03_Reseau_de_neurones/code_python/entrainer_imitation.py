"""Apprentissage du reseau du bras 50 cm par imitation de l'expert, puis correction DAgger.

  Tour 0 : l'expert fait des milliers d'essais (une balle chacun : balle 1, 2 ou 3 d'une disposition au hasard,
           depart bras replie au repos), le reseau l'imite.
  Tours 1..N : le reseau pilote ; a chaque etat visite, l'expert dit ce qu'il aurait fallu faire (DAgger).
A chaque tour, le reseau est evalue seul sur des dispositions qu'il n'a jamais vues.

Usage : python3 entrainer_imitation.py                 (premier entrainement)
        python3 entrainer_imitation.py --reprendre 3    (3 tours de correction de plus)
        python3 entrainer_imitation.py --affiner        (avec les corrections collectees dans Gazebo)
Sortie : modeles/politique_bras.pt (le meilleur reseau), historique.json, donnees.npz, entrainement.log
"""
import argparse
import json
import multiprocessing
import os
import time

import numpy as np
import torch
from torch import nn

from cinematique import SERVO_OUVERT
from expert import Expert
from politique import I_SERVO, I_ZOOM_BALLE, Reseau
from sim_cinematique import SimReceptacles
from zone import tirer_scene

DOSSIER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'modeles')
TOURS = [  # (nombre d'essais, part des pas pilotes par l'expert)
    (1500, 1.0),
    (700, 0.5),
    (700, 0.25),
    (700, 0.0),
    (700, 0.0),
    (700, 0.0),
    (700, 0.0),
    (700, 0.0),
    (700, 0.0),
]
N_EVAL = 240
if os.environ.get('RAPIDE'):          # verification de la chaine complete en quelques minutes
    TOURS, N_EVAL = [(24, 1.0), (12, 0.0)], 12
OUVRIERS = int(os.environ.get('OUVRIERS', min(6, max(1, (os.cpu_count() or 2) - 2))))

_sim = _expert = _reseau = None


def journal(texte):
    ligne = f'[{time.strftime("%H:%M:%S")}] {texte}'
    print(ligne, flush=True)
    with open(os.path.join(DOSSIER, 'entrainement.log'), 'a') as f:
        f.write(ligne + '\n')


def _init_ouvrier(poids):
    global _sim, _expert, _reseau
    torch.set_num_threads(1)
    _sim = SimReceptacles()
    _expert = Expert(_sim.cine)
    _reseau = None
    if poids is not None:
        _reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
        _reseau.load_state_dict(poids)
        _reseau.eval()


def depart(rng, sim, expert):
    """Disposition, numero de balle k et posture de depart d'un essai. Chaque balle part du repos (bras replie) :
    entre deux receptacles, le robot roule bras replie. Un depart sur trois est legerement decale (le reseau doit
    savoir repartir d'une posture voisine)."""
    while True:
        scene = tirer_scene(rng)
        k = int(rng.choice(3))
        q0 = expert.q_repos.copy()
        if rng.random() < 0.33:
            q0 = np.clip(q0 + rng.normal(0, 0.08, 4), sim.cine.bas, sim.cine.haut)
        if expert.preparer(scene, k, q0):
            return scene, k, q0, SERVO_OUVERT


def _essai(args):
    """Un essai complet. Renvoie les (observation, action de l'expert) visites et le resultat."""
    graine, part_expert, enregistrer = args
    rng = np.random.default_rng(graine)
    scene, k, q0, servo = depart(rng, _sim, _expert)
    obs = _sim.reset(scene, k, q0, servo, graine=graine)
    X, Y, fini = [], [], False
    while not fini:
        expert_pilote = _reseau is None or rng.random() < part_expert
        a_expert = _expert.action(_sim.q, _sim.servo, _sim.balle, _sim.tenue) if enregistrer or expert_pilote else None
        if enregistrer:
            X.append(obs)
            Y.append(a_expert.astype(np.float32))
        if expert_pilote:
            a = a_expert
        else:
            with torch.no_grad():
                a = _reseau(torch.from_numpy(obs)[None])[0].numpy()
        obs, fini = _sim.step(a)
    return X, Y, (_sim.resultat, k)


def jouer(poids, graines, part_expert, enregistrer=True):
    with multiprocessing.get_context('spawn').Pool(OUVRIERS, initializer=_init_ouvrier, initargs=(poids,)) as pool:
        return pool.map(_essai, [(g, part_expert, enregistrer) for g in graines], chunksize=4)


def entrainer(reseau, X, Y, epoques=40, lr=1e-3, poids_source=None):
    """Apprend a imiter les actions Y dans les etats X (memes ponderations que pour le bras actuel)."""
    torch.set_num_threads(OUVRIERS)
    X, Y = torch.from_numpy(X), torch.from_numpy(Y)
    poids_sortie = torch.tensor([1, 1, 1, 1, 3.0])
    poids_exemple = 1.0 + 2.0 * (X[:, I_ZOOM_BALLE].abs().max(dim=1).values < 1.0).float()
    poids_exemple = poids_exemple * (1.0 + 2.0 * ((Y[:, 4] > 0) & (X[:, I_SERVO] < 0)).float())
    poids_exemple = poids_exemple / Y[:, :4].abs().max(dim=1).values.clamp(min=0.25)
    if poids_source is not None:
        poids_exemple = poids_exemple * torch.from_numpy(poids_source)
    opt = torch.optim.Adam(reseau.parameters(), lr=lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epoques)
    ouvrir = (Y[:, 4] > 0).float()
    for ep in range(epoques):
        perm = torch.randperm(len(X))
        total = 0.0
        for i in range(0, len(X), 1024):
            idx = perm[i:i + 1024]
            z = reseau.brut(X[idx])
            bras = ((torch.tanh(z[:, :4]) - Y[idx, :4]) ** 2).mean(dim=1)
            pince = nn.functional.binary_cross_entropy_with_logits(2 * z[:, 4], ouvrir[idx], reduction='none')
            perte = ((bras + poids_sortie[4] * pince / 5) * poids_exemple[idx]).mean()
            opt.zero_grad()
            perte.backward()
            opt.step()
            total += perte.item() * len(idx)
        sched.step()
    return total / len(X)


def bilan_eval(evals):
    res = [r for _, _, (r, _) in evals]
    par_balle = {f'balle {k + 1}': round(float(np.mean([r == 'reussite' for _, _, (r, kk) in evals if kk == k])), 3)
                 for k in range(3)}
    return res.count('reussite') / len(res), {k: res.count(k) for k in set(res) if k != 'reussite'}, par_balle


def affiner(poids_gazebo, epoques=15):
    """Affine le meilleur reseau avec les corrections de l'expert collectees dans Gazebo (vraie physique)."""
    poids = torch.load(os.path.join(DOSSIER, 'politique_bras.pt'))
    reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    reseau.load_state_dict(poids)
    sim, gz = np.load(os.path.join(DOSSIER, 'donnees.npz')), np.load(os.path.join(DOSSIER, 'donnees_gazebo.npz'))
    X = np.concatenate([sim['X'], gz['X']])
    Y = np.concatenate([sim['Y'], gz['Y']])
    source = np.concatenate([np.ones(len(sim['X'])), np.full(len(gz['X']), poids_gazebo)]).astype(np.float32)
    journal(f'=== Affinage : {len(sim["X"])} exemples du simulateur + {len(gz["X"])} corrections Gazebo (x{poids_gazebo:g}) ===')
    perte = entrainer(reseau, X, Y, epoques=epoques, lr=3e-4, poids_source=source)
    torch.save(reseau.state_dict(), os.path.join(DOSSIER, 'politique_affine.pt'))
    taux, echecs, par_balle = bilan_eval(jouer({k: v.clone() for k, v in reseau.state_dict().items()},
                                               list(range(10_000_000, 10_000_000 + N_EVAL)), 0.0, enregistrer=False))
    journal(f'  erreur {perte:.4f} ; simulateur rapide : {taux * 100:.0f} % {par_balle} -> modeles/politique_affine.pt')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reprendre', type=int, default=0, metavar='N')
    ap.add_argument('--affiner', action='store_true')
    ap.add_argument('--poids-gazebo', type=float, default=5.0)
    ap.add_argument('--reinit', action='store_true')
    ap.add_argument('--cache', type=int, default=512)
    args = ap.parse_args()
    os.makedirs(DOSSIER, exist_ok=True)
    if args.affiner:
        return affiner(args.poids_gazebo)
    X_tout, Y_tout, historique = [], [], []
    reseau, graine, meilleur = Reseau(cache=args.cache), 0, -1.0
    tours, premier = TOURS, 0
    chemin_donnees = os.path.join(DOSSIER, 'donnees.npz')
    if args.reprendre:
        if not args.reinit:
            poids = torch.load(os.path.join(DOSSIER, 'politique_bras.pt'))
            reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
            reseau.load_state_dict(poids)
        if os.path.exists(chemin_donnees):
            d = np.load(chemin_donnees)
            X_tout, Y_tout = list(d['X']), list(d['Y'])
        if os.path.exists(os.path.join(DOSSIER, 'historique.json')):
            with open(os.path.join(DOSSIER, 'historique.json')) as f:
                historique = json.load(f)
        premier = len(historique)
        graine = 1_000_000 + premier * 1000
        tours = [(700, 0.0)] * args.reprendre
        if args.reinit:
            tours = [(0, 0.0)] + tours
    graines_eval = list(range(10_000_000, 10_000_000 + N_EVAL))
    journal(f'=== Debut : {len(tours)} tours, {OUVRIERS} processus en parallele, {len(X_tout)} exemples deja acquis ===')
    for tour, (n, part_expert) in enumerate(tours, start=premier):
        t = time.time()
        qui = "l'expert" if part_expert == 1 else f"le reseau ({100 - part_expert * 100:.0f} % des pas), corrige par l'expert"
        if n:
            journal(f'Tour {tour} : {n} essais pilotes par {qui}...')
            poids = None if part_expert == 1 else {k: v.clone() for k, v in reseau.state_dict().items()}
            essais = jouer(poids, range(graine, graine + n), part_expert)
            graine += n
            for X, Y, _ in essais:
                X_tout += X
                Y_tout += Y
        else:
            journal(f'Tour {tour} : reseau neuf ({sum(p.numel() for p in reseau.parameters())} poids)')
        journal(f'  {len(X_tout)} exemples au total ; apprentissage du reseau...')
        # premier tour (reseau neuf) : 40 passages ; ensuite le reseau repart du precedent, 15 passages suffisent
        # (avec un million d'exemples, 40 passages par tour prenaient 3 heures)
        premier_apprentissage = tour == premier and not args.reprendre
        perte = entrainer(reseau, np.array(X_tout), np.array(Y_tout),
                          epoques=60 if not n else 40 if premier_apprentissage else 15)
        journal(f'  erreur d\'imitation {perte:.4f} ; examen sur {N_EVAL} essais jamais vus...')
        taux, echecs, par_balle = bilan_eval(jouer({k: v.clone() for k, v in reseau.state_dict().items()},
                                                   graines_eval, 0.0, enregistrer=False))
        bilan = dict(tour=tour, essais=n, part_expert=part_expert, exemples=len(X_tout), perte=round(perte, 5),
                     reussite=taux, par_balle=par_balle, echecs=echecs, duree_s=round(time.time() - t))
        historique.append(bilan)
        journal(f'  >>> RESEAU SEUL : {taux * 100:.0f} % de balles deposees {par_balle} (echecs : {echecs}, '
                f'{bilan["duree_s"]} s)')
        torch.save(reseau.state_dict(), os.path.join(DOSSIER, f'politique_tour{tour}.pt'))
        if taux >= meilleur:
            meilleur = taux
            torch.save(reseau.state_dict(), os.path.join(DOSSIER, 'politique_bras.pt'))
        with open(os.path.join(DOSSIER, 'historique.json'), 'w') as f:
            json.dump(historique, f, indent=2, ensure_ascii=False)
        np.savez_compressed(chemin_donnees, X=np.array(X_tout), Y=np.array(Y_tout))
    journal(f'=== Fin : meilleur reseau {meilleur * 100:.0f} % -> modeles/politique_bras.pt ===')


if __name__ == '__main__':
    main()
