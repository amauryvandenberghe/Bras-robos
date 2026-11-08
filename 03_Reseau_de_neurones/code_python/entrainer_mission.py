"""Complement d'apprentissage pour le parcours reel (programme ESP32 de mission_esp32/).

Le reseau final (politique_finale.pt) n'a jamais vu de balle droit devant le robot (le grand receptacle etait
toujours en face, la reglette a 40 degres de lui) : 0 % de reussite a +-20 degres (mission_esp32/essai_angles.py).
Or en fin de parcours les balles sont devant le robot. Ce complement part du reseau final et ajoute, par DAgger :
  - des PRISES : balle dans toute direction (-85 a +85 degres), sur la reglette ou posee par terre, seule ou avec
    des voisines ; petit receptacle vise (pendant la prise, l'ESP32 donne une cible provisoire) ;
  - des DEPOTS en partant du repos, balle deja tenue (l'ESP32 replie le bras avec la balle pendant que le robot
    roule, puis depose) : petit receptacle n'importe ou, ou grand receptacle gare a 20,5-25 cm (pilotage manuel) ;
  - un quart d'essais de la scene d'origine, pour ne rien oublier.
Les anciens exemples (donnees.npz) et les corrections Gazebo (x5) restent dans l'apprentissage.

Usage : OUVRIERS=5 nice -n 10 python3 -u -W ignore entrainer_mission.py [tours]   (5 tours : ~1 h 15)
        RAPIDE=1 ... : verification de la chaine en quelques minutes (dossier modeles/mission_rapide)
Sortie : modeles/mission/politique_mission.pt (meilleur reseau), mission.log, historique.json, donnees_mission.npz
"""
import json
import multiprocessing
import os
import sys
import time

import numpy as np
import torch

import entrainer_imitation as ei
from cinematique import SERVO_FERME, SERVO_OUVERT
from expert import Expert
from politique import Reseau
from sim_cinematique import SimReceptacles
from zone import GRAND, PETIT, R_BALLE, Z_BALLE, distance_robot, segment_points, tirer_scene

ICI = os.path.dirname(os.path.abspath(__file__))
DOSSIER = os.path.join(ICI, 'modeles', 'mission')
OUVRIERS = int(os.environ.get('OUVRIERS', 5))
N_EVAL, N_TOUR0, N_TOUR, N_ORIGINE = 160, 1000, 700, 240
if os.environ.get('RAPIDE'):          # verification de la chaine en quelques minutes (ne pas garder le resultat)
    N_EVAL, N_TOUR0, N_TOUR, N_ORIGINE = 10, 12, 8, 12
    DOSSIER += '_rapide'
D_GRAND_MISSION = (0.205, 0.25)

_sim = _expert = _reseau = None


def journal(texte):
    ligne = f'[{time.strftime("%H:%M:%S")}] {texte}'
    print(ligne, flush=True)
    with open(os.path.join(DOSSIER, 'mission.log'), 'a') as f:
        f.write(ligne + '\n')


def _pt(r, a):
    return np.array([r * np.cos(a), r * np.sin(a)])


def _balles(rng):
    """Reglette (3 balles, z reglette) ou balles posees par terre (1 a 3), dans toute direction. Renvoie
    (balles, points de la reglette pour les distances, ou les balles elles-memes)."""
    lim = np.radians(83)
    while True:
        if rng.random() < 0.5:
            a_r = rng.uniform(-lim + 0.08, lim - 0.08)
            psi = a_r + np.radians(rng.uniform(-15, 15))
            u = np.array([np.cos(psi), np.sin(psi)])
            centre = _pt(rng.uniform(0.18, 0.36), a_r)
            trous = rng.choice(5, 3, replace=False)
            balles = [np.r_[centre + (t - 2) * 0.05 * u, Z_BALLE] for t in trous]
            bord = segment_points(centre - u * 0.135, centre + u * 0.135, 20)
            if min(distance_robot(p) for p in bord) < 0.012 + 0.015:
                continue
        else:
            balles = []
            for _ in range(int(rng.integers(1, 4))):
                b = np.r_[_pt(rng.uniform(0.19, 0.35), rng.uniform(-lim, lim)), R_BALLE]
                if all(np.linalg.norm(b[:2] - c[:2]) > 0.045 for c in balles):
                    balles.append(b)
            bord = [b[:2] for b in balles]
        r = [np.hypot(*b[:2]) for b in balles]
        if min(r) >= 0.19 and max(r) <= 0.35 and all(abs(np.arctan2(b[1], b[0])) <= lim for b in balles):
            return balles, bord


def _petit(rng, eviter):
    for _ in range(200):
        c = _pt(rng.uniform(0.21, 0.34), rng.uniform(-np.radians(76), np.radians(76)))
        if distance_robot(c) >= PETIT['r_ext'] + 0.01 and all(np.linalg.norm(p - c) >= PETIT['r_ext'] + 0.045
                                                              for p in eviter):
            return c
    return None


def scene_mission(rng):
    """(type, scene) : 'origine', 'prise' ou 'depot' (depart du repos, balle tenue)."""
    t = rng.random()
    if t < 0.25:
        return 'origine', tirer_scene(rng)
    balles, bord = _balles(rng)
    i = int(rng.integers(len(balles)))
    balles = [balles[i]] + [b for j, b in enumerate(balles) if j != i]
    if t < 0.65 or rng.random() < 0.6:
        c = _petit(rng, bord)
        if c is None:
            return None
        return ('prise' if t < 0.65 else 'depot'), dict(receptacles=[c], types=[PETIT], balles=balles)
    g = _pt(rng.uniform(*D_GRAND_MISSION), np.radians(rng.uniform(-15, 15)))
    return 'depot', dict(receptacles=[g], types=[GRAND], balles=balles[:1])


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


def _depart(rng):
    while True:
        tirage = scene_mission(rng)
        if tirage is None:
            continue
        genre, scene = tirage
        k = int(rng.choice(3)) if genre == 'origine' else 0
        q0 = _expert.q_repos.copy()
        if rng.random() < 0.33:
            q0 = np.clip(q0 + rng.normal(0, 0.06 if genre == 'depot' else 0.08, 4), _sim.cine.bas, _sim.cine.haut)
        if _expert.preparer(scene, k, q0):
            return genre, scene, k, q0


def _essai(args):
    graine, part_expert, enregistrer = args
    rng = np.random.default_rng(graine)
    genre, scene, k, q0 = _depart(rng)
    obs = _sim.reset(scene, k, q0, SERVO_FERME if genre == 'depot' else SERVO_OUVERT, graine=graine)
    if genre == 'depot':                      # balle deja serree dans la pince (prise faite avant de rouler)
        d = rng.normal(size=3)
        _sim.tenue, _sim.z_prise, _sim.z_sol_balle = True, -1.0, scene['balles'][0][2]
        _sim.decalage = d / np.linalg.norm(d) * rng.uniform(0.0015, 0.007)
        _sim.balle = _sim.outil_reel() + _sim.decalage
        obs = _sim.obs()
    X, Y, fini = [], [], False
    while not fini:
        expert_pilote = _reseau is None or rng.random() < part_expert
        a_expert = _expert.action(_sim.q, _sim.servo, _sim.balle, _sim.tenue) if enregistrer or expert_pilote else None
        if enregistrer:
            X.append(obs)
            Y.append(a_expert.astype(np.float32))
        a = a_expert if expert_pilote else _reseau(torch.from_numpy(obs)[None])[0].detach().numpy()
        obs, fini = _sim.step(a)
    return X, Y, (_sim.resultat, genre)


def jouer(poids, graines, part_expert, enregistrer=True):
    with multiprocessing.get_context('spawn').Pool(OUVRIERS, initializer=_init_ouvrier, initargs=(poids,)) as pool:
        return pool.map(_essai, [(g, part_expert, enregistrer) for g in graines], chunksize=4)


def bilan(evals):
    par = {}
    for _, _, (r, g) in evals:
        par.setdefault(g, []).append(r == 'reussite')
    return {g: round(float(np.mean(v)), 3) for g, v in sorted(par.items())}


def main():
    tours = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    os.makedirs(DOSSIER, exist_ok=True)
    poids = torch.load(os.path.join(ICI, 'modeles', 'politique_finale.pt'), map_location='cpu')
    reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    reseau.load_state_dict(poids)
    ancien = np.load(os.path.join(ICI, 'modeles', 'donnees.npz'))
    gz = np.load(os.path.join(ICI, 'modeles', 'donnees_gazebo.npz'))
    Xm, Ym, historique = [], [], []
    graines_eval = list(range(20_000_000, 20_000_000 + N_EVAL))
    journal(f'=== Complement "mission" : {tours} tours, {OUVRIERS} processus ; {len(ancien["X"])} anciens exemples + '
            f'{len(gz["X"])} corrections Gazebo (x5) ===')
    b0 = bilan(jouer({k: v.clone() for k, v in reseau.state_dict().items()}, graines_eval, 0.0, False))
    journal(f'  reseau final AVANT complement, essais mission : {b0}')
    meilleur, graine = -1.0, 3_000_000
    for tour in range(tours):
        t = time.time()
        part = 0.5 if tour == 0 else 0.0
        n = N_TOUR0 if tour == 0 else N_TOUR
        journal(f'Tour {tour} : {n} essais pilotes par le reseau ({100 - part * 100:.0f} % des pas), corriges par l\'expert...')
        for X, Y, _ in jouer({k: v.clone() for k, v in reseau.state_dict().items()}, range(graine, graine + n), part):
            Xm += X
            Ym += Y
        graine += n
        X = np.concatenate([ancien['X'], gz['X'], np.array(Xm)])
        Y = np.concatenate([ancien['Y'], gz['Y'], np.array(Ym)])
        source = np.concatenate([np.ones(len(ancien['X'])), np.full(len(gz['X']), 5.0),
                                 np.full(len(Xm), 2.0)]).astype(np.float32)
        journal(f'  {len(Xm)} exemples "mission" ({len(X)} au total) ; apprentissage...')
        perte = ei.entrainer(reseau, X, Y, epoques=1 if os.environ.get('RAPIDE') else 10, lr=3e-4,
                             poids_source=source)
        etat = {k: v.clone() for k, v in reseau.state_dict().items()}
        b = bilan(jouer(etat, graines_eval, 0.0, False))
        orig = ei.bilan_eval(jouer_origine(etat))[0]
        note = min(b.get('prise', 0), b.get('depot', 0)) * 0.7 + orig * 0.3
        journal(f'  erreur {perte:.4f} | essais mission {b} | scene d\'origine (240 essais) {orig * 100:.0f} % '
                f'({time.time() - t:.0f} s)')
        historique.append(dict(tour=tour, perte=perte, mission=b, origine=orig, duree_s=round(time.time() - t)))
        torch.save(etat, os.path.join(DOSSIER, f'politique_mission_tour{tour}.pt'))
        if note >= meilleur and orig >= 0.97:
            meilleur = note
            torch.save(etat, os.path.join(DOSSIER, 'politique_mission.pt'))
            journal('  -> meilleur jusqu\'ici : modeles/mission/politique_mission.pt')
        with open(os.path.join(DOSSIER, 'historique.json'), 'w') as f:
            json.dump(historique, f, indent=2, ensure_ascii=False)
        np.savez_compressed(os.path.join(DOSSIER, 'donnees_mission.npz'), X=np.array(Xm), Y=np.array(Ym))
    journal('=== Fin du complement ===')


def jouer_origine(poids):
    """Examen sur la scene d'origine (memes 240 graines que entrainer_imitation.py)."""
    ei.OUVRIERS = OUVRIERS
    return ei.jouer(poids, list(range(10_000_000, 10_000_000 + N_ORIGINE)), 0.0, enregistrer=False)


if __name__ == '__main__':
    main()
