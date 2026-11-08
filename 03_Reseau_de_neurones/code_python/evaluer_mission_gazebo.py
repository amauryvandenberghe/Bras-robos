"""Validation dans Gazebo du reseau complete pour le parcours (modeles/mission/politique_mission.pt).
Reprend la boucle d'evaluer_receptacles.py, mais ecrit ses resultats dans modeles/mission/ : le resultat officiel du
reseau final (modeles/eval_gazebo.json, utilise par le PDF) n'est pas touche.

  --scene officielle : la scene d'origine (graine 40 000 000), a comparer au reseau final (20/20, 60/60)
  --scene devant     : reglette DEVANT le robot (-35 a +35 degres, la ou le reseau final ratait tout) ;
                       2 balles par sequence, chacune dans un petit receptacle (le grand reste hors de portee)
  --collecter        : le reseau pilote, l'expert note ses corrections (vraie physique) dans
                       modeles/mission/donnees_gazebo_mission.npz (pour affiner_mission.py)
Usage (Gazebo sans fenetre deja lance) : python3 evaluer_mission_gazebo.py 20 --scene devant
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy

from cinematique import SERVO_OUVERT
from evaluer_receptacles import episode
from expert import Expert
from pilote_gazebo import Pilote
from politique import Politique
from scene_gazebo import MONDE, apres_depot, disposer, ranger, viser
from zone import (GRAND, LACET_MAX, LARG_REGLETTE, LONG_REGLETTE, N_TROUS, PAS_REGLETTE, PETIT, R_BALLES, R_PETITS,
                  TYPES, Z_BALLE, distance_robot, point, segment_points, tirer_scene)

ICI = os.path.dirname(os.path.abspath(__file__))
DOSSIER = os.path.join(ICI, 'modeles', 'mission')
ANGLE_DEVANT = np.radians(35)
LOIN = (-0.6, -0.6)                   # grand receptacle hors de portee, derriere le robot


def scene_devant(rng):
    """Reglette devant le robot ; deux petits receptacles sur les cotes. Meme format que tirer_scene."""
    while True:
        a_r = rng.uniform(-ANGLE_DEVANT, ANGLE_DEVANT)
        psi = a_r + rng.uniform(-np.radians(15), np.radians(15))
        u = np.array([np.cos(psi), np.sin(psi)])
        trous = np.sort(rng.choice(N_TROUS, 3, replace=False))
        centre = point(rng.uniform(0.18, 0.36), a_r)
        pos_trous = [centre + (k - 2) * PAS_REGLETTE * u for k in range(N_TROUS)]
        balles = [pos_trous[k] for k in trous]
        r_b = [np.linalg.norm(b) for b in balles]
        if min(r_b) < R_BALLES[0] or max(r_b) > R_BALLES[1]:
            continue
        if any(abs(np.arctan2(b[1], b[0])) > LACET_MAX for b in balles):
            continue
        bord_regl = segment_points(centre - u * LONG_REGLETTE / 2, centre + u * LONG_REGLETTE / 2, 20)
        if min(distance_robot(p) for p in bord_regl) < 0.012 + LARG_REGLETTE / 2:
            continue
        petits, essais = [], 0
        while len(petits) < 2 and essais < 200:
            essais += 1
            c = point(rng.uniform(*R_PETITS), rng.uniform(-LACET_MAX + 0.15, LACET_MAX - 0.15))
            if distance_robot(c) < PETIT['r_ext'] + 0.01:
                continue
            if min(np.linalg.norm(p - c) for p in bord_regl) < PETIT['r_ext'] + 0.045:
                continue
            if any(np.linalg.norm(c - p) < 2 * PETIT['r_ext'] + 0.01 for p in petits):
                continue
            petits.append(c)
        if len(petits) < 2:
            continue
        ordre = rng.permutation(3)
        recs = sorted([(petits[0], 1), (petits[1], 2)], key=lambda r: np.linalg.norm(r[0] - centre))
        recs.append((np.array(LOIN), 3))
        return dict(receptacles=[r[0] for r in recs], modeles=[r[1] for r in recs],
                    types=[TYPES[r[1] - 1] for r in recs], reglette=(centre, psi),
                    trous=[int(trous[i]) for i in ordre], balles=[np.r_[balles[i], Z_BALLE] for i in ordre])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('n', nargs='?', type=int, default=20)
    ap.add_argument('--scene', choices=('officielle', 'devant'), default='devant')
    ap.add_argument('--modele', default=os.path.join(DOSSIER, 'politique_mission.pt'))
    ap.add_argument('--graine', type=int, default=None)
    ap.add_argument('--reprendre', action='store_true')
    ap.add_argument('--collecter', action='store_true')
    args = ap.parse_args()
    graine = args.graine if args.graine is not None else (40_000_000 if args.scene == 'officielle' else 41_000_000)
    tirer = tirer_scene if args.scene == 'officielle' else scene_devant
    n_balles = 3 if args.scene == 'officielle' else 2

    os.makedirs(DOSSIER, exist_ok=True)
    cle = dict(scene=args.scene, modele=os.path.abspath(args.modele), graine=graine, n=args.n, collecter=args.collecter)
    nom_modele = os.path.splitext(os.path.basename(args.modele))[0]
    suffixe = args.scene if nom_modele == 'politique_mission' else f'{args.scene}_{nom_modele}'
    if args.collecter:
        suffixe = f'collecte_{suffixe}'
    chemin_avancement = os.path.join(DOSSIER, f'avancement_{suffixe}.json')
    bilans, avancement = [], None
    if args.reprendre and os.path.exists(chemin_avancement):
        with open(chemin_avancement) as f:
            avancement = json.load(f)
        if avancement['cle'] == cle:
            bilans = avancement['bilans']
            print(f'reprise apres {len(bilans)} sequences deja faites', flush=True)
    if len(bilans) >= args.n:
        return 0

    rclpy.init()
    pilote = Pilote(MONDE)
    if pilote.q_mesure is None or 'balle_1' not in pilote.suivi.positions:
        print('ARRET : Gazebo ne repond pas -> relancer Gazebo')
        pilote.suivi.arreter()
        rclpy.try_shutdown()
        return 3
    politique = Politique(args.modele)
    expert = Expert(pilote.cine)
    rng = np.random.default_rng(graine)
    rng_expert = np.random.default_rng(graine + 1)
    collecte = [] if args.collecter else None
    mal_placees, arret = 0, False
    suivante = avancement['suivante'] if bilans else 0
    k_seq = -1
    try:
        while len(bilans) < args.n:
            k_seq += 1
            while True:
                scene = tirer(rng)
                if all(expert.preparer(scene, k, expert.q_repos) for k in range(n_balles)):
                    break
            if k_seq < suivante:
                continue
            pilote.aller(pilote.q_mesure, SERVO_OUVERT, 1.0, precision=1.0)
            ranger()
            pilote.attendre_sim(0.5)
            pilote.aller(expert.q_repos, SERVO_OUVERT, 3.0)
            disposer(scene)
            pilote.attendre_sim(1.5)
            if any(np.linalg.norm(pilote.suivi.positions.get(f'balle_{i + 1}', np.zeros(3))[:2] - scene['balles'][i][:2]) > 0.01
                   for i in range(3)):
                print(f'sequence {k_seq + 1}: balles mal placees, ignoree', flush=True)
                mal_placees += 1
                if mal_placees >= 3:
                    print('ARRET : Gazebo ne repond plus (3 dispositions ratees de suite) -> relancer Gazebo', flush=True)
                    arret = True
                    break
                continue
            mal_placees = 0
            res, ecarts = [], np.zeros(4)
            t = time.time()
            for k in range(n_balles):
                if k > 0:
                    pilote.aller(expert.q_repos, SERVO_OUVERT, 3.0)
                q, servo = expert.q_repos.copy(), SERVO_OUVERT
                viser(scene, k)
                pilote.attendre_sim(0.3)
                if args.collecter:
                    expert.preparer(scene, k, q, pilote.suivi.positions[f'balle_{k + 1}'])
                r, pas, q, servo, ecart = episode(pilote, scene, k, q, servo, politique,
                                                  expert if args.collecter else None, collecte, 0.0, rng_expert)
                ecarts = np.maximum(ecarts, ecart)
                res.append((r, pas))
                if r != 'reussite':
                    break
                apres_depot(scene, k, pilote.suivi.positions[f'balle_{k + 1}'].copy())
            ok = len(res) == n_balles and all(r == 'reussite' for r, _ in res)
            angles = [round(float(np.degrees(np.arctan2(b[1], b[0]))), 1) for b in scene['balles'][:n_balles]]
            bilans.append(dict(scene=dict(receptacles=[np.round(c, 3).tolist() for c in scene['receptacles']],
                                          balles=[np.round(b, 3).tolist() for b in scene['balles']]),
                               angles_balles_deg=angles, resultats=[r for r, _ in res], reussite=ok,
                               duree_s=round(time.time() - t, 1), ecart_servos_deg=np.round(np.degrees(ecarts), 1).tolist()))
            detail = ', '.join(f'balle {j + 1}: {r} ({p / 10:.1f} s)' for j, (r, p) in enumerate(res))
            print(f'sequence {k_seq + 1}: {"REUSSIE" if ok else "ratee  "}  {detail}  [angles balles {angles} deg]',
                  flush=True)
            with open(chemin_avancement, 'w') as f:
                json.dump(dict(cle=cle, bilans=bilans, suivante=k_seq + 1), f)
            if any(r == 'bloquee' for r, _ in res):
                print('ARRET : la simulation s est figee -> relancer Gazebo', flush=True)
                arret = True
                break
        if not arret:
            pilote.aller(expert.q_repos, SERVO_OUVERT, 3.0)
    finally:
        pilote.suivi.arreter()
        rclpy.try_shutdown()
    n_ok = sum(b['reussite'] for b in bilans)
    n_reussies = sum(r == 'reussite' for b in bilans for r in b['resultats'])
    print(f'BILAN {args.scene} : {n_ok}/{len(bilans)} sequences reussies, '
          f'{n_reussies}/{n_balles * len(bilans)} balles dans leur receptacle')
    if collecte:
        chemin = os.path.join(DOSSIER, 'donnees_gazebo_mission.npz')
        X, Y = np.array([o for o, _ in collecte]), np.array([a for _, a in collecte])
        if os.path.exists(chemin):
            d = np.load(chemin)
            X, Y = np.concatenate([d['X'], X]), np.concatenate([d['Y'], Y])
        np.savez_compressed(chemin, X=X, Y=Y)
        print(f'{len(collecte)} corrections ajoutees ({len(X)} au total) -> {chemin}', flush=True)
    with open(os.path.join(DOSSIER, f'eval_gazebo_{suffixe}.json'), 'w') as f:
        json.dump(dict(modele=args.modele, scene=args.scene, graine=graine, reussite=n_ok / max(1, len(bilans)),
                       sequences=bilans), f, indent=2)
    return 3 if arret else 0


if __name__ == '__main__':
    raise SystemExit(main())
