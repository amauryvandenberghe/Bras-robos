"""Bras 50 cm dans Gazebo : trois balles dans trois receptacles (deux a 15 cm, un a 50 cm), a la suite, du receptacle le
plus proche des balles au plus eloigne. Disposition tiree au hasard a chaque sequence.
Comme sur le parcours (le robot se gare devant un receptacle a la fois, bras replie entre deux) : chaque balle part du
repos, seul le receptacle vise est solide ; les autres sont affiches en fantomes, avec les balles deja deposees.

Gazebo : ros2 launch mon_bras_robot_moveit_config gazebo.launch.py robot:=bras_50cm monde:=scene_trois_receptacles
  python3 evaluer_receptacles.py 20             : 20 sequences pilotees par le reseau
  python3 evaluer_receptacles.py 5 --expert     : c'est l'expert qui pilote (verification de la physique)
  python3 evaluer_receptacles.py 20 --collecter : le reseau pilote, l'expert note ses corrections (vraie physique)
Une sequence est reussie si les trois balles finissent dans leur receptacle.
Affiche aussi l'ecart maxi entre consigne et angle reel de chaque servo : un servo trop faible pour le poids
du bras (couple maxi = celui du vrai servo dans l'URDF) prendrait du retard ou s'affaisserait.
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy

from cinematique import SERVO_CONTACT, SERVO_OUVERT
from expert import Expert
from obstacles import heurte
from pilote_gazebo import Pilote
from politique import PERIODE, Politique, observation, securite, servo_suivant
from scene_gazebo import MONDE, apres_depot, disposer, ranger, viser
from zone import GRAND, Z_BALLE, cible_lacher, dans_receptacle, tirer_scene

ICI = os.path.dirname(os.path.abspath(__file__))
DOSSIER = os.path.join(ICI, 'modeles')
PAS_MAX = 700
TRACE = False


def episode(pilote, scene, k, q, servo, politique=None, expert=None, collecte=None, part_expert=0.0, rng=None):
    """Balle k (0, 1, 2) -> receptacle k. Renvoie (resultat, pas, q, servo, ecart maxi par servo)."""
    nom = f'balle_{k + 1}'
    centre, t = np.asarray(scene['receptacles'][k], float), scene['types'][k]
    cible, grand = cible_lacher(scene, k), t is GRAND
    autres = [b for j, b in enumerate(scene['balles']) if j > k]
    depart = pilote.suivi.positions[nom].copy()
    levee, lachee, pose_depuis, figee = False, False, None, 0
    ecart_max = np.zeros(4)
    # horloge a jour d'abord : partir d'une heure perimee enverrait une rafale de consignes sans attendre le bras
    pilote.attendre_sim(0.05)
    prochain = pilote.t_sim
    for pas in range(1, PAS_MAX + 1):
        balle = pilote.suivi.positions[nom].copy()
        outil = pilote.cine.outil(q)[0]
        obs = observation(q, servo, balle, cible, outil, grand)
        a_expert = None
        if expert is not None:
            tenue = servo <= SERVO_CONTACT and (balle[2] > Z_BALLE + 0.008 or np.linalg.norm(balle - outil) < 0.008)
            a_expert = expert.action(q, servo, balle, tenue)
        if collecte is not None:
            collecte.append((obs, a_expert.astype(np.float32)))
        if politique is None or (collecte is not None and rng.random() < part_expert):
            action = a_expert
        else:
            action = politique(obs)
        if lachee:
            # balle lachee en l'air : comme dans le simulateur rapide (l'essai s'y arrete au lacher), le bras ne bouge
            # plus pendant qu'on regarde ou elle tombe (sinon il repartirait la chercher dans le receptacle)
            action = np.r_[np.zeros(4), 1.0]
        q, pince = securite(pilote.cine, q, action, servo)
        servo = servo_suivant(servo, pince)
        pilote.commander(q, servo, PERIODE)
        prochain += PERIODE
        figee = 0 if pilote.attendre_jusqua(prochain) else figee + 1
        if figee >= 3:
            return 'bloquee', pas, q, servo, ecart_max
        ecart = np.abs(pilote.q_mesure - q)
        if ecart.max() > max(ecart_max.max(), np.radians(15)):
            print(f'    [balle {k + 1}] pas {pas}: ecart {np.round(np.degrees(ecart), 1)} deg, consigne {np.round(q, 2)}, '
                  f'mesure {np.round(pilote.q_mesure, 2)}, pince {np.round(pilote.cine.outil(q)[0], 3)}, '
                  f'servo {servo:+.2f}, balle {np.round(pilote.suivi.positions[nom], 3)}', flush=True)
        ecart_max = np.maximum(ecart_max, ecart)
        balle = pilote.suivi.positions[nom]
        if levee and servo > SERVO_CONTACT and np.linalg.norm(balle - pilote.cine.outil(q)[0]) > 0.02:
            lachee = True
        if TRACE and pas % 10 == 0:
            print(f'    [balle {k + 1}] pas {pas:3d} consigne {np.round(q, 2)} mesure {np.round(pilote.q_mesure, 2)} '
                  f'pince {np.round(outil, 3)} servo {servo:+.2f} balle {np.round(balle, 3)} action {np.round(action, 2)}',
                  flush=True)
        choc = heurte(pilote.cine, pilote.q_mesure, scene, k, False, autres)
        if choc:
            print(f'    choc : {choc}', flush=True)
            return 'collision', pas, q, servo, ecart_max
        if balle[2] > Z_BALLE + 0.025:
            levee = True
            if dans_receptacle(balle, centre, t):
                pose_depuis = pose_depuis or pilote.t_sim
                if pilote.t_sim - pose_depuis > 1.0:
                    return 'reussite', pas, q, servo, ecart_max
            else:
                pose_depuis = None
        elif levee and balle[2] < Z_BALLE + 0.008:
            pose_depuis = pose_depuis or pilote.t_sim
            if pilote.t_sim - pose_depuis > 1.0:
                return 'tombee', pas, q, servo, ecart_max
        elif levee:
            pose_depuis = None
        elif balle[2] < Z_BALLE + 0.003 and np.linalg.norm(balle[:2] - depart[:2]) > 0.03:
            return 'poussee', pas, q, servo, ecart_max
    return 'temps', PAS_MAX, q, servo, ecart_max


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('n', nargs='?', type=int, default=20)
    ap.add_argument('--modele', default=os.path.join(DOSSIER, 'politique_bras.pt'))
    ap.add_argument('--graine', type=int, default=40_000_000)
    ap.add_argument('--expert', action='store_true')
    ap.add_argument('--collecter', action='store_true')
    ap.add_argument('--part-expert', type=float, default=0.0)
    ap.add_argument('--reprendre', action='store_true')
    ap.add_argument('--trace', action='store_true', help='affiche consignes, angles mesures et balle tous les 10 pas')
    args = ap.parse_args()
    global TRACE
    TRACE = args.trace

    mode = 'expert' if args.expert else 'collecter' if args.collecter else 'test'
    cle = dict(mode=mode, modele=None if args.expert else os.path.abspath(args.modele), graine=args.graine, n=args.n)
    chemin_avancement = os.path.join(DOSSIER, 'avancement.json')
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
    politique = None if args.expert else Politique(args.modele)
    expert = Expert(pilote.cine)
    rng = np.random.default_rng(args.graine)
    rng_expert = np.random.default_rng(args.graine + 1)
    collecte = [] if args.collecter else None
    mal_placees, arret = 0, False
    suivante = avancement['suivante'] if bilans else 0
    k_seq = -1
    try:
        while len(bilans) < args.n:
            k_seq += 1
            while True:                       # disposition realisable par l'expert pour les trois balles
                scene = tirer_scene(rng)
                if all(expert.preparer(scene, k, expert.q_repos) for k in range(3)):
                    break
            if k_seq < suivante:
                continue
            pilote.aller(pilote.q_mesure, SERVO_OUVERT, 1.0, precision=1.0)
            ranger()                          # rien devant le bras pendant qu'il rejoint sa posture de repos
            pilote.attendre_sim(0.5)
            ecart = pilote.aller(expert.q_repos, SERVO_OUVERT, 3.0)
            if ecart > 0.05:
                print(f'  repos atteint a {np.degrees(ecart):.1f} deg pres seulement', flush=True)
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
            q, servo, res, ecarts = expert.q_repos.copy(), SERVO_OUVERT, [], np.zeros(4)
            t = time.time()
            for k in range(3):
                # le robot "roule" jusqu'au receptacle k, bras replie : seul ce receptacle devient solide
                if k > 0:
                    pilote.aller(expert.q_repos, SERVO_OUVERT, 3.0)
                q, servo = expert.q_repos.copy(), SERVO_OUVERT
                viser(scene, k)
                pilote.attendre_sim(0.3)
                if args.expert or args.collecter:
                    expert.preparer(scene, k, q, pilote.suivi.positions[f'balle_{k + 1}'])
                r, pas, q, servo, ecart = episode(pilote, scene, k, q, servo, politique,
                                                  expert if (args.expert or args.collecter) else None,
                                                  collecte, args.part_expert, rng_expert)
                ecarts = np.maximum(ecarts, ecart)
                res.append((r, pas))
                if r != 'reussite':
                    break
                apres_depot(scene, k, pilote.suivi.positions[f'balle_{k + 1}'].copy())
            ok = len(res) == 3 and all(r == 'reussite' for r, _ in res)
            bilans.append(dict(scene=dict(receptacles=[np.round(c, 3).tolist() for c in scene['receptacles']],
                                          balles=[np.round(b, 3).tolist() for b in scene['balles']]),
                               resultats=[r for r, _ in res], reussite=ok, duree_s=round(time.time() - t, 1),
                               ecart_servos_deg=np.round(np.degrees(ecarts), 1).tolist()))
            detail = ', '.join(f'balle {j + 1}: {r} ({p / 10:.1f} s)' for j, (r, p) in enumerate(res))
            print(f'sequence {k_seq + 1}: {"REUSSIE" if ok else "ratee  "}  {detail}  '
                  f'[ecart maxi servos {np.round(np.degrees(ecarts), 1)} deg]', flush=True)
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
    n_balles = sum(r == 'reussite' for b in bilans for r in b['resultats'])
    print(f'BILAN : {n_ok}/{len(bilans)} sequences reussies, {n_balles}/{3 * len(bilans)} balles dans leur receptacle')
    if collecte:
        chemin = os.path.join(DOSSIER, 'donnees_gazebo.npz')
        X, Y = np.array([o for o, _ in collecte]), np.array([a for _, a in collecte])
        if os.path.exists(chemin):
            d = np.load(chemin)
            X, Y = np.concatenate([d['X'], X]), np.concatenate([d['Y'], Y])
        np.savez_compressed(chemin, X=X, Y=Y)
        print(f'{len(collecte)} corrections ajoutees ({len(X)} au total) -> {chemin}')
    else:
        nom = 'eval_gazebo_expert.json' if args.expert else 'eval_gazebo.json'
        with open(os.path.join(DOSSIER, nom), 'w') as f:
            json.dump(dict(modele=None if args.expert else args.modele, reussite=n_ok / max(1, len(bilans)),
                           sequences=bilans), f, indent=2)
    return 3 if arret else 0


if __name__ == '__main__':
    raise SystemExit(main())
