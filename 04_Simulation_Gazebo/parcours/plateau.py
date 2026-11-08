"""Plateau du parcours (dimensions SUPPOSEES en attendant le plan cote) : positions des balles et des receptacles vues
depuis chaque arret du robot, verification de la zone apprise par le reseau, effet des erreurs d'arret, essais dans le
simulateur rapide.

LE JOUR OU LE PLAN COTE ARRIVE : ne changer que le bloc "MESURES DU PLATEAU" ci-dessous, puis relancer :
    cd ~/ros2_ws/src/mon_bras_robot/ia_50cm && nice -n 10 python3 -W ignore parcours/plateau.py [modele] [essais]
Sorties : parcours/resultats_plateau.txt, parcours/plateau.json, parcours/plateau.png

Convention (pour chaque station) : repere du TRAIT. Origine = milieu du trait, sur l'axe de la ligne noire.
X = le long de la ligne, dans le sens de marche du robot ; Y = vers la gauche du robot. Unites : METRES.
Une reglette est donnee par la position de son trou 1 (premier trou rencontre par le robot), la direction trou 1 ->
trou 5 (en degres, 0 = sens de marche, +90 = vers la gauche) et les numeros des trous occupes (1 a 5).
"""
import json
import multiprocessing
import os
import sys

import numpy as np
import torch

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..'))
from cinematique import SERVO_FERME, SERVO_OUVERT, Bras  # noqa: E402
from expert import Expert  # noqa: E402
from politique import Reseau, observation  # noqa: E402
import sim_cinematique  # noqa: E402
from zone import GRAND, PETIT, Z_BALLE, cible_lacher  # noqa: E402

# =============================================================================== MESURES DU PLATEAU (a remplacer)
PAS_TROUS = 0.050                       # distance entre deux trous de la reglette
STATION_1 = dict(                       # petit trait, juste apres le 1er lot de balles (debut du parcours)
    nom='Station 1 (1er lot)',
    trou1=(-0.250, -0.200), direction_deg=0.0,                      # reglette a droite, parallele a la ligne
    trous=(1, 2, 3, 4, 5),                                          # trous occupes : les 5 (10 balles en tout)
    prises=(1,),                                                    # balles prises ici, dans l'ordre
    petit=(-0.300, 0.250),                                          # centre du petit receptacle 1 (a gauche)
    # Le trait est APRES les balles : arrete dessus, le bras les aurait derriere lui (hors de portee). On s'arrete
    # donc sur NOTRE repere en scotch, pose avec le gabarit a DECALAGE_REPERE du trait (negatif = avant le trait).
    decalage_repere=-0.315,
)
STATION_2 = dict(                       # grand trait, 2e lot de balles (sur le chemin du robot, FAQ precision n°3)
    nom='Station 2 (2e lot)',
    trou1=(0.170, -0.100), direction_deg=90.0,                      # meme reglette, devant, en travers de la ligne
    trous=(1, 2, 3, 4, 5),
    prises=(3, 2),                      # milieu -> petit receptacle 2, puis sa voisine de droite, gardee (grand)
    petit=(0.000, 0.270),                                           # centre du petit receptacle 2 (a gauche)
    decalage_repere=0.0,                                            # on s'arrete directement sur le grand trait
)
# Grand receptacle : le robot y est conduit a la main (application) apres la fin de la ligne. Position visee :
GRAND_GARAGE = (0.225, 0.0)             # centre du grand receptacle dans le repere du robot (22,5 cm devant l'axe)

# =============================================================================== ROBOT (a mesurer sur le vrai robot)
D_CAPTEURS_AVANT = 0.085                # capteurs de ligne avant : distance devant l'axe du bras
D_CAPTEUR_ARRIERE = 0.085               # capteur de ligne arriere : distance derriere l'axe du bras

# =============================================================================== ZONE APPRISE PAR LE RESEAU
R_BALLE_MIN, R_BALLE_SURE, R_BALLE_MAX = 0.19, 0.28, 0.35      # sure : 100 % dans Gazebo en dessous de ~28 cm
ANGLE_BALLE_MAX = 83.0
R_PETIT_MIN, R_PETIT_MAX, ANGLE_PETIT_MAX = 0.21, 0.34, 76.0
D_GRAND_MIN, D_GRAND_MAX, ANGLE_GRAND_MAX = 0.205, 0.25, 15.0

# =============================================================================== ERREURS D'ARRET ESSAYEES
# (ecart-type le long de la ligne, en travers, angle) : ligne + repere en scotch + capteur arriere ; puis sans
# capteur arriere ; puis recul mesure par les codeurs (sans repere a nous).
NIVEAUX = [('parfait', 0.0, 0.0, 0.0), ('repere + capteur arriere', 0.002, 0.002, 0.5),
           ('repere, sans capteur arriere', 0.002, 0.002, 1.5), ('recul aux codeurs', 0.008, 0.002, 1.0)]
NIVEAUX_GRAND = [('parfait', 0.0, 0.0, 0.0), ('garage a la main, soigne', 0.01, 0.01, 3.0),
                 ('garage a la main, approximatif', 0.02, 0.02, 6.0)]


if os.environ.get('PRISES_2'):          # essai d'un autre ordre a la station 2, ex. PRISES_2=3,1
    STATION_2['prises'] = tuple(int(k) for k in os.environ['PRISES_2'].split(','))


def trous(st):
    u = np.array([np.cos(np.radians(st['direction_deg'])), np.sin(np.radians(st['direction_deg']))])
    return {k: np.array(st['trou1']) + (k - 1) * PAS_TROUS * u for k in range(1, 6)}


def axe_bras(st):
    """Position de l'axe du bras dans le repere du trait quand les capteurs avant sont sur le repere d'arret."""
    return np.array([st['decalage_repere'] - D_CAPTEURS_AVANT, 0.0])


def vers_robot(p, st):
    return np.asarray(p, float) - axe_bras(st)


def polaire(p):
    return float(np.hypot(*p[:2])), float(np.degrees(np.arctan2(p[1], p[0])))


def taches_station(st, n_balles):
    """Balles a prendre (dans l'ordre) et objets vus depuis l'arret, repere du robot."""
    t = trous(st)
    balles = {k: vers_robot(t[k], st) for k in st['trous']}
    petit = vers_robot(st['petit'], st)
    return balles, petit


def verdict_balle(p):
    r, a = polaire(p)
    if not (R_BALLE_MIN <= r <= R_BALLE_MAX) or abs(a) > ANGLE_BALLE_MAX:
        return 'HORS ZONE'
    return 'ok' if r <= R_BALLE_SURE else 'limite (> 28 cm)'


def verdict_petit(p):
    r, a = polaire(p)
    return 'ok' if R_PETIT_MIN <= r <= R_PETIT_MAX and abs(a) <= ANGLE_PETIT_MAX else 'HORS ZONE'


def erreur_max(p, s_long, s_trav, s_ang):
    """Erreur de position (a 2 ecarts-types) d'un objet vu depuis le robot, pour ces erreurs d'arret."""
    r = np.hypot(*p[:2])
    return 2 * np.sqrt(s_long ** 2 + s_trav ** 2 + (r * np.radians(s_ang)) ** 2)


# =============================================================================== simulateur rapide
class SimArret(sim_cinematique.SimReceptacles):
    """Le reseau voit les positions CROYEES (arret parfait) ; la scene reelle est decalee par l'erreur d'arret."""
    biais = np.zeros(3)

    def obs(self):
        vue = self.balle + self.biais + self.rng.normal(0, sim_cinematique.BRUIT_VUE, 3)
        return observation(self.q, self.servo, vue, self.cible, self.outil(), self.grand)


def _deplacer(p, t, dtheta):
    """Objet vu depuis le robot reel (decale de t, tourne de dtheta) au lieu du robot prevu."""
    c, s = np.cos(-dtheta), np.sin(-dtheta)
    q = np.asarray(p[:2], float) - t
    return np.r_[c * q[0] - s * q[1], s * q[0] + c * q[1], p[2:]] if len(p) > 2 else np.array([c * q[0] - s * q[1],
                                                                                                s * q[0] + c * q[1]])


def _init(poids):
    global _sim, _expert, _reseau
    torch.set_num_threads(1)
    _sim = SimArret(Bras())
    _expert = Expert(_sim.cine)
    _reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    _reseau.load_state_dict(poids)
    _reseau.eval()


def _essai(args):
    nom, graine, scene_cru, s_long, s_trav, s_ang, tenue = args
    # le passage au processus ouvrier copie les dictionnaires : zone.py reconnait le grand receptacle par "is GRAND"
    scene_cru['types'] = [GRAND if t['bord'] == GRAND['bord'] else PETIT for t in scene_cru['types']]
    rng = np.random.default_rng(graine)
    t = np.array([rng.normal(0, s_long), rng.normal(0, s_trav)])
    dth = np.radians(rng.normal(0, s_ang))
    reel = dict(receptacles=[_deplacer(c, t, dth) for c in scene_cru['receptacles']], types=scene_cru['types'],
                balles=[_deplacer(b, t, dth) for b in scene_cru['balles']])
    if tenue:                                         # balle deja dans la pince, depart du repos (grand receptacle)
        _sim.reset(reel, 0, _expert.q_repos, SERVO_FERME, graine=graine)
        _sim.tenue, _sim.z_prise, _sim.z_sol_balle = True, -1.0, reel['balles'][0][2]
        d = rng.normal(size=3)
        _sim.decalage = d / np.linalg.norm(d) * rng.uniform(0.0015, 0.007)
        _sim.balle = _sim.outil_reel() + _sim.decalage
        _sim.biais = np.zeros(3)
    else:
        _sim.reset(reel, 0, _expert.q_repos, SERVO_OUVERT, graine=graine)
        _sim.biais = np.asarray(scene_cru['balles'][0]) - reel['balles'][0]
    _sim.cible = cible_lacher(scene_cru, 0)           # le robot vise la ou il CROIT qu'est le receptacle
    obs, fini = _sim.obs(), False
    while not fini:
        with torch.no_grad():
            act = _reseau(torch.from_numpy(obs)[None])[0].numpy()
        obs, fini = _sim.step(act)
    return nom, _sim.resultat


def main():
    modele = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ICI, '..', 'modeles', 'mission', 'politique_mission_affine.pt')
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    lignes, resultat = [], dict(modele=os.path.basename(modele), stations={})

    def dire(texte=''):
        lignes.append(texte)
        print(texte, flush=True)

    dire(f'Plateau SUPPOSE (a remplacer par le plan cote) - reseau {os.path.basename(modele)}')
    scenarios = []
    for st in (STATION_1, STATION_2):
        balles, petit = taches_station(st, len(st['prises']))
        dire(f'\n=== {st["nom"]} : repere d\'arret a {st["decalage_repere"] * 100:+.1f} cm du trait ; '
             f'axe du bras a {axe_bras(st)[0] * 100:+.1f} cm du trait')
        ordre = list(st['prises'])
        d_st = dict(axe_bras=axe_bras(st).round(4).tolist(), balles={}, petit=None)
        for k, p in balles.items():
            r, a = polaire(p)
            role = f' <- prise n°{ordre.index(k) + 1}' if k in ordre else ''
            dire(f'  trou {k} : x {p[0] * 100:6.1f} cm, y {p[1] * 100:6.1f} cm  ({r * 100:4.1f} cm, {a:+6.1f} deg) '
                 f'{verdict_balle(p)}{role}')
            d_st['balles'][k] = dict(x=round(float(p[0]), 4), y=round(float(p[1]), 4), r=round(r, 4), angle=round(a, 1))
        r, a = polaire(petit)
        dire(f'  petit receptacle : x {petit[0] * 100:6.1f} cm, y {petit[1] * 100:6.1f} cm  ({r * 100:4.1f} cm, '
             f'{a:+6.1f} deg) {verdict_petit(petit)}')
        d_st['petit'] = dict(x=round(float(petit[0]), 4), y=round(float(petit[1]), 4))
        d_st['ordre'] = ordre
        resultat['stations'][st['nom']] = d_st
        for k in ordre:
            p = balles[k]
            dire(f'  erreur sur la balle du trou {k} (2 ecarts-types) : ' + ' ; '.join(
                f'{nom} {erreur_max(p, a_, b_, c_) * 1000:.1f} mm' for nom, a_, b_, c_ in NIVEAUX[1:]))
            restantes = [np.r_[balles[j], Z_BALLE] for j in st['trous'] if j != k and j not in ordre[:ordre.index(k)]]
            scene = dict(receptacles=[petit], types=[PETIT], balles=[np.r_[p, Z_BALLE]] + restantes)
            for niv in NIVEAUX:
                scenarios.append((f'{st["nom"]} | trou {k} -> petit | {niv[0]}', scene, niv, False))
    g = np.array(GRAND_GARAGE)
    r, a = polaire(g)
    ok = D_GRAND_MIN <= r <= D_GRAND_MAX and abs(a) <= ANGLE_GRAND_MAX
    dire(f'\n=== Grand receptacle (conduite a la main) : centre a {r * 100:.1f} cm, {a:+.1f} deg {"ok" if ok else "HORS ZONE"}')
    # balle prise au 2e lot (trou du 2e choix), deja dans la pince : sa position d'origine ne sert qu'au simulateur
    scene_g = dict(receptacles=[g], types=[GRAND], balles=[np.r_[0.15, -0.25, Z_BALLE]])
    for niv in NIVEAUX_GRAND:
        scenarios.append((f'Grand receptacle | balle tenue -> grand | {niv[0]}', scene_g, niv, True))

    poids = torch.load(modele, map_location='cpu')
    taches = [(nom, 60_000_000 + i, sc, a_, b_, c_, tenue) for nom, sc, (_, a_, b_, c_), tenue in scenarios
              for i in range(n)]
    with multiprocessing.get_context('spawn').Pool(4, initializer=_init, initargs=(poids,)) as pool:
        res = pool.map(_essai, taches, chunksize=2)
    dire(f'\n=== Simulateur rapide ({n} essais par ligne ; erreurs d\'arret tirees au hasard) ===')
    resultat['simulation'] = {}
    for nom, *_ in scenarios:
        r_ = [x for nn, x in res if nn == nom]
        echecs = {k: r_.count(k) for k in set(r_) if k != 'reussite'}
        taux = r_.count('reussite') / len(r_)
        resultat['simulation'][nom] = dict(reussite=round(taux, 3), echecs=echecs)
        dire(f'  {nom:75s} {taux * 100:5.1f} %  {echecs if echecs else ""}')
    with open(os.path.join(ICI, 'resultats_plateau.txt'), 'w') as f:
        f.write('\n'.join(lignes) + '\n')
    with open(os.path.join(ICI, 'plateau.json'), 'w') as f:
        json.dump(resultat, f, indent=2, ensure_ascii=False)
    dessiner()


def dessiner():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    for ax, st in zip(axes, (STATION_1, STATION_2)):
        ax.axhline(0, color='k', lw=6, alpha=0.8, zorder=0)
        ax.plot([0, 0], [-0.04, 0.04] if st is STATION_1 else [-0.12, 0.12], color='k', lw=4)
        ax.annotate('trait officiel', (0, 0.05 if st is STATION_1 else 0.13), ha='center', fontsize=8)
        if st['decalage_repere'] != 0:
            x = st['decalage_repere']
            ax.plot([x, x], [-0.04, 0.04], color='#d62728', lw=4)
            ax.annotate('notre repère\n(gabarit)', (x, -0.09), ha='center', fontsize=8, color='#d62728')
        a = axe_bras(st)
        ax.add_patch(Rectangle((a[0] - 0.1, -0.1), 0.2, 0.2, fill=False, ec='#1f77b4', lw=1.5))
        ax.plot(*a, 'o', color='#1f77b4')
        ax.annotate('axe du bras', (a[0], a[1] - 0.025), ha='center', fontsize=8, color='#1f77b4')
        t = trous(st)
        for k, p in t.items():
            ax.add_patch(Circle(p, 0.0115, fill=k in st['trous'], fc='#ff7f0e', ec='#ff7f0e'))
            ax.annotate(str(k), (p[0] + (0.025 if st['direction_deg'] else 0), p[1] + (0 if st['direction_deg'] else 0.02)),
                        ha='center', va='center', fontsize=7)
        ax.add_patch(Circle(st['petit'], PETIT['r_ext'], fill=False, ec='#2ca02c', lw=1.5))
        ax.annotate('petit réceptacle', (st['petit'][0], st['petit'][1] + 0.06), ha='center', fontsize=8, color='#2ca02c')
        ang = np.radians(np.linspace(-ANGLE_BALLE_MAX, ANGLE_BALLE_MAX, 60))
        bord = np.r_[R_BALLE_SURE * np.c_[np.cos(ang), np.sin(ang)], R_BALLE_MIN * np.c_[np.cos(ang), np.sin(ang)][::-1]]
        ax.fill(a[0] + bord[:, 0], a[1] + bord[:, 1], color='#1f77b4', alpha=0.10, lw=0)
        ax.annotate('zone sûre des balles\n(19-28 cm)', (a[0] + 0.21, 0.2), ha='center', fontsize=7, color='#1f77b4')
        ax.set_aspect('equal')
        ax.set_xlim(a[0] - 0.15, a[0] + 0.45)
        ax.set_ylim(-0.32, 0.38)
        ax.set_title(st['nom'] + ' (repère du trait, m)', fontsize=10)
        ax.grid(alpha=0.3)
        ax.annotate('sens de marche →', (a[0] - 0.14, 0.015), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(ICI, 'plateau.png'), dpi=130)


if __name__ == '__main__':
    main()
