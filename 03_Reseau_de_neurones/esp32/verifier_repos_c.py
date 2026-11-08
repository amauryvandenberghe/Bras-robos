"""Verifie aller_repos() du programme ESP32 (C) contre le retour a rebours teste en Python (essai_retour_repos.py) :
un vrai trajet du reseau vers le grand receptacle est donne au C, qui doit produire les memes consignes, sans choc.
Usage : python3 esp32/verifier_repos_c.py
"""
import os
import subprocess
import sys
import numpy as np
import torch

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..'))
import essai_retour_repos as E  # noqa: E402
from zone import GRAND  # noqa: E402

E._init(torch.load(os.path.join(ICI, '..', 'modeles', 'politique_finale.pt')))
sim, cine = E._sim, E._sim.cine
for g in range(30_000_000, 30_000_200):                  # premier lacher reussi au grand receptacle
    rng = np.random.default_rng(g)
    scene, k, q0, servo = E.depart(rng, sim, E._expert)
    if scene['types'][k] is not GRAND:
        continue
    obs = sim.reset(scene, k, q0, servo, graine=g)
    fini, histo = False, []
    while not fini:
        with torch.no_grad():
            obs, fini = sim.step(E._reseau(torch.from_numpy(obs)[None])[0].numpy())
        histo.append(sim.q.copy())
    if sim.resultat == 'reussite':
        break
attendu = np.array(list(E.chemin(histo[-1], E._expert.q_repos, 'a_rebours', cine, None, histo[:-1])))
D = os.path.join(ICI, 'bras_esp32')
STUB = '/mnt/c/Users/amaur/AppData/Local/Temp/bras50/stub'
src = open(os.path.join(D, 'bras_esp32.ino')).read()
src = src.replace('static void commander() {', 'static void commander() {\n    printf("Q %.6f %.6f %.6f %.6f\\n", etat.q[0], etat.q[1], etat.q[2], etat.q[3]);')
src += '\nstatic void poser(float a, float b, float c, float d) { etat.q[0] = a; etat.q[1] = b; etat.q[2] = c; etat.q[3] = d; }\n'
src += 'int main() {\n    cerveau_repos(&etat);\n'
for q in histo[:-1]:
    src += '    poser(%ff, %ff, %ff, %ff); memoriser();\n' % tuple(q)
src += '    poser(%ff, %ff, %ff, %ff);\n    aller_repos();\n    return 0;\n}\n' % tuple(histo[-1])
open('/tmp/repos_c.cpp', 'w').write(src)
subprocess.run(['g++', '-std=gnu++17', '-w', f'-I{STUB}', f'-I{D}', '/tmp/repos_c.cpp', os.path.join(D, 'cerveau.cpp'),
                '-o', '/tmp/repos_c', '-lm'], check=True)
sortie = subprocess.run(['/tmp/repos_c'], capture_output=True, text=True).stdout
c = np.array([list(map(float, l.split()[1:])) for l in sortie.splitlines() if l.startswith('Q ')])
n = min(len(c), len(attendu))
print(f'graine {g} : {len(histo)} pas de reseau ; retour : {len(c)} consignes en C, {len(attendu)} en Python')
print(f'ecart maxi C / Python : {np.abs(c[:n] - attendu[:n]).max():.2e} rad ; fin en C {np.round(c[-1], 4)} ; repos {np.round(E._expert.q_repos, 4)}')
q_reel, choc = np.array(histo[-1]), None
for qq in list(c) + [c[-1]] * 10:
    q_reel += E.SUIVI_SERVO * (qq - q_reel)
    choc = choc or E.probleme(cine, q_reel, scene, k)
print('chocs pendant le retour (C) :', choc or 'aucun')
