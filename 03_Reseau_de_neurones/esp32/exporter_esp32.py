"""Exporte le reseau entraine et la geometrie du bras pour l'ESP32, puis verifie le code C contre Python.

  1. ecrit bras_esp32/reseau_poids.h : poids du reseau (float32, en memoire flash : ~1,1 Mo) + constantes du bras
     (transformations de l'URDF, butees, point de prise, reglages de pince) ;
  2. copie cerveau.h / cerveau.cpp dans le croquis Arduino bras_esp32/ ;
  3. verification : compile cerveau.cpp sur le PC (g++) et compare, sur 2000 etats au hasard, l'observation,
     la sortie du reseau, les nouvelles consignes et le point de prise avec ceux de politique.py / cinematique.py.
Usage : python3 exporter_esp32.py [modele.pt]   (defaut : ../modeles/politique_bras.pt)
"""
import os
import shutil
import subprocess
import sys

import numpy as np
import torch

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..'))
from cinematique import Bras, BRAS, OUTIL, SERVO_CONTACT, SERVO_FERME, SERVO_OUVERT  # noqa: E402
from expert import Expert  # noqa: E402
from politique import (PAS_PINCE, PAS_SERRAGE, PAS_SERVO, Z_MIN_OUTIL, Reseau, observation, securite,  # noqa: E402
                       servo_suivant)
from zone import DEMI_ROBOT, HAUT_ROBOT  # noqa: E402

CROQUIS_DEFAUT = os.path.join(ICI, 'bras_esp32')
# CROQUIS=chemin : ecrire reseau_poids.h dans un autre croquis (ex. ../mission_esp32/robot_esp32, qui a son propre
# cerveau.cpp : il n'est pas ecrase, mais c'est lui qui est verifie contre Python)
CROQUIS = os.path.abspath(os.environ.get('CROQUIS', CROQUIS_DEFAUT))


def fl(x):
    """Litteral float C ("1.0f", "-3.5e-06f")."""
    s = f'{float(x):.9g}'
    if '.' not in s and 'e' not in s:
        s += '.0'
    return s + 'f'


def tableau(nom, v, par_ligne=8):
    v = np.asarray(v, np.float32).ravel()
    lignes = [', '.join(fl(x) for x in v[i:i + par_ligne]) for i in range(0, len(v), par_ligne)]
    return f'static const float {nom}[{len(v)}] = {{\n  ' + ',\n  '.join(lignes) + '\n};\n'


def entete(poids, cine):
    W1, B1 = poids['couches.0.weight'].numpy(), poids['couches.0.bias'].numpy()
    W2, B2 = poids['couches.2.weight'].numpy(), poids['couches.2.bias'].numpy()
    W3, B3 = poids['couches.4.weight'].numpy(), poids['couches.4.bias'].numpy()
    j = {v['nom']: v for v in cine.joints.values()}
    base = j['world_joint']['T'] @ j['robot_socle']['T'] @ j['joint_base_bras_1']['T']
    origines = [np.eye(4)] + [j[n]['T'] for n in BRAS[1:]]
    axes = [j[n]['axe'] / np.linalg.norm(j[n]['axe']) for n in BRAS]
    q_repos = Expert(cine).q_repos
    s = ['// FICHIER GENERE par ia_50cm/esp32/exporter_esp32.py : ne pas modifier a la main.',
         '#pragma once', f'#define CACHE {W1.shape[0]}',
         f'#define SERVO_OUVERT {SERVO_OUVERT}f', f'#define SERVO_CONTACT {SERVO_CONTACT}f',
         f'#define SERVO_FERME {SERVO_FERME}f', f'#define PAS_SERVO {PAS_SERVO}f', f'#define PAS_PINCE {PAS_PINCE}f',
         f'#define PAS_SERRAGE {PAS_SERRAGE}f', f'#define Z_MIN_OUTIL {Z_MIN_OUTIL}f',
         f'#define DEMI_ROBOT {DEMI_ROBOT}f', f'#define HAUT_ROBOT {HAUT_ROBOT}f', '']
    s.append(tableau('OUTIL', OUTIL))
    s.append(tableau('Q_REPOS', q_repos))
    s.append(tableau('BUTEE_BAS', cine.bas))
    s.append(tableau('BUTEE_HAUT', cine.haut))
    s.append(tableau('BASE_VERS_LACET', base))
    s.append('static const float ORIGINE_JOINT[4][16] = {\n' + ',\n'.join(
        '  {' + ', '.join(fl(x) for x in o.ravel()) + '}' for o in origines) + '\n};\n')
    s.append('static const float AXE_JOINT[4][3] = {\n' + ',\n'.join(
        '  {' + ', '.join(fl(x) for x in a) + '}' for a in axes) + '\n};\n')
    for nom, v in (('W1', W1), ('B1', B1), ('W2', W2), ('B2', B2), ('W3', W3), ('B3', B3)):
        s.append(tableau(nom, v))
    return '\n'.join(s)


TEST_CPP = r'''
#include <cstdio>
#include <cmath>
#include "cerveau.h"
int main() {
    FILE *f = fopen("cas_test.txt", "r");
    int n; if (fscanf(f, "%d", &n) != 1) return 1;
    double e_obs = 0, e_act = 0, e_q = 0, e_servo = 0, e_outil = 0; int n_diff = 0;
    for (int c = 0; c < n; c++) {
        EtatBras e; float balle[3], cible[3], obs_ref[N_OBS], act_ref[N_ACT], q_ref[4], servo_ref, outil_ref[3]; int grand;
        for (int i = 0; i < 4; i++) fscanf(f, "%f", &e.q[i]);
        fscanf(f, "%f", &e.servo);
        for (int i = 0; i < 3; i++) fscanf(f, "%f", &balle[i]);
        for (int i = 0; i < 3; i++) fscanf(f, "%f", &cible[i]);
        fscanf(f, "%d", &grand);
        for (int i = 0; i < N_OBS; i++) fscanf(f, "%f", &obs_ref[i]);
        for (int i = 0; i < N_ACT; i++) fscanf(f, "%f", &act_ref[i]);
        for (int i = 0; i < 4; i++) fscanf(f, "%f", &q_ref[i]);
        fscanf(f, "%f", &servo_ref);
        for (int i = 0; i < 3; i++) fscanf(f, "%f", &outil_ref[i]);
        float obs[N_OBS], act[N_ACT], outil[3];
        cerveau_outil(e.q, outil);
        for (int i = 0; i < 3; i++) e_outil = fmax(e_outil, fabs(outil[i] - outil_ref[i]));
        cerveau_observation(&e, balle, cible, grand, obs);
        for (int i = 0; i < N_OBS; i++) e_obs = fmax(e_obs, fabs(obs[i] - obs_ref[i]));
        cerveau_pas(&e, balle, cible, grand, act);
        for (int i = 0; i < N_ACT; i++) e_act = fmax(e_act, fabs(act[i] - act_ref[i]));
        double ec = 0;
        for (int i = 0; i < 4; i++) ec = fmax(ec, fabs(e.q[i] - q_ref[i]));
        if (ec > 1e-4) { n_diff++; float o2[3]; cerveau_outil(q_ref, o2);
            printf("  cas %d : garde-fou a la limite (pince a z = %.5f m)\n", c, o2[2]); }
        e_q = fmax(e_q, ec);
        e_servo = fmax(e_servo, fabs(e.servo - servo_ref));
    }
    printf("%d cas : ecart maxi point de prise %.2e m, observation %.2e, sorties du reseau %.2e, "
           "consignes %.2e rad (%d cas differents), pince %.2e rad\n", n, e_outil, e_obs, e_act, e_q, n_diff, e_servo);
    return 0;
}
'''


def verifier(poids, cine, n=2000):
    reseau = Reseau(cache=poids['couches.0.weight'].shape[0])
    reseau.load_state_dict(poids)
    reseau.eval()
    rng = np.random.default_rng(0)
    lignes = [str(n)]
    for _ in range(n):
        q = rng.uniform(cine.bas, cine.haut).astype(np.float32).astype(float)
        servo = float(np.float32(rng.uniform(SERVO_FERME, SERVO_OUVERT)))
        balle = np.float32(rng.uniform([0.1, -0.3, 0.0], [0.35, 0.3, 0.3])).astype(float)
        cible = np.float32(rng.uniform([0.1, -0.3, 0.15], [0.35, 0.3, 0.56])).astype(float)
        grand = int(rng.random() < 0.4)
        outil = cine.outil(q)[0]
        obs = observation(q, servo, balle, cible, outil, grand)
        with torch.no_grad():
            act = reseau(torch.from_numpy(obs)[None])[0].numpy()
        q2, pince = securite(cine, q, act, servo)
        servo2 = servo_suivant(servo, pince)
        vals = list(q) + [servo] + list(balle) + list(cible) + [grand] + list(obs) + list(act) + list(q2) + [servo2] + list(outil)
        lignes.append(' '.join(f'{v:.9g}' if not isinstance(v, int) else str(v) for v in vals))
    dossier = '/tmp/test_cerveau'
    os.makedirs(dossier, exist_ok=True)
    open(os.path.join(dossier, 'cas_test.txt'), 'w').write('\n'.join(lignes))
    open(os.path.join(dossier, 'test.cpp'), 'w').write(TEST_CPP)
    subprocess.run(['g++', '-O2', '-Wno-unused-result', '-I', CROQUIS, os.path.join(dossier, 'test.cpp'), os.path.join(CROQUIS, 'cerveau.cpp'),
                    '-o', os.path.join(dossier, 'test')], check=True)
    print(subprocess.run([os.path.join(dossier, 'test')], cwd=dossier, capture_output=True, text=True).stdout)


if __name__ == '__main__':
    modele = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ICI, '..', 'modeles', 'politique_bras.pt')
    poids = torch.load(modele, map_location='cpu')
    cine = Bras()
    os.makedirs(CROQUIS, exist_ok=True)
    if CROQUIS == os.path.abspath(CROQUIS_DEFAUT):
        for f in ('cerveau.h', 'cerveau.cpp'):
            shutil.copy(os.path.join(ICI, f), os.path.join(CROQUIS, f))
    with open(os.path.join(CROQUIS, 'reseau_poids.h'), 'w') as f:
        f.write(entete(poids, cine))
    n = sum(v.numel() for v in poids.values())
    print(f'{modele} : {n} poids = {n * 4 / 1e6:.2f} Mo en float32 -> {CROQUIS}/reseau_poids.h')
    verifier(poids, cine)
