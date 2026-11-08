"""Analyse des maillages de l'avant-bras (bras trois) et du corps de pince (bras quatre) : corps separes, boites, rendu."""
import os
import sys
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from apercu import dessiner, cadrer  # noqa: E402

ICI = os.path.dirname(os.path.abspath(__file__))
M = os.path.join(ICI, '..', 'meshes')
COUL = ['tab:green', 'tab:red', 'tab:blue', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:olive', 'tab:cyan', 'k', 'y']
fig = plt.figure(figsize=(20, 10))
for i, (nom, pivots) in enumerate([('bras_trois', {'J3': (0, 0, 0), 'J4': (-0.1234, 0.016, -0.0236)}),
                                   ('bras_quatre', {'J4': (0, 0, 0), 'pignon': (-0.022617, 0.026325, -0.01764)})]):
    m = trimesh.load(os.path.join(M, nom + '.stl'))
    corps = m.split(only_watertight=False)
    print(f'== {nom} : {len(m.faces)} facettes, {len(corps)} corps, boite {np.round(m.bounds * 1000, 1).tolist()}')
    corps = sorted(corps, key=lambda c: -len(c.faces))
    for k, c in enumerate(corps[:12]):
        b = c.bounds * 1000
        print(f'  corps {k}: {len(c.faces):6d} fac, x[{b[0,0]:7.1f} {b[1,0]:7.1f}] y[{b[0,1]:7.1f} {b[1,1]:7.1f}] '
              f'z[{b[0,2]:7.1f} {b[1,2]:7.1f}] taille {np.round(b[1]-b[0],1).tolist()} etanche={c.is_watertight}')
    for v, (elev, azim) in enumerate([(25, -60), (-20, 120)]):
        ax = fig.add_subplot(2, 2, i + 1 + 2 * v, projection='3d')
        for k, c in enumerate(corps[:10]):
            dessiner(ax, c.vertices[c.faces], COUL[k % len(COUL)], 3000)
        for n, p in pivots.items():
            ax.scatter(*p, s=60, color='k'); ax.text(*p, n)
        cadrer(ax, m.vertices); ax.view_init(elev, azim); ax.set_title(nom)
plt.tight_layout(); plt.savefig(os.path.join(ICI, 'pieces', 'analyse_avant_bras.png'), dpi=70)
