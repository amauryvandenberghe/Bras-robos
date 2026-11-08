"""Vues du corps de pince (bras quatre) : le servo de pince (corps fusionne) en rouge, le reste en gris."""
import os
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from apercu import dessiner, cadrer

ICI = os.path.dirname(os.path.abspath(__file__))
m = trimesh.load(os.path.join(ICI, '..', 'meshes', 'bras_quatre.stl'))
corps = sorted(m.split(only_watertight=False), key=lambda c: -len(c.faces))
fig = plt.figure(figsize=(20, 7))
for v, (elev, azim) in enumerate([(20, -60), (20, 30), (85, -90)]):
    ax = fig.add_subplot(1, 3, v + 1, projection='3d')
    for k, c in enumerate(corps):
        dessiner(ax, c.vertices[c.faces], 'tab:red' if k == 0 else ['tab:gray', 'tab:blue', 'tab:green', 'tab:olive'][k % 4], 4000)
    cadrer(ax, m.vertices); ax.view_init(elev, azim)
plt.tight_layout(); plt.savefig(os.path.join(ICI, 'pieces', 'analyse_pince.png'), dpi=65)
# distance entre le servo et le corps de pince
s, p = corps[0], corps[2]
d = trimesh.proximity.closest_point(p, s.vertices[::5])[1]
print('servo de pince : distance mini au corps de pince %.2f mm, %d %% des points a moins de 0,5 mm'
      % (d.min() * 1000, 100 * np.mean(d < 0.0005)))
