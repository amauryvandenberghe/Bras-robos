"""Apercus PNG : chaque piece neuve (2 vues) et, avec --assemblage, le bras complet tel que decrit par l'URDF.
Usage : ~/venv_cao/bin/python apercu.py [--assemblage] [--q q1 q2 q3 q4]"""
import argparse
import os
import sys
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..', 'ia'))


def dessiner(ax, tris, couleur, n_max=8000):
    if len(tris) > n_max:
        tris = tris[np.random.default_rng(0).choice(len(tris), n_max, replace=False)]
    # ombrage simple : la couleur depend de l'orientation de la facette
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    lum = 0.45 + 0.55 * np.abs(n @ np.array([0.3, -0.5, 0.8]))
    base = np.array(matplotlib.colors.to_rgb(couleur))
    ax.add_collection3d(Poly3DCollection(tris, facecolors=np.clip(lum[:, None] * base, 0, 1), edgecolor='none'))


def cadrer(ax, pts):
    c, r = (pts.max(0) + pts.min(0)) / 2, (pts.max(0) - pts.min(0)).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')


def pieces():
    noms = ['socle', 'plateau', 'bras', 'bride_coude']
    fig = plt.figure(figsize=(20, 10))
    for i, nom in enumerate(noms):
        m = trimesh.load(os.path.join(ICI, 'pieces', nom + '.stl'))
        for k, (elev, azim) in enumerate([(30, -55), (-25, 120)]):
            ax = fig.add_subplot(2, len(noms), i + 1 + k * len(noms), projection='3d')
            dessiner(ax, m.vertices[m.faces], 'tab:orange')
            cadrer(ax, m.vertices); ax.view_init(elev, azim); ax.set_title(nom)
    plt.tight_layout(); plt.savefig(os.path.join(ICI, 'pieces', 'apercu_pieces.png'), dpi=75)


def assemblage(q, nom_png):
    import xml.etree.ElementTree as ET
    from cinematique import Bras
    urdf = os.path.join(ICI, '..', 'urdf', 'bras_50cm.urdf')
    b = Bras(urdf)
    qd = dict(zip(['joint_base_bras_1', 'joint_base_bras_2', 'joint_base_bras_3', 'joint_base_bras_4'], q))
    fig = plt.figure(figsize=(18, 9))
    vues = [(15, -60), (0, -90)]
    racine = ET.parse(urdf).getroot()
    tous = []
    axes = [fig.add_subplot(1, 2, k + 1, projection='3d') for k in range(2)]
    couleurs = {'robot': 'lightgray', 'base_link': 'tab:gray', 'bras un': 'tab:blue', 'bras deux': 'tab:orange',
                'bras trois': 'tab:green', 'bras quatre': 'tab:red'}
    for lien in racine.findall('link'):
        nom = lien.get('name')
        if nom == 'world':
            continue
        T = b.pose(nom, qd)
        for vis in lien.findall('visual'):
            g = vis.find('geometry')
            o = vis.find('origin')
            To = np.eye(4)
            if o is not None:
                from cinematique import _rpy
                To[:3, :3] = _rpy(*map(float, o.get('rpy', '0 0 0').split()))
                To[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
            if g.find('mesh') is not None:
                f = g.find('mesh').get('filename').replace('package://mon_bras_robot/', os.path.join(ICI, '..') + '/')
                m = trimesh.load(f)
                ech = float(g.find('mesh').get('scale', '1 1 1').split()[0])
                v = m.vertices * ech
                tri = v[m.faces]
            elif g.find('box') is not None:
                m = trimesh.creation.box(list(map(float, g.find('box').get('size').split())))
                tri = m.vertices[m.faces]
            else:
                continue
            Tt = T @ To
            w = tri.reshape(-1, 3) @ Tt[:3, :3].T + Tt[:3, 3]
            tous.append(w)
            for ax in axes:
                dessiner(ax, w.reshape(-1, 3, 3), couleurs.get(nom, 'tab:purple'), 4000)
    p, a = b.outil(np.array(q))
    for ax, (elev, azim) in zip(axes, vues):
        ax.scatter(*p, color='k', s=40)
        cadrer(ax, np.vstack(tous)); ax.view_init(elev, azim)
    fig.suptitle(f'q = {np.round(q, 2)}   outil = {np.round(p, 3)} m')
    plt.tight_layout(); plt.savefig(os.path.join(ICI, 'pieces', nom_png), dpi=75)
    print('outil', p.round(4), 'approche', a.round(3))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--assemblage', action='store_true')
    ap.add_argument('--q', type=float, nargs=4, default=[0, 0, 0, 0])
    ap.add_argument('--png', default='apercu_assemblage.png')
    args = ap.parse_args()
    if args.assemblage:
        assemblage(args.q, args.png)
    else:
        pieces()
