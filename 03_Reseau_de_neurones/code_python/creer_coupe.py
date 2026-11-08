"""Genere le maillage de la coupe : demi-sphere creuse, ouverture vers le haut.

Diametre exterieur 10 cm, epaisseur 1 cm (rayon exterieur 5 cm, interieur 4 cm).
Origine du maillage = centre de la sphere, donc le bord de la coupe est a z = 0 et le fond exterieur a z = -5 cm.
Dans la scene, la coupe est posee sur un poteau de 5 cm : fond a 5 cm du sol, bord a 10 cm.

Usage : python3 creer_coupe.py   ->   ../meshes/coupe.stl (affichage, fin)
                                   ../meshes/coupe_collision.stl (collisions, grossier : la physique reste rapide)
"""
import os
import struct
import numpy as np

R_EXT, R_INT = 0.05, 0.04
N_LAT, N_LON = 24, 64          # finesse du maillage d'affichage (latitude x longitude)
N_LAT_COLL, N_LON_COLL = 6, 20 # maillage de collision : ~500 triangles au lieu de ~6000
MESHES = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')
SORTIE = os.path.join(MESHES, 'coupe.stl')


def point(r, lat, lon):
    """lat = 0 : bord (equateur) ; lat = pi/2 : fond de la coupe."""
    return np.array([r * np.cos(lat) * np.cos(lon), r * np.cos(lat) * np.sin(lon), -r * np.sin(lat)])


def triangles(n_lat=N_LAT, n_lon=N_LON):
    lats = np.linspace(0, np.pi / 2, n_lat + 1)
    lons = np.linspace(0, 2 * np.pi, n_lon + 1)
    for i in range(n_lat):
        for j in range(n_lon):
            for r, dehors in ((R_EXT, True), (R_INT, False)):
                a, b = point(r, lats[i], lons[j]), point(r, lats[i], lons[j + 1])
                c, d = point(r, lats[i + 1], lons[j]), point(r, lats[i + 1], lons[j + 1])
                if dehors:            # face exterieure : normale vers l'exterieur
                    yield a, c, b
                    yield b, c, d
                else:                 # face interieure : normale vers le centre (cote balle)
                    yield a, b, c
                    yield b, d, c
    for j in range(n_lon):            # anneau du bord, normale vers le haut
        a, b = point(R_INT, 0, lons[j]), point(R_INT, 0, lons[j + 1])
        c, d = point(R_EXT, 0, lons[j]), point(R_EXT, 0, lons[j + 1])
        yield a, c, b
        yield b, c, d


def ecrire_stl(chemin, n_lat=N_LAT, n_lon=N_LON):
    tris = list(triangles(n_lat, n_lon))
    with open(chemin, 'wb') as f:
        f.write(b'coupe demi-sphere creuse'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))
    return len(tris)


if __name__ == '__main__':
    os.makedirs(MESHES, exist_ok=True)
    n = ecrire_stl(SORTIE)
    print(f'{n} triangles -> {os.path.abspath(SORTIE)}')
    coll = os.path.join(MESHES, 'coupe_collision.stl')
    n = ecrire_stl(coll, N_LAT_COLL, N_LON_COLL)
    print(f'{n} triangles -> {os.path.abspath(coll)}')
