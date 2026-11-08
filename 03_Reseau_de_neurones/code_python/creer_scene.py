"""Genere worlds/scene_trois_receptacles.world (bras 50 cm) et les maillages des receptacles.

  - receptacle_1, receptacle_2 : demi-sphere creuse de 100 mm (1 cm d'epaisseur), bord a 150 mm, sur une colonne
    de 100 mm de diametre ; receptacle_3 : demi-sphere de 200 mm, bord a 500 mm, sur une colonne de 200 mm ;
  - reglette : plaque de 270 x 30 x 2 mm, 5 supports de balle au pas de 50 mm (la balle repose a 14,8 mm) ;
  - balle_1 (rouge), balle_2 (jaune), balle_3 (orange) : 23 mm (50 g dans la simulation, voir MASSE_BALLE).
Le robot roulant (200 x 200 x 80 mm) fait partie du modele du bras (urdf/bras_50cm.urdf).
Comme pour les coupes du bras actuel, l'affichage utilise un maillage fin, mais les COLLISIONS des demi-spheres
sont faites de plaques (contact maillage contre maillage = moteur physique fige).
Usage : python3 creer_scene.py
"""
import os
import numpy as np

import creer_coupe

ICI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(ICI, '..'))
SORTIE = os.path.join(PKG, 'worlds', 'scene_trois_receptacles.world')
MESHES = os.path.join(PKG, 'meshes', 'bras_50cm')
EPAISSEUR = 0.01
BANDES = [10, 30, 50, 70]
LARGEUR_BANDE = 20
# Vraie balle : 2 g. Dans Gazebo, une balle si legere est ejectee par la pince au serrage (essais : 2, 10 et 20 g
# ratent, 50 g tient) : on simule 50 g, et l'URDF ajoute aux servos le couple de ces 48 g en trop (generer_urdf.py).
MASSE_BALLE = float(os.environ.get('MASSE_BALLE', 0.050))
FROTTEMENT_BALLE = float(os.environ.get('FROTTEMENT_BALLE', 2.0))
R_BALLE = 0.0115
# Au depart, tout est range a l'ecart (hors de portee du bras) : la disposition de chaque essai est faite ensuite
# par evaluer_receptacles.py. Un receptacle laisse devant le robot bloquerait le bras pendant qu'il se met au repos.
GARAGE = {'receptacle_1': (-0.6, 0.4), 'receptacle_2': (-0.6, -0.4), 'receptacle_3': (-0.9, 0.0), 'reglette': (-0.6, 0.0),
          'fantome_1': (-1.3, 0.4), 'fantome_2': (-1.3, -0.4), 'fantome_3': (-1.6, 0.0),
          'balle_fixe_1': (-1.0, 0.8), 'balle_fixe_2': (-1.0, 0.85), 'balle_fixe_3': (-1.0, 0.9)}
COULEURS_BALLES = [('0.9 0 0 1', '1 0 0 1'), ('0.9 0.8 0 1', '1 0.9 0 1'), ('0.9 0.45 0 1', '1 0.55 0 1')]
COULEURS_RECEPTACLES = [('0 0.3 0.8 1', '0 0.4 1 1'), ('0.1 0.5 0.1 1', '0.1 0.7 0.1 1'), ('0.8 0.1 0.1 1', '0.9 0.2 0.2 1')]


def rpy(R):
    p = -np.arcsin(np.clip(R[2, 0], -1, 1))
    return np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])


def collisions_demi_sphere(r_moy, z_centre, n_lon):
    lignes = []
    for b, phi_deg in enumerate(BANDES):
        phi = np.radians(phi_deg)
        h = r_moy * np.radians(LARGEUR_BANDE) * 1.1
        w = 2 * np.pi * r_moy * np.cos(phi) / n_lon * 1.15
        for k in range(n_lon):
            th = 2 * np.pi * k / n_lon
            n = np.array([np.cos(phi) * np.cos(th), np.cos(phi) * np.sin(th), -np.sin(phi)])
            t_th = np.array([-np.sin(th), np.cos(th), 0.0])
            R = np.column_stack([t_th, np.cross(n, t_th), n])
            c = r_moy * n + [0, 0, z_centre]
            r, p, y = rpy(R)
            lignes.append(f'        <collision name="paroi_{b}_{k}"><pose>{c[0]:.5f} {c[1]:.5f} {c[2]:.5f} {r:.5f} {p:.5f} {y:.5f}</pose>'
                          f'<geometry><box><size>{w:.5f} {h:.5f} {EPAISSEUR}</size></box></geometry></collision>')
    lignes.append(f'        <collision name="fond"><pose>0 0 {z_centre - r_moy:.5f} 0 0 0</pose>'
                  f'<geometry><cylinder><radius>{0.4 * r_moy:.4f}</radius><length>{EPAISSEUR}</length></cylinder></geometry></collision>')
    return '\n'.join(lignes)


def receptacle(nom, x, y, r_ext, bord, maillage, couleur, n_lon):
    h_col = bord - r_ext                       # la colonne monte jusque sous la demi-sphere
    mat = f'<material><ambient>{couleur[0]}</ambient><diffuse>{couleur[1]}</diffuse></material>'
    return f'''    <model name="{nom}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="colonne"><pose>0 0 {h_col / 2} 0 0 0</pose><geometry><cylinder><radius>{r_ext}</radius><length>{h_col}</length></cylinder></geometry></collision>
        <visual name="v_colonne"><pose>0 0 {h_col / 2} 0 0 0</pose><geometry><cylinder><radius>{r_ext}</radius><length>{h_col}</length></cylinder></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
{collisions_demi_sphere(r_ext - EPAISSEUR / 2, bord, n_lon)}
        <visual name="v_coupe"><pose>0 0 {bord} 0 0 0</pose><geometry><mesh><uri>{maillage}</uri></mesh></geometry>{mat}</visual>
      </link>
    </model>
'''


def fantome(nom, x, y, r_ext, bord, maillage, couleur):
    """Receptacle affiche seulement (sans collision, semi-transparent) : un receptacle qui n'est pas la cible du
    moment. Sur le vrai parcours, il serait hors de portee du bras : le bras peut le traverser."""
    h_col = bord - r_ext
    return f'''    <model name="{nom}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <visual name="v_colonne"><pose>0 0 {h_col / 2} 0 0 0</pose><geometry><cylinder><radius>{r_ext}</radius><length>{h_col}</length></cylinder></geometry>
          <transparency>0.6</transparency><material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
        <visual name="v_coupe"><pose>0 0 {bord} 0 0 0</pose><geometry><mesh><uri>{maillage}</uri></mesh></geometry>
          <transparency>0.5</transparency><material><ambient>{couleur[0]}</ambient><diffuse>{couleur[1]}</diffuse></material></visual>
      </link>
    </model>
'''


def balle_fixe(nom, x, y, couleur):
    """Balle deposee, affichee seulement (elle reste visible dans son receptacle devenu fantome)."""
    return f'''    <model name="{nom}">
      <static>true</static>
      <pose>{x} {y} 0.0115 0 0 0</pose>
      <link name="link">
        <visual name="visual"><geometry><sphere><radius>{R_BALLE}</radius></sphere></geometry>
          <material><ambient>{couleur[0]}</ambient><diffuse>{couleur[1]}</diffuse></material></visual>
      </link>
    </model>
'''


def reglette(x, y, psi):
    """Plaque + 5 anneaux de 15 mm de diametre exterieur, 5 mm de haut (comme la vraie reglette).
    Chaque anneau est fait de 8 petits blocs (rayon 6 a 7,5 mm) : la balle repose sur leurs bords interieurs, a 14,8 mm.
    (Un premier modele en barrettes de 18 mm depassait l'anneau : une machoire butait dessus et la prise ratait.)"""
    c, v = [], []
    gris = '<material><ambient>0.1 0.2 0.7 1</ambient><diffuse>0.1 0.25 0.85 1</diffuse></material>'
    c.append('        <collision name="plaque"><pose>0 0 0.001 0 0 0</pose><geometry><box><size>0.27 0.03 0.002</size></box></geometry></collision>')
    v.append(f'        <visual name="v_plaque"><pose>0 0 0.001 0 0 0</pose><geometry><box><size>0.27 0.03 0.002</size></box></geometry>{gris}</visual>')
    for k in range(5):
        xk = (k - 2) * 0.05
        for n in range(8):
            a = n * np.pi / 4
            g = (f'<pose>{xk + 0.00675 * np.cos(a):.5f} {0.00675 * np.sin(a):.5f} 0.0025 0 0 {a:.4f}</pose>'
                 f'<geometry><box><size>0.0015 0.004 0.005</size></box></geometry>')
            c.append(f'        <collision name="s{k}_{n}">{g}</collision>')
            v.append(f'        <visual name="v_s{k}_{n}">{g}{gris}</visual>')
    return f'''    <model name="reglette">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {psi}</pose>
      <link name="link">
{chr(10).join(c)}
{chr(10).join(v)}
      </link>
    </model>
'''


def balle(nom, x, y, couleur):
    i = float(os.environ.get('INERTIE_BALLE', 0.4 * MASSE_BALLE * R_BALLE ** 2))
    return f'''    <model name="{nom}">
      <pose>{x} {y} 0.0148 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{MASSE_BALLE}</mass>
          <inertia><ixx>{i:.3e}</ixx><iyy>{i:.3e}</iyy><izz>{i:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><sphere><radius>{R_BALLE}</radius></sphere></geometry>
          <material><ambient>{couleur[0]}</ambient><diffuse>{couleur[1]}</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><sphere><radius>{R_BALLE}</radius></sphere></geometry>
          <surface><friction><ode><mu>{FROTTEMENT_BALLE}</mu><mu2>{FROTTEMENT_BALLE}</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
'''


def maillages():
    os.makedirs(MESHES, exist_ok=True)
    for nom, r_ext, r_int in (('coupe_petite', 0.05, 0.04), ('coupe_grande', 0.10, 0.09)):
        creer_coupe.R_EXT, creer_coupe.R_INT = r_ext, r_int
        creer_coupe.ecrire_stl(os.path.join(MESHES, nom + '.stl'))
    return os.path.join(MESHES, 'coupe_petite.stl'), os.path.join(MESHES, 'coupe_grande.stl')


def objets(petite, grande):
    formes = [(0.05, 0.15, petite, 16), (0.05, 0.15, petite, 16), (0.10, 0.50, grande, 32)]
    s = ''
    for i, (r, bord, maillage, n) in enumerate(formes, start=1):
        s += receptacle(f'receptacle_{i}', *GARAGE[f'receptacle_{i}'], r, bord, maillage, COULEURS_RECEPTACLES[i - 1], n)
        s += fantome(f'fantome_{i}', *GARAGE[f'fantome_{i}'], r, bord, maillage, COULEURS_RECEPTACLES[i - 1])
    s += reglette(*GARAGE['reglette'], 0.0)
    for i in range(1, 4):
        g = GARAGE['reglette']
        s += balle(f'balle_{i}', g[0] + (i - 2) * 0.05, g[1], COULEURS_BALLES[i - 1])
        s += balle_fixe(f'balle_fixe_{i}', *GARAGE[f'balle_fixe_{i}'], COULEURS_BALLES[i - 1])
    return s


def monde():
    petite, grande = maillages()
    return f'''<?xml version="1.0" ?>
<!-- FICHIER GENERE par ia_50cm/creer_scene.py : modifier le script plutot que ce fichier.
     Receptacles 1 et 2 : bord a 15 cm ; receptacle 3 : bord a 50 cm. Une balle par receptacle, du plus proche des balles
     au plus eloigne ; seul le receptacle vise est solide, les autres sont affiches en fantomes (fantome_i).
     Positions : ia_50cm/evaluer_receptacles.py les tire au hasard. -->
<sdf version="1.6">
  <world name="scene_trois_receptacles">
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.025 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>20 20 0.05</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>20 20 0.02</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>
{objets(petite, grande)}  </world>
</sdf>
'''


if __name__ == '__main__':
    with open(SORTIE, 'w') as f:
        f.write(monde())
    print('->', os.path.abspath(SORTIE))
