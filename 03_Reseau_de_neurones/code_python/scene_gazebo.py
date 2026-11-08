"""Disposer les objets de la scene a trois receptacles dans Gazebo (deplacer, recreer une balle immobile)."""
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
MONDE = 'scene_trois_receptacles'
FICHIER_MONDE = os.path.expanduser(f'~/ros2_ws/src/mon_bras_robot/worlds/{MONDE}.world')


def _service(nom, reqtype, requete):
    return subprocess.run(['ign', 'service', '-s', f'/world/{MONDE}/{nom}', '--reqtype', reqtype,
                           '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', requete],
                          capture_output=True, text=True, timeout=20).stdout


def deplacer(nom, x, y, z=0.0, lacet=0.0):
    """Deplace un objet (statique ou non), avec son orientation autour de la verticale."""
    _service('set_pose', 'ignition.msgs.Pose',
             f'name: "{nom}", position: {{x: {x}, y: {y}, z: {z}}}, '
             f'orientation: {{x: 0, y: 0, z: {np.sin(lacet / 2)}, w: {np.cos(lacet / 2)}}}')


def recreer(nom, x, y, z):
    """Supprime un objet et le recree, immobile, a (x, y, z), d'apres sa definition dans le fichier du monde."""
    racine = ET.parse(FICHIER_MONDE).getroot()
    modele = next(m for m in racine.iter('model') if m.get('name') == nom)
    sdf = os.path.join(ICI, 'modeles', f'{MONDE}_{nom}.sdf')
    os.makedirs(os.path.dirname(sdf), exist_ok=True)
    with open(sdf, 'w') as f:
        f.write('<?xml version="1.0" ?>\n<sdf version="1.6">\n' + ET.tostring(modele, encoding='unicode') + '\n</sdf>\n')
    _service('remove', 'ignition.msgs.Entity', f'name: "{nom}", type: MODEL')
    time.sleep(0.5)
    _service('create', 'ignition.msgs.EntityFactory',
             f'sdf_filename: "{sdf}", name: "{nom}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}')


def ranger():
    """Receptacles et reglette a l'ecart (derriere le robot, hors de portee) : le bras peut bouger librement."""
    from creer_scene import GARAGE
    for nom, (x, y) in GARAGE.items():
        deplacer(nom, x, y)


def disposer(scene):
    """Reglette et balles a leur place ; les trois receptacles affiches en fantomes (aucun n'est encore vise)."""
    from creer_scene import GARAGE
    for nom in ('balle_fixe_1', 'balle_fixe_2', 'balle_fixe_3'):
        deplacer(nom, *GARAGE[nom], 0.0115)
    for c, m in zip(scene['receptacles'], scene['modeles']):
        deplacer(f'fantome_{m}', c[0], c[1])
    centre, psi = scene['reglette']
    deplacer('reglette', centre[0], centre[1], 0.0, psi)
    for i, b in enumerate(scene['balles'], start=1):
        recreer(f'balle_{i}', b[0], b[1], b[2])


def viser(scene, k):
    """Le receptacle k devient la cible : le vrai (solide) prend la place de son fantome."""
    from creer_scene import GARAGE
    m, c = scene['modeles'][k], scene['receptacles'][k]
    deplacer(f'fantome_{m}', *GARAGE[f'fantome_{m}'])
    deplacer(f'receptacle_{m}', c[0], c[1])


def apres_depot(scene, k, position_balle):
    """Balle k deposee : le receptacle redevient fantome (le robot repart), la balle reste affichee dedans."""
    from creer_scene import GARAGE
    m, c = scene['modeles'][k], scene['receptacles'][k]
    deplacer(f'receptacle_{m}', *GARAGE[f'receptacle_{m}'])
    deplacer(f'fantome_{m}', c[0], c[1])
    deplacer(f'balle_fixe_{k + 1}', *position_balle)
    deplacer(f'balle_{k + 1}', -2.0, 0.3 * k, 0.0115)        # la vraie balle (physique) part hors de la scene
