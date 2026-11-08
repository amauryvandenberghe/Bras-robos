"""Ecrit urdf/bras_50cm.urdf : pieces neuves (socle, plateau, bras long) + avant-bras et pince du bras actuel.

Les noms des joints et des liens de la chaine sont ceux du bras actuel (joint_base_bras_1..4, 44/45/46,
"bras un".."bras quatre") : les controleurs, les scripts d'IA et le point de prise (cinematique.OUTIL) restent valables.

Limites des joints = vrais servos :
  - debattement 180 degres (fenetre choisie pour couvrir la tache, voir BUTEES) ;
  - couple maxi (effort) = couple de blocage a 4,8 V : MG996R 0,92 N.m, SG90 0,18 N.m.
    Gazebo ne peut donc pas tenir une pose que les vrais servos ne tiendraient pas.
Usage : ~/venv_cao/bin/python generer_urdf.py
"""
import json
import os
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

ICI = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(ICI, '..'))
ANCIEN = os.path.join(PKG, 'urdf', 'Assemblage1.urdf')
SORTIE = os.path.join(PKG, 'urdf', 'bras_50cm.urdf')
MESH = 'package://mon_bras_robot/meshes/bras_50cm/'

CAO = json.load(open(os.path.join(ICI, 'pieces', 'masses.json')))
L_BRAS = CAO['L_BRAS'] / 1000
Z_PLATEAU = CAO['Z_PLATEAU'] / 1000
H_EPAULE = CAO['H_EPAULE'] / 1000
HAUTEUR_ROBOT = 0.080      # dessus du robot roulant (le bras est fixe au centre de ce dessus)
COTE_ROBOT = 0.200

MG996R = dict(masse=0.055, couple=9.4 * 0.0981, vitesse=3.0)
SG90 = dict(masse=0.009, couple=1.8 * 0.0981, vitesse=3.0)
# Dans Gazebo, une balle de 2 g est ejectee par la pince au serrage (instabilite du calcul des contacts sur un objet
# si leger) : la scene utilise une balle de 50 g, comme celle du bras actuel. Pour que la marge des servos reste celle
# des vrais servos avec une vraie balle de 2 g, chaque servo recoit en plus le couple de ces 48 g en trop, au pire bras
# de levier (axe -> balle, bras tendu) : epaule 47 cm, coude 22 cm, poignet 10 cm.
SURPLUS_BALLE = (0.050 - 0.002) * 9.81
LEVIER_BALLE = {'joint_base_bras_2': 0.474, 'joint_base_bras_3': 0.224, 'joint_base_bras_4': 0.100}


def servo_gazebo(servo, nom_joint):
    return dict(servo, couple=servo['couple'] + SURPLUS_BALLE * LEVIER_BALLE.get(nom_joint, 0.0))
# Fenetres de 180 degres des servos (rad), choisies pour couvrir toutes les poses de la tache (voir verifier_tache.py)
BUTEES = {   # angles mesures pour la tache : lacet +-85, epaule -0,29..1,52, coude 0,82..2,66, poignet -0,79..1,53
    'joint_base_bras_1': (-np.pi / 2, np.pi / 2),
    'joint_base_bras_2': (-0.95, np.pi - 0.95),
    'joint_base_bras_3': (-0.05, np.pi - 0.05),     # inclut 0 : posture d'apparition dans Gazebo
    'joint_base_bras_4': (-1.20, np.pi - 1.20),
}

# Coude : l'avant-bras actuel (repere "bras trois") est pose pour que, a q3 = 0, il prolonge le bras, et que le point
# de prise de la pince soit dans le plan vertical qui contient l'axe de lacet.
POIGNET_DANS_AVANT_BRAS = np.array([-0.1234, 0.016, -0.0236])
Z_OUTIL_DANS_AVANT_BRAS = -0.0126
PSI_COUDE = -np.arctan2(POIGNET_DANS_AVANT_BRAS[1], POIGNET_DANS_AVANT_BRAS[0])


def rpy_de(R):
    """Angles roulis-tangage-lacet (convention URDF) d'une matrice de rotation."""
    p = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-6:
        r, y = np.arctan2(R[2, 1], R[2, 2]), np.arctan2(R[1, 0], R[0, 0])
    else:
        r, y = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    return r, p, y


def f(v):
    return ' '.join(f'{x:.6g}' for x in v)


def inertie(masses):
    """masses = [(m, centre(3), demi-cotes de boite(3))] -> masse, centre, tenseur (somme de boites pleines)."""
    m_tot = sum(m for m, _, _ in masses)
    c = sum(m * np.asarray(p) for m, p, _ in masses) / m_tot
    I = np.zeros((3, 3))
    for m, p, h in masses:
        a, b, cc = 2 * np.asarray(h)
        Ib = m / 12 * np.diag([b * b + cc * cc, a * a + cc * cc, a * a + b * b])
        d = np.asarray(p) - c
        I += Ib + m * (d @ d * np.eye(3) - np.outer(d, d))
    return m_tot, c, I


def piece(nom):
    """(masse, centre, demi-cotes) d'une piece imprimee, depuis la CAO."""
    info = CAO['pieces'][nom]
    b = np.array(info['boite_mm']) / 1000
    return (info['masse_g'] / 1000, np.array(info['centre_mm']) / 1000,
            [(b[1] - b[0]) / 4, (b[3] - b[2]) / 4, (b[5] - b[4]) / 4])   # boite reduite de moitie : pieces creuses


def bloc_inertie(masses):
    m, c, I = inertie(masses)
    return (f'<inertial><origin xyz="{f(c)}" rpy="0 0 0"/><mass value="{m:.5f}"/>'
            f'<inertia ixx="{I[0, 0]:.4e}" ixy="{I[0, 1]:.4e}" ixz="{I[0, 2]:.4e}" iyy="{I[1, 1]:.4e}" '
            f'iyz="{I[1, 2]:.4e}" izz="{I[2, 2]:.4e}"/></inertial>')


def visuel_mesh(fichier, couleur, collision=True):
    s = (f'<visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{MESH}{fichier}" scale="0.001 0.001 0.001"/>'
         f'</geometry><material name="{fichier}"><color rgba="{couleur}"/></material></visual>')
    return s


def boite(nom, centre, cotes, couleur=None, collision=True, visuel=True):
    g = f'<geometry><box size="{f(cotes)}"/></geometry>'
    o = f'<origin xyz="{f(centre)}" rpy="0 0 0"/>'
    s = ''
    if visuel and couleur:
        s += f'<visual>{o}{g}<material name="{nom}"><color rgba="{couleur}"/></material></visual>'
    if collision:
        s += f'<collision name="{nom}">{o}{g}</collision>'
    return s


def joint(nom, parent, enfant, xyz, rpy, axe=None, butees=None, servo=None, mimic=''):
    if axe is None:
        return f'<joint name="{nom}" type="fixed"><parent link="{parent}"/><child link="{enfant}"/><origin xyz="{f(xyz)}" rpy="{f(rpy)}"/></joint>'
    lo, hi = butees
    return (f'<joint name="{nom}" type="revolute"><parent link="{parent}"/><child link="{enfant}"/>'
            f'<origin xyz="{f(xyz)}" rpy="{f(rpy)}"/><axis xyz="{f(axe)}"/>'
            f'<limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{servo["couple"]:.3f}" velocity="{servo["vitesse"]}"/>'
            f'<dynamics damping="0.01" friction="0.0"/>{mimic}</joint>')


SERVO_NOIR = '0.1 0.1 0.12 1'
ORANGE = '0.95 0.55 0.15 1'


def generer(sortie=SORTIE, large=False):
    """large=True : butees a +-pi (seulement pour mesurer les angles dont la tache a besoin)."""
    if large:
        BUTEES.update({n: (-np.pi, np.pi) for n in BUTEES})
    ancien = ET.parse(ANCIEN).getroot()
    liens_anciens = {l.get('name'): l for l in ancien.findall('link')}
    joints_anciens = {j.get('name'): j for j in ancien.findall('joint')}
    out = ['<?xml version="1.0" encoding="utf-8"?>',
           '<!-- FICHIER GENERE par bras_50cm/generer_urdf.py : modifier le script plutot que ce fichier. -->',
           '<robot name="bras_50cm">', '<link name="world"/>']

    # robot roulant (boite 200 x 200 x 80 mm), le bras est fixe au centre de son dessus
    out.append(f'<link name="robot">{bloc_inertie([(1.0, (0, 0, HAUTEUR_ROBOT / 2), (0.1, 0.1, 0.04))])}'
               f'{boite("chassis", (0, 0, HAUTEUR_ROBOT / 2), (COTE_ROBOT, COTE_ROBOT, HAUTEUR_ROBOT), "0.25 0.35 0.55 1")}</link>')
    out.append(joint('world_joint', 'world', 'robot', (0, 0, 0), (0, 0, 0)))

    # socle + servo de lacet
    mg_lacet_c, mg_lacet_h = (0.01015, 0, 0.004 + 0.0185), (0.02035, 0.00985, 0.0185)
    out.append('<link name="base_link">'
               + bloc_inertie([piece('socle'), (MG996R['masse'], mg_lacet_c, mg_lacet_h)])
               + visuel_mesh('socle.stl', ORANGE)
               + boite('servo_lacet', mg_lacet_c, 2 * np.array(mg_lacet_h), SERVO_NOIR, collision=False)
               + boite('socle_c', (0, 0, 0.025), (0.08, 0.08, 0.05)) + '</link>')
    out.append(joint('robot_socle', 'robot', 'base_link', (0, 0, HAUTEUR_ROBOT), (0, 0, 0)))

    # plateau tournant + servo d'epaule (couche, axe le long de y)
    mg_ep_c, mg_ep_h = (0, -0.0054, H_EPAULE - 0.01015), (0.00985, 0.0185, 0.02035)
    out.append('<link name="bras un">'
               + bloc_inertie([piece('plateau'), (MG996R['masse'], mg_ep_c, mg_ep_h)])
               + visuel_mesh('plateau.stl', ORANGE)
               + boite('servo_epaule', mg_ep_c, 2 * np.array(mg_ep_h), SERVO_NOIR, collision=False)
               + boite('plateau_c', (0, 0, 0.002), (0.08, 0.08, 0.004))
               + boite('berceau_c', (0, -0.012, 0.034), (0.03, 0.04, 0.06)) + '</link>')
    out.append(joint('joint_base_bras_1', 'base_link', 'bras un', (0, 0, Z_PLATEAU), (0, 0, 0), (0, 0, 1),
                     BUTEES['joint_base_bras_1'], MG996R))

    # bras long + servo de coude (SG90) + bride
    sg_c, sg_h = (L_BRAS - 0.00535, 0, 0.0235 + 0.01135), (0.01125, 0.0061, 0.01135)
    out.append('<link name="bras deux">'
               + bloc_inertie([piece('bras'), piece('bride_coude'), (SG90['masse'], sg_c, sg_h)])
               + visuel_mesh('bras.stl', ORANGE)
               + visuel_mesh('bride_coude.stl', ORANGE)
               + boite('servo_coude', sg_c, 2 * np.array(sg_h), SERVO_NOIR, collision=False)
               + boite('poutre_c', ((40 + L_BRAS * 1000 - 24) / 2000, 0, 0.028), ((L_BRAS * 1000 - 64) / 1000, 0.02, 0.016))
               + boite('bout_c', (L_BRAS - 0.005, 0, 0.034), (0.038, 0.025, 0.028)) + '</link>')
    R_epaule = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)   # x du bras vers le haut, axe z du bras = y du plateau
    out.append(joint('joint_base_bras_2', 'bras un', 'bras deux', (0, 0, H_EPAULE), rpy_de(R_epaule), (0, 0, 1),
                     BUTEES['joint_base_bras_2'], servo_gazebo(MG996R, 'joint_base_bras_2')))

    # avant-bras et pince : liens du bras actuel, inchanges
    for nom in ['bras trois', 'bras quatre', 'vis un 4', 'vis deux 4', 'vis trois 4']:
        out.append(ET.tostring(liens_anciens[nom], encoding='unicode'))
    out.append(joint('joint_base_bras_3', 'bras deux', 'bras trois', (L_BRAS, 0, -Z_OUTIL_DANS_AVANT_BRAS),
                     (0, 0, PSI_COUDE), (0, 0, 1), BUTEES['joint_base_bras_3'], servo_gazebo(SG90, 'joint_base_bras_3')))
    j4 = joints_anciens['joint_base_bras_4']
    o = j4.find('origin')
    out.append(joint('joint_base_bras_4', 'bras trois', 'bras quatre', list(map(float, o.get('xyz').split())),
                     list(map(float, o.get('rpy').split())), list(map(float, j4.find('axis').get('xyz').split())),
                     BUTEES['joint_base_bras_4'], servo_gazebo(SG90, 'joint_base_bras_4')))
    for nom in ['joint_base_bras_44', 'joint_base_bras_45', 'joint_base_bras_46']:
        j = joints_anciens[nom]
        lim = j.findall('limit')
        for l in lim[1:]:
            j.remove(l)
        if os.environ.get('EFFORT_PINCE'):     # essais : couple de pince realiste (MG90S ~0,2 N.m) au lieu de 10
            lim[0].set('effort', os.environ['EFFORT_PINCE'])
        out.append(ET.tostring(j, encoding='unicode'))

    # ros2_control et frottement des machoires : comme le bras actuel
    for g in ancien.findall('gazebo'):
        out.append(ET.tostring(g, encoding='unicode'))
    out.append(ET.tostring(ancien.find('ros2_control'), encoding='unicode'))
    out.append('</robot>')
    texte = '\n'.join(out)
    ET.fromstring(texte.split('\n', 1)[1])   # verifie que le XML est bien forme
    open(sortie, 'w').write(texte)
    print('ecrit', sortie)


if __name__ == '__main__':
    import sys
    if '--large' in sys.argv:
        generer(sys.argv[sys.argv.index('--large') + 1], large=True)
    else:
        generer()
