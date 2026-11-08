"""CAO des pieces neuves du bras 50 cm (CadQuery) : socle, plateau tournant, bras long, avant-bras et brides des servos.

Chaque piece est dessinee dans le repere de son lien URDF (mm), puis exportee en STL (Gazebo, impression 3D)
et en STEP (assemblage SolidWorks : voir generer_assemblage_sw.py). La pince est celle du bras actuel.

Servos : les logements sont cotes sur les modeles SolidWorks fournis (dossier servos/, mesures avec mesurer_servos.py) :
  MG996R : boitier 41 x 20,5 mm, fond -> dessous des pattes 28,1 mm, fond -> dessus du palonnier 47,0 mm,
           pattes de 54 mm (trous a 49,5 x 10 mm), axe a 10,3 mm d'un bout, sortie du cable sous l'axe.
  MG90S / SG90 : boitier 22,6 x 12 mm, pattes 32,3 mm de long a 4,4 mm sous la face de sortie,
           axe a 6 mm d'un bout ; bout de la cannelure a 4,1 mm de la face de sortie.
Visserie : M3 (trous de 3,5) + ecrous dans des empreintes. Les SG90 sont serres par une bride (4 vis M3 a cote du boitier) :
les trous de leurs pattes sont trop petits pour du M3.
Guidage (06/11) : roulements 608 a l'epaule et au coude (le servo ne porte plus seul la liaison), gorge sur le pourtour
du plateau et 3 patins a 120 degres sur le socle (frottement PLA/PLA) : la cinematique de l'URDF ne change pas.
Chaque servo est place (SERVOS) a l'endroit exact que lui impose la cinematique du bras (axes des joints de l'URDF).

Usage : ~/venv_cao/bin/python cao_bras.py     -> pieces/*.stl, pieces/*.step, pieces/masses.json
"""
import json
import os
import numpy as np
import cadquery as cq

ICI = os.path.dirname(os.path.abspath(__file__))
SORTIE = os.path.join(ICI, 'pieces')
DOSSIER_SERVOS = os.path.join(ICI, 'servos')

# ---- Cotes principales (mm) : ce sont elles qu'il faut changer pour modifier le bras ----
L_BRAS = 250.0            # axe d'epaule -> axe de coude
Z_PLATEAU = 50.0          # dessous du plateau tournant, au-dessus du dessus du robot
H_EPAULE = 43.0           # axe d'epaule au-dessus du dessous du plateau
E_PLATEAU = 4.0
JEU = 0.3                 # jeu d'impression autour des MG996R
JEU_SG = 0.15             # jeu autour des SG90/MG90S : 0,4 mm les laissait bouger
M3 = 3.5                  # trou de passage M3
M3_ECROU = 5.8            # ecrou M3 : 5,5 sur plats + jeu
# Roulement 608 (modele et support de l'utilisateur : alesage 8, logement 22, largeur 7)
ROUL_D, ROUL_B = 22.0, 7.0
SIEGE_R = 11.05           # logement du roulement (serre ; poncer ou coller au besoin)
LEVRE_R = 9.0             # trou de la levre qui retient la bague exterieure, la bague interieure reste libre
AXE_R = 3.975             # axe imprime dans l'alesage de 8
COLLET_R = 5.5            # collet qui appuie sur la bague interieure seulement
# Base : gorge sur le pourtour du plateau, dans laquelle entrent 3 patins vissés sur 3 poteaux du socle
R_PLATEAU = 40.0
JANTE_RI, JANTE_H = 33.0, 7.0
GORGE_Z, GORGE_R = (2.5, 5.5), 37.0
ANGLES_PATINS = (0.0, 120.0, 240.0)   # 0 degre : cote +x ; evite les vis de fixation du socle
POTEAU_R, POTEAU_L = (41.0, 48.0), 8.0    # rayons, demi-largeur
POTEAU_HAUT = 46.0        # dessus du poteau (repere du socle)
PATIN_HAUT = 57.0
PATIN_VIS_R, PATIN_VIS_L = 44.5, 5.0
DENSITE_PLA = 1.24e-3     # g/mm3
TAUX_REMPLI = 0.8         # pieces a parois fines : 4 perimetres + 20 % de remplissage ~ 80 % de matiere
# cinematique imposee (URDF) : coude -> poignet dans le repere de l'avant-bras
POIGNET = (-123.4, 16.0, -23.6)
# le servo de poignet est recule de 2 mm (le long de son axe : la cinematique ne change pas) : sa broche, posee sur
# la tour du servo, s'enfonce alors entierement dans l'empreinte de la plaque de la pince (5 mm d'epaisseur)
DECALAGE_POIGNET = -2.0          # > 0 : vers la pince

# ---- Servos, dans le repere de leur modele SolidWorks (mm) ----
# MG996R (modele "MG996R SBORKA") : axe en (15,27 ; 37,23), sortie vers +z
MG_AXE_XY = (15.27, 37.23)
MG_REF_Z = 58.6           # dessus du palonnier
MG_FOND, MG_PATTE_BAS, MG_PATTE_HAUT, MG_DESSUS = 11.6, 39.55, 41.95, 48.1   # coupes au pas de 0,05 mm
MG_X, MG_Y = (5.0, 46.0), (27.0, 47.5)          # boitier
MG_PATTE_X = (-1.5, 52.5)
MG_TROUS = [(0.75, 32.25), (0.75, 42.25), (50.25, 32.25), (50.25, 42.25)]
MG_CABLE = dict(x=(-2.0, 5.0), z=(15.3, 19.3))  # sortie du cable, sous l'axe
# MG90S (modele "Servo Motor Micro MG90S") : axe en (0, 0), sortie vers +z. Le modele s'arrete au dessus de la tour
# des engrenages (r 6,2, 4,3 mm au-dessus du boitier) : la cannelure (~3 mm) n'y est pas dessinee.
SG_REF_Z = -3.2           # dessus de la tour (la broche se pose 0,2 mm au-dessus)
SG_FOND, SG_PATTE_BAS, SG_PATTE_HAUT, SG_DESSUS = -31.8, -14.35, -11.9, -7.5
SG_X, SG_Y = (-6.0, 16.6), (-6.0, 6.0)
SG_PATTE_X = (-10.85, 21.45)


def _pose(R_colonnes, ref_modele, position):
    """Transformation 4x4 modele -> lien : le point `ref_modele` va en `position`, les axes du modele selon R."""
    R = np.array(R_colonnes, float).T          # colonnes = images des axes x, y, z du modele
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(position, float) - R @ np.array(ref_modele, float)
    return T


MG_REF = (MG_AXE_XY[0], MG_AXE_XY[1], MG_REF_Z)
SG_REF = (0.0, 0.0, SG_REF_Z)
# nom -> (modele, lien URDF, pose modele -> lien)
SERVOS = {
    # lacet : axe vertical, palonnier sous le plateau (z = 50), boitier vers +x
    'servo_lacet': ('mg996r', 'base_link', _pose([(1, 0, 0), (0, 1, 0), (0, 0, 1)], MG_REF, (0, 0, Z_PLATEAU))),
    # epaule : axe selon +y, palonnier contre la joue du bras (y = 20), boitier vers le bas
    'servo_epaule': ('mg996r', 'bras un', _pose([(0, 0, -1), (-1, 0, 0), (0, 1, 0)], MG_REF, (0, 20.0, H_EPAULE))),
    # coude : axe selon -z, bout de cannelure sur l'axe du coude (z = 12,6), boitier vers l'epaule
    'servo_coude': ('mg90s', 'bras deux', _pose([(-1, 0, 0), (0, 1, 0), (0, 0, -1)], SG_REF, (L_BRAS, 0, 12.6))),
    # poignet : axe selon -z, bout de cannelure sur l'axe du poignet, boitier vers le coude (au-dela du poignet,
    # la paroi de la pince tourne autour de l'axe a 17 mm et plus : rien ne doit y depasser)
    'servo_poignet': ('mg90s', 'bras trois', _pose([(1, 0, 0), (0, -1, 0), (0, 0, -1)], SG_REF,
                                                   (POIGNET[0], POIGNET[1], POIGNET[2] - DECALAGE_POIGNET))),
}

# ---- Palonniers : pieces d'origine de l'utilisateur (servos/arm01.step = "Servo MG996R arm01", servos/broche.step),
# emboites dans une EMPREINTE a leur forme creusee dans la piece entrainee (comme dans les supports d'origine) :
# c'est l'empreinte qui transmet la rotation ; des vis autotaraudeuses de 2 mm (livrees avec les servos) traversent
# la piece et se vissent dans les trous du palonnier pour le tenir. Cotes dans le repere propre de chaque palonnier.
PALONNIER = {
    # arm01 : moyeu r 6, deux bras (bout r 2,5 a 16,5 mm), plaque z 3,2..5,5, dessus z 5,5 (= MG_REF_Z du servo)
    'arm01': dict(fichier='arm01.step', moyeu=6.0, bout=2.5, l_bras=16.5, bras=(1, -1), plaque=(3.2, 5.5), ref_z=5.5,
                  moyeu_haut=None),
    # broche : moyeu r 3,6 (z -2,8..2,2) avec un alesage cannele de 2,8 mm de profondeur, un bras (bout r 2,5 a
    # 15,4 mm), plaque z -0,8..1,2, nervure de renfort au pied du bras (x 0..5,3, y +-3,9, jusqu'a z 2,05).
    # Le dessous du moyeu se pose 0,2 mm au-dessus de la tour du servo (SG_REF) : ref_z = -3,0.
    'broche': dict(fichier='broche.step', moyeu=3.6, bout=2.5, l_bras=15.4, bras=(1,), plaque=(-0.8, 1.2), ref_z=-3.0,
                   moyeu_haut=2.2, nervure=(-0.5, 5.6, 4.1)),
    # palonnier rond du MG996R : COTES A VERIFIER AU PIED A COULISSE sur le palonnier reel (D_ROND, E_ROND, R_VIS_ROND)
    'rond': dict(fichier='Palonnier rond.step', moyeu=10.5, bout=0.0, l_bras=0.0, bras=(), plaque=(3.5, 5.5), ref_z=5.5,
                 moyeu_haut=None),
}
JEU_EMPREINTE = 0.15      # jeu autour du palonnier dans son empreinte
PAROI_EMPREINTE = 2.5     # paroi de la collerette autour de l'empreinte
VIS_PAL, TETE_VIS_PAL = 2.2, 4.4   # vis 2 mm : trou de passage, lamage de la tete
# palonnier -> servo, type, piece entrainee, joint, direction des bras (repere de la piece), rayons des vis (signe =
# bras), collerette (empreinte en saillie sous une piece mince), longueur du trou de vis, debut du lamage
PALONNIERS = {
    # lacet : palonnier rond, 4 vis autotaraudeuses par le dessous, dans des avant-trous (plateau monte AVANT le servo d'epaule)
    'palonnier_lacet': dict(servo='servo_lacet', type='rond', piece='plateau', joint='joint_base_bras_1',
                            bras=(1, 0, 0), vis=((7.5, 0), (-7.5, 0), (0, 7.5), (0, -7.5)), collerette=True, l_vis=3.2,
                            lamage=None, d_vis=1.7),
    'palonnier_epaule': dict(servo='servo_epaule', type='arm01', piece='bras', joint='joint_base_bras_2',
                             bras=(1, 0, 0), vis=(10.5, 16.5, -10.5, -16.5), collerette=True, l_vis=30, lamage=2.5),
    # broche du coude : sa plaque s'enfonce dans le moyeu de l'avant-bras (dessus a z = -2), pas de collerette
    'broche_coude': dict(servo='servo_coude', type='broche', piece='avant_bras', joint='joint_base_bras_3',
                         bras=(POIGNET[0], POIGNET[1], 0), vis=(7.9, 12.9), collerette=False, l_vis=40, lamage=4.0),
    # la plaque de la pince (45 x 24 mm) n'a la place pour le bras de la broche (20 mm) que le long de son axe x,
    # vers le bloc des machoires (l'empreinte s'arrete 2 mm avant lui)
    'broche_poignet': dict(servo='servo_poignet', type='broche', piece='corps_pince', joint='joint_base_bras_4',
                           bras='x de la pince', vis=(7.9, 12.9), collerette=False, l_vis=3.5, lamage=None),
}


def recalage(nom):
    """Recalages calcules par preparer_palonniers.py (servos/recalages.json)."""
    import json
    return np.array(json.load(open(os.path.join(DOSSIER_SERVOS, 'recalages.json')))[nom])
_JOINTS = {}


def T_joint(nom):
    """Transformation parent -> enfant du joint a q = 0 (mm), lue dans l'URDF."""
    if not _JOINTS:
        import sys
        sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
        from cinematique import Bras
        for j in Bras().joints.values():
            T = j['T'].copy()
            T[:3, 3] *= 1000
            _JOINTS[j['nom']] = T
    return _JOINTS[nom]


def pose_palonnier(nom):
    """Repere propre du palonnier -> repere de la piece entrainee (mm) : sur l'axe du servo, au bon endroit de la
    cannelure, bras dans la direction voulue."""
    p = PALONNIERS[nom]
    d = PALONNIER[p['type']]
    modele, _, T = SERVOS[p['servo']]
    M = np.linalg.inv(T_joint(p['joint'])) @ T                       # modele du servo -> piece entrainee
    ref = np.array(MG_REF if modele == 'mg996r' else SG_REF, float)
    z = M[:3, 2]
    u = recalage('pince_vers_bras_quatre')[:3, 0] if p['bras'] == 'x de la pince' else np.array(p['bras'], float)
    u = u - z * (u @ z)
    u /= np.linalg.norm(u)
    P = np.eye(4)
    P[:3, :3] = np.column_stack([u, np.cross(z, u), z])
    P[:3, 3] = M[:3, :3] @ ref + M[:3, 3] - P[:3, :3] @ np.array([0, 0, d['ref_z']])
    return P


def _contour(d, jeu):
    """Silhouette du palonnier (vue de dessus) agrandie de `jeu` : enveloppe du moyeu et des bouts de bras."""
    from scipy.spatial import ConvexHull
    pts = [(cx + r * np.cos(a), r * np.sin(a)) for cx, r in [(0.0, d['moyeu'] + jeu)] +
           [(s * d['l_bras'], d['bout'] + jeu) for s in d['bras']] for a in np.linspace(0, 2 * np.pi, 96, endpoint=False)]
    pts = np.array(pts)
    return pts[ConvexHull(pts).vertices]


def _dans(P, forme):
    return forme.transformGeometry(cq.Matrix([list(P[0]), list(P[1]), list(P[2])]))


def _prisme(contour, z0, z1, P):
    w = cq.Workplane('XY').workplane(offset=z0).polyline([tuple(p) for p in contour]).close().extrude(z1 - z0).val()
    return _dans(P, w)


def empreinte(nom):
    """(collerette a ajouter ou None, formes a oter) dans le repere de la piece entrainee : empreinte du palonnier
    (ouverte cote servo), logement du moyeu de la broche, trous des vis (+ lamage cote exterieur)."""
    p = PALONNIERS[nom]
    d = PALONNIER[p['type']]
    P = pose_palonnier(nom)
    z0, z1 = d['plaque']
    sortie = 8.0 if p['collerette'] else 1.5                         # l'empreinte debouche du cote du servo
    a_oter = [_prisme(_contour(d, JEU_EMPREINTE), z0 - sortie, z1, P)]
    if d['moyeu_haut'] is not None:                                  # le moyeu de la broche depasse de la plaque
        a_oter.append(_dans(P, cylindre(d['moyeu'] + 0.2, (0, 0, z1 - 0.1), (0, 0, d['moyeu_haut'] + 0.3))))
    if d.get('nervure'):                                             # ... et la nervure au pied de son bras
        x0, x1, y = d['nervure']
        a_oter.append(_dans(P, boite(x0, x1, -y, y, z1 - 0.1, d['moyeu_haut'] + 0.3)))
    for v in p['vis']:
        x, y = v if isinstance(v, tuple) else (v, 0)
        a_oter.append(_dans(P, cylindre(p.get('d_vis', VIS_PAL) / 2, (x, y, z1 - 0.1), (x, y, z1 + p['l_vis']))))
        if p['lamage'] is not None:
            a_oter.append(_dans(P, cylindre(TETE_VIS_PAL / 2, (x, y, z1 + p['lamage']), (x, y, z1 + p['l_vis'] + 1))))
    collerette = None
    if p['collerette']:
        collerette = _prisme(_contour(d, JEU_EMPREINTE + PAROI_EMPREINTE), z0, z1 + 2.0, P)
    return collerette, a_oter


def avec_empreinte(s, nom):
    collerette, a_oter = empreinte(nom)
    if collerette is not None:
        s = unir(s, collerette)
    return oter(s, *a_oter)


def vers_lien(nom, p):
    """Point du modele du servo `nom` -> repere de son lien."""
    T = SERVOS[nom][2]
    return T[:3, :3] @ np.array(p, float) + T[:3, 3]


def boite(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def boite_servo(nom, xs, ys, zs, jeu=0.0):
    """Boite donnee dans le repere du modele du servo, placee dans le repere du lien (axes alignes)."""
    coins = np.array([vers_lien(nom, (x, y, z)) for x in xs for y in ys for z in zs])
    a, b = coins.min(0) - jeu, coins.max(0) + jeu
    return boite(a[0], b[0], a[1], b[1], a[2], b[2])


def cylindre(r, p0, p1):
    v = cq.Vector(*p1) - cq.Vector(*p0)
    return cq.Solid.makeCylinder(r, v.Length, cq.Vector(*p0), v.normalized())


def ecrou(centre, axe, profondeur=2.8, ouverture=None):
    """Empreinte d'ecrou M3 (hexagone) d'axe x, y ou z ; `ouverture` = fente jusqu'au bord pour glisser l'ecrou."""
    plan = {'z': 'XY', 'y': 'XZ', 'x': 'YZ'}[axe]
    h = cq.Workplane(plan).polygon(6, M3_ECROU / 0.866).extrude(profondeur, both=True).val()
    h = h.translate(cq.Vector(*centre))
    if ouverture:
        h = h.fuse(boite(*ouverture))
    return h


def unir(*formes):
    s = formes[0]
    for f in formes[1:]:
        s = s.fuse(f)
    return s.clean()


def oter(s, *formes):
    for f in formes:
        s = s.cut(f)
    return s.clean()


def _tourne(forme, angle):
    return forme.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), angle)


def poteau_patin(angle):
    """Oreille de la plaque + poteau radial qui porte un patin."""
    a, b = POTEAU_R
    return _tourne(unir(boite(30, b + 1, -POTEAU_L - 1, POTEAU_L + 1, 0, 4), boite(a, b, -POTEAU_L, POTEAU_L, 4, POTEAU_HAUT),
                        boite(a - 6, a, -3, 3, 4, 28)), angle)


def trous_poteau(angle):
    """2 vis M3 verticales, ecrous glisses par l'exterieur du poteau."""
    formes = []
    for y in (-PATIN_VIS_L, PATIN_VIS_L):
        formes.append(cylindre(M3 / 2, (PATIN_VIS_R, y, POTEAU_HAUT - 14), (PATIN_VIS_R, y, POTEAU_HAUT + 1)))
        fente = (PATIN_VIS_R, POTEAU_R[1] + 1, y - M3_ECROU / 2, y + M3_ECROU / 2, POTEAU_HAUT - 9, POTEAU_HAUT - 6)
        formes.append(ecrou((PATIN_VIS_R, y, POTEAU_HAUT - 7.5), 'z', 1.5, fente))
    return [_tourne(f, angle) for f in formes]


def patin(angle):
    """Patin de la base : se pose sur le poteau, glisse vers l'axe jusqu'a entrer dans la gorge du plateau, 2 vis M3."""
    z0 = Z_PLATEAU + GORGE_Z[0] + 0.2
    z1 = Z_PLATEAU + GORGE_Z[1] - 0.2
    bloc = oter(boite(38, POTEAU_R[1], -POTEAU_L, POTEAU_L, POTEAU_HAUT + 0.05, PATIN_HAUT), cylindre(R_PLATEAU + 0.4, (0, 0, POTEAU_HAUT), (0, 0, PATIN_HAUT + 1)))
    doigt = cylindre(R_PLATEAU + 1, (0, 0, z0), (0, 0, z1)).cut(cylindre(GORGE_R + 0.4, (0, 0, z0 - 1), (0, 0, z1 + 1))).intersect(boite(30, 45, -POTEAU_L, POTEAU_L, z0, z1))
    vis = [cylindre(M3 / 2, (PATIN_VIS_R, y, POTEAU_HAUT), (PATIN_VIS_R, y, PATIN_HAUT + 1)) for y in (-PATIN_VIS_L, PATIN_VIS_L)]
    return _tourne(oter(unir(bloc, doigt), *vis), angle)


def patin_a():
    return patin(ANGLES_PATINS[0])


def patin_b():
    return patin(ANGLES_PATINS[1])


def patin_c():
    return patin(ANGLES_PATINS[2])


# =========================== SOCLE (lien base_link : z = 0 sur le dessus du robot) ===========================
def socle():
    """Plaque + deux blocs qui portent les pattes du MG996R de lacet (vis M3, ecrous glisses de cote).
    Le fond du servo descend dans une fenetre de la plaque (palonnier sous le plateau a z = 50)."""
    n = 'servo_lacet'
    z_patte = vers_lien(n, (0, 0, MG_PATTE_BAS))[2]                    # dessous des pattes (31,1)
    caisse = boite_servo(n, MG_X, MG_Y, (MG_FOND, MG_DESSUS), JEU)    # boitier
    xa, xb = caisse.BoundingBox().xmin, caisse.BoundingBox().xmax
    plaque = boite(-40, 40, -40, 40, 0, 4)
    bloc_a = boite(-21, xa, -14, 14, 4, z_patte)                       # cote axe (sortie du cable)
    bloc_b = boite(xb, 39.5, -14, 14, 4, z_patte)
    joues = [boite(-21, 39.5, s * 10.9, s * 14, 4, 20) for s in (-1, 1)]
    s = unir(plaque, bloc_a, bloc_b, *joues, *[poteau_patin(a) for a in ANGLES_PATINS])
    fenetre = boite(caisse.BoundingBox().xmin, xb, -10.67, 10.67, -1, z_patte + 1)
    cable = boite(-22, xa + 0.5, -7, 7, 4, vers_lien(n, (0, 0, MG_CABLE['z'][1]))[2] + 2.5)
    trous = []
    for xm, ym in MG_TROUS:
        x, y, _ = vers_lien(n, (xm, ym, 0))
        trous.append(cylindre(M3 / 2, (x, y, 4), (x, y, z_patte + 1)))
        fente = (x - M3_ECROU / 2, x + M3_ECROU / 2, y, 15 * np.sign(y), z_patte - 9, z_patte - 6)
        trous.append(ecrou((x, y, z_patte - 7.5), 'z', 3.0, fente))
    fixation = [cylindre(M3 / 2, (x, y, -1), (x, y, 5)) for x in (-33, 33) for y in (-33, 33)]
    return oter(s, fenetre, cable, *trous, *fixation, *[o for a in ANGLES_PATINS for o in trous_poteau(a)])


# =========================== PLATEAU TOURNANT + BERCEAU D'EPAULE (lien "bras un", z = 0 sous le plateau) ===========================
Y_JOUE_EPAULE = (-37.5, -29.0)   # joue du plateau qui porte le roulement d'epaule (repere du plateau, axe selon y)
Z_JOUE_B = (-42.0, -38.0)        # joue du bras cote roulement (repere du bras : z du bras = y du plateau)


def plateau():
    """Disque vissé sur le palonnier du servo de lacet + face verticale qui porte les pattes du MG996R d'epaule."""
    n = 'servo_epaule'
    za = H_EPAULE                                  # axe d'epaule (le long de y)
    y_patte = vers_lien(n, (0, 0, MG_PATTE_BAS))[1]                    # dessous des pattes (1,1)
    caisse = boite_servo(n, MG_X, MG_Y, (MG_FOND, MG_DESSUS), JEU)
    bb = caisse.BoundingBox()
    y0, y1 = Y_JOUE_EPAULE
    disque = cylindre(R_PLATEAU, (0, 0, 0), (0, 0, E_PLATEAU))
    jante = cylindre(R_PLATEAU, (0, 0, 0), (0, 0, JANTE_H)).cut(cylindre(JANTE_RI, (0, 0, -1), (0, 0, JANTE_H + 1)))
    face = boite(-15, 15, y_patte - 4, y_patte, E_PLATEAU, bb.zmax + 10)
    joue = boite(-13, 13, y0, y1, E_PLATEAU, za)
    bossage = cylindre(14, (0, y0, za), (0, y1, za))                   # porte le roulement d'epaule
    limite = cylindre(R_PLATEAU - 0.5, (0, 0, 0), (0, 0, 30))         # rien ne depasse du plateau : les patins sont autour
    nervures = [boite(sx * 15 - 2, sx * 15 + 2, y0, y_patte, E_PLATEAU, 22).intersect(limite) for sx in (-1, 1)]
    s = unir(disque, jante, face, joue, bossage, *nervures)
    gorge = cylindre(R_PLATEAU + 1, (0, 0, GORGE_Z[0]), (0, 0, GORGE_Z[1])).cut(cylindre(GORGE_R, (0, 0, GORGE_Z[0] - 1), (0, 0, GORGE_Z[1] + 1)))
    siege = cylindre(SIEGE_R, (0, y0 - 1, za), (0, y0 + ROUL_B, za))    # le roulement entre par l'exterieur
    levre = cylindre(LEVRE_R, (0, y0 + ROUL_B - 0.1, za), (0, y1 + 1, za))
    # acces a la vis centrale du palonnier de lacet, par le dessus, avant de monter le servo d'epaule
    acces = cylindre(3.5, (0, 0, E_PLATEAU - 1), (0, 0, bb.zmin + 1))
    fenetre = boite(bb.xmin, bb.xmax, y_patte - 5, y_patte + 1, bb.zmin, bb.zmax)
    trous = []
    for xm, ym in MG_TROUS:
        x, _, z = vers_lien(n, (xm, ym, 0))
        trous.append(cylindre(M3 / 2, (x, y_patte - 5, z), (x, y_patte + 1, z)))
        trous.append(ecrou((x, y_patte - 4, z), 'y', 1.5))
    centre = cylindre(4, (0, 0, -1), (0, 0, 5))                       # tete de la vis centrale du palonnier
    allegement = [cylindre(7, (26 * c, 26 * s, -1), (26 * c, 26 * s, 5)) for c, s in ((0.707, 0.707), (-0.707, 0.707))]
    return avec_empreinte(oter(s, gorge, fenetre, *trous, siege, levre, acces, centre, *allegement), 'palonnier_lacet')


# =========================== BRAS (lien "bras deux" : x le long du bras, z = axe d'epaule) ===========================
BRIDE_VIS = [(sx * 9.0, sy * 9.3) for sx in (-1, 1) for sy in (-1, 1)]   # 4 vis, de part et d'autre du boitier


def logement_sg90(n):
    """Logement d'un SG90/MG90S dont la sortie est vers -z dans le lien : le servo entre par le haut, ses pattes
    se posent au fond d'une poche, la bride les plaque (4 vis M3 a cote du boitier, ecrous par en dessous).
    Renvoie (formes a oter du bloc porteur, z du dessous du bloc, z du dessus du bloc = dessous de la bride, cx, cy)."""
    z1, z2 = (vers_lien(n, (0, 0, z))[2] for z in (SG_PATTE_BAS, SG_PATTE_HAUT))
    z_appui, z_bride = min(z1, z2), max(z1, z2)    # face des pattes cote sortie, face cote fond
    caisse = boite_servo(n, SG_X, SG_Y, (SG_FOND, SG_DESSUS), JEU_SG).BoundingBox()
    pattes = boite_servo(n, SG_PATTE_X, SG_Y, (SG_PATTE_BAS, SG_PATTE_HAUT), JEU_SG).BoundingBox()
    cx, cy = (caisse.xmin + caisse.xmax) / 2, (caisse.ymin + caisse.ymax) / 2
    z_bas = z_appui - 3.0
    a_oter = [boite(caisse.xmin, caisse.xmax, caisse.ymin, caisse.ymax, z_bas - 1, z_bride + 1),
              boite(pattes.xmin, pattes.xmax, pattes.ymin, pattes.ymax, z_appui, z_bride + 1)]
    for dx, dy in BRIDE_VIS:
        a_oter.append(cylindre(M3 / 2, (cx + dx, cy + dy, z_bas - 1), (cx + dx, cy + dy, z_bride + 1)))
        a_oter.append(ecrou((cx + dx, cy + dy, z_bas), 'z', 2.6))
    return a_oter, z_bas, z_bride, cx, cy


def bras():
    L = L_BRAS
    # cote palonnier (z 20..24), cote roulement (Z_JOUE_B), traverse qui les relie au-dela du servo (rayon >= 40)
    zb0, zb1 = Z_JOUE_B
    joue_a = unir(cylindre(20, (0, 0, 20), (0, 0, 24)), boite(0, 46, -10, 10, 20, 24))
    joue_b = unir(cylindre(11, (0, 0, zb0), (0, 0, zb1)), boite(0, 46, -8, 8, zb0, zb1),
                  cylindre(COLLET_R, (0, 0, zb1), (0, 0, Y_JOUE_EPAULE[0])))      # collet sur la bague interieure
    traverse = boite(40, 46, -10, 10, zb0, 24)
    # poutre creuse 20 x 16 mm, parois 1,6 mm, arretee avant les pattes du servo de coude
    a_oter, z_bas, z_pattes, cx, cy = logement_sg90('servo_coude')
    x0, x1 = 40, cx - 18.5
    poutre = oter(boite(x0, x1, -10, 10, 20, 36), boite(x0 - 1, x1 + 1, -8.4, 8.4, 21.6, 34.4))
    # bout du coude : semelle sous la poutre qui porte le SG90 (cannelure sur l'axe du coude)
    bout = boite(L - 50, L + 14, -12.5, 12.5, z_bas, z_pattes)
    fond = boite(L - 50, x1, -10, 10, z_bas, 36)                       # bouchon de la poutre sur la semelle
    s = unir(joue_a, joue_b, traverse, poutre, oter(fond, boite(L - 49, x1 - 1.6, -8.4, 8.4, z_pattes, 34.4)), bout,
             *colonne_coude(z_bas, z_pattes))
    # palonnier arm01 d'epaule : empreinte sur la face interieure de la joue (bras le long du bras), 4 vis par
    # l'exterieur ; passage de la vis centrale ; pivot M3 cote oppose
    centre = cylindre(4, (0, 0, 19), (0, 0, 25))
    trou_axe = cylindre(4.1, (0, 0, zb0 - 1), (0, 0, Y_JOUE_EPAULE[0] + 1))   # l'axe d'epaule se monte par l'exterieur
    allegement = [cylindre(5, (x, 0, 19), (x, 0, 37)) for x in range(int(x0) + 16, int(L - 60), 20)]
    return avec_empreinte(oter(s, *a_oter, centre, trou_axe, *allegement, *trous_colonne()), 'palonnier_epaule')


# ---- Palier du coude : une colonne sort du bras avant le coude, du cote oppose au debattement de l'avant-bras (y < 0),
# descend sous le moyeu et recoit une plaque vissee qui porte le roulement ; l'axe est sous le moyeu de l'avant-bras.
COL_X, COL_VIS_X = (-22.0, 10.0), (-15.0, 3.0)    # par rapport a l'axe du coude
COL_VIS_Y = -21.0


def z_plaque_coude():
    """Dessus de la plaque (repere du bras) : 0,5 mm sous le moyeu de l'avant-bras."""
    return logement_sg90('servo_poignet')[1] + 12.6 - 0.5


def colonne_coude(z_bas, z_pattes):
    x0, x1 = L_BRAS + COL_X[0], L_BRAS + COL_X[1]
    z = z_plaque_coude()
    return [boite(x0, x1, -25, -17, z - 1, 8), boite(x0, x1, -22, -17, 8, z_bas), boite(x0, x1, -22, -12.5, z_bas, z_pattes)]


def trous_colonne():
    z = z_plaque_coude()
    formes = []
    for dx in COL_VIS_X:
        x = L_BRAS + dx
        formes.append(cylindre(M3 / 2, (x, COL_VIS_Y, z - 2), (x, COL_VIS_Y, 7)))
        formes.append(ecrou((x, COL_VIS_Y, 3.4), 'z', 1.4, (x - M3_ECROU / 2, x + M3_ECROU / 2, COL_VIS_Y - 5, COL_VIS_Y, 2.0, 4.8)))
    return formes


def palier_coude():
    """Plaque du roulement du coude, vissee sous la colonne apres avoir mis l'avant-bras en place (2 vis M3)."""
    L, z = L_BRAS, z_plaque_coude()
    z0 = z - ROUL_B - 1.5
    x0, x1 = L + COL_X[0], L + COL_X[1]
    s = unir(cylindre(14, (L, 0, z0), (L, 0, z)), boite(x0, x1, -27, 0, z0, z))
    pose = boite(x0 - 0.1, x1 + 0.1, -25.1, -16.9, z - 1, z + 1)        # la colonne s'emboite de 1 mm : plaque bien calee
    siege = cylindre(SIEGE_R, (L, 0, z - ROUL_B), (L, 0, z + 1))
    levre = cylindre(LEVRE_R, (L, 0, z0 - 1), (L, 0, z - ROUL_B + 0.1))
    vis = [cylindre(M3 / 2, (L + dx, COL_VIS_Y, z0 - 1), (L + dx, COL_VIS_Y, z)) for dx in COL_VIS_X]
    return oter(s, pose, siege, levre, *vis)


def axe_epaule():
    """Axe de l'epaule, monte en dernier par l'exterieur a travers la joue du bras jusque dans le roulement.
    Une vis M3 x 16 le traverse et se serre dans un ecrou loge au bout de l'axe : elle le renforce (couches d'impression)."""
    zb0, _ = Z_JOUE_B
    y0 = Y_JOUE_EPAULE[0]
    bout = y0 + ROUL_B - 0.2
    s = unir(cylindre(7, (0, 0, zb0 - 2.5), (0, 0, zb0)), cylindre(AXE_R, (0, 0, zb0), (0, 0, bout)))
    return oter(s, cylindre(M3 / 2, (0, 0, zb0 - 3), (0, 0, bout + 1)), ecrou((0, 0, bout - 1.2), 'z', 1.3))


def roulement_608():
    """Roulement dans son repere propre : axe z, faces z = 0 et z = 7."""
    return cylindre(ROUL_D / 2, (0, 0, 0), (0, 0, ROUL_B)).cut(cylindre(4, (0, 0, -1), (0, 0, ROUL_B + 1)))


def palonnier_rond():
    """Palonnier rond du MG996R, simplifie, repere comme arm01 : dessus en z = 5,5."""
    d = PALONNIER['rond']
    s = unir(cylindre(d['moyeu'], (0, 0, d['plaque'][0]), (0, 0, d['plaque'][1])), cylindre(6, (0, 0, 0), (0, 0, d['plaque'][0])))
    trous = [cylindre(0.9, (x, y, d['plaque'][0] - 1), (x, y, d['plaque'][1] + 1)) for x, y in PALONNIERS['palonnier_lacet']['vis']]
    return oter(s, *trous)


def detourer_poignet(s, z0, z1):
    """Au-dela de l'axe du poignet (cote bout), ne garde que ce qui est a moins de R_LIBRE de l'axe :
    la paroi de la pince tourne autour de cet axe a 17 mm et plus."""
    x, y, _ = POIGNET
    garde = unir(boite(x, x + 200, y - 100, y + 100, z0 - 1, z1 + 1), cylindre(R_LIBRE, (x, y, z0 - 1), (x, y, z1 + 1)))
    return s.intersect(garde).clean()


R_LIBRE = 15.0


def bride_sg90(n):
    """Cadre qui plaque les pattes d'un SG90 sur son logement (4 vis M3 + ecrous)."""
    _, _, z_pattes, cx, cy = logement_sg90(n)
    caisse = boite_servo(n, SG_X, SG_Y, (SG_FOND, SG_DESSUS), JEU_SG).BoundingBox()
    s = boite(cx - 18, cx + 18, cy - 12.5, cy + 12.5, z_pattes, z_pattes + 3)
    fenetre = boite(caisse.xmin, caisse.xmax, caisse.ymin, caisse.ymax, z_pattes - 1, z_pattes + 4)
    vis = [cylindre(M3 / 2, (cx + dx, cy + dy, z_pattes - 1), (cx + dx, cy + dy, z_pattes + 4)) for dx, dy in BRIDE_VIS]
    s = oter(s, fenetre, *vis)
    return detourer_poignet(s, z_pattes, z_pattes + 3) if SERVOS[n][1] == 'bras trois' else s


def bride_coude():
    return bride_sg90('servo_coude')


# =========================== AVANT-BRAS (lien "bras trois" : z = axe du coude, poignet en POIGNET) ===========================
def avant_bras():
    """Poutre creuse coude -> poignet, moyeu ou s'emboite la broche du SG90 de coude, logement du SG90 de poignet.
    Les efforts du poids sont dans le plan (x, y) : la poutre est haute de 18 mm dans ce plan."""
    a_oter, z_bas, z_pattes, cx, cy = logement_sg90('servo_poignet')
    z_face = -2.0                                  # dessus du moyeu (bout de cannelure du coude en z = 0)
    # poutre et moyeu dessines le long de -x, puis tournes vers le poignet
    angle = np.degrees(np.arctan2(POIGNET[1], -POIGNET[0]))
    long_poutre = 98.0                             # s'arrete avant la bride, pose sur la semelle
    moyeu = cylindre(12, (0, 0, z_bas), (0, 0, z_face))
    poutre = oter(boite(-long_poutre, -6, -9, 9, z_bas, -6), boite(-long_poutre + 3, -7, -7.4, 7.4, z_bas + 1.6, -7.6))
    cale = boite(-21, -6, -6.5, 6.5, z_bas, z_face)   # porte le bras de la broche au-dela du moyeu
    trous = [cylindre(4, (x, 0, z_bas - 1), (x, 0, -5)) for x in (-30, -50, -70)]          # allegement
    bras_local = oter(unir(moyeu, poutre, cale), *trous)
    bras_local = bras_local.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), -angle)
    # semelle qui porte le servo du poignet (boitier vers le coude), detouree autour de l'axe cote bout
    semelle = detourer_poignet(boite(POIGNET[0] - R_LIBRE, cx + 30, cy - 12.5, cy + 12.5, z_bas, z_pattes), z_bas, z_pattes)
    # axe du coude sous le moyeu : collet (bague interieure) + axe dans le roulement ; vis M3 par le dessous avec rondelle,
    # ecrou loge sous la broche. La broche se visse d'abord sur le servo : plus besoin d'acces central.
    collet = cylindre(COLLET_R, (0, 0, z_bas - 0.5), (0, 0, z_bas))
    axe = cylindre(AXE_R, (0, 0, z_bas - 0.5 - ROUL_B + 0.2), (0, 0, z_bas - 0.5))
    centre = cylindre(M3 / 2, (0, 0, z_bas - 0.5 - ROUL_B), (0, 0, -6.0))
    logement_ecrou = ecrou((0, 0, -6.9), 'z', 1.4)
    # broche du coude : empreinte dans le moyeu (bras de la broche vers le poignet), 2 vis par le dessous
    return avec_empreinte(oter(unir(bras_local, semelle, collet, axe), *a_oter, centre, logement_ecrou), 'broche_coude')


def bride_poignet():
    return bride_sg90('servo_poignet')


# =========================== CORPS DE PINCE (lien "bras quatre") ===========================
def corps_pince():
    """La pince d'origine de l'utilisateur (servos/pince_origine.step, recalee dans le repere du lien par
    preparer_palonniers.py) + empreinte de la broche du servo de poignet dans sa plaque de 5 mm (2 vis par
    l'exterieur ; le trou de 5 mm d'origine, sur l'axe, laisse passer le tournevis de la vis centrale)."""
    M = recalage('pince_vers_bras_quatre')
    s = _dans(M, cq.importers.importStep(os.path.join(DOSSIER_SERVOS, 'pince_origine.step')).val())
    return avec_empreinte(s, 'broche_poignet')


PIECES = {'socle': socle, 'plateau': plateau, 'bras': bras, 'bride_coude': bride_coude,
          'avant_bras': avant_bras, 'bride_poignet': bride_poignet, 'corps_pince': corps_pince,
          'palier_coude': palier_coude, 'axe_epaule': axe_epaule, 'patin_a': patin_a, 'patin_b': patin_b, 'patin_c': patin_c}
LIEN_PIECE = {'socle': 'base_link', 'plateau': 'bras un', 'bras': 'bras deux', 'bride_coude': 'bras deux',
              'avant_bras': 'bras trois', 'bride_poignet': 'bras trois', 'corps_pince': 'bras quatre',
              'palier_coude': 'bras deux', 'axe_epaule': 'bras deux', 'patin_a': 'base_link', 'patin_b': 'base_link',
              'patin_c': 'base_link'}
# pieces du commerce dessinees ici, non imprimees : exportees en STEP seulement
COMMERCE_CAO = {'Roulement 608': roulement_608, 'Palonnier rond': palonnier_rond}


def pose_roulement(nom):
    """Repere propre du roulement -> repere de la piece qui le porte (mm)."""
    T = np.eye(4)
    if nom == 'roulement_epaule':                 # axe selon +y du plateau, face exterieure en y0
        T[:3, :3] = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)   # z du roulement = +y du plateau
        T[:3, 3] = (0, Y_JOUE_EPAULE[0], H_EPAULE)
    else:                                         # plaque du coude
        T[:3, 3] = (L_BRAS, 0, z_plaque_coude() - ROUL_B)
    return T


def palonnier_place(nom):
    """Modele du palonnier (piece d'origine) place dans le repere de sa piece entrainee."""
    d = PALONNIER[PALONNIERS[nom]['type']]
    corps = [s for s in cq.importers.importStep(os.path.join(DOSSIER_SERVOS, d['fichier'])).val().Solids()
             if s.Volume() > 0.5]                                    # la broche contient un corps degenere (un point)
    return _dans(pose_palonnier(nom), cq.Compound.makeCompound(corps))


def enveloppe_servo(nom):
    """Forme simplifiee d'un servo place (boitier + pattes + sortie) pour les essais de collision."""
    if SERVOS[nom][0] == 'mg996r':
        ax, ay = MG_AXE_XY
        return unir(boite_servo(nom, MG_X, MG_Y, (MG_FOND, MG_DESSUS)),
                    boite_servo(nom, MG_PATTE_X, MG_Y, (MG_PATTE_BAS, MG_PATTE_HAUT)),
                    boite_servo(nom, (ax - 6, ax + 6), (ay - 6, ay + 6), (MG_DESSUS, MG_REF_Z - 5.5)))
    return unir(boite_servo(nom, SG_X, SG_Y, (SG_FOND, SG_DESSUS)),
                boite_servo(nom, SG_PATTE_X, SG_Y, (SG_PATTE_BAS, SG_PATTE_HAUT)),
                boite_servo(nom, (-6, 8.2), (-6, 6), (SG_DESSUS, SG_REF_Z)))


def modele_servo(modele):
    """Solide du modele SolidWorks (repere du modele, mm)."""
    return cq.importers.importStep(os.path.join(DOSSIER_SERVOS, modele + '.step')).val()


def servo_place(nom):
    modele, _, T = SERVOS[nom]
    m = cq.Matrix([list(T[0]), list(T[1]), list(T[2])])
    return modele_servo(modele).transformGeometry(m)


def preparer_servos():
    """servos/mg996r_origine.step (export de "MG996R SBORKA") -> mg996r.step : boitier SANS l'arbre metallique
    (la rotation se fait entre le palonnier arm01 et le servo)."""
    sortie = os.path.join(DOSSIER_SERVOS, 'mg996r.step')
    s = cq.importers.importStep(os.path.join(DOSSIER_SERVOS, 'mg996r_origine.step')).solids().vals()
    cq.exporters.export(cq.Workplane().add(cq.Compound.makeCompound(s[:5])), sortie)


if __name__ == '__main__':
    os.makedirs(SORTIE, exist_ok=True)
    preparer_servos()
    infos = {}
    for nom, fonction in PIECES.items():
        s = fonction()
        cq.exporters.export(cq.Workplane().add(s), os.path.join(SORTIE, nom + '.stl'), tolerance=0.05, angularTolerance=0.2)
        cq.exporters.export(cq.Workplane().add(s), os.path.join(SORTIE, nom + '.step'))
        bb = s.BoundingBox()
        masse = s.Volume() * DENSITE_PLA * TAUX_REMPLI
        cdg = s.Center()
        infos[nom] = dict(volume_cm3=round(s.Volume() / 1000, 2), masse_g=round(masse, 1),
                          centre_mm=[round(c, 2) for c in (cdg.x, cdg.y, cdg.z)],
                          boite_mm=[round(v, 1) for v in (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)],
                          valide=s.isValid())
        print(f'{nom:14s} {infos[nom]}')
    for nom, fonction in COMMERCE_CAO.items():
        cq.exporters.export(cq.Workplane().add(fonction()), os.path.join(DOSSIER_SERVOS, nom + '.step'))
    json.dump(dict(pieces=infos, L_BRAS=L_BRAS, Z_PLATEAU=Z_PLATEAU, H_EPAULE=H_EPAULE, E_PLATEAU=E_PLATEAU),
              open(os.path.join(SORTIE, 'masses.json'), 'w'), indent=1)
