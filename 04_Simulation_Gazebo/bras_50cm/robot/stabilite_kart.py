"""28/11/2026 : EQUILIBRE du robot KART (chassis_kart.py ; copie de stabilite_chassis_diff.py) avec le bras (URDF).
- masses : pieces imprimees (volume x 1,24 g/cm3 x REMPLISSAGE), composants du commerce (valeurs typiques, NON pesees) ;
- centre de gravite au repos et pour TOUT le domaine du bras (lacet -90..90, epaule, coude, poignet sur leur plage) ;
- marge de basculement = distance du centre de gravite au bord du polygone d'appui (billes avant, roues, patins arriere) ;
- effet des accelerations (3 m/s2 en ligne, 3,2 m/s2 en virage : 0,8 m/s dans un rayon de 200 mm) bras replie ;
- inertie de lacet autour du milieu de l'essieu.
Usage : ~/venv_cao/bin/python stabilite_chassis_diff.py <dossier des pieces (chassis_differentiel.py)> <urdf>
"""
import json
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

REMPLISSAGE = 0.5            # gyroide 20 % + 2-3 perimetres : environ la moitie de la matiere pleine
G = 9.81
out, urdf = sys.argv[1], sys.argv[2]
P = json.load(open(os.path.join(out, 'chassis_kart.json')))
XA, ZM, YS, XE, YE = P['XA'], P['ZM'], P['YS'], P['XE'], P['YE']

# ---------------------------------------------------------------- bras (URDF)
U = ET.parse(urdf).getroot()


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def axang(a, t):
    a = np.asarray(a, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def T(R, p):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = p; return M


LIENS = {}
for l in U.findall('link'):
    i = l.find('inertial')
    if i is not None and l.get('name') not in ('robot', 'world'):
        o = i.find('origin')
        LIENS[l.get('name')] = (float(i.find('mass').get('value')), np.array([float(v) for v in o.get('xyz').split()]))
ART = []
for j in U.findall('joint'):
    o = j.find('origin'); ax = j.find('axis'); mi = j.find('mimic')
    ART.append(dict(nom=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'), enfant=j.find('child').get('link'),
                    xyz=np.array([float(v) for v in o.get('xyz').split()]), rpy=[float(v) for v in o.get('rpy').split()],
                    axe=[float(v) for v in ax.get('xyz').split()] if ax is not None else [0, 0, 1],
                    mimic=(mi.get('joint'), float(mi.get('multiplier', 1)), float(mi.get('offset', 0))) if mi is not None else None))


def bras(q):
    pose = {'world': np.eye(4)}; reste = list(ART)
    while reste:
        for a in list(reste):
            if a['parent'] in pose:
                t = 0.0
                if a['type'] == 'revolute':
                    t = q.get(a['nom'], 0.0) if a['mimic'] is None else q.get(a['mimic'][0], 0.0) * a['mimic'][1] + a['mimic'][2]
                pose[a['enfant']] = pose[a['parent']] @ T(rpy(*a['rpy']), a['xyz']) @ T(axang(a['axe'], t), np.zeros(3))
                reste.remove(a)
    M, S = 0.0, np.zeros(3)
    for n, (m, c) in LIENS.items():
        if n in pose:
            M += m; S += m * (pose[n] @ np.r_[c, 1])[:3]
    c = S / M * 1000.0
    return M * 1000.0, np.array([XA + c[0], YS + c[2], ZM - c[1]])     # repere de l'assemblage (x avant, y haut, z travers)


# ---------------------------------------------------------------- robot sans le bras
def piece(nom):
    m = trimesh.load(os.path.join(out, nom + '.stl'))
    return m.volume / 1000.0 * 1.24 * REMPLISSAGE, m.center_mass


masses = []          # (nom, g, xyz)
PIECES = ('platine kart', 'tour du bras', 'nez 5 capteurs et roulette', 'berceau batteries kart')
for nom in PIECES:
    g, c = piece(nom); masses.append((nom, g, c))
for sg in (-1, 1):
    masses.append(('moteur', 29.0, np.array([XE + 21.9, YE, ZM + sg * P['W_MOT']])))
    masses.append(('roue', 12.0, np.array([XE, YE, ZM + sg * P['W_ROUE']])))
    masses.append(('batterie', 45.0, np.array([P['XC_BAT'], P['Y_BAT'], ZM + sg * P['W_BAT']])))
    pass
masses += [('PCB + ESP32 + barrettes (place prevue devant la tour)', 35.0, np.array([XA + 75.0, P['YP1'] + 14.0, ZM])), ('roulette a bille', 15.0, np.array([P['X_ROUL'], YS + 10.0, ZM])),
           ('L298N', 26.0, np.array([P['L298N'][0], P['L298N'][1] + 12.0, ZM])), ('abaisseur + interrupteur', 15.0, np.array([P['abaisseur'][0], P['abaisseur'][1] + 8.0, ZM])),
           ('5 cartes capteurs', 15.0, np.array([XE + P['L_CAPT'] - 12.0, YS + 19.0, ZM])), ('vis, ecrous', 25.0, np.array([XA, P['YP1'], ZM])),
           ('fils', 15.0, np.array([XA, P['YP1'] + 5.0, ZM]))]
m_robot = sum(g for _, g, _ in masses)
c_robot = sum(g * c for _, g, c in masses) / m_robot

# polygone d'appui (x, z) : billes avant, roues (milieu du pneu), patins arriere (a 1 mm du sol)
if 'appui' in P:                                                  # 28/11 : appuis donnes par chassis_kart.py (roues + roulette)
    APPUI = [tuple(q) for q in P['appui']]
    APPUI_NORMAL = APPUI
else:
    xp, wp = P['patins'][0]
    APPUI = [(P['X_BILLE'], ZM + P['W_BILLE']), (XE, ZM + P['W_ROUE']), (xp, ZM + wp), (xp, ZM - wp), (XE, ZM - P['W_ROUE']), (P['X_BILLE'], ZM - P['W_BILLE'])]
    APPUI_NORMAL = [APPUI[0], APPUI[1], APPUI[4], APPUI[5]]
if os.environ.get('APPUI'):                                       # essai : "x1,z1;x2,z2;..." relatifs a l'essieu et au milieu
    APPUI = [(XE + float(q.split(',')[0]), ZM + float(q.split(',')[1])) for q in os.environ['APPUI'].split(';')]
    APPUI_NORMAL = APPUI
if os.environ.get('DX_BRAS'):                                     # essai : bras deplace de DX_BRAS mm vers l'avant
    XA = XA + float(os.environ['DX_BRAS'])


def marge(pt, poly):
    """Distance signee de pt au bord du polygone convexe (positive = dedans)."""
    d = 1e9
    n = len(poly)
    for i in range(n):
        a, b = np.array(poly[i]), np.array(poly[(i + 1) % n])
        t = b - a; nrm = np.array([t[1], -t[0]]) / np.linalg.norm(t)
        cen = np.mean(poly, axis=0)
        if np.dot(cen - a, nrm) < 0:
            nrm = -nrm
        d = min(d, float(np.dot(np.asarray(pt) - a, nrm)))
    return d


print('=== MASSES (pieces imprimees : %.0f %% de matiere ; composants : valeurs typiques, non pesees) ===' % (REMPLISSAGE * 100))
for n, g, c in masses:
    print('  %-34s %6.1f g   x %6.1f  haut. %5.1f  z %6.1f' % (n, g, c[0], c[1] - YS, c[2] - ZM))
home = {'joint_base_bras_2': -0.189, 'joint_base_bras_3': 2.286, 'joint_base_bras_4': 1.170}
mb, cb = bras(home)
M = m_robot + mb
c = (m_robot * c_robot + mb * cb) / M
print('robot sans bras %.0f g, bras %.0f g, TOTAL %.0f g' % (m_robot, mb, M))
print('centre de gravite, bras replie : %.1f mm devant l\'essieu, %.1f mm de haut, %.1f mm de cote' % (c[0] - XE, c[1] - YS, c[2] - ZM))
part_billes = (c[0] - XE) / (P['X_ROUL'] - XE)
print('  -> la roulette avant porte %.0f %% du poids, les roues %.0f %% (adherence)' % (100 * part_billes, 100 * (1 - part_billes)))
print('  marge de basculement (sans les patins) %.1f mm' % marge((c[0], c[2]), APPUI_NORMAL))
h = c[1] - YS
for nom, ax, az in (('freinage 3 m/s2', 3.0, 0.0), ('acceleration 3 m/s2', -3.0, 0.0), ('virage 0,8 m/s R 200 (3,2 m/s2)', 0.0, 3.2)):
    p = (c[0] + h * ax / G, c[2] + h * az / G)
    print('  %-32s : point d\'appui equivalent %+.1f / %+.1f mm, marge %.1f mm (patins compris), %s' % (
        nom, p[0] - XE, p[1] - ZM, marge(p, APPUI), 'sur les billes' if p[0] > XE else 'sur les PATINS ARRIERE (1 mm)'))

# domaine du bras
lim = [(-1.5708, 1.5708), (-0.95, 2.19), (-0.05, 3.09), (-1.2, 1.94)]
pire = (1e9, None); n_tot = 0; n_arriere = 0; pire_normal = (1e9, None); n_bascule = 0; bascule_lacet = {}
for q2 in np.linspace(*lim[1], 17):
    for q3 in np.linspace(*lim[2], 17):
        for q4 in np.linspace(*lim[3], 7):
            mb, cb0 = bras({'joint_base_bras_2': q2, 'joint_base_bras_3': q3, 'joint_base_bras_4': q4})
            d = cb0 - np.array([XA, 0, ZM])
            for q1 in np.radians(np.arange(-90, 91, 15)):
                ca, sa = np.cos(q1), np.sin(q1)
                cbx, cbz = XA + d[0] * ca - d[2] * sa, ZM + d[0] * sa + d[2] * ca
                cx = (m_robot * c_robot[0] + mb * cbx) / M; cz = (m_robot * c_robot[2] + mb * cbz) / M
                n_tot += 1
                mg = marge((cx, cz), APPUI)
                if cx < XE:
                    n_arriere += 1
                if mg < 0:
                    k = int(round(np.degrees(q1))); n_bascule += 1; bascule_lacet[k] = bascule_lacet.get(k, 0) + 1
                if mg < pire[0]:
                    pire = (mg, (np.degrees(q1), q2, q3, q4, cx - XE, cz - ZM))
print('\n=== TOUT LE DOMAINE DU BRAS (%d positions, sans balle) ===' % n_tot)
print('pire marge de basculement : %.1f mm (lacet %.0f deg, q2 %.2f q3 %.2f q4 %.2f rad ; centre de gravite %+.1f / %+.1f mm de l\'essieu)' % (
    pire[0], *pire[1]))
print("positions ou le centre de gravite passe derriere l'essieu : %.0f %%" % (100.0 * n_arriere / n_tot))
print('positions ou le robot BASCULE : %d sur %d (%.1f %%) ; par angle de lacet : %s' % (n_bascule, n_tot, 100.0 * n_bascule / n_tot, dict(sorted(bascule_lacet.items()))))

# inertie de lacet (points + pieces imprimees en masse repartie)
I = 0.0
for n, g, cc in masses:
    if n in PIECES:
        m = trimesh.load(os.path.join(out, n + '.stl'))
        pts = m.sample(4000)
        I += g * np.mean((pts[:, 0] - XE) ** 2 + (pts[:, 2] - ZM) ** 2)
    else:
        I += g * ((cc[0] - XE) ** 2 + (cc[2] - ZM) ** 2)
mb, cb = bras(home)
I += mb * ((cb[0] - XE) ** 2 + (cb[2] - ZM) ** 2)
print('\ninertie de lacet autour du milieu de l\'essieu (bras replie) : %.0f g.cm2 = %.2e kg.m2 ; rayon de giration %.0f mm' % (
    I / 100, I * 1e-9, np.sqrt(I / M)))
