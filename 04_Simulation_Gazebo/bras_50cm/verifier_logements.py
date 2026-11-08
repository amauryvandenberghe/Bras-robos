"""Verifie que les vrais modeles de servos (SolidWorks) entrent dans leurs logements, sans toucher les pieces
imprimees, et que les pieces ne se touchent pas quand les joints tournent. Ecrit aussi des vues PNG.

  - interference servo / pieces de son lien (volume commun en mm3, doit etre ~0) ;
  - jeu entre le servo et les pieces (distance mini) ;
  - coude : bras + servo de coude contre avant-bras + servo de poignet, sur toute la course du joint ;
  - poignet : avant-bras contre la pince (maillage), sur toute la course du joint.
Usage : ~/venv_cao/bin/python verifier_logements.py
"""
import os
import sys
import numpy as np
import cadquery as cq
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
import cao_bras as C                     # noqa: E402
from cinematique import Bras             # noqa: E402  (avant apercu, qui ajoute ../ia au chemin)
from apercu import dessiner, cadrer      # noqa: E402

pieces = {n: cq.importers.importStep(os.path.join(C.SORTIE, n + '.step')).val() for n in C.PIECES}
servos = {n: C.servo_place(n) for n in C.SERVOS}
# palonniers dans le repere de leur piece entrainee : le corps du palonnier seul (la broche contient aussi sa vis et
# un insert cannele, qui se chevauchent : les operations booleennes d'OpenCascade plantent dessus)
palonniers = {p: max(C.palonnier_place(p).Solids(), key=lambda x: x.Volume()) for p in C.PALONNIERS}


def commun(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return float('nan')


def balayage(joint, q):
    """Volumes communs pour un angle d'un balayage, calcules par un programme Python neuf : OpenCascade plante
    parfois (erreur de segmentation) apres une longue suite d'operations ; un plantage ne coute que cet angle."""
    import subprocess
    r = subprocess.run([sys.executable, '-W', 'ignore', os.path.join(ICI, 'balayage_un_angle.py'), joint, f'{float(q):.6f}'],
                       capture_output=True, text=True, timeout=900)
    try:
        return [float(x) for x in r.stdout.strip().splitlines()[-1].split()]
    except (IndexError, ValueError):
        return [float('nan'), float('nan')]


def maillage(forme, tol=0.2):
    v, f = forme.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([(p.x, p.y, p.z) for p in v]), np.array(f))


print('== Servos dans leurs logements (volume commun mm3 / jeu mini mm)')
bilan_ok = True
for n, (modele, lien, T) in C.SERVOS.items():
    m_servo = maillage(servos[n])
    for p, lp in C.LIEN_PIECE.items():
        if lp != lien:
            continue
        v = commun(servos[n], pieces[p])
        m_p = maillage(pieces[p])
        d = trimesh.proximity.closest_point(m_p, m_servo.sample(4000, ))[1].min()
        ok = v < 1.0
        bilan_ok &= ok
        print(f'  {n:14s} / {p:13s} : {v:8.2f} mm3   jeu mini {d:5.2f} mm   {"ok" if ok else "INTERFERENCE"}')
print('== Palonniers dans leur empreinte (volume commun mm3 ; ecart palonnier / fond de l\'empreinte en mm)')
for p, d in C.PALONNIERS.items():
    piece = pieces[d['piece']]
    v = commun(palonniers[p], piece)
    # le servo, ramene dans le repere de la piece entrainee (a q = 0)
    M = np.linalg.inv(C.T_joint(d['joint'])) @ C.SERVOS[d['servo']][2]
    s = C.modele_servo(C.SERVOS[d['servo']][0]).transformGeometry(cq.Matrix([list(M[0]), list(M[1]), list(M[2])]))
    # contre le servo, hors de la cannelure (r < 3 mm : le modele du palonnier n'a pas d'alesage cannele, la
    # cannelure du servo le traverse ; dans la realite elle y entre)
    P = C.pose_palonnier(p)
    cannelure = C._dans(P, C.cylindre(3.0, (0, 0, -12), (0, 0, 8)))
    vs = commun(palonniers[p].cut(cannelure), s)
    # contact : points du dessus de la plaque, sur les bras, a cote de l'axe des trous de vis (~ 0 : appui au fond)
    t = C.PALONNIER[d['type']]
    haut = [P[:3, :3] @ (sg * f * t['l_bras'], cote, t['plaque'][1] + 0.05) + P[:3, 3]
            for sg in t['bras'] for f in (0.55, 0.75, 0.9) for cote in (-1.6, 1.6)]
    m_p = maillage(piece, 0.1)
    fond = np.abs(trimesh.proximity.signed_distance(m_p, np.array(haut))).max()
    ok = v < 1.0 and vs < 1.0 and fond < 0.3
    bilan_ok &= ok
    print(f'  {p:16s} / {d["piece"]:11s} : {v:6.2f} mm3 | / servo : {vs:6.2f} mm3 | ecart au fond {fond:4.2f} mm  '
          f'{"ok" if ok else "A CORRIGER"}')

# ---- coude : rotation de l'avant-bras sous le bras ----
b = Bras()
j3 = [j for j in b.joints.values() if j['nom'] == 'joint_base_bras_3'][0]
j4 = [j for j in b.joints.values() if j['nom'] == 'joint_base_bras_4'][0]


def tf(T, q, axe):
    """Transformation parent <- enfant (mm) pour l'angle q du joint."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_rotvec(np.array(axe) * q).as_matrix()
    M = T.copy()
    M[:3, 3] *= 1000
    D = np.eye(4)
    D[:3, :3] = R
    return M @ D


groupe_ab = [pieces['avant_bras'], pieces['bride_poignet'], C.enveloppe_servo('servo_poignet')]
for titre, joint, i in (('Coude : bras + servo de coude contre avant-bras + servo de poignet ; broche du coude contre '
                         'le bras', 'coude', 2),
                        ('Poignet : pince d\'origine (avec son empreinte) contre avant-bras + vrai servo de poignet ; '
                         'broche du poignet contre l\'avant-bras', 'poignet', 3)):
    print(f'== {titre} (volume commun mm3)')
    pire = 0
    for q in np.linspace(b.bas[i], b.haut[i], 7):
        v, vb = balayage(joint, q)
        pire = max(pire, v + vb) if v + vb == v + vb else float('nan')
        print(f'  q = {q:5.2f} rad : {v:7.2f}  (broche {vb:5.2f})', flush=True)
    bilan_ok &= pire < 1.0

# ---- poignet : la pince tourne sous l'avant-bras ----
print('== Poignet : pince contre avant-bras (points de la pince a l\'interieur / jeu mini mm)')
m_ab = [maillage(g) for g in groupe_ab]
pince = trimesh.load(os.path.join(ICI, '..', 'meshes', 'bras_quatre.stl'))
pince.apply_scale(1000)
pts = pince.sample(6000)
for q in np.linspace(b.bas[3], b.haut[3], 9):
    M = tf(j4['T'], q, j4['axe'])
    p = pts @ M[:3, :3].T + M[:3, 3]
    d = np.max([trimesh.proximity.signed_distance(m, p) for m in m_ab], axis=0)   # > 0 a l'interieur
    print(f'  q4 = {q:5.2f} rad : {int((d > 0.05).sum()):4d} points dedans, jeu mini {-d.max():6.2f} mm')

print('BILAN :', 'tous les servos entrent, aucune interference' if bilan_ok else 'A CORRIGER')

# ---- vues ----
fig = plt.figure(figsize=(22, 11))
vues = [('socle', ['socle'], ['servo_lacet'], (25, -50)), ('plateau', ['plateau'], ['servo_epaule'], (20, -130)),
        ('bout du bras (coude)', ['bras', 'bride_coude'], ['servo_coude'], (30, -60)),
        ('avant-bras (poignet)', ['avant_bras', 'bride_poignet'], ['servo_poignet'], (35, -120))]
for i, (titre, ps, ss, (el, az)) in enumerate(vues):
    for k, (el2, az2) in enumerate([(el, az), (-25, az + 180)]):
        ax = fig.add_subplot(2, 4, i + 1 + 4 * k, projection='3d')
        tout = []
        for p in ps:
            m = maillage(pieces[p], 0.3)
            if p == 'bras':
                m = m.slice_plane([200, 0, 0], [1, 0, 0])
            dessiner(ax, m.vertices[m.faces], 'tab:orange', 6000)
            tout.append(m.vertices)
        for s in ss:
            m = maillage(servos[s], 0.3)
            dessiner(ax, m.vertices[m.faces], '#303040', 4000)
            tout.append(m.vertices)
        cadrer(ax, np.vstack(tout)); ax.view_init(el2, az2); ax.set_title(titre)
plt.tight_layout(); plt.savefig(os.path.join(C.SORTIE, 'logements_servos.png'), dpi=70)
