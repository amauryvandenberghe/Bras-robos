"""Assemblage SolidWorks CONTRAINT du bras 50 cm (toute la chaine : socle -> pince), servos et palonniers compris.

Chaque piece imprimee est dans le repere de son lien URDF. Pieces du commerce : les fichiers SolidWorks D'ORIGINE de
l'utilisateur (copies) : "Servo MG996R", "Servo MG996R arm01" (palonnier), "Servo MG90S", "broche" (palonnier du
SG90). Chacun recoit une esquisse 3D "Repere" (axe de sortie / de rotation + une direction) ; la piece qui le porte
recoit une esquisse "Servo_<nom>" ou "Palonnier_<nom>" au meme endroit (repere visuel). Chaque piece est inseree a sa
place exacte, puis UNE contrainte de verrouillage la fixe : le servo dans son logement, le palonnier dans son EMPREINTE
de la piece entrainee (il en devient solidaire), la bride sur sa piece. (Avant le 30/10 : axe + centre + parallele,
en partie redondants : 18 contraintes en conflit a la reconstruction.)
Pivots des 4 servos du bras : comme dans la realite, la rotation se fait entre le PALONNIER et le SERVO (sans l'arbre
metallique) : axes colineaires + centres coincidents. Pivots de la pince (pignon, machoires) : esquisses "Axe_<piece>"
dans la piece parente et "Axe_propre" dans la piece enfant.
Le corps de pince est la pince d'origine (avec l'empreinte de la broche) ; son servo est un MG90S contraint.

Usage : ~/venv_cao/bin/python generer_assemblage_sw.py <dossier Windows de sortie (chemin WSL)>
        puis sous Windows : powershell -ExecutionPolicy Bypass -File <dossier>\\assembler.ps1
"""
import os
import shutil
import sys
import numpy as np
import trimesh

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
from cinematique import Bras, BRAS  # noqa: E402
import cao_bras as C  # noqa: E402

MESHES = os.path.join(ICI, '..', 'meshes')
POSTURE = np.array([0.0, 0.5, 1.4, 0.9])
DLL = 'C:\\Program Files\\SOLIDWORKS Corp\\SOLIDWORKS\\api\\redist\\'
# piece SolidWorks -> (lien URDF, source)
PIECES = {
    'socle': ('base_link', 'cao'), 'plateau': ('bras un', 'cao'), 'bras': ('bras deux', 'cao'),
    'bride_coude': ('bras deux', 'cao'), 'avant_bras': ('bras trois', 'cao'), 'bride_poignet': ('bras trois', 'cao'),
    'corps_pince': ('bras quatre', 'cao'),
    'pignon_pince': ('vis un 4', 'vis_un_4.stl'), 'machoire_a': ('vis deux 4', 'vis_deux_4.stl'),
    'machoire_b': ('vis trois 4', 'vis_trois_4.stl'),
}
BRIDES = {'bride_coude': 'bras', 'bride_poignet': 'avant_bras'}
# pieces du commerce : fichiers SolidWorks d'origine de l'utilisateur (chemins WSL) ; copies dans le dossier de sortie
ORIGINE = '/mnt/c/Users/amaur/Desktop/ICAM/Pieces tipe/'
COMMERCE = {'Servo MG996R': ORIGINE + 'MG996R SERVO/Servo MG996R.SLDPRT',
            'Servo MG996R arm01': ORIGINE + 'MG996R SERVO/Servo MG996R arm01.SLDPRT',
            'Servo MG90S': '/mnt/c/Users/amaur/Desktop/Bras robotisé/01_SolidWorks/2_Pieces d\'origine (copies de vos fichiers)/'
                           'Servo Motor Micro MG90S.SLDPRT',
            # broche.SLDPRT n'est plus sur le Bureau (30/10) : copie d'origine rangee dans 01_SolidWorks/2_Pieces d'origine
            'broche': '/mnt/c/Users/amaur/Desktop/Bras robotisé/01_SolidWorks/2_Pieces d\'origine (copies de vos fichiers)/'
                      'broche.SLDPRT'}
PALONNIER_SW = {'arm01': 'Servo MG996R arm01', 'broche': 'broche'}
# pivots des pieces de la pince (les 4 servos du bras tournent par leur palonnier)
PIVOTS = ['joint_base_bras_44', 'joint_base_bras_45', 'joint_base_bras_46']
JOINTS_SERVOS = ['joint_base_bras_1', 'joint_base_bras_2', 'joint_base_bras_3', 'joint_base_bras_4']


def corps_mg996r_vers_sborka():
    """Repere propre de "Servo MG996R" -> repere de "MG996R SBORKA" (celui des cotes MG_*), mm."""
    return C.recalage('mg996r_corps_vers_sborka')


def repere_commerce(piece_sw):
    """Esquisse "Repere" dans le repere propre de la piece (mm) : point de reference, axe de sortie, direction x."""
    if piece_sw == 'Servo MG996R':
        Minv = np.linalg.inv(corps_mg996r_vers_sborka())
        return Minv[:3, :3] @ np.array(C.MG_REF) + Minv[:3, 3], Minv[:3, 2], Minv[:3, 0]
    if piece_sw == 'Servo MG90S':
        return np.array(C.SG_REF), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
    if piece_sw == 'Servo MG996R arm01':
        return np.array([0, 0, C.PALONNIER['arm01']['ref_z']]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
    return np.array([0, 0, C.PALONNIER['broche']['ref_z']]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
FACETTES_SERVO_PINCE = (3850, 402, 152)        # corps du maillage bras_quatre qui sont le servo de pince


def morceaux_pince():
    m = trimesh.load(os.path.join(MESHES, 'bras_quatre.stl'))
    return m.split(only_watertight=False)


def pose_servo_pince():
    """Pose du MG90S de pince dans le repere "bras quatre" (mm) : son maillage est le meme que celui du servo de poignet
    de l'ancien avant-bras (meme ordre des sommets), dont la pose est connue -> recalage par moindres carres."""
    trois = max(trimesh.load(os.path.join(MESHES, 'bras_trois.stl')).split(only_watertight=False),
                key=lambda c: len(c.faces) if len(c.faces) == 3850 else 0)
    quatre = [c for c in morceaux_pince() if len(c.faces) == 3850][0]
    A, B = trois.vertices * 1000, quatre.vertices * 1000
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    residu = np.sqrt(((A @ R.T + t - B) ** 2).sum(1).mean())
    ancien = np.eye(4)                                   # MG90S -> ancien avant-bras (boitier vers le bout)
    ancien[:3, :3] = np.diag([-1.0, 1.0, -1.0])
    ancien[:3, 3] = (-123.4, 16.0, -26.8)
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, t
    return M @ ancien, residu


def pince_sans_servo(step):
    """Maillage de la pince d'origine sans le servo de pince -> STEP."""
    garde = [c for c in morceaux_pince() if len(c.faces) not in FACETTES_SERVO_PINCE]
    stl = step[:-5] + '_tmp.stl'
    trimesh.util.concatenate(garde).export(stl)
    stl_vers_step(stl, step)
    os.remove(stl)


def stl_vers_step(stl, step):
    """Maillage (m) -> STEP (mm) : facettes cousues en coque, solide si elle est fermee."""
    from OCP.StlAPI import StlAPI_Reader
    from OCP.TopoDS import TopoDS_Shape
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Pnt
    from OCP.TopoDS import TopoDS
    from OCP.TopAbs import TopAbs_SHELL
    forme = TopoDS_Shape()
    StlAPI_Reader().Read(forme, stl)
    couture = BRepBuilderAPI_Sewing(1e-6)
    couture.Add(forme)
    couture.Perform()
    s = couture.SewedShape()
    try:
        if s.ShapeType() == TopAbs_SHELL:
            s = BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(s)).Solid()
    except Exception:
        pass
    t = gp_Trsf()
    t.SetScale(gp_Pnt(0, 0, 0), 1000.0)
    s = BRepBuilderAPI_Transform(s, t, True).Shape()
    # STEP a un seul produit (sinon SolidWorks l'importe comme un assemblage)
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    ecrivain = STEPControl_Writer()
    ecrivain.Transfer(s, STEPControl_AsIs)
    ecrivain.Write(step)


def servos_a_monter(b):
    """(instance, piece SolidWorks, piece porteuse, pose repere propre de la piece -> repere de la porteuse en mm).
    Les palonniers sont portes par la piece qu'ils entrainent (ils sont dans son empreinte)."""
    lien_vers_piece = {}
    for nom, (lien, _) in PIECES.items():
        lien_vers_piece.setdefault(lien, nom)
    liste = []
    for nom, (modele, lien, T) in C.SERVOS.items():
        if modele == 'mg996r':
            liste.append((nom, 'Servo MG996R', lien_vers_piece[lien], T @ corps_mg996r_vers_sborka()))
        else:
            liste.append((nom, 'Servo MG90S', lien_vers_piece[lien], T))
    for pal, d in C.PALONNIERS.items():
        liste.append((pal, PALONNIER_SW[d['type']], d['piece'], C.pose_palonnier(pal)))
    pose, residu = pose_servo_pince()
    print(f'servo de pince recale sur le maillage d\'origine : ecart moyen {residu:.3f} mm')
    liste.append(('servo_pince', 'Servo MG90S', 'corps_pince', pose))
    return liste, lien_vers_piece


def donnees():
    b = Bras()
    q = dict(zip(BRAS, POSTURE))
    servos, lien_vers_piece = servos_a_monter(b)
    parts = list(PIECES) + list(COMMERCE)
    esquisses = {nom: [] for nom in parts}             # piece -> [(nom esquisse, [p0, p1, p0', p1' ...])] (m)
    liaisons, fixations = [], []
    for lien, j in b.joints.items():
        if j['nom'] not in PIVOTS:
            continue
        parent, enfant = lien_vers_piece[j['parent']], lien_vers_piece[lien]
        axe_parent = j['T'][:3, :3] @ j['axe']
        p0 = j['T'][:3, 3]
        esquisses[parent].append((f'Axe_{enfant}', [p0, p0 + 0.03 * axe_parent]))
        esquisses[enfant].append(('Axe_propre', [np.zeros(3), 0.03 * j['axe']]))
        liaisons.append((parent, f'Axe_{enfant}', enfant))
    for piece_sw in COMMERCE:                          # repere propre : axe de sortie / de rotation + direction x
        r, z, x = (v / 1000 if i == 0 else v for i, v in enumerate(repere_commerce(piece_sw)))
        esquisses[piece_sw].append(('Repere', [r, r + 0.03 * z, r, r + 0.03 * x]))
    for inst, piece_sw, porteuse, T in servos:
        r0, z0, x0 = repere_commerce(piece_sw)
        r = (T[:3, :3] @ r0 + T[:3, 3]) / 1000
        z, x = T[:3, :3] @ z0, T[:3, :3] @ x0
        prefixe = 'Palonnier' if piece_sw in PALONNIER_SW.values() else 'Servo'
        esquisses[porteuse].append((f'{prefixe}_{inst}', [r, r + 0.03 * z, r, r + 0.03 * x]))
        fixations.append((porteuse, f'{prefixe}_{inst}', inst))
    # pivots des 4 servos du bras : palonnier / servo (axes colineaires, centres coincidents)
    pivots_servos = [(d['servo'], pal) for pal, d in C.PALONNIERS.items()]
    vers_sw = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)   # SolidWorks : Y en haut
    from cinematique import train_engrenages, PINCE, SERVO_OUVERT
    qd = dict(q)
    qd.update(dict(zip(PINCE, train_engrenages(SERVO_OUVERT))))
    instances = []                                      # (instance, piece SolidWorks, placement 4x4 en m)
    for nom, (lien, _) in PIECES.items():
        instances.append((nom, nom, vers_sw @ b.pose(lien, qd)))
    piece_lien = {nom: lien for nom, (lien, _) in PIECES.items()}
    for inst, piece_sw, porteuse, T in servos:
        S = T.copy()
        S[:3, 3] /= 1000
        instances.append((inst, piece_sw, vers_sw @ b.pose(piece_lien[porteuse], qd) @ S))
    placements = [(i, p, list(T[:3, :3].T.flatten()) + list(T[:3, 3]) + [1.0, 0.0, 0.0, 0.0]) for i, p, T in instances]
    return parts, esquisses, liaisons, fixations, placements, pivots_servos


def cs(v):
    return ', '.join(f'{x:.7f}' for x in v)


CSHARP = r'''
using System;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class AssemblageBras {
    static string journal = "";
    static int nOk = 0, nEchec = 0;
    static void Contrainte(AssemblyDoc ad, int type, string quoi) {
        int err = 0;
        var m = ad.AddMate5(type, (int)swMateAlign_e.swMateAlignCLOSEST, false,
                            0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
        if (m == null) { nEchec++; journal += quoi + " : ECHEC (" + err + ")\n"; } else nOk++;
        ((ModelDoc2)ad).ClearSelection2(true);
    }
    static SketchLine Ligne(Component2 c, string esquisse, int k) {
        var f = (Feature)c.FeatureByName(esquisse);
        if (f == null) return null;
        var segs = (object[])((Sketch)f.GetSpecificFeature2()).GetSketchSegments();
        return k < segs.Length ? (SketchLine)segs[k] : null;
    }
    public static string Lancer(string d) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        try { return Construire(sw, d); }
        catch (Exception ex) { return journal + "ERREUR : " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
    }
    static string Construire(SldWorks sw, string d) {
        // DONNEES
        int err = 0, att = 0;
        // 0. import des STEP -> pieces SolidWorks (PLA pour les pieces imprimees) ; les pieces du commerce sont deja
        //    des copies des fichiers SolidWorks d'origine (pas de STEP) : on les garde telles quelles
        foreach (var nom in pieces) {
            if (!System.IO.File.Exists(d + nom + ".step")) continue;
            if (System.IO.File.Exists(d + nom + ".SLDPRT")) System.IO.File.Delete(d + nom + ".SLDPRT");
            object donnees = sw.GetImportFileData(d + nom + ".step");
            var imp = (ModelDoc2)sw.LoadFile4(d + nom + ".step", "r", donnees, ref err);
            if (imp == null) { journal += nom + " : import impossible (" + err + ")\n"; continue; }
            bool commerce = nom.StartsWith("Servo") || nom.StartsWith("Palonnier");
            if (imp.GetType() == 1 && !commerce)
                ((PartDoc)imp).SetMaterialPropertyName2("", "SOLIDWORKS Materials", "PLA");
            // couleurs : PLA orange, servos noirs, palonniers blancs
            double[] c = nom.StartsWith("Servo") ? new double[] { 0.12, 0.12, 0.16 }
                       : nom.StartsWith("Palonnier") ? new double[] { 0.92, 0.92, 0.92 } : new double[] { 0.95, 0.55, 0.15 };
            imp.MaterialPropertyValues = new double[] { c[0], c[1], c[2], 1, 1, 0.5, 0.3, 0, 0 };
            imp.Extension.SaveAs(d + nom + ".SLDPRT", 0, 1, null, ref err, ref att);
            sw.CloseDoc(imp.GetTitle());
        }
        // 1. esquisses 3D (segments) dans chaque piece
        for (int i = 0; i < pieces.Length; i++) {
            var doc = (ModelDoc2)sw.OpenDoc6(d + pieces[i] + ".SLDPRT", 1, 1, "", ref err, ref att);
            if (doc == null) { journal += pieces[i] + " : ouverture impossible\n"; continue; }
            for (int k = 0; k < esqNoms[i].Length; k++) {
                if (((PartDoc)doc).FeatureByName(esqNoms[i][k]) != null) continue;
                doc.ClearSelection2(true);
                doc.SketchManager.Insert3DSketch(true);
                doc.SketchManager.AddToDB = true;            // pas d'accrochage automatique sur la geometrie voisine
                var p = esqPts[i][k];
                bool ok = true;
                for (int s = 0; s + 5 < p.Length; s += 6)
                    ok &= doc.SketchManager.CreateLine(p[s], p[s + 1], p[s + 2], p[s + 3], p[s + 4], p[s + 5]) != null;
                doc.SketchManager.AddToDB = false;
                doc.SketchManager.Insert3DSketch(true);
                Feature esq = null;
                var f = (Feature)doc.FirstFeature();
                while (f != null) { if (f.GetTypeName2() == "3DProfileFeature") esq = f; f = (Feature)f.GetNextFeature(); }
                if (esq != null) esq.Name = esqNoms[i][k];
                if (!ok || esq == null || ((PartDoc)doc).FeatureByName(esqNoms[i][k]) == null)
                    journal += pieces[i] + " : esquisse " + esqNoms[i][k] + " ECHEC\n";
            }
            doc.Extension.SaveAs(d + pieces[i] + ".SLDPRT", 0, 1, null, ref err, ref att);   // reste ouverte : AddComponent5 le demande
        }
        // 2. assemblage
        string gabarit = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
        var assy = (ModelDoc2)sw.NewDocument(gabarit, 0, 0, 0);
        var ad = (AssemblyDoc)assy;
        var mu = (MathUtility)sw.GetMathUtility();
        var comps = new Dictionary<string, Component2>();
        for (int i = 0; i < instNoms.Length; i++) {
            var c = (Component2)ad.AddComponent5(d + instPieces[i] + ".SLDPRT", 0, "", false, "", 0, 0, 0);
            if (c == null) { journal += instNoms[i] + " : non insere\n"; continue; }
            c.Transform2 = (MathTransform)mu.CreateTransform(placements[i]);
            try { c.Name2 = instNoms[i]; } catch { }
            comps[instNoms[i]] = c;
        }
        comps["socle"].Select4(false, null, false);
        ad.FixComponent();
        assy.ClearSelection2(true);
        // 3. liaisons pivot : segments colineaires + debuts coincidents
        foreach (var l in liaisons) {
            var sp = Ligne(comps[l[0]], l[1], 0); var se = Ligne(comps[l[2]], "Axe_propre", 0);
            if (sp == null || se == null) { journal += l[0] + " -> " + l[2] + " : esquisse introuvable\n"; nEchec++; continue; }
            ((SketchSegment)sp).Select4(false, null); ((SketchSegment)se).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "pivot " + l[0] + " / " + l[2] + " (axe)");
            ((SketchPoint)sp.GetStartPoint2()).Select4(false, null); ((SketchPoint)se.GetStartPoint2()).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "pivot " + l[0] + " / " + l[2] + " (centre)");
        }
        // 3 bis. rotation des 4 servos du bras : entre le palonnier et le servo (axes colineaires, centres coincidents)
        foreach (var l in pivotsServo) {
            var s0 = Ligne(comps[l[0]], "Repere", 0); var p0 = Ligne(comps[l[1]], "Repere", 0);
            if (s0 == null || p0 == null) { journal += l[0] + " / " + l[1] + " : esquisse introuvable\n"; nEchec++; continue; }
            ((SketchSegment)s0).Select4(false, null); ((SketchSegment)p0).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "rotation " + l[0] + " / " + l[1] + " (axe)");
            ((SketchPoint)s0.GetStartPoint2()).Select4(false, null); ((SketchPoint)p0.GetStartPoint2()).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "rotation " + l[0] + " / " + l[1] + " (centre)");
        }
        // 4. servos dans leurs logements, palonniers dans leurs empreintes : UNE contrainte de verrouillage (les deux
        //    pieces sont deja a leur place exacte). Avant le 30/10 : axe + centre + parallele, en partie redondants ->
        //    SolidWorks signalait 18 contraintes sur-contraintes / mal definies a la reconstruction.
        foreach (var l in fixations) {
            comps[l[0]].Select4(false, null, false); comps[l[2]].Select4(true, null, false);
            Contrainte(ad, (int)swMateType_e.swMateLOCK, l[2] + " dans " + l[0] + " (verrouillage)");
        }
        // 5. brides : verrouillees sur leur piece
        foreach (var l in brides) {
            comps[l[0]].Select4(false, null, false); comps[l[1]].Select4(true, null, false);
            Contrainte(ad, (int)swMateType_e.swMateLOCK, l[1] + " sur " + l[0] + " (verrouillage)");
        }
        assy.ForceRebuild3(false);
        journal += string.Format("{0} contraintes ajoutees, {1} echecs\n", nOk, nEchec);
        foreach (var kv in comps) {
            var t = (double[])kv.Value.Transform2.ArrayData;
            journal += string.Format("  {0} en ({1:F0}, {2:F0}, {3:F0}) mm\n", kv.Key, t[9] * 1000, t[10] * 1000, t[11] * 1000);
        }
        assy.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swDisplaySketches, false);   // esquisses d'axes masquees
        assy.ShowNamedView2("*Isometric", 7);
        assy.ViewZoomtofit2();
        int e2 = 0, w2 = 0;
        bool ok2 = assy.Extension.SaveAs(d + "Bras 50cm.SLDASM", 0, 1, null, ref e2, ref w2);
        journal += "Bras 50cm.SLDASM : " + (ok2 ? "enregistre" : "erreur " + e2) + "\n";
        assy.Extension.SaveAs(d + "Bras 50cm.png", 0, 1, null, ref e2, ref w2);
        return journal;
    }
}
'''


def ps1(dossier_win, parts, esquisses, liaisons, fixations, placements, pivots_servos):
    """Script PowerShell : le code C# avec les donnees ecrites directement dedans."""
    def tab(xs):
        return ', '.join(f'"{x}"' for x in xs)
    donnees_cs = (
        f'string[] pieces = {{ {tab(parts)} }};\n'
        + 'string[][] esqNoms = {\n' + ',\n'.join('new string[] { ' + tab(e[0] for e in esquisses[n]) + ' }'
                                                  for n in parts) + ' };\n'
        + 'double[][][] esqPts = {\n' + ',\n'.join('new double[][] { ' + ', '.join(
            'new double[] { ' + cs(np.concatenate(e[1])) + ' }' for e in esquisses[n]) + ' }' for n in parts) + ' };\n'
        + f'string[] instNoms = {{ {tab(p[0] for p in placements)} }};\n'
        + f'string[] instPieces = {{ {tab(p[1] for p in placements)} }};\n'
        + 'double[][] placements = {\n' + ',\n'.join('new double[] { ' + cs(p[2]) + ' }' for p in placements) + ' };\n'
        + 'string[][] liaisons = {\n' + ',\n'.join(f'new string[] {{ {tab(l)} }}' for l in liaisons) + ' };\n'
        + 'string[][] fixations = {\n' + ',\n'.join(f'new string[] {{ {tab(l)} }}' for l in fixations) + ' };\n'
        + 'string[][] pivotsServo = {\n' + ',\n'.join(f'new string[] {{ {tab(l)} }}' for l in pivots_servos) + ' };\n'
        + 'string[][] brides = {\n' + ',\n'.join(f'new string[] {{ {tab(l)} }}' for l in BRIDES.items()) + ' };\n')
    code = CSHARP.replace('        // DONNEES', donnees_cs)
    return f'''# FICHIER GENERE par bras_50cm/generer_assemblage_sw.py : assemblage contraint du bras 50 cm (servos compris)
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) {{ 'SolidWorks est ouvert : enregistre ton travail, ferme-le, puis relance ce script.'; exit 1 }}
$dll = '{DLL}'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
{code}
'@
[AssemblageBras]::Lancer('{dossier_win}')
'''


if __name__ == '__main__':
    dossier = sys.argv[1]       # chemin WSL, ex. ".../Bras robotisé/01_SolidWorks/1_ASSEMBLAGE_FINAL (a utiliser)"
    dossier_win = 'C:\\' + dossier.split('/mnt/c/', 1)[1].replace('/', '\\') + '\\'
    os.makedirs(dossier, exist_ok=True)
    for nom, (lien, source) in PIECES.items():
        step = os.path.join(dossier, nom + '.step')
        if source == 'cao':
            shutil.copy(os.path.join(ICI, 'pieces', nom + '.step'), step)
        elif source == 'pince_sans_servo':
            pince_sans_servo(step)
        else:
            stl_vers_step(os.path.join(MESHES, source), step)
    for piece_sw, source in COMMERCE.items():             # copies des fichiers d'origine (l'original n'est pas touche)
        for ext in ('.step', '.SLDPRT'):
            if os.path.exists(os.path.join(dossier, piece_sw + ext)):
                os.remove(os.path.join(dossier, piece_sw + ext))
        shutil.copy(source, os.path.join(dossier, piece_sw + '.SLDPRT'))
    for ancien in ('Palonnier MG996R.step', 'Palonnier MG996R.SLDPRT'):
        if os.path.exists(os.path.join(dossier, ancien)):
            os.remove(os.path.join(dossier, ancien))
    parts, esquisses, liaisons, fixations, placements, pivots_servos = donnees()
    with open(os.path.join(dossier, 'assembler.ps1'), 'w', encoding='utf-8-sig') as f:
        f.write(ps1(dossier_win, parts, esquisses, liaisons, fixations, placements, pivots_servos))
    print('->', dossier_win)
    print('liaisons (pince) :', liaisons)
    print('fixations :', fixations)
    print('rotations palonnier / servo :', pivots_servos)
