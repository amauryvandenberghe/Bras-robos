# Robot complet (Assemblage1v2) : poutre des roues avant droite, support 2 rangees de capteurs, vis bras/bati.
for ($i = 0; $i -lt 20 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 15 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$asm = Join-Path $bureau 'Assemblage1v2.SLDASM'
$tipe = Join-Path $bureau 'ICAM\Pieces tipe'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$impr = Join-Path (Split-Path $racine) '02_Impression_3D\Robot (chassis)'
New-Item -ItemType Directory -Force $impr | Out-Null
$png = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)\Robot complet (Assemblage1v2).png'
$step = Join-Path $tipe 'support capteurs 2 rangees.step'
Copy-Item '\\wsl.localhost\Ubuntu-22.04\home\amaury\ros2_ws\src\mon_bras_robot\bras_50cm\robot\support_capteurs.step' $step -Force
Copy-Item '\\wsl.localhost\Ubuntu-22.04\home\amaury\ros2_ws\src\mon_bras_robot\bras_50cm\robot\support_capteurs.stl' (Join-Path $impr 'support capteurs 2 rangees.stl') -Force
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Robot {
    static MathUtility mu;
    static string journal = "";
    static string Sans(string s) {
        var sb = new StringBuilder();
        foreach (var ch in s.Normalize(NormalizationForm.FormD)) if (CharUnicodeInfo.GetUnicodeCategory(ch) != UnicodeCategory.NonSpacingMark) sb.Append(ch);
        return sb.ToString();
    }
    static double[] Pt(MathTransform t, double x, double y, double z) { return (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { x, y, z })).MultiplyTransform(t)).ArrayData; }
    static List<Face2> Faces(Component2 c) {
        object info; var l = new List<Face2>();
        foreach (Body2 b in (object[])c.GetBodies3(0, out info)) foreach (Face2 f in (object[])b.GetFaces()) l.Add(f);
        return l;
    }
    static Face2 Trou(Component2 c, double x, double z) {
        foreach (var f in Faces(c)) {
            var s = (Surface)f.GetSurface();
            if (!s.IsCylinder()) continue;
            var a = (double[])s.CylinderParams;
            if (Math.Abs(a[6] - 0.00175) > 0.0001) continue;
            var o = Pt(c.Transform2, a[0], a[1], a[2]);
            if (Math.Abs(o[0] - x) < 0.0006 && Math.Abs(o[2] - z) < 0.0006) return f;
        }
        return null;
    }
    static Face2 Plan(Component2 c, double y, double airemin) {
        Face2 mieux = null; double aire = airemin;
        foreach (var f in Faces(c)) {
            var s = (Surface)f.GetSurface();
            if (!s.IsPlane()) continue;
            var a = (double[])s.PlaneParams;
            var o = Pt(c.Transform2, a[3], a[4], a[5]);
            if (Math.Abs(o[1] - y) < 0.0002 && f.GetArea() > aire) { aire = f.GetArea(); mieux = f; }
        }
        return mieux;
    }
    static bool Lie(Mate2 mm, HashSet<string> noms) {
        for (int i = 0; i < mm.GetMateEntityCount(); i++) {
            var cc = mm.MateEntity(i).ReferenceComponent; if (cc == null) continue;
            var n = cc.Name2;
            foreach (var k in noms) if (n == k || n.StartsWith(k + "/")) return true;
        }
        return false;
    }
    static int SupprimerContraintes(ModelDoc2 doc, HashSet<string> noms) {
        doc.ClearSelection2(true); int n = 0;
        var fe = (Feature)doc.FirstFeature();
        while (fe != null) {
            if (fe.GetTypeName2() == "MateGroup") {
                var m = (Feature)fe.GetFirstSubFeature();
                while (m != null) { var mm = m.GetSpecificFeature2() as Mate2; if (mm != null && Lie(mm, noms)) { m.Select2(true, 0); n++; } m = (Feature)m.GetNextSubFeature(); }
            }
            fe = (Feature)fe.GetNextFeature();
        }
        if (n > 0) doc.Extension.DeleteSelection2(0);
        doc.ClearSelection2(true);
        return n;
    }
    static Mate2 Verrou(AssemblyDoc ad, ModelDoc2 doc, Component2 a, Component2 b, string quoi) {
        doc.ClearSelection2(true);
        a.Select4(false, null, false); b.Select4(true, null, false);
        int err; var m = ad.AddMate5((int)swMateType_e.swMateLOCK, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
        if (m == null) journal += "  ECHEC verrou " + quoi + " err " + err + "\n";
        doc.ClearSelection2(true);
        return m;
    }
    static MathTransform Deplacement(double[] centre, double angle, double dx, double dz) {
        var p = (MathPoint)mu.CreatePoint(centre);
        var ax = (MathVector)mu.CreateVector(new double[] { 0, 1, 0 });
        var r = (MathTransform)mu.CreateTransformRotateAxis(p, ax, angle);
        var t = (MathTransform)mu.CreateTransform(new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, dx, 0, dz, 1, 0, 0, 0 });
        return (MathTransform)r.Multiply(t);
    }
    static double Volume(ModelDoc2 d) { var mp = (MassProperty)d.Extension.CreateMassProperty(); return mp.Volume * 1e9; }

    // ---------- 1. poutre des roues avant : U supprime, 2 trous pour les vis du support ----------
    static void Poutre(SldWorks sw, string f, string stl) {
        int e = 0, w = 0;
        var d = (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
        double v0 = Volume(d);
        var cibles = new[] { "Boss.-Extru.8", "Enlev. mat.-Extru.3", "Conge3", "Enlev. mat.-Extru.4" };
        d.ClearSelection2(true); int n = 0;
        var fe = (Feature)d.FirstFeature();
        while (fe != null) { if (cibles.Contains(Sans(fe.Name))) { fe.Select2(true, 0); n++; } fe = (Feature)fe.GetNextFeature(); }
        bool sup = d.EditSuppress2();
        d.ClearSelection2(true);
        journal += "poutre : " + n + "/4 fonctions supprimees (" + sup + ")\n";
        bool deja = false;
        fe = (Feature)d.FirstFeature();
        while (fe != null) { if (fe.Name == "Trous vis support capteurs") deja = true; fe = (Feature)fe.GetNextFeature(); }
        if (!deja) {
            bool sel = d.Extension.SelectByID2("Plan de face", "PLANE", 0, 0, 0, false, 0, null, 0);
            d.SketchManager.InsertSketch(true);
            d.SketchManager.CreateCircleByRadius(0, 0.020, 0, 0.00175);
            d.SketchManager.CreateCircleByRadius(0, -0.020, 0, 0.00175);
            d.SketchManager.InsertSketch(true);
            var cut = d.FeatureManager.FeatureCut4(true, false, false, (int)swEndConditions_e.swEndCondThroughAll, (int)swEndConditions_e.swEndCondThroughAll, 0.01, 0.01, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, false);
            if (cut == null) {
                d.ClearSelection2(true);
                var esq = (Feature)d.FeatureByPositionReverse(0);
                esq.Select2(false, 0);
                cut = d.FeatureManager.FeatureCut4(false, true, false, (int)swEndConditions_e.swEndCondThroughAll, (int)swEndConditions_e.swEndCondThroughAll, 0.01, 0.01, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, false);
            }
            if (cut != null) cut.Name = "Trous vis support capteurs";
            journal += "poutre : plan trouve " + sel + ", trous " + (cut != null) + "\n";
        }
        d.ForceRebuild3(false);
        double v1 = Volume(d);
        journal += string.Format("poutre : volume {0:F0} -> {1:F0} mm3\n", v0, v1);
        var b = (double[])((PartDoc)d).GetPartBox(true);
        journal += "poutre : boite " + string.Join(" ", b.Select(x => (x * 1000).ToString("F1"))) + "\n";
        d.Save3(1, ref e, ref w);
        journal += "poutre enregistree err " + e + "\n";
        d.Extension.SaveAs(stl, 0, 1, null, ref e, ref w);
        sw.CloseDoc(d.GetTitle());
    }

    public static string Lancer(string asm, string tipe, string step, string impr, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        mu = (MathUtility)sw.GetMathUtility();
        try {
            int e = 0, w = 0, err = 0;
            Poutre(sw, Path.Combine(tipe, "piece avec les bites.SLDPRT"), Path.Combine(impr, "piece avec les bites (poutre droite).stl"));
            // ---------- 2. support importe ----------
            string sup_f = Path.Combine(tipe, "support capteurs 2 rangees.SLDPRT");
            if (File.Exists(sup_f)) File.Delete(sup_f);
            var imp = (ModelDoc2)sw.LoadFile4(step, "r", sw.GetImportFileData(step), ref err);
            if (imp == null) return journal + "import du support impossible " + err;
            imp.Extension.SaveAs(sup_f, 0, 1, null, ref e, ref w);
            journal += "support : " + sup_f + " (err " + e + "), volume " + Volume(imp).ToString("F0") + " mm3\n";
            // ---------- 3. assemblage ----------
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            if (doc == null) return journal + "assemblage non ouvert " + e;
            var ad = (AssemblyDoc)doc;
            doc.ForceRebuild3(false);
            Func<Component2[]> enfants = () => ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var enf = enfants();
            Func<string, Component2> C = nm => enf.FirstOrDefault(c => c.Name2 == nm);
            var bras = enf.First(c => c.Name2.StartsWith("Bras 50cm"));
            var bati = enf.First(c => c.Name2.StartsWith("bati"));
            var socle = ((object[])bras.GetChildren()).Cast<Component2>().First(c => c.Name2.EndsWith("socle-1"));
            // 3a. bras sur le bati : face + 2 trous en diagonale (comme reparer_v2)
            journal += "contraintes du bras supprimees : " + SupprimerContraintes(doc, new HashSet<string> { bras.Name2 }) + "\n";
            double yb = 0.5297;
            var fb = Plan(bati, yb, 0.01); var fs = Plan(socle, yb, 0.001);
            if (fb == null || fs == null) return journal + "ARRET : plan bati/socle introuvable\n";
            ((Entity)fb).Select4(false, null); ((Entity)fs).Select4(true, null);
            ad.AddMate5((int)swMateType_e.swMateCOINCIDENT, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
            doc.ClearSelection2(true);
            var trous = new[] { new[] { 0.19726, 0.54572 }, new[] { 0.13126, 0.47972 }, new[] { 0.19726, 0.47972 }, new[] { 0.13126, 0.54572 } };
            for (int i = 0; i < 4; i++) {
                var tb = Trou(bati, trous[i][0], trous[i][1]); var ts = Trou(socle, trous[i][0], trous[i][1]);
                journal += string.Format("trou socle ({0:F2},{1:F2}) : bati {2}, socle {3}\n", trous[i][0] * 1000, trous[i][1] * 1000, tb != null, ts != null);
                if (i < 2) {
                    if (tb == null || ts == null) return journal + "ARRET : trou introuvable\n";
                    ((Entity)tb).Select4(false, null); ((Entity)ts).Select4(true, null);
                    ad.AddMate5((int)swMateType_e.swMateCONCENTRIC, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
                    doc.ClearSelection2(true);
                }
            }
            doc.ForceRebuild3(false);
            // 3b. composants qui bougent : on retient leurs positions, on supprime leurs contraintes
            var ancien = enf.First(c => c.Name2.StartsWith("support capteur-"));
            var capteurs = new[] { "capteur-2", "capteur-1", "capteur-3" };                 // gauche, milieu, droite (z croissant)
            var vis = new[] { "socket head cap screw_iso-69", "socket head cap screw_iso-70", "socket head cap screw_iso-71" };
            var ecrous = new[] { "hex nut style 1 gradeab_iso-38", "hex nut style 1 gradeab_iso-39", "hex nut style 1 gradeab_iso-40" };
            var poutreVis = new[] { "socket head cap screw_iso-66", "socket head cap screw_iso-65" };
            var poutreEcr = new[] { "hex nut style 1 gradeab_iso-29", "hex nut style 1 gradeab_iso-30" };
            var colonne = new[] { "socket head cap screw_iso-58", "plain washer large grade ac_iso-14", "plain washer large grade ac_iso-15", "hex nut style 1 gradeab_iso-31" };
            var tous = capteurs.Concat(vis).Concat(ecrous).Concat(poutreVis).Concat(poutreEcr).Concat(colonne).ToArray();
            foreach (var nm in tous) if (C(nm) == null) return journal + "ARRET : composant absent " + nm + "\n";
            var T0 = tous.ToDictionary(nm => nm, nm => (double[])C(nm).Transform2.ArrayData);
            var noms = new HashSet<string>(tous); noms.Add(ancien.Name2);
            journal += "contraintes supprimees (support, capteurs, visserie) : " + SupprimerContraintes(doc, noms) + "\n";
            doc.ClearSelection2(true);
            ancien.Select4(false, null, false);
            journal += "ancien support retire : " + doc.Extension.DeleteSelection2(0) + "\n";
            doc.ClearSelection2(true);
            // 3c. nouveau support a l'origine de l'assemblage, verrouille sur le bati
            var sup = (Component2)ad.AddComponent5(Path.Combine(tipe, "support capteurs 2 rangees.SLDPRT"), 0, "", false, "", 0, 0, 0);
            if (sup == null) return journal + "ARRET : support non insere\n";
            sup.Transform2 = (MathTransform)mu.CreateTransform(new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 });
            Verrou(ad, doc, bati, sup, "support / bati");
            sw.CloseDoc(imp.GetTitle());
            // 3d. rangee arriere : capteurs a 33 mm d'ecart ; rangee avant : copies tournees de 90 degres
            double[] zAnc = { 0.48272, 0.51272, 0.54272 }, zNeu = { 0.47972, 0.51272, 0.54572 };
            int[] sens = { -1, 1, 1 };
            double xa = 0.11826, xv = 0.19930, ya = 0.4707;
            for (int i = 0; i < 3; i++) {
                var mv = Deplacement(new[] { xa, ya, zAnc[i] }, 0, 0, zNeu[i] - zAnc[i]);
                foreach (var nm in new[] { capteurs[i], vis[i], ecrous[i] }) {
                    var c = C(nm);
                    c.Transform2 = (MathTransform)((MathTransform)mu.CreateTransform(T0[nm])).Multiply(mv);
                    Verrou(ad, doc, sup, c, nm);
                }
                // avant : on cherche le sens de rotation qui envoie le connecteur (x + 25 mm) du bon cote
                double ang = Math.PI / 2;
                var essai = Deplacement(new[] { xa, ya, zAnc[i] }, ang, 0, 0);
                var p = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { xa + 0.025, ya, zAnc[i] })).MultiplyTransform(essai)).ArrayData;
                if (Math.Sign(p[2] - zAnc[i]) != sens[i]) ang = -ang;
                var mav = Deplacement(new[] { xa, ya, zAnc[i] }, ang, xv - xa, zNeu[i] - zAnc[i]);
                foreach (var nm in new[] { capteurs[i], vis[i], ecrous[i] }) {
                    var src = C(nm);
                    var cfg = src.ReferencedConfiguration;
                    var c = (Component2)ad.AddComponent5(src.GetPathName(), 0, "", false, cfg, 0, 0, 0);
                    if (c == null) { journal += "  ECHEC copie " + nm + "\n"; continue; }
                    c.Transform2 = (MathTransform)((MathTransform)mu.CreateTransform(T0[nm])).Multiply(mav);
                    Verrou(ad, doc, sup, c, "copie " + nm);
                    journal += "  avant " + i + " : " + c.Name2 + " (" + c.ReferencedConfiguration + ")\n";
                }
            }
            // 3e. vis de la poutre : deplacees sur les nouveaux trous ; colonne : memes positions, reverrouillees
            double[] zpA = { 0.49522, 0.53022 }, zpN = { 0.49272, 0.53272 };
            for (int k = 0; k < 2; k++) {
                var mv = Deplacement(new[] { 0.0, 0, 0 }, 0, 0.17726 - 0.15476, zpN[k] - zpA[k]);
                foreach (var nm in new[] { poutreVis[k], poutreEcr[k] }) {
                    var c = C(nm);
                    c.Transform2 = (MathTransform)((MathTransform)mu.CreateTransform(T0[nm])).Multiply(mv);
                    Verrou(ad, doc, sup, c, nm);
                }
            }
            foreach (var nm in colonne) { var c = C(nm); c.Transform2 = (MathTransform)mu.CreateTransform(T0[nm]); Verrou(ad, doc, sup, c, nm); }
            // 3f. vis du bras dans le bati : 4 x M3 x 12, tete sur la plaque du socle, ecrou sous le bati
            var vref = C("socket head cap screw_iso-26"); var eref = C("hex nut style 1 gradeab_iso-10");
            var Rv = ((double[])vref.Transform2.ArrayData).Take(9).ToArray(); var Re = ((double[])eref.Transform2.ArrayData).Take(9).ToArray();
            foreach (var t in trous) {
                var v = (Component2)ad.AddComponent5(vref.GetPathName(), 0, "", false, vref.ReferencedConfiguration, 0, 0, 0);
                var n2 = (Component2)ad.AddComponent5(eref.GetPathName(), 0, "", false, eref.ReferencedConfiguration, 0, 0, 0);
                if (v == null || n2 == null) { journal += "  ECHEC vis du bras\n"; continue; }
                v.Transform2 = (MathTransform)mu.CreateTransform(Rv.Concat(new[] { t[0], 0.5367, t[1], 1, 0, 0, 0 }).ToArray());
                n2.Transform2 = (MathTransform)mu.CreateTransform(Re.Concat(new[] { t[0], 0.5223, t[1], 1, 0, 0, 0 }).ToArray());
                Verrou(ad, doc, bati, v, "vis bras"); Verrou(ad, doc, bati, n2, "ecrou bras");
                journal += "  vis du bras : " + v.Name2 + " (" + v.ReferencedConfiguration + "), " + n2.Name2 + "\n";
            }
            doc.ForceRebuild3(false);
            // ---------- 4. bilan ----------
            int total = 0, mauvais = 0;
            var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") {
                    var m = (Feature)fe.GetFirstSubFeature();
                    while (m != null) {
                        if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; journal += "ERREUR " + m.Name + " code " + code + "\n"; } }
                        m = (Feature)m.GetNextSubFeature();
                    }
                }
                fe = (Feature)fe.GetNextFeature();
            }
            journal += "contraintes : " + total + ", en erreur : " + mauvais + ", composants : " + enfants().Length + "\n";
            var t2 = (double[])bras.Transform2.ArrayData;
            journal += string.Format("Bras en ({0:F2},{1:F2},{2:F2}) mm\n", t2[9] * 1000, t2[10] * 1000, t2[11] * 1000);
            if (mauvais > 0) return journal + "ARRET : erreurs, assemblage NON enregistre\n";
            // esquisses masquees (traits gris), vue, enregistrement du seul document de tete
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            bool ok = doc.Save3(1, ref e, ref w);
            journal += "assemblage enregistre : " + ok + " err " + e + "\n";
            doc.Extension.SaveAs(png, 0, 1, null, ref e, ref w);
            // interferences
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            if (lst != null) {
                var lignes = new List<KeyValuePair<double, string>>();
                foreach (IInterference it in lst) lignes.Add(new KeyValuePair<double, string>(it.Volume * 1e9, string.Join(" / ", ((object[])it.Components).Cast<Component2>().Select(c => c.Name2))));
                journal += "interferences : " + lignes.Count + "\n";
                foreach (var l in lignes.OrderByDescending(k => k.Key)) journal += string.Format("  {0:F2} mm3 : {1}\n", l.Key, l.Value);
            } else journal += "aucune interference\n";
            mgr.Done();
        } catch (Exception ex) { journal += "EXCEPTION " + ex.Message + "\n" + ex.StackTrace; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return journal;
    }
}
'@
[Robot]::Lancer($asm, $tipe, $step, $impr, $png) | Out-File "$PSScriptRoot\robot_v3.txt" -Encoding utf8
Get-Content "$PSScriptRoot\robot_v3.txt"
