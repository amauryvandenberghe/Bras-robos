# 08/11 : capteurs resserres a 13 mm. Encoche de la poutre, reimport du support, 6 capteurs + visserie repositionnes. A NE PAS RELANCER (historique).
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
Get-Process SLDWORKS -ErrorAction SilentlyContinue | Where-Object { -not $_.MainWindowTitle } | Stop-Process -Force -Confirm:$false
Start-Sleep 3
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$asm = Join-Path $bureau 'Assemblage1v2.SLDASM'
$tipe = Join-Path $bureau 'ICAM\Pieces tipe'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$impr = Join-Path (Split-Path $racine) '02_Impression_3D\Robot (chassis)'
$arch = Join-Path $racine '3_Anciennes versions\3j_08-11 capteurs a 33 mm'
New-Item -ItemType Directory -Force $arch | Out-Null
if (-not (Test-Path (Join-Path $arch 'Assemblage1v2.SLDASM'))) {
    Copy-Item $asm $arch
    foreach ($n in 'piece avec les bites.SLDPRT', 'support capteurs 2 rangees.SLDPRT', 'support capteurs 2 rangees.step') { Copy-Item (Join-Path $tipe $n) $arch }
    Copy-Item (Join-Path $impr 'support capteurs 2 rangees.stl') $arch
}
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\ros2_ws\src\mon_bras_robot\bras_50cm\robot\'
Copy-Item ($src + 'support_capteurs.step') (Join-Path $tipe 'support capteurs 2 rangees.step') -Force
Copy-Item ($src + 'support_capteurs.stl') (Join-Path $impr 'support capteurs 2 rangees.stl') -Force
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Cap {
    static string j = "";
    static double Vol(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Volume * 1e9; }
    static bool Lie(Mate2 mm, HashSet<string> noms) {
        for (int i = 0; i < mm.GetMateEntityCount(); i++) { var cc = mm.MateEntity(i).ReferenceComponent; if (cc == null) continue; var n = cc.Name2; foreach (var k in noms) if (n == k || n.StartsWith(k + "/")) return true; }
        return false;
    }
    static int Suppr(ModelDoc2 doc, HashSet<string> noms) {
        doc.ClearSelection2(true); int n = 0; var fe = (Feature)doc.FirstFeature();
        while (fe != null) {
            if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { var mm = m.GetSpecificFeature2() as Mate2; if (mm != null && Lie(mm, noms)) { m.Select2(true, 0); n++; } m = (Feature)m.GetNextSubFeature(); } }
            fe = (Feature)fe.GetNextFeature();
        }
        if (n > 0) doc.Extension.DeleteSelection2(0);
        doc.ClearSelection2(true); return n;
    }
    static void Encoche(SldWorks sw, string f, string stl, double[] Tp) {
        int e = 0, w = 0;
        var d = (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
        sw.Visible = true;
        d = (ModelDoc2)sw.ActivateDoc3(d.GetTitle(), false, 0, ref e);
        double v0 = Vol(d);
        d.ClearSelection2(true);
        var e3 = (Feature)((PartDoc)d).FeatureByName("Esquisse3");
        Feature plan = null;
        foreach (var p in (object[])e3.GetParents()) { var pf = p as Feature; if (pf != null && pf.GetTypeName2() == "RefPlane") plan = pf; }
        plan.Select2(false, 0);
        d.SketchManager.InsertSketch(true);
        d.SketchManager.AddToDB = true;
        var ska = (Sketch)d.SketchManager.ActiveSketch;
        var mu = (MathUtility)sw.GetMathUtility();
        var m2s = (MathTransform)ska.ModelToSketchTransform;
        var a2p = (MathTransform)((MathTransform)mu.CreateTransform(Tp)).Inverse();
        Func<double, double, double[]> S = (xa, za) => {
            var pp = (MathPoint)((MathPoint)mu.CreatePoint(new[] { xa, 0.49, za })).MultiplyTransform(a2p);
            return (double[])((MathPoint)pp.MultiplyTransform(m2s)).ArrayData;
        };
        var q1 = S(0.1833, 0.5077); var q2 = S(0.1880, 0.5411);
        d.SketchManager.CreateCornerRectangle(q1[0], q1[1], 0, q2[0], q2[1], 0);
        d.SketchManager.AddToDB = false;
        d.SketchManager.InsertSketch(true);
        var esq = (Feature)d.FeatureByPositionReverse(0);
        Feature cut = null;
        foreach (var dir in new[] { false, true }) {
            d.ClearSelection2(true);
            d.Extension.SelectByID2(esq.Name, "SKETCH", 0, 0, 0, false, 0, null, 0);
            cut = d.FeatureManager.FeatureCut4(true, false, dir, (int)swEndConditions_e.swEndCondThroughAll, (int)swEndConditions_e.swEndCondThroughAll, 0.01, 0.01, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, false);
            j += "encoche dir=" + dir + " : " + (cut != null) + "\n";
            if (cut != null) break;
        }
        if (cut == null) throw new Exception("encoche impossible");
        cut.Name = "Encoche carte capteur";
        d.ForceRebuild3(false);
        double dv = v0 - Vol(d);
        j += string.Format("encoche : volume enleve {0:F0} mm3\n", dv);
        if (dv < 50 || dv > 3000) throw new Exception("volume enleve anormal, poutre NON enregistree");
        d.Save3(1, ref e, ref w); j += "poutre enregistree err " + e + "\n";
        d.Extension.SaveAs(stl, 0, 1, null, ref e, ref w);
        sw.CloseDoc(d.GetTitle());
        sw.Visible = false;
    }
    public static string Lancer(string asm, string tipe, string impr, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var Tp = (double[])enf.First(c => c.Name2.StartsWith("piece avec les bites")).Transform2.ArrayData;
            sw.CloseAllDocuments(true);
            Encoche(sw, Path.Combine(tipe, "piece avec les bites.SLDPRT"), Path.Combine(impr, "piece avec les bites (poutre droite).STL"), Tp);
            // support reimporte
            string sf = Path.Combine(tipe, "support capteurs 2 rangees.SLDPRT"), st = Path.Combine(tipe, "support capteurs 2 rangees.step");
            File.Delete(sf);
            var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
            imp.Extension.SaveAs(sf, 0, 1, null, ref e, ref w);
            j += "support reimporte err " + e + ", volume " + Vol(imp).ToString("F0") + "\n";
            sw.CloseDoc(imp.GetTitle());
            // assemblage
            doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var ad = (AssemblyDoc)doc;
            doc.ForceRebuild3(false);
            enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            Func<string, Component2> C = nm => { var c = enf.FirstOrDefault(k => k.Name2 == nm); if (c == null) throw new Exception("absent " + nm); return c; };
            var sup = enf.First(c => c.Name2.StartsWith("support capteurs 2 rangees"));
            string V = "socket head cap screw_iso-", N = "hex nut style 1 gradeab_iso-";
            // cible, source de l'orientation, deplacement (mm)
            var plan = new[] {
                new { c = new[] { "capteur-1", V + "70", N + "39" }, s = new[] { "capteur-1", V + "70", N + "39" }, d = new[] { 0.84, -3.0, 0.0 } },
                new { c = new[] { "capteur-2", V + "69", N + "38" }, s = new[] { "capteur-4", V + "77", N + "41" }, d = new[] { -80.5, -3.0, 19.7 } },
                new { c = new[] { "capteur-3", V + "71", N + "40" }, s = new[] { "capteur-6", V + "79", N + "43" }, d = new[] { -80.5, -3.0, -19.7 } },
                new { c = new[] { "capteur-4", V + "77", N + "41" }, s = new[] { "capteur-4", V + "77", N + "41" }, d = new[] { 5.75, 0.0, 19.7 } },
                new { c = new[] { "capteur-5", V + "78", N + "42" }, s = new[] { "capteur-5", V + "78", N + "42" }, d = new[] { -8.75, 0.0, 0.3 } },
                new { c = new[] { "capteur-6", V + "79", N + "43" }, s = new[] { "capteur-6", V + "79", N + "43" }, d = new[] { 5.75, 0.0, -19.7 } },
            };
            var noms = new HashSet<string>(plan.SelectMany(p => p.c));
            var T0 = plan.SelectMany(p => p.s).Distinct().ToDictionary(nm => nm, nm => (double[])C(nm).Transform2.ArrayData);
            j += "contraintes supprimees : " + Suppr(doc, noms) + "\n";
            foreach (var p in plan) for (int i = 0; i < 3; i++) {
                var t = (double[])T0[p.s[i]].Clone();
                t[9] += p.d[0] / 1000; t[10] += p.d[1] / 1000; t[11] += p.d[2] / 1000;
                var c = C(p.c[i]);
                c.Transform2 = (MathTransform)mu.CreateTransform(t);
                doc.ClearSelection2(true);
                sup.Select4(false, null, false); c.Select4(true, null, false);
                var m = ad.AddMate5((int)swMateType_e.swMateLOCK, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
                if (m == null) j += "ECHEC verrou " + p.c[i] + " err " + err + "\n";
            }
            doc.ClearSelection2(true); doc.ForceRebuild3(false);
            int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; j += "ERREUR " + m.Name + " code " + code + "\n"; } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            j += "contraintes : " + total + ", en erreur : " + mauvais + ", composants : " + enf.Length + "\n";
            foreach (var nm in new[] { "capteur-1", "capteur-2", "capteur-3", "capteur-4", "capteur-5", "capteur-6" }) { var b = (double[])C(nm).GetBox(false, false); j += string.Format("{0} : x {1:F2}..{2:F2} y {3:F2}..{4:F2} z {5:F2}..{6:F2}\n", nm, b[0]*1000, b[3]*1000, b[1]*1000, b[4]*1000, b[2]*1000, b[5]*1000); }
            if (mauvais > 0) return j + "ARRET : assemblage NON enregistre\n";
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            j += "enregistre : " + doc.Save3(1, ref e, ref w) + " err " + e + "\n";
            doc.Extension.SaveAs(png, 0, 1, null, ref e, ref w);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            var cles = new[] { "support capteurs", "capteur-", "piece avec" };
            int nb = 0;
            if (lst != null) foreach (IInterference it in lst) { nb++; var cs = ((object[])it.Components).Cast<Component2>().Select(c => c.Name2).ToArray(); if (cs.Any(s => cles.Any(k => s.StartsWith(k)))) j += string.Format("  {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", cs)); }
            j += "interferences totales : " + nb + "\n";
            mgr.Done();
        } catch (Exception ex) { j += "EXCEPTION " + ex.Message + "\n" + ex.StackTrace; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j;
    }
}
'@
$png = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)\Robot complet (Assemblage1v2).png'
[Cap]::Lancer($asm, $tipe, $impr, $png) | Out-File "$PSScriptRoot\capteurs_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\capteurs_sw.txt"
