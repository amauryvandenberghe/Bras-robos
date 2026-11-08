# Robot complet : direction MG996R + 2e roulement (08/11). Se lance une fois : l'ancien support doit etre present.
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
Get-Process SLDWORKS -ErrorAction SilentlyContinue | Where-Object { -not $_.MainWindowTitle } | Stop-Process -Force
Start-Sleep 3
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$asm = Join-Path $bureau 'Assemblage1v2.SLDASM'
$tipe = Join-Path $bureau 'ICAM\Pieces tipe'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$fin = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)'
$impr = Join-Path (Split-Path $racine) '02_Impression_3D\Robot (chassis)'
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\ros2_ws\src\mon_bras_robot\bras_50cm\robot\'
$arch = Join-Path $tipe 'anciennes versions 08-11'
New-Item -ItemType Directory -Force $arch | Out-Null
foreach ($n in 'bati(en cours).SLDPRT') { Copy-Item (Join-Path $tipe $n) $arch -Force }
Copy-Item $asm $arch -Force
$arch2 = Join-Path $racine '3_Anciennes versions\3i_08-11 Assemblage1v2 avant direction MG996R'
New-Item -ItemType Directory -Force $arch2 | Out-Null
Copy-Item $asm $arch2 -Force
$noms = @{ 'support_mg996r_direction' = 'support MG996R direction'; 'cuve_direction' = 'cuve direction'; 'entretoise_direction' = 'entretoise direction'; 'liaison_mg996r_tige' = 'liaison MG996R tige' }
foreach ($k in $noms.Keys) { Copy-Item ($src + $k + '.step') (Join-Path $tipe ($noms[$k] + '.step')) -Force; Copy-Item ($src + $k + '.stl') (Join-Path $impr ($noms[$k] + '.stl')) -Force }
$pos = Get-Content ($src + 'direction_positions.json') -Raw | ConvertFrom-Json
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
public static class Dir {
    static MathUtility mu; static string j = "";
    static MathTransform Tr(double[] a) { return (MathTransform)mu.CreateTransform(a); }
    static double[] Avec(double[] r, double x, double y, double z) { return r.Take(9).Concat(new[] { x, y, z, 1, 0, 0, 0 }).ToArray(); }
    static double[] Id(double x, double y, double z) { return new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, x, y, z, 1, 0, 0, 0 }; }
    static bool Lie(Mate2 mm, HashSet<string> noms) {
        for (int i = 0; i < mm.GetMateEntityCount(); i++) { var cc = mm.MateEntity(i).ReferenceComponent; if (cc == null) continue; foreach (var k in noms) if (cc.Name2 == k || cc.Name2.StartsWith(k + "/")) return true; }
        return false;
    }
    static int Sup(ModelDoc2 doc, HashSet<string> noms) {
        doc.ClearSelection2(true); int n = 0; var fe = (Feature)doc.FirstFeature();
        while (fe != null) {
            if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { var mm = m.GetSpecificFeature2() as Mate2; if (mm != null && Lie(mm, noms)) { m.Select2(true, 0); n++; } m = (Feature)m.GetNextSubFeature(); } }
            fe = (Feature)fe.GetNextFeature();
        }
        if (n > 0) doc.Extension.DeleteSelection2(0);
        doc.ClearSelection2(true); return n;
    }
    static void V(AssemblyDoc ad, ModelDoc2 doc, Component2 a, Component2 b, string q) {
        doc.ClearSelection2(true); a.Select4(false, null, false); b.Select4(true, null, false);
        int err; var m = ad.AddMate5((int)swMateType_e.swMateLOCK, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
        if (m == null) j += "  ECHEC verrou " + q + " err " + err + "\n";
        doc.ClearSelection2(true);
    }
    static double Vol(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Volume * 1e9; }
    static void Trou(SldWorks sw, string f, double[] pm, string stl) {
        int e = 0, w = 0;
        var d = (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
        d = (ModelDoc2)sw.ActivateDoc3(d.GetTitle(), false, 0, ref e);
        double v0 = Vol(d);
        var e1 = (Feature)((PartDoc)d).FeatureByName("Esquisse1"); Feature plan = null;
        foreach (var p in (object[])e1.GetParents()) { var pf = p as Feature; if (pf != null && pf.GetTypeName2() == "RefPlane") plan = pf; }
        plan.Select2(false, 0);
        d.SketchManager.InsertSketch(true); d.SketchManager.AddToDB = true;
        var sk = (Sketch)d.SketchManager.ActiveSketch;
        var q = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { pm[0], pm[1], pm[2] })).MultiplyTransform((MathTransform)sk.ModelToSketchTransform)).ArrayData;
        d.SketchManager.CreateCircleByRadius(q[0], q[1], 0, 0.00175);
        d.SketchManager.AddToDB = false; d.SketchManager.InsertSketch(true);
        var esq = (Feature)d.FeatureByPositionReverse(0); Feature cut = null;
        foreach (var dir in new[] { true, false }) {
            d.ClearSelection2(true); d.Extension.SelectByID2(esq.Name, "SKETCH", 0, 0, 0, false, 0, null, 0);
            cut = d.FeatureManager.FeatureCut4(true, false, dir, (int)swEndConditions_e.swEndCondThroughAll, (int)swEndConditions_e.swEndCondThroughAll, 0.01, 0.01, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, false);
            if (cut != null) break;
        }
        d.ForceRebuild3(false);
        j += string.Format("bati : nouveau trou {0}, volume enleve {1:F1} mm3 (attendu 48,1)\n", cut != null, v0 - Vol(d));
        if (cut != null) { cut.Name = "Trou cuve direction"; d.Save3(1, ref e, ref w); d.Extension.SaveAs(stl, 0, 1, null, ref e, ref w); }
        sw.CloseDoc(d.GetTitle());
    }
    public static string Lancer(string asm, string tipe, string fin, string impr, string png, double[] servo, double[] arm, double[] tz, double[] tzs, double yTete, double yBasLiaison, double[] vz) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; mu = (MathUtility)sw.GetMathUtility();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var ad = (AssemblyDoc)doc;
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            Func<string, Component2> C = n => enf.First(c => c.Name2 == n);
            Func<string, Component2> Deb = n => enf.First(c => c.Name2.StartsWith(n));
            var bati = Deb("bati"); var tige = Deb("tige pour"); var maitient = Deb("maitient");
            var roul1 = enf.First(c => c.Name2.StartsWith("roulement") && c.Name2.EndsWith("-1"));
            string S = "socket head cap screw_iso-", N = "hex nut style 1 gradeab_iso-", W = "plain washer large grade ac_iso-";
            var ptB = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { 0.01826, 0.5297, 0.51272 })).MultiplyTransform((MathTransform)bati.Transform2.Inverse())).ArrayData;
            j += "trou neuf, repere du bati (mm) : " + string.Join(" ", ptB.Select(v => (v * 1000).ToString("F2"))) + "\n";
            Trou(sw, bati.GetPathName(), ptB, Path.Combine(impr, "bati (avec le trou de la cuve).stl"));
            doc = (ModelDoc2)sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
            doc.ForceRebuild3(false);
            // 1. composants conserves : on retient leurs positions, on retire leurs contraintes
            var gardes = new[] { N + "2", N + "3", S + "31", S + "32", W + "6", W + "9", N + "13", N + "14", S + "24", N + "7" };
            var T0 = gardes.ToDictionary(n => n, n => (double[])C(n).Transform2.ArrayData);
            var Tm = (double[])maitient.Transform2.ArrayData; var Tr1 = (double[])roul1.Transform2.ArrayData;
            var Tige0 = (double[])tige.Transform2.ArrayData;
            j += "contraintes retirees : " + Sup(doc, new HashSet<string>(gardes)) + "\n";
            // 2. ancienne direction retiree
            doc.ClearSelection2(true);
            var aSuppr = new[] { Deb("support servo moteur direction"), Deb("piece de liason"), Deb("Servo Motor Micro"), C(S + "75"), C(S + "76"), C(W + "7"), C(W + "8") };
            bool prem = true; foreach (var c in aSuppr) { c.Select4(!prem, null, false); prem = false; }
            j += "anciens composants retires : " + doc.Extension.DeleteSelection2(0) + "\n";
            doc.ClearSelection2(true);
            // 3. nouvelles pieces imprimees
            var fp = new Dictionary<string, Component2>();
            foreach (var n in new[] { "support MG996R direction", "cuve direction", "entretoise direction", "liaison MG996R tige" }) {
                string f = Path.Combine(tipe, n + ".SLDPRT"), st = Path.Combine(tipe, n + ".step");
                if (File.Exists(f)) File.Delete(f);
                var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
                if (imp == null) return j + "ARRET import " + n + " err " + err;
                imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
                doc = (ModelDoc2)sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
                var c = (Component2)ad.AddComponent5(f, 0, "", false, "", 0, 0, 0);
                if (c == null) return j + "ARRET insertion " + n;
                c.Transform2 = Tr(Id(0, 0, 0)); fp[n] = c;
                sw.CloseDoc(imp.GetTitle());
                doc = (ModelDoc2)sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
            }
            var sup = fp["support MG996R direction"]; var cuve = fp["cuve direction"]; var ent = fp["entretoise direction"]; var lia = fp["liaison MG996R tige"];
            V(ad, doc, bati, sup, "support"); V(ad, doc, bati, cuve, "cuve"); V(ad, doc, tige, lia, "liaison"); V(ad, doc, tige, ent, "entretoise");
            // 4. pieces du commerce (fichiers deja charges par le bras)
            Func<string, double[], Component2> Ajout = (nom, t) => {
                string f = Path.Combine(fin, nom + ".SLDPRT");
                var o = (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
                doc = (ModelDoc2)sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
                var c = (Component2)ad.AddComponent5(f, 0, "", false, "", 0, 0, 0);
                if (c != null) c.Transform2 = Tr(t); else j += "  ECHEC " + nom + "\n";
                return c;
            };
            var srv = Ajout("Servo MG996R", servo); V(ad, doc, sup, srv, "servo");
            var a01 = Ajout("Servo MG996R arm01", arm); V(ad, doc, lia, a01, "arm01");
            foreach (var z in vz) { var v = Ajout("Vis 2x8", new double[] { 1, 0, 0, 0, 0, -1, 0, 1, 0, 0.03726, yBasLiaison / 1000, z / 1000, 1, 0, 0, 0 }); V(ad, doc, lia, v, "vis arm"); }
            var r2 = (Component2)ad.AddComponent5(roul1.GetPathName(), 0, "", false, roul1.ReferencedConfiguration, 0, 0, 0);
            r2.Transform2 = Tr(Avec(Tr1, 0.03726, 0.514, 0.51272)); V(ad, doc, cuve, r2, "roulement 2"); j += "roulement ajoute : " + r2.Name2 + "\n";
            // 5. visserie gardee : repositionnee et verrouillee
            foreach (var n in new[] { N + "2", N + "3", S + "31", S + "32", W + "6", W + "9" }) { C(n).Transform2 = Tr(T0[n]); V(ad, doc, sup, C(n), n); }
            V(ad, doc, sup, maitient, "maitient");
            C(S + "31").ReferencedConfiguration = "ISO 4762 M3 x 20 - 20N"; C(S + "32").ReferencedConfiguration = "ISO 4762 M3 x 20 - 20N";
            foreach (var n in new[] { N + "13", N + "14" }) { C(n).Transform2 = Tr(Avec(T0[n], T0[n][9], 0.51829, T0[n][11])); V(ad, doc, cuve, C(n), n); }
            C(S + "24").Transform2 = Tr(Avec(T0[S + "24"], 0.03726, 0.54948, 0.52272)); C(S + "24").ReferencedConfiguration = "ISO 4762 M3 x 20 - 20N"; V(ad, doc, lia, C(S + "24"), "goupille");
            C(N + "7").Transform2 = Tr(Avec(T0[N + "7"], 0.03726, 0.54948, 0.50572)); V(ad, doc, lia, C(N + "7"), "ecrou goupille");
            // 6. nouvelle visserie
            var vr = C(S + "26"); var er = C(N + "10");
            var Rv = (double[])vr.Transform2.ArrayData; var Re = (double[])er.Transform2.ArrayData;
            string pv = vr.GetPathName(), pe = er.GetPathName(), cv = vr.ReferencedConfiguration, ce = er.ReferencedConfiguration;
            for (int i = 0; i < tz.Length / 2; i++) {
                var v = (Component2)ad.AddComponent5(pv, 0, "", false, cv, 0, 0, 0); var n = (Component2)ad.AddComponent5(pe, 0, "", false, ce, 0, 0, 0);
                v.ReferencedConfiguration = cv; n.ReferencedConfiguration = ce;
                v.Transform2 = Tr(Avec(Rv, tz[2 * i] / 1000, (yTete + 3.0) / 1000, tz[2 * i + 1] / 1000));
                n.Transform2 = Tr(Avec(Re, tz[2 * i] / 1000, 0.57185, tz[2 * i + 1] / 1000));
                V(ad, doc, sup, v, "vis servo"); V(ad, doc, sup, n, "ecrou servo");
            }
            { var v = (Component2)ad.AddComponent5(pv, 0, "", false, cv, 0, 0, 0); var n = (Component2)ad.AddComponent5(pe, 0, "", false, ce, 0, 0, 0);
              v.ReferencedConfiguration = cv; n.ReferencedConfiguration = ce;
              v.Transform2 = Tr(Avec(Rv, 0.01826, 0.5327, 0.51272)); n.Transform2 = Tr(Avec(Re, 0.01826, 0.51829, 0.51272));
              V(ad, doc, bati, v, "vis cuve"); V(ad, doc, cuve, n, "ecrou cuve"); }
            doc.ForceRebuild3(false);
            // 7. bilan
            int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; j += "ERREUR " + m.Name + " code " + code + "\n"; } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            var fin2 = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            j += "contraintes : " + total + ", en erreur : " + mauvais + ", composants : " + fin2.Length + "\n";
            var Tige1 = (double[])tige.Transform2.ArrayData;
            j += "tige deplacee de " + Enumerable.Range(0, 12).Max(i => Math.Abs(Tige1[i] - Tige0[i])).ToString("E2") + "\n";
            if (mauvais > 0) return j + "ARRET : erreurs, NON enregistre\n";
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            j += "enregistre : " + doc.Save3(1, ref e, ref w) + " err " + e + "\n";
            doc.Extension.SaveAs(png, 0, 1, null, ref e, ref w);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            var cles = new[] { "support MG996R", "cuve direction", "entretoise", "liaison MG996R", "Servo MG996R", "tige pour", "maitient", "Vis 2x8", "roulement" };
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
$tz = @(); foreach ($p in $pos.trous_servo) { $tz += [double]$p[0]; $tz += [double]$p[1] }
$png = Join-Path $fin 'Robot complet (Assemblage1v2).png'
[Dir]::Lancer($asm, $tipe, $fin, $impr, $png, [double[]]$pos.servo, [double[]]$pos.arm, [double[]]$tz, [double[]]@(), [double]$pos.y_tete, [double]$pos.y_bas_liaison, [double[]]$pos.vis_pal_z) | Out-File "$PSScriptRoot\direction_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\direction_sw.txt"
