# Archive Assemblage1v2.SLDASM, remplace les 4 contraintes cassees du socle, enregistre, controle.
for ($i = 0; $i -lt 20 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 15 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$f = Join-Path $bureau 'Assemblage1v2.SLDASM'
$sw_dir = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$arch = Join-Path $sw_dir "3_Anciennes versions\3f_06-11 Assemblage1v2 avant mise a jour"
New-Item -ItemType Directory -Force $arch | Out-Null
if (-not (Test-Path (Join-Path $arch 'Assemblage1v2.SLDASM'))) { Copy-Item $f $arch; "archive faite : $arch" } else { "archive deja presente" }
$png = Join-Path (Join-Path $sw_dir '1_ASSEMBLAGE_FINAL (a utiliser)') 'Robot complet (Assemblage1v2).png'
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.Linq;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Reparer {
    static MathUtility mu;
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
    public static string Lancer(string f, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string r = "";
        try {
            int e = 0, w = 0, ee = 0, ww = 0;
            var doc = (ModelDoc2)sw.OpenDoc6(f, 2, 1, "", ref e, ref w);
            if (doc == null) return "ouverture impossible " + e;
            mu = (MathUtility)sw.GetMathUtility();
            var ad = (AssemblyDoc)doc;
            doc.ForceRebuild3(false);
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var bras = enf.First(c => c.Name2.StartsWith("Bras 50cm"));
            var bati = enf.First(c => c.Name2.StartsWith("bati"));
            var socle = ((object[])bras.GetChildren()).Cast<Component2>().First(c => c.Name2.EndsWith("socle-1"));
            // 1. supprimer les contraintes cassees
            doc.ClearSelection2(true);
            int n = 0;
            var noms = new[] { "Coïncidente1325", "Coaxiale1067", "Coaxiale1068", "Coaxiale1069" };
            var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") {
                    var m = (Feature)fe.GetFirstSubFeature();
                    while (m != null) {
                        var mm = m.GetSpecificFeature2() as Mate2; bool liee = false;
                        if (mm != null) for (int i = 0; i < mm.GetMateEntityCount(); i++) { var cc = mm.MateEntity(i).ReferenceComponent; if (cc != null && cc.Name2.StartsWith("Bras 50cm")) liee = true; }
                        if (liee) { m.Select2(true, 0); n++; }
                        m = (Feature)m.GetNextSubFeature();
                    }
                }
                fe = (Feature)fe.GetNextFeature();
            }
            bool sup = doc.Extension.DeleteSelection2(0);
            r += "contraintes cassees supprimees : " + n + " (" + sup + ")\n";
            doc.ClearSelection2(true);
            // 2. nouvelles contraintes : face contre face + 2 trous en diagonale (assez pour tout bloquer, sans redondance)
            double y = 0.5297;
            var fb = Plan(bati, y, 0.01); var fs = Plan(socle, y, 0.001);
            var cibles = new[] { new[] { 0.19726, 0.54572 }, new[] { 0.13126, 0.47972 } };
            r += "plans trouves : bati " + (fb != null) + ", socle " + (fs != null) + "\n";
            if (fb == null || fs == null) return r + "ARRET : plan introuvable, rien n'est enregistre\n";
            var trous = new List<Tuple<Face2, Face2>>();
            foreach (var c in cibles) {
                var tb = Trou(bati, c[0], c[1]); var ts = Trou(socle, c[0], c[1]);
                r += string.Format("trou ({0:F2},{1:F2}) : bati {2}, socle {3}\n", c[0]*1000, c[1]*1000, tb != null, ts != null);
                if (tb == null || ts == null) return r + "ARRET : trou introuvable, rien n'est enregistre\n";
                trous.Add(Tuple.Create(tb, ts));
            }
            ((Entity)fb).Select4(false, null); ((Entity)fs).Select4(true, null);
            int err; var m1 = ad.AddMate5((int)swMateType_e.swMateCOINCIDENT, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
            r += "face contre face : " + (m1 != null) + " err " + err + "\n";
            doc.ClearSelection2(true);
            foreach (var t in trous) {
                ((Entity)t.Item1).Select4(false, null); ((Entity)t.Item2).Select4(true, null);
                var m2 = ad.AddMate5((int)swMateType_e.swMateCONCENTRIC, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
                r += "trou coaxial : " + (m2 != null) + " err " + err + "\n";
                doc.ClearSelection2(true);
            }
            doc.ForceRebuild3(false);
            // 3. bilan
            int total = 0, mauvais = 0;
            fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") {
                    var m = (Feature)fe.GetFirstSubFeature();
                    while (m != null) {
                        if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; r += "ERREUR " + m.Name + " code " + code + "\n"; } }
                        m = (Feature)m.GetNextSubFeature();
                    }
                }
                fe = (Feature)fe.GetNextFeature();
            }
            r += "contraintes : " + total + ", en erreur : " + mauvais + "\n";
            var t2 = (double[])bras.Transform2.ArrayData;
            r += string.Format("Bras en ({0:F2},{1:F2},{2:F2}) mm, etat {3}\n", t2[9]*1000, t2[10]*1000, t2[11]*1000, bras.GetConstrainedStatus());
            if (mauvais > 0) return r + "ARRET : erreurs, rien n'est enregistre\n";
            doc.ShowNamedView2("*Isometric", 7); doc.ViewZoomtofit2();
            bool ok = doc.Save3(1, ref ee, ref ww);
            r += "enregistre : " + ok + " err " + ee + "\n";
            doc.Extension.SaveAs(png, 0, 1, null, ref ee, ref ww);
            // 4. interferences du bras avec le reste
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = false; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            int vus = 0;
            if (lst != null) {
                var lignes = new List<KeyValuePair<double,string>>();
                foreach (IInterference it in lst) {
                    var cs = ((object[])it.Components).Cast<Component2>().Select(c => c.Name2).ToArray();
                    int nb = cs.Count(s => s.StartsWith("Bras 50cm"));
                    if (nb == 1) lignes.Add(new KeyValuePair<double,string>(it.Volume*1e9, string.Join(" / ", cs)));
                    vus++;
                }
                r += "interferences totales : " + vus + ", bras contre robot : " + lignes.Count + "\n";
                foreach (var l in lignes.OrderByDescending(k => k.Key).Take(30)) r += string.Format("  {0:F1} mm3 : {1}\n", l.Key, l.Value);
            } else r += "aucune interference\n";
            mgr.Done();
            sw.CloseDoc(doc.GetTitle());
        } catch (Exception ex) { r += "EXCEPTION " + ex.Message + "\n" + ex.StackTrace; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r;
    }
}
'@
[Reparer]::Lancer($f, $png) | Out-File "$PSScriptRoot\reparation.txt" -Encoding utf8
Get-Content "$PSScriptRoot\reparation.txt"
Get-ChildItem $f | ForEach-Object { '{0:dd/MM HH:mm} {1,10} {2}' -f $_.LastWriteTime, $_.Length, $_.Name }
