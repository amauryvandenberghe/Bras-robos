# 27/11/2026 : PROPOSITION direction differentielle, dans un ASSEMBLAGE A PART (Assemblage1v2 est seulement LU).
# Bati, moteurs (mcc), roues, capteurs : pieces existantes, positions reprises d'Assemblage1v2 (moteurs et roues deplaces sur
# l'essieu x 150, voie 150). Colonne + bille + plaque des capteurs, etriers des moteurs : CadQuery (proposition_differentiel.py).
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$racine = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks')).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$prop = Join-Path $racine '4_PROPOSITION direction differentielle (a part)'
New-Item -ItemType Directory -Force $prop | Out-Null
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\differentiel_out\'
$noms = @('colonne bille et capteurs', 'bille 16', 'etrier moteur gauche', 'etrier moteur droit')
foreach ($n in $noms) { Copy-Item -LiteralPath ($src + $n + '.step') (Join-Path $prop ($n + '.step')) -Force; Copy-Item -LiteralPath ($src + $n + '.stl') (Join-Path $prop ($n + '.stl')) -Force }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Prop {
    public static string Lancer(string asmRef, string dossier, string[] noms) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility(); var j = new StringBuilder();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            // 1. positions et fichiers des pieces existantes (lecture seule)
            var refd = (ModelDoc2)sw.OpenDoc6(asmRef, 2, 1 | 256, "", ref e, ref w);
            var enf = ((object[])((Configuration)refd.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var pos = new List<Tuple<string, double[]>>();
            Func<string, Component2> C = n => enf.First(c => c.Name2 == n);
            Action<string, double, double, double> prendre = (n, ddx, ddy, ddz) => {
                var c = C(n); var t = (double[])c.Transform2.ArrayData;
                t[9] += ddx / 1000; t[10] += ddy / 1000; t[11] += ddz / 1000;
                pos.Add(Tuple.Create(c.GetPathName(), t));
            };
            prendre(enf.First(c => c.Name2.StartsWith("bati")).Name2, 0, 0, 0);
            var m0 = (double[])C("mcc-1").Transform2.ArrayData;
            double dx = 150.0 - m0[9] * 1000, dzA = (512.72 - 55.6) - m0[11] * 1000, dzB = (512.72 + 55.6) - m0[11] * 1000;
            prendre("mcc-1", dx, 0, dzA); prendre("mcc-1", dx, 0, dzB);
            prendre("roue arriere motrice-1", dx, 0, dzA); prendre("roue arriere motrice-2", dx, 0, dzB);
            foreach (var n in new[] { "capteur-7", "capteur-8", "capteur-9", "capteur-12" }) prendre(n, 0, 0, 0);
            sw.CloseDoc(refd.GetTitle());
            // 2. pieces CadQuery -> SLDPRT
            foreach (var n in noms) {
                string f = Path.Combine(dossier, n + ".SLDPRT"), st = Path.Combine(dossier, n + ".step");
                if (File.Exists(f)) File.Delete(f);
                var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
                imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
                sw.CloseDoc(imp.GetTitle());
                var id = new double[16]; id[0] = id[4] = id[8] = 1; id[12] = 1;
                pos.Add(Tuple.Create(f, id));
            }
            // 3. nouvel assemblage
            string modele = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
            var doc = (ModelDoc2)sw.NewDocument(modele, 0, 0, 0);
            var ad = (AssemblyDoc)doc;
            foreach (var p in pos) {
                if (p.Item1.EndsWith(".SLDASM", StringComparison.OrdinalIgnoreCase)) sw.OpenDoc6(p.Item1, 2, 1, "", ref e, ref w);
                else sw.OpenDoc6(p.Item1, 1, 1, "", ref e, ref w);
                sw.ActivateDoc3(doc.GetTitle(), false, 0, ref err);
                var c = ad.AddComponent5(p.Item1, 0, "", false, "", 0, 0, 0);
                if (c == null) { j.Append("ECHEC insertion " + p.Item1 + "\n"); continue; }
                c.Transform2 = (MathTransform)mu.CreateTransform(p.Item2);
                doc.ClearSelection2(true); c.Select4(false, null, false); ad.FixComponent(); doc.ClearSelection2(true);
                j.Append("  " + c.Name2 + "\n");
            }
            doc.ForceRebuild3(false);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            if (lst != null) foreach (IInterference it in lst) j.AppendFormat("  INTERFERENCE {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", ((object[])it.Components).Cast<Component2>().Select(c => c.Name2)));
            mgr.Done();
            j.Append("interferences : " + (lst == null ? 0 : lst.Length) + "\n");
            string nom = Path.Combine(dossier, "Proposition direction differentielle.SLDASM");
            j.Append("enregistre : " + doc.Extension.SaveAs(nom, 0, 1, null, ref e, ref w) + " err " + e + "\n");
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            doc.Extension.SaveAs(Path.Combine(dossier, "Proposition (vue isometrique).png"), 0, 1, null, ref e, ref w);
            doc.ShowNamedView2("", 6); doc.ViewZoomtofit2();
            doc.Extension.SaveAs(Path.Combine(dossier, "Proposition (vue de dessous).png"), 0, 1, null, ref e, ref w);
        } catch (Exception ex) { j.Append("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j.ToString();
    }
}
'@
[Prop]::Lancer((Join-Path $fin 'Assemblage1v2.SLDASM'), $prop, $noms) | Out-File "$PSScriptRoot\proposition_differentiel_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\proposition_differentiel_sw.txt"
