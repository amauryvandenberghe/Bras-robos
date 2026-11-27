# 27/11/2026 : PROPOSITION robot differentiel compact, ASSEMBLAGE A PART (Assemblage1v2 est seulement LU).
# Pieces neuves : CadQuery bras_50cm/robot/chassis_differentiel.py (dessinees dans le repere d'Assemblage1v2, inserees a l'origine).
# Pieces reprises : moteurs mcc, roues motrices, batteries, 4 capteurs, L298N, ESP32, bras (fichiers de 1_VERSION_FINALE, non modifies).
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$racine = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks')).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$prop = Join-Path $racine '5_PROPOSITION robot differentiel compact (a part)'
New-Item -ItemType Directory -Force $prop | Out-Null
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\chassis_diff_out\'
$noms = @('bati differentiel', 'etrier moteur gauche', 'etrier moteur droit', 'nez capteurs et billes', 'bille 16 gauche', 'bille 16 droite',
          'berceau batteries PCB et patins', 'PCB MAVE (encombrement)')
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
public static class Chassis {
    // geometrie (mm) : voir chassis_differentiel.json
    const double XA = 164.25, ZM = 512.72, XE = 154.25, YE = 479.6, W_ROUE = 93.25, W_MOT = 76.7, Y_BAT = 470.1, W_BAT = 11.0;
    const double X_RET = 253.75, PCB_X0 = 70.0, PCB_HAUT = 489.6;
    class Pose { public string Fichier; public double[] Data; public double[] R; public double[] Centre; public double YMin = double.NaN; public string Nom; }
    static double[] Ident() { var d = new double[16]; d[0] = d[4] = d[8] = 1; d[12] = 1; return d; }
    // R (ligne par ligne, assemblage = R . local) -> ArrayData (lignes = images des axes locaux)
    static double[] VersData(double[] R, double[] t) { var d = new double[16]; for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) d[3 * i + k] = R[3 * k + i]; d[9] = t[0]; d[10] = t[1]; d[11] = t[2]; d[12] = 1; return d; }
    static double[] RDe(double[] d) { var R = new double[9]; for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) R[3 * k + i] = d[3 * i + k]; return R; }
    public static string Lancer(string asmRef, string dossier, string[] noms) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility(); var j = new StringBuilder();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            var refd = (ModelDoc2)sw.OpenDoc6(asmRef, 2, 1 | 256, "", ref e, ref w);
            var enf = ((object[])((Configuration)refd.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            Func<string, Component2> C = n => enf.First(c => c.Name2 == n);
            var poses = new List<Pose>();
            // capteurs : rotation de 180 deg autour de la verticale (x -> X_RET - x, z -> 2 ZM - z)
            foreach (var n in new[] { "capteur-7", "capteur-8", "capteur-9", "capteur-12" }) {
                var d = (double[])C(n).Transform2.ArrayData; var R = RDe(d);
                var Rn = new double[9]; for (int k = 0; k < 3; k++) { Rn[k] = -R[k]; Rn[3 + k] = R[3 + k]; Rn[6 + k] = -R[6 + k]; }
                var t = new[] { X_RET / 1000 - d[9], d[10], 2 * ZM / 1000 - d[11] };
                poses.Add(new Pose { Fichier = C(n).GetPathName(), Data = VersData(Rn, t), Nom = n });
            }
            poses.Add(new Pose { Fichier = C("L298N Motor Driver.step-1").GetPathName(), Data = (double[])C("L298N Motor Driver.step-1").Transform2.ArrayData, Nom = "L298N" });
            poses.Add(new Pose { Fichier = C("Bras 50cm-2").GetPathName(), Data = (double[])C("Bras 50cm-2").Transform2.ArrayData, Nom = "bras" });
            var esp = C("esp32_dev_kit_v1_asm-1");
            poses.Add(new Pose { Fichier = esp.GetPathName(), R = RDe((double[])esp.Transform2.ArrayData), Centre = new[] { PCB_X0 + 46.22, double.NaN, ZM - 3.3 }, YMin = PCB_HAUT + 2.5, Nom = "ESP32" });
            string mcc = C("mcc-1").GetPathName(), roue = C("roue arriere motrice-1").GetPathName(), bat = C("batterie 18x110-3").GetPathName();
            sw.CloseDoc(refd.GetTitle());
            // moteurs : longueur locale z (arbre au bout z mini) -> +x ; hauteur locale x -> y ; arbre local y -> z
            poses.Add(new Pose { Fichier = mcc, R = new double[] { 0, 0, 1, 1, 0, 0, 0, 1, 0 }, Centre = new[] { XE - 10.92 + 65.65 / 2, YE, ZM + W_MOT }, Nom = "moteur gauche" });
            poses.Add(new Pose { Fichier = mcc, R = new double[] { 0, 0, 1, -1, 0, 0, 0, -1, 0 }, Centre = new[] { XE - 10.92 + 65.65 / 2, YE, ZM - W_MOT }, Nom = "moteur droit" });
            // roues : axe local z, voile du cote z local mini -> vers l'exterieur (moyeu cote moteur)
            poses.Add(new Pose { Fichier = roue, R = new double[] { 1, 0, 0, 0, -1, 0, 0, 0, -1 }, Centre = new[] { XE, YE, ZM + W_ROUE }, Nom = "roue gauche" });
            poses.Add(new Pose { Fichier = roue, R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, Centre = new[] { XE, YE, ZM - W_ROUE }, Nom = "roue droite" });
            // batteries couchees le long de x
            foreach (var s in new[] { 1.0, -1.0 })
                poses.Add(new Pose { Fichier = bat, R = new double[] { 0, 0, 1, 0, 1, 0, -1, 0, 0 }, Centre = new[] { XE, Y_BAT, ZM + s * W_BAT }, Nom = "batterie" });
            foreach (var n in noms) {
                string f = Path.Combine(dossier, n + ".SLDPRT"), st = Path.Combine(dossier, n + ".step");
                if (File.Exists(f)) File.Delete(f);
                var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
                imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
                sw.CloseDoc(imp.GetTitle());
                poses.Add(new Pose { Fichier = f, Data = Ident(), Nom = n });
            }
            string modele = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
            var doc = (ModelDoc2)sw.NewDocument(modele, 0, 0, 0);
            var ad = (AssemblyDoc)doc;
            foreach (var p in poses) {
                if (p.Fichier.EndsWith(".SLDASM", StringComparison.OrdinalIgnoreCase)) sw.OpenDoc6(p.Fichier, 2, 1, "", ref e, ref w);
                else sw.OpenDoc6(p.Fichier, 1, 1, "", ref e, ref w);
                sw.ActivateDoc3(doc.GetTitle(), false, 0, ref err);
                var c = ad.AddComponent5(p.Fichier, 0, "", false, "", 0, 0, 0);
                if (c == null) { j.Append("ECHEC insertion " + p.Fichier + "\n"); continue; }
                double[] data = p.Data;
                if (data == null) {
                    c.Transform2 = (MathTransform)mu.CreateTransform(Ident());
                    var bx = (double[])c.GetBox(false, false);                        // bornes locales (m)
                    double[] mn = { 1e9, 1e9, 1e9 }, mx = { -1e9, -1e9, -1e9 };
                    for (int k = 0; k < 8; k++) {
                        double[] q = { bx[(k & 1) == 0 ? 0 : 3], bx[(k & 2) == 0 ? 1 : 4], bx[(k & 4) == 0 ? 2 : 5] };
                        for (int a = 0; a < 3; a++) { double v = p.R[3 * a] * q[0] + p.R[3 * a + 1] * q[1] + p.R[3 * a + 2] * q[2]; mn[a] = Math.Min(mn[a], v); mx[a] = Math.Max(mx[a], v); }
                    }
                    var t = new double[3];
                    for (int a = 0; a < 3; a++) t[a] = (double.IsNaN(p.Centre[a]) ? (p.YMin / 1000 - mn[a]) : (p.Centre[a] / 1000 - (mn[a] + mx[a]) / 2));
                    data = VersData(p.R, t);
                }
                c.Transform2 = (MathTransform)mu.CreateTransform(data);
                doc.ClearSelection2(true); c.Select4(false, null, false); ad.FixComponent(); doc.ClearSelection2(true);
                var b2 = (double[])c.GetBox(false, false);
                j.AppendFormat("  {0,-34} ({1}) boite x {2:F1}..{3:F1} y {4:F1}..{5:F1} z {6:F1}..{7:F1}\n", c.Name2, p.Nom, b2[0] * 1000, b2[3] * 1000, b2[1] * 1000, b2[4] * 1000, b2[2] * 1000, b2[5] * 1000);
            }
            doc.ForceRebuild3(false);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            if (lst != null) foreach (IInterference it in lst) j.AppendFormat("  INTERFERENCE {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", ((object[])it.Components).Cast<Component2>().Select(c => c.Name2)));
            mgr.Done();
            j.Append("interferences : " + (lst == null ? 0 : lst.Length) + "\n");
            string nom = Path.Combine(dossier, "Robot differentiel compact.SLDASM");
            j.Append("enregistre : " + doc.Extension.SaveAs(nom, 0, 1, null, ref e, ref w) + " err " + e + "\n");
            foreach (var v in new[] { Tuple.Create(7, "vue isometrique"), Tuple.Create(6, "vue de dessous"), Tuple.Create(5, "vue de dessus"), Tuple.Create(3, "vue de gauche"), Tuple.Create(1, "vue de face") }) {
                doc.ShowNamedView2("", v.Item1); doc.ViewZoomtofit2();
                doc.Extension.SaveAs(Path.Combine(dossier, "Robot differentiel (" + v.Item2 + ").png"), 0, 1, null, ref e, ref w);
            }
        } catch (Exception ex) { j.Append("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j.ToString();
    }
}
'@
[Chassis]::Lancer((Join-Path $fin 'Assemblage1v2.SLDASM'), $prop, $noms) | Out-File "$PSScriptRoot\chassis_differentiel_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\chassis_differentiel_sw.txt"
