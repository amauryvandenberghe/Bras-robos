# 28/11/2026 : PROPOSITION 2, robot KART compact, version 2 (5 capteurs en fleche, 1 roulette a bille, toute la visserie, PCB
# pas modelise), ASSEMBLAGE A PART (Assemblage1v2 est seulement LU).
# Pieces neuves : CadQuery bras_50cm/robot/chassis_kart.py (dessinees dans le repere d'Assemblage1v2, inserees a l'origine),
# visserie (vis M3 x 8/10/12/16/30, ecrou M3) placee d'apres chassis_kart.json.
# Pieces reprises : moteurs mcc, roues motrices, batteries, 5 capteurs, L298N, bras, 4 vis du socle du bras (fichiers non modifies).
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$racine = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks')).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$prop = Join-Path $racine '5_PROPOSITION robot kart compact (a part)'
$visd = Join-Path $prop 'visserie'
New-Item -ItemType Directory -Force $prop, $visd | Out-Null
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\kart_out2\'
$noms = @('platine kart', 'tour du bras', 'nez 5 capteurs et roulette', 'berceau batteries kart', 'roulette a bille (encombrement)', 'support ESP32')
foreach ($n in $noms) { Copy-Item -LiteralPath ($src + $n + '.step') (Join-Path $prop ($n + '.step')) -Force; Copy-Item -LiteralPath ($src + $n + '.stl') (Join-Path $prop ($n + '.stl')) -Force }
$types = @('kart vis M3x8', 'kart vis M3x10', 'kart vis M3x12', 'kart vis M3x16', 'kart vis M3x30', 'kart vis M2x8', 'kart ecrou M3')   # noms propres : 'Vis M3x12' existe deja dans 1_VERSION_FINALE\Visserie
foreach ($n in $types) { Copy-Item -LiteralPath ($src + $n + '.step') (Join-Path $visd ($n + '.step')) -Force }
$json = Get-Content -Raw ($src + 'chassis_kart.json') | ConvertFrom-Json
$inv = [Globalization.CultureInfo]::InvariantCulture
$lignes = @(foreach ($v in $json.visserie) { $v[0] + '|' + ((($v[1] + $v[2]) | ForEach-Object { ([double]$_).ToString($inv) }) -join ';') })
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
public static class Kart {
    // geometrie (mm) : voir chassis_kart.json
    const double XA = 164.25, ZM = 512.72, XE = 154.25, YE = 479.6, W_ROUE = 68.25, W_MOT = 51.7, Y_BAT = 470.1, W_BAT = 11.0, XC_BAT = 172.85;
    const double X_RET = 261.75, X_L298N = 152.25, Y_PLOTS = 498.02, X_ESP = 263.25, Y_PLOTS_ESP = 502.52;
    class Pose { public string Fichier; public string Config = ""; public double[] Data; public double[] R; public double[] Centre; public double YMin = double.NaN; public string Nom; }
    static double[] Ident() { var d = new double[16]; d[0] = d[4] = d[8] = 1; d[12] = 1; return d; }
    static double[] VersData(double[] R, double[] t) { var d = new double[16]; for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) d[3 * i + k] = R[3 * k + i]; d[9] = t[0]; d[10] = t[1]; d[11] = t[2]; d[12] = 1; return d; }
    static double[] RDe(double[] d) { var R = new double[9]; for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) R[3 * k + i] = d[3 * i + k]; return R; }
    static string Importer(SldWorks sw, string dossier, string nom) {
        int e = 0, w = 0, err = 0;
        string f = Path.Combine(dossier, nom + ".SLDPRT"), st = Path.Combine(dossier, nom + ".step");
        if (File.Exists(f)) File.Delete(f);
        var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
        imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
        sw.CloseDoc(imp.GetTitle());
        return f;
    }
    public static string Lancer(string asmRef, string dossier, string dossierVis, string[] noms, string[] types, string[] visserie) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility(); var j = new StringBuilder();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            var refd = (ModelDoc2)sw.OpenDoc6(asmRef, 2, 1 | 256, "", ref e, ref w);
            var enf = ((object[])((Configuration)refd.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            Func<string, Component2> C = n => enf.First(c => c.Name2 == n);
            var poses = new List<Pose>();
            // capteurs : pose de la version 1 (fleche retournee : x -> X_RET - x, z -> 2 ZM - z), puis decalage vers les 5 elements
            Func<string, double[]> V1 = n => {
                var d = (double[])C(n).Transform2.ArrayData; var R = RDe(d);
                var Rn = new double[9]; for (int k = 0; k < 3; k++) { Rn[k] = -R[k]; Rn[3 + k] = R[3 + k]; Rn[6 + k] = -R[6 + k]; }
                return VersData(Rn, new[] { X_RET / 1000 - d[9], d[10], 2 * ZM / 1000 - d[11] });
            };
            // (capteur de reference, element de reference x/w en version 1, nouvel element x/w)
            var cap = new[] {
                Tuple.Create("capteur-7", 256.25, 19.5, 256.25, 33.2), Tuple.Create("capteur-9", 264.25, 7.6, 260.25, 15.2),
                Tuple.Create("capteur-9", 264.25, 7.6, 264.25, 0.0), Tuple.Create("capteur-12", 264.25, -7.6, 260.25, -15.2),
                Tuple.Create("capteur-8", 256.25, -19.5, 256.25, -33.2) };
            foreach (var c in cap) {
                var d = V1(c.Item1); d[9] += (c.Item4 - c.Item2) / 1000; d[11] += (c.Item5 - c.Item3) / 1000;
                poses.Add(new Pose { Fichier = C(c.Item1).GetPathName(), Data = d, Nom = "capteur " + c.Item5.ToString(CultureInfo.InvariantCulture) });
            }
            var l298 = C("L298N Motor Driver.step-1");     // dans la tour, sur ses 4 plots ; trous centres sur (X_L298N, ZM)
            var bl = (double[])l298.GetBox(false, false);  // dans Assemblage1v2, centre des 4 trous du bati : x 100, z ZM - 56,22
            double ox = (bl[0] + bl[3]) * 500 - 100.0, oz = (bl[2] + bl[5]) * 500 - (ZM - 56.22);
            j.AppendFormat("L298N : centre de la boite - centre des trous = {0:F2} / {1:F2} mm" + System.Environment.NewLine, ox, oz);
            poses.Add(new Pose { Fichier = l298.GetPathName(), R = RDe((double[])l298.Transform2.ArrayData), Centre = new[] { X_L298N + ox, double.NaN, ZM + oz }, YMin = Y_PLOTS, Nom = "L298N" });
            // ESP32 seul (sans PCB), RETOURNE (broches en haut) sur les 4 plots du support, longueur en travers, USB vers -z
            var esp = C("esp32_dev_kit_v1_asm-1"); var Re = RDe((double[])esp.Transform2.ArrayData);
            double[] Rf = { 1, 0, 0, 0, -1, 0, 0, 0, -1 }, Ry = { 0, 0, 1, 0, 1, 0, -1, 0, 0 };
            Func<double[], double[], double[]> Mul = (A, B) => { var M = new double[9]; for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) for (int m = 0; m < 3; m++) M[3 * i + k] += A[3 * i + m] * B[3 * m + k]; return M; };
            poses.Add(new Pose { Fichier = esp.GetPathName(), R = Mul(Ry, Mul(Rf, Re)), Centre = new[] { X_ESP, double.NaN, ZM }, YMin = Y_PLOTS_ESP - 2.9, Nom = "ESP32 (retourne)" });
            poses.Add(new Pose { Fichier = C("Bras 50cm-2").GetPathName(), Data = (double[])C("Bras 50cm-2").Transform2.ArrayData, Nom = "bras" });
            // vis du socle du bras : reprises d'Assemblage1v2 (meme socle, meme hauteur), a +-33 mm de l'axe du bras
            int nb = 0;
            foreach (var c in enf.Where(c => c.Name2.StartsWith("socket head cap screw_iso"))) {
                var b = (double[])c.GetBox(false, false);
                double cx = (b[0] + b[3]) * 500, cy = (b[1] + b[4]) * 500, cz = (b[2] + b[5]) * 500;
                if (Math.Abs(Math.Abs(cx - XA) - 33) < 3 && Math.Abs(Math.Abs(cz - ZM) - 33) < 3 && cy > 515 && cy < 560) {
                    poses.Add(new Pose { Fichier = c.GetPathName(), Config = c.ReferencedConfiguration, Data = (double[])c.Transform2.ArrayData, Nom = "vis du socle du bras" });
                    nb++;
                }
            }
            j.Append("vis du socle du bras reprises : " + nb + "\n");
            string mcc = C("mcc-1").GetPathName(), roue = C("roue arriere motrice-1").GetPathName(), bat = C("batterie 18x110-3").GetPathName();
            sw.CloseDoc(refd.GetTitle());
            poses.Add(new Pose { Fichier = mcc, R = new double[] { 0, 0, 1, 1, 0, 0, 0, 1, 0 }, Centre = new[] { XE - 10.92 + 65.65 / 2, YE, ZM + W_MOT }, Nom = "moteur gauche" });
            poses.Add(new Pose { Fichier = mcc, R = new double[] { 0, 0, 1, -1, 0, 0, 0, -1, 0 }, Centre = new[] { XE - 10.92 + 65.65 / 2, YE, ZM - W_MOT }, Nom = "moteur droit" });
            poses.Add(new Pose { Fichier = roue, R = new double[] { 1, 0, 0, 0, -1, 0, 0, 0, -1 }, Centre = new[] { XE, YE, ZM + W_ROUE }, Nom = "roue gauche" });
            poses.Add(new Pose { Fichier = roue, R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, Centre = new[] { XE, YE, ZM - W_ROUE }, Nom = "roue droite" });
            foreach (var s in new[] { 1.0, -1.0 })
                poses.Add(new Pose { Fichier = bat, R = new double[] { 0, 0, 1, 0, 1, 0, -1, 0, 0 }, Centre = new[] { XC_BAT, Y_BAT, ZM + s * W_BAT }, Nom = "batterie" });
            foreach (var n in noms) poses.Add(new Pose { Fichier = Importer(sw, dossier, n), Data = Ident(), Nom = n });
            var fv = new Dictionary<string, string>();
            foreach (var t in types) fv[t] = Importer(sw, dossierVis, t);
            foreach (var l in visserie) {
                var p = l.Split('|'); var v = p[1].Split(';').Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray();
                var R = v.Take(9).ToArray(); var t = new[] { v[9] / 1000, v[10] / 1000, v[11] / 1000 };
                poses.Add(new Pose { Fichier = fv[p[0]], Data = VersData(R, t), Nom = p[0] });
            }
            string modele = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
            var doc = (ModelDoc2)sw.NewDocument(modele, 0, 0, 0);
            var ad = (AssemblyDoc)doc;
            var ouverts = new HashSet<string>();
            int nv = 0;
            var lot = poses.Where(p => p.Nom.StartsWith("kart ")).ToList();
            foreach (var p in poses.Except(lot)) {
                if (p.Fichier.EndsWith(".SLDASM", StringComparison.OrdinalIgnoreCase)) sw.OpenDoc6(p.Fichier, 2, 1, p.Config, ref e, ref w);
                else sw.OpenDoc6(p.Fichier, 1, 1, p.Config, ref e, ref w);
                sw.ActivateDoc3(doc.GetTitle(), false, 0, ref err);
                var c = ad.AddComponent5(p.Fichier, 0, "", false, p.Config, 0, 0, 0);
                if (c == null) { j.Append("ECHEC insertion " + p.Fichier + " " + p.Config + "\n"); continue; }
                double[] data = p.Data;
                if (data == null) {
                    c.Transform2 = (MathTransform)mu.CreateTransform(Ident());
                    var bx = (double[])c.GetBox(false, false);
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
                if (p.Nom.StartsWith("kart ")) { nv++; continue; }
                var b2 = (double[])c.GetBox(false, false);
                j.AppendFormat("  {0,-40} ({1}) boite x {2:F1}..{3:F1} y {4:F1}..{5:F1} z {6:F1}..{7:F1}\n", c.Name2, p.Nom, b2[0] * 1000, b2[3] * 1000, b2[1] * 1000, b2[4] * 1000, b2[2] * 1000, b2[5] * 1000);
            }
            // visserie : insertion groupee (AddComponents3), une piece ouverte par type
            foreach (var f in lot.Select(p => p.Fichier).Distinct()) sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
            sw.ActivateDoc3(doc.GetTitle(), false, 0, ref err);
            var tr = new List<double>(); foreach (var p in lot) tr.AddRange(p.Data);
            var cs = ad.AddComponents3(lot.Select(p => p.Fichier).ToArray(), tr.ToArray(), lot.Select(p => "").ToArray()) as object[];
            if (cs != null) foreach (var o in cs) { var c = o as Component2; if (c == null) continue; doc.ClearSelection2(true); c.Select4(false, null, false); ad.FixComponent(); nv++; }
            doc.ClearSelection2(true);
            j.Append("visserie inseree : " + nv + " pieces sur " + lot.Count + "\n");
            doc.ForceRebuild3(false);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            if (lst != null) foreach (IInterference it in lst) j.AppendFormat("  INTERFERENCE {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", ((object[])it.Components).Cast<Component2>().Select(c => c.Name2)));
            mgr.Done();
            j.Append("interferences : " + (lst == null ? 0 : lst.Length) + "\n");
            string nom = Path.Combine(dossier, "Robot kart compact.SLDASM");
            j.Append("enregistre : " + doc.Extension.SaveAs(nom, 0, 1, null, ref e, ref w) + " err " + e + "\n");
            foreach (var v in new[] { Tuple.Create(7, "vue isometrique"), Tuple.Create(6, "vue de dessous"), Tuple.Create(5, "vue de dessus"), Tuple.Create(3, "vue de gauche"), Tuple.Create(1, "vue de face") }) {
                doc.ShowNamedView2("", v.Item1); doc.ViewZoomtofit2();
                doc.Extension.SaveAs(Path.Combine(dossier, "Robot kart (" + v.Item2 + ").png"), 0, 1, null, ref e, ref w);
            }
        } catch (Exception ex) { j.Append("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j.ToString();
    }
}
'@
[Kart]::Lancer((Join-Path $fin 'Assemblage1v2.SLDASM'), $prop, $visd, $noms, $types, $lignes) | Out-File "$PSScriptRoot\chassis_kart_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\chassis_kart_sw.txt"
