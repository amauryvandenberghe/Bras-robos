# 2e passage : pieces reimportees, roulement remonte, controle.
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
$noms = @{ 'support_mg996r_direction' = 'support MG996R direction'; 'cuve_direction' = 'cuve direction'; 'entretoise_direction' = 'entretoise direction'; 'liaison_mg996r_tige' = 'liaison MG996R tige' }
foreach ($k in $noms.Keys) { Copy-Item ($src + $k + '.step') (Join-Path $tipe ($noms[$k] + '.step')) -Force; Copy-Item ($src + $k + '.stl') (Join-Path $impr ($noms[$k] + '.stl')) -Force }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Dir2 {
    public static string Lancer(string asm, string tipe, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility(); string j = "";
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            foreach (var n in new[] { "support MG996R direction", "cuve direction", "entretoise direction", "liaison MG996R tige" }) {
                string f = Path.Combine(tipe, n + ".SLDPRT"), st = Path.Combine(tipe, n + ".step");
                if (File.Exists(f)) File.Delete(f);
                var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
                imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
                j += n + " reimporte, err " + e + "\n";
                sw.CloseDoc(imp.GetTitle());
            }
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var ad = (AssemblyDoc)doc;
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var r4 = enf.First(c => c.Name2.StartsWith("roulement") && c.Name2.EndsWith("-4"));
            var cuve = enf.First(c => c.Name2.StartsWith("cuve direction"));
            doc.ClearSelection2(true); int n0 = 0;
            var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { var mm = m.GetSpecificFeature2() as Mate2; if (mm != null) { bool l = false; for (int i = 0; i < mm.GetMateEntityCount(); i++) { var cc = mm.MateEntity(i).ReferenceComponent; if (cc != null && cc.Name2 == r4.Name2) l = true; } if (l) { m.Select2(true, 0); n0++; } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            doc.Extension.DeleteSelection2(0); doc.ClearSelection2(true);
            var t = (double[])r4.Transform2.ArrayData; t[10] = 0.5217;
            r4.Transform2 = (MathTransform)mu.CreateTransform(t);
            cuve.Select4(false, null, false); r4.Select4(true, null, false);
            ad.AddMate5((int)swMateType_e.swMateLOCK, (int)swMateAlign_e.swMateAlignCLOSEST, false, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
            doc.ClearSelection2(true); doc.ForceRebuild3(false);
            int total = 0, mauvais = 0; fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; j += "ERREUR " + m.Name + " code " + code + "\n"; } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            j += "contraintes : " + total + ", en erreur : " + mauvais + ", composants : " + enf.Length + "\n";
            if (mauvais > 0) return j + "ARRET, non enregistre\n";
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
[Dir2]::Lancer($asm, $tipe, (Join-Path $fin 'Robot complet (Assemblage1v2).png')) | Out-File "$PSScriptRoot\direction_sw2.txt" -Encoding utf8
Get-Content "$PSScriptRoot\direction_sw2.txt"
