# 2e passage : support reimporte, tailles de la visserie copiee, controle, enregistrement.
for ($i = 0; $i -lt 20 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 15 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$asm = Join-Path $bureau 'Assemblage1v2.SLDASM'
$tipe = Join-Path $bureau 'ICAM\Pieces tipe'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$impr = Join-Path (Split-Path $racine) '02_Impression_3D\Robot (chassis)'
$png = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)\Robot complet (Assemblage1v2).png'
$step = Join-Path $tipe 'support capteurs 2 rangees.step'
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\ros2_ws\src\mon_bras_robot\bras_50cm\robot\'
Copy-Item ($src + 'support_capteurs.step') $step -Force
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
public static class Robot2 {
    public static string Lancer(string asm, string tipe, string step, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string r = "";
        try {
            int e = 0, w = 0, err = 0;
            string sup_f = Path.Combine(tipe, "support capteurs 2 rangees.SLDPRT");
            if (File.Exists(sup_f)) File.Delete(sup_f);
            var imp = (ModelDoc2)sw.LoadFile4(step, "r", sw.GetImportFileData(step), ref err);
            if (imp == null) return "import impossible " + err;
            imp.Extension.SaveAs(sup_f, 0, 1, null, ref e, ref w);
            r += "support reimporte, err " + e + ", volume " + (((MassProperty)imp.Extension.CreateMassProperty()).Volume * 1e9).ToString("F0") + " mm3\n";
            sw.CloseDoc(imp.GetTitle());
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var ad = (AssemblyDoc)doc;
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            var tailles = new Dictionary<string, string>();
            foreach (var n in new[] { 77, 78, 79 }) tailles["socket head cap screw_iso-" + n] = "ISO 4762 M2 x 8 - 8N";
            foreach (var n in new[] { 41, 42, 43 }) tailles["hex nut style 1 gradeab_iso-" + n] = "ISO - 4032 - M2 - D - N";
            foreach (var n in new[] { 80, 81, 82, 83 }) tailles["socket head cap screw_iso-" + n] = "ISO 4762 M3 x 12 - 12N";
            foreach (var n in new[] { 44, 45, 46, 47 }) tailles["hex nut style 1 gradeab_iso-" + n] = "ISO - 4032 - M3 - D - N";
            foreach (var kv in tailles) {
                var c = enf.FirstOrDefault(k => k.Name2 == kv.Key);
                if (c == null) { r += "absent " + kv.Key + "\n"; continue; }
                c.ReferencedConfiguration = kv.Value;
            }
            doc.ForceRebuild3(false);
            foreach (var kv in tailles) { var c = enf.First(k => k.Name2 == kv.Key); if (c.ReferencedConfiguration != kv.Value) r += "TAILLE NON APPLIQUEE " + kv.Key + " : " + c.ReferencedConfiguration + "\n"; }
            r += "tailles de la visserie copiee : " + tailles.Count + " composants\n";
            int total = 0, mauvais = 0;
            var fe = (Feature)doc.FirstFeature();
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
            r += "contraintes : " + total + ", en erreur : " + mauvais + ", composants : " + enf.Length + "\n";
            if (mauvais > 0) return r + "ARRET : erreurs, non enregistre\n";
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            r += "enregistre : " + doc.Save3(1, ref e, ref w) + " err " + e + "\n";
            doc.Extension.SaveAs(png, 0, 1, null, ref e, ref w);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            var miens = new[] { "support capteurs", "capteur-", "piece avec les bites", "-77", "-78", "-79", "-41", "-42", "-43", "-44", "-45", "-46", "-47", "-80", "-81", "-82", "-83", "-65", "-66", "-29", "-30", "-69", "-70", "-71", "-38", "-39", "-40", "-58", "-31", "-14", "-15", "Bras 50cm" };
            int n0 = 0;
            if (lst != null) foreach (IInterference it in lst) {
                var cs = ((object[])it.Components).Cast<Component2>().Select(c => c.Name2).ToArray();
                n0++;
                if (cs.Any(s => miens.Any(k => s.StartsWith(k) || s.EndsWith(k)))) r += string.Format("  {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", cs));
            }
            r += "interferences totales : " + n0 + " (lignes ci-dessus : celles qui touchent les pieces modifiees)\n";
            mgr.Done();
        } catch (Exception ex) { r += "EXCEPTION " + ex.Message + "\n" + ex.StackTrace; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r;
    }
}
'@
[Robot2]::Lancer($asm, $tipe, $step, $png) | Out-File "$PSScriptRoot\robot_v3b.txt" -Encoding utf8
Get-Content "$PSScriptRoot\robot_v3b.txt"
