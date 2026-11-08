for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
Get-Process SLDWORKS -ErrorAction SilentlyContinue | Where-Object { -not $_.MainWindowTitle } | Stop-Process -Force -Confirm:$false
Start-Sleep 3
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_VERSION_FINALE (a ouvrir)') } | Select-Object -First 1).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$vieux = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)'
$arch = Join-Path $racine '3_Anciennes versions\3k_08-11 etat avant rangement final'
$impr = Join-Path (Split-Path $racine) '02_Impression_3D'
$docs = Join-Path $fin 'Documents'
New-Item -ItemType Directory -Force $docs, $arch | Out-Null
foreach ($n in 'MONTAGE.txt', 'CE QUI A CHANGE.txt', 'Robot complet (Assemblage1v2).png', 'Bras 50cm.png', 'Vues 07-11', 'Vues 08-11', 'Vues visserie') { Copy-Item -LiteralPath (Join-Path $vieux $n) -Destination $docs -Recurse -Force }
Move-Item -LiteralPath $vieux -Destination $arch
Move-Item -LiteralPath (Join-Path $bureau 'Assemblage1v2.SLDASM') -Destination $arch
"archive : ancien dossier present = $(Test-Path -LiteralPath (Join-Path $arch '1_ASSEMBLAGE_FINAL (a utiliser)')), reste au Bureau = $(Test-Path -LiteralPath (Join-Path $bureau 'Assemblage1v2.SLDASM'))"
$tmp = Join-Path $PSScriptRoot 'stl_export'
New-Item -ItemType Directory -Force $tmp | Out-Null
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
public static class Ar {
    static IEnumerable<Component2> Tous(Component2 c) { var f = (object[])c.GetChildren(); if (f == null) yield break; foreach (Component2 e in f) { yield return e; foreach (var x in Tous(e)) yield return x; } }
    static string Bilan(ModelDoc2 doc, string fin) {
        var tous = Tous(((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true)).ToArray();
        int hors = 0, supp = 0, manq = 0; var r = new StringBuilder();
        foreach (var c in tous) {
            var p = c.GetPathName();
            if (c.IsSuppressed()) { supp++; r.AppendLine("  SUPPRIME " + c.Name2 + " -> " + p); }
            if (p.IndexOf("SOLIDWORKS Data", StringComparison.OrdinalIgnoreCase) >= 0 || p.IndexOf("\\swx", StringComparison.OrdinalIgnoreCase) >= 0) continue;
            if (!File.Exists(p)) manq++;
            if (fin != null && !p.StartsWith(fin, StringComparison.OrdinalIgnoreCase)) { hors++; r.AppendLine("  HORS " + c.Name2 + " -> " + p); }
        }
        int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
        while (fe != null) {
            if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) mauvais++; } m = (Feature)m.GetNextSubFeature(); } }
            fe = (Feature)fe.GetNextFeature();
        }
        r.AppendLine(string.Format("contraintes {0}, en erreur {1}, composants {2}, hors dossier {3}, fichiers manquants {4}, supprimes {5}", total, mauvais, tous.Length, hors, manq, supp));
        return r.ToString();
    }
    public static string Lancer(string fin, string arch, string tmp) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var r = new StringBuilder(); int e = 0, w = 0;
        try {
            sw.CloseAllDocuments(true);
            var doc = (ModelDoc2)sw.OpenDoc6(Path.Combine(fin, "Assemblage1v2.SLDASM"), 2, 3, "", ref e, ref w);
            r.AppendLine("FINALE seule, err " + e); doc.ForceRebuild3(false); r.Append(Bilan(doc, fin));
            sw.CloseAllDocuments(true);
            doc = (ModelDoc2)sw.OpenDoc6(Path.Combine(fin, "Bras 50cm.SLDASM"), 2, 3, "", ref e, ref w);
            r.AppendLine("BRAS seul, err " + e); r.Append(Bilan(doc, fin));
            sw.CloseAllDocuments(true);
            foreach (var n in new[] { "maitient roulement a bille", "pièce moteur direction", "roue arriere motrice", "roue avant libre", "tige pour direction" }) {
                var f = Directory.GetFiles(Path.Combine(fin, "Pieces robot"), "*.SLDPRT").FirstOrDefault(x => Path.GetFileNameWithoutExtension(x).Normalize() == n.Normalize());
                if (f == null) { r.AppendLine("ABSENT " + n); continue; }
                var d = (ModelDoc2)sw.OpenDoc6(f, 1, 3, "", ref e, ref w);
                d.Extension.SaveAs(Path.Combine(tmp, Path.GetFileNameWithoutExtension(f) + ".stl"), 0, 1, null, ref e, ref w);
                r.AppendLine("stl " + n + " err " + e);
                sw.CloseDoc(d.GetTitle());
            }
            sw.CloseAllDocuments(true);
            var a = Path.Combine(arch, "Assemblage1v2.SLDASM");
            doc = (ModelDoc2)sw.OpenDoc6(a, 2, 3, "", ref e, ref w);
            if (doc == null) r.AppendLine("ARCHIVE non ouverte err " + e);
            else { r.AppendLine("ARCHIVE, err " + e); var b = Bilan(doc, null); r.AppendLine(b.Length > 800 ? b.Substring(b.Length - 800) : b); }
        } catch (Exception ex) { r.AppendLine("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r.ToString();
    }
}
'@
[Ar]::Lancer($fin, $arch, $tmp) | Out-File "$PSScriptRoot\archiver.txt" -Encoding utf8
Get-Content "$PSScriptRoot\archiver.txt"
Get-ChildItem $tmp | ForEach-Object { '{0,9} {1}' -f $_.Length, $_.Name }
