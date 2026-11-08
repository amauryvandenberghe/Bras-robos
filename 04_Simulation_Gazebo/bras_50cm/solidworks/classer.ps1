for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
Get-Process SLDWORKS -ErrorAction SilentlyContinue | Where-Object { -not $_.MainWindowTitle } | Stop-Process -Force -Confirm:$false
Start-Sleep 3
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_VERSION_FINALE (a ouvrir)') } | Select-Object -First 1).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$vieux = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)'
$dep = @{}
foreach ($f in Get-ChildItem $fin -File) {
    $n = $f.Name
    if ($n -in 'Assemblage1v2.SLDASM', 'Bras 50cm.SLDASM') { continue }
    if ($n -match '^(Vis |Ecrou|Rondelle|socket head|hex nut|plain washer)') { $s = 'Visserie' }
    elseif (Test-Path -LiteralPath (Join-Path $vieux $n)) { $s = 'Pieces bras' }
    else { $s = 'Pieces robot' }
    $dep[$n] = $s
    Move-Item -LiteralPath $f.FullName -Destination (Join-Path (Join-Path $fin $s) $n)
}
$table = ($dep.GetEnumerator() | ForEach-Object { $_.Key + '|' + $_.Value }) -join "`n"
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
public static class Cl {
    static IEnumerable<Component2> Tous(Component2 c) { var f = (object[])c.GetChildren(); if (f == null) yield break; foreach (Component2 e in f) { yield return e; foreach (var x in Tous(e)) yield return x; } }
    public static string Lancer(string fin, string table, string liste) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var r = new StringBuilder(); int e = 0, w = 0;
        try {
            sw.CloseAllDocuments(true);
            var ou = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var l in table.Split('\n')) { var p = l.Split('|'); ou[p[0].Trim()] = p[1].Trim(); }
            var asms = Directory.GetFiles(fin, "*.SLDASM", SearchOption.AllDirectories);
            int nb = 0, echec = 0;
            foreach (var a in asms) {
                var deps = (string[])sw.GetDocumentDependencies2(a, false, true, false);
                if (deps == null) continue;
                for (int i = 1; i < deps.Length; i += 2) {
                    var old = deps[i]; var nom = Path.GetFileName(old);
                    if (!ou.ContainsKey(nom) && !nom.Equals("Bras 50cm.SLDASM", StringComparison.OrdinalIgnoreCase)) continue;
                    var neu = ou.ContainsKey(nom) ? Path.Combine(fin, ou[nom], nom) : Path.Combine(fin, nom);
                    if (old.Equals(neu, StringComparison.OrdinalIgnoreCase)) continue;
                    if (sw.ReplaceReferencedDocument(a, old, neu)) nb++; else { echec++; r.AppendLine("ECHEC " + Path.GetFileName(a) + " : " + old); }
                }
            }
            r.AppendLine("assemblages " + asms.Length + ", references remplacees " + nb + ", echecs " + echec);
            var f = Path.Combine(fin, "Assemblage1v2.SLDASM");
            var doc = (ModelDoc2)sw.OpenDoc6(f, 2, 1, "", ref e, ref w);
            if (doc == null) return r + "FINALE NON OUVERTE err " + e;
            r.AppendLine("ouverture err " + e + " avert " + w);
            doc.ForceRebuild3(false);
            var tous = Tous(((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true)).ToArray();
            int ext = 0, supp = 0, vide = 0;
            var vus = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
            foreach (var c in tous) {
                var p = c.GetPathName();
                if (c.IsSuppressed()) { supp++; r.AppendLine("  SUPPRIME " + c.Name2); }
                if (p.IndexOf("SOLIDWORKS Data", StringComparison.OrdinalIgnoreCase) >= 0 || p.IndexOf("\\swx", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                if (!p.StartsWith(fin, StringComparison.OrdinalIgnoreCase)) { ext++; r.AppendLine("  HORS DOSSIER " + c.Name2 + " -> " + p); }
                if (p.EndsWith(".SLDPRT", StringComparison.OrdinalIgnoreCase) && !vus.ContainsKey(p)) {
                    object info; var cb = (object[])c.GetBodies3(0, out info); double v = 0;
                    if (cb == null || cb.Length == 0) vide++;
                    else foreach (Body2 b in cb) { var mp = (double[])b.GetMassProperties(1.0); if (mp != null) v += mp[3]; }
                    vus[p] = v * 1e9;
                }
            }
            int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; r.AppendLine("ERREUR " + m.Name + " code " + code); } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            r.AppendLine("contraintes " + total + ", en erreur " + mauvais + ", composants " + tous.Length + ", vraiment hors dossier " + ext + ", supprimes " + supp + ", sans corps " + vide);
            if (mauvais == 0 && ext == 0 && supp == 0) r.AppendLine("enregistre : " + doc.Save3(5, ref e, ref w) + " err " + e);
            File.WriteAllLines(liste, vus.OrderBy(k => k.Key).Select(k => string.Format(CultureInfo.InvariantCulture, "{0:F1}\t{1}", k.Value, k.Key.Substring(fin.Length + 1))));
        } catch (Exception ex) { r.AppendLine("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r.ToString();
    }
}
'@
[Cl]::Lancer($fin, $table, "$PSScriptRoot\volumes_finale.txt") | Out-File "$PSScriptRoot\classer.txt" -Encoding utf8
Get-Content "$PSScriptRoot\classer.txt"
Get-ChildItem $fin | ForEach-Object { $_.Name }
foreach ($d in 'Pieces bras', 'Pieces robot', 'Visserie') { "$d : " + (Get-ChildItem (Join-Path $fin $d) -File).Count }
