for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
Get-Process SLDWORKS -ErrorAction SilentlyContinue | Where-Object { -not $_.MainWindowTitle } | Stop-Process -Force -Confirm:$false
Start-Sleep 3
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$bureau = [Environment]::GetFolderPath('Desktop')
$asm = Join-Path $bureau 'Assemblage1v2.SLDASM'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
if (Test-Path $fin) { 'dossier final deja present : abandon'; exit 1 }
foreach ($d in '', 'Pieces bras', 'Pieces robot', 'Visserie') { New-Item -ItemType Directory -Force (Join-Path $fin $d) | Out-Null }
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
public static class Pg {
    static IEnumerable<Component2> Tous(Component2 c) { var f = (object[])c.GetChildren(); if (f == null) yield break; foreach (Component2 e in f) { yield return e; foreach (var x in Tous(e)) yield return x; } }
    static string Dest(string src, string fin) {
        var n = Path.GetFileName(src); var low = src.ToLowerInvariant();
        if (n.Equals("Assemblage1v2.SLDASM", StringComparison.OrdinalIgnoreCase) || n.Equals("Bras 50cm.SLDASM", StringComparison.OrdinalIgnoreCase)) return Path.Combine(fin, n);
        if (low.Contains("1_assemblage_final")) {
            if (n.StartsWith("Vis ") || n.StartsWith("Ecrou") || n.StartsWith("Rondelle")) return Path.Combine(fin, "Visserie", n);
            return Path.Combine(fin, "Pieces bras", n);
        }
        if (low.Contains("toolbox") || low.Contains("browser") || low.Contains("solidworks data")) return Path.Combine(fin, "Visserie", n);
        return Path.Combine(fin, "Pieces robot", n);
    }
    public static string Lancer(string asm, string fin, string liste) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var r = new StringBuilder(); int e = 0, w = 0;
        try {
            sw.CloseAllDocuments(true);
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var pg = doc.Extension.GetPackAndGo();
            pg.IncludeDrawings = false; pg.IncludeSimulationResults = false; pg.IncludeToolboxComponents = true; pg.IncludeSuppressed = true; pg.FlattenToSingleFolder = false;
            object o;
            pg.GetDocumentNames(out o);
            var src = ((object[])o).Cast<string>().ToArray();
            var dst = src.Select(s => Dest(s, fin)).ToArray();
            r.AppendLine("documents : " + src.Length + ", destinations distinctes : " + dst.Distinct(StringComparer.OrdinalIgnoreCase).Count());
            foreach (var g in dst.GroupBy(d => Path.GetDirectoryName(d))) r.AppendLine("  " + Path.GetFileName(g.Key) + " : " + g.Count());
            foreach (var g in dst.GroupBy(d => d, StringComparer.OrdinalIgnoreCase).Where(g => g.Count() > 1)) r.AppendLine("  DOUBLON " + g.Key);
            pg.SetSaveToName(true, fin);
            pg.FlattenToSingleFolder = true;
            object o2; pg.GetDocumentSaveToNames(out o2, out o);
            r.AppendLine("exemple cible proposee : " + ((object[])o2).Cast<string>().First());
            bool okn = pg.SetDocumentSaveToNames((object)dst.Select(d => (object)d).ToArray());
            if (!okn) okn = pg.SetDocumentSaveToNames((object)dst);
            r.AppendLine(okn ? "sous-dossiers acceptes" : "sous-dossiers refuses : dossier unique");
            if (!okn) { pg.FlattenToSingleFolder = true; pg.SetSaveToName(true, fin); }
            var st = (int[])doc.Extension.SavePackAndGo(pg);
            r.AppendLine("noms " + okn + ", statuts : " + string.Join(" ", st.GroupBy(x => x).Select(g => g.Key + "x" + g.Count())));
            sw.CloseAllDocuments(true);
            // controle : on rouvre la version finale
            var f = Path.Combine(fin, "Assemblage1v2.SLDASM");
            doc = (ModelDoc2)sw.OpenDoc6(f, 2, 1, "", ref e, ref w);
            if (doc == null) return r + "FINALE NON OUVERTE err " + e;
            r.AppendLine("ouverture err " + e + " avert " + w);
            doc.ForceRebuild3(false);
            var root = ((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true);
            var tous = Tous(root).ToArray();
            int ext = 0, supp = 0;
            var vus = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
            foreach (var c in tous) {
                var p = c.GetPathName();
                if (!p.StartsWith(fin, StringComparison.OrdinalIgnoreCase)) { ext++; r.AppendLine("  HORS DOSSIER " + c.Name2 + " -> " + p); }
                if (c.IsSuppressed()) { supp++; r.AppendLine("  SUPPRIME " + c.Name2); }
                if (p.EndsWith(".SLDPRT", StringComparison.OrdinalIgnoreCase) && !vus.ContainsKey(p)) {
                    object info; var cb = (object[])c.GetBodies3(0, out info); double v = 0;
                    if (cb != null) foreach (Body2 b in cb) { var mp = (double[])b.GetMassProperties(1.0); if (mp != null) v += mp[3]; }
                    vus[p] = v * 1e9;
                }
            }
            int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; r.AppendLine("ERREUR " + m.Name + " code " + code); } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            r.AppendLine("contraintes " + total + ", en erreur " + mauvais + ", composants (tous niveaux) " + tous.Length + ", hors dossier " + ext + ", supprimes " + supp);
            r.AppendLine("enregistre : " + doc.Save3(1, ref e, ref w) + " err " + e);
            File.WriteAllLines(liste, vus.OrderBy(k => k.Key).Select(k => string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0:F1}\t{1}", k.Value, k.Key.Substring(fin.Length + 1))));
        } catch (Exception ex) { r.AppendLine("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r.ToString();
    }
}
'@
[Pg]::Lancer($asm, $fin, "$PSScriptRoot\volumes_finale.txt") | Out-File "$PSScriptRoot\packgo.txt" -Encoding utf8
Get-Content "$PSScriptRoot\packgo.txt"
