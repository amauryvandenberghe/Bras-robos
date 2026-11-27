# 27/11/2026 : LECTURE SEULE : exporte en STL (repere de la piece) le moteur mcc et la roue motrice, pour dessiner les etriers du chassis differentiel.
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : le fermer puis relancer'; exit 1 }
$pr = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks\1_VERSION_FINALE (a ouvrir)\Pieces robot')).Path
$out = $args[0]
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll') -TypeDefinition @'
using System;
using System.IO;
using SolidWorks.Interop.sldworks;
public static class Exp {
    public static string Lancer(string dossier, string sortie, string[] noms) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; string r = "";
        try {
            foreach (var n in noms) {
                int e = 0, w = 0;
                var d = (ModelDoc2)sw.OpenDoc6(Path.Combine(dossier, n + ".SLDPRT"), 1, 1 | 256, "", ref e, ref w);
                if (d == null) { r += n + " : ouverture impossible " + e + "\n"; continue; }
                bool ok = d.Extension.SaveAs(Path.Combine(sortie, n + ".stl"), 0, 2, null, ref e, ref w);
                r += n + " : " + ok + " err " + e + "\n";
                sw.CloseDoc(d.GetTitle());
            }
        } catch (Exception ex) { r += "EXCEPTION " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r;
    }
}
'@
[Exp]::Lancer($pr, $out, @('mcc', 'roue arriere motrice', 'batterie 18x110'))
