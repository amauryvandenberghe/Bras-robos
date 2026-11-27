# 27/11/2026 : LECTURE SEULE de Assemblage1v2 : TOUS les composants de premier niveau (position, boite), pour le chassis differentiel compact.
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : le fermer puis relancer'; exit 1 }
$fin = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks\1_VERSION_FINALE (a ouvrir)')).Path
$asm = Join-Path $fin 'Assemblage1v2.SLDASM'
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.Linq;
using System.Text;
using SolidWorks.Interop.sldworks;
public static class Insp {
    public static string Lancer(string asm) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var r = new StringBuilder();
        try {
            int e = 0, w = 0;
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1 | 256, "", ref e, ref w);   // lecture seule
            r.AppendLine("ouverture err " + e + " avert " + w);
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            r.AppendLine("composants de premier niveau : " + enf.Length);
            var cles = new[] { "capteur", "pièce moteur", "piece moteur", "embout", "batterie", "support capteurs", "roue arriere", "mcc", "cuve" };
            foreach (var c in enf) {
                
                var t = (double[])c.Transform2.ArrayData;
                double[] bx = c.GetBox(false, false) as double[];
                r.AppendLine(string.Format("{0} | fixe {1} | supprime {2} | {3}", c.Name2, c.IsFixed(), c.IsSuppressed(), c.GetPathName()));
                r.AppendLine("   R " + string.Join(" ", t.Take(9).Select(v => v.ToString("F4"))) + " | T(mm) " + string.Join(" ", t.Skip(9).Take(3).Select(v => (v * 1000).ToString("F2"))));
                if (bx != null) r.AppendLine("   boite(mm) " + string.Join(" ", bx.Select(v => (v * 1000).ToString("F1"))));
            }
        } catch (Exception ex) { r.AppendLine("EXCEPTION " + ex.Message); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r.ToString();
    }
}
'@
[Insp]::Lancer($asm) | Out-File "$PSScriptRoot\diff_inspecter.txt" -Encoding utf8
Get-Content "$PSScriptRoot\diff_inspecter.txt"
