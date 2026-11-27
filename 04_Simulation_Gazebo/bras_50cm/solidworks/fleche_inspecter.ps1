# 27/11/2026 : LECTURE SEULE de Assemblage1v2 (capteurs, piece moteur direction, embouts, batteries) : positions et contraintes.
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
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1 | 256, "", ref e, ref w);   // silencieux + lecture seule
            r.AppendLine("ouverture err " + e + " avert " + w);
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            r.AppendLine("composants de premier niveau : " + enf.Length);
            var cles = new[] { "capteur", "pièce moteur", "piece moteur", "embout", "batterie", "support capteurs", "roue arriere", "mcc", "cuve" };
            foreach (var c in enf) {
                if (!cles.Any(k => c.Name2.ToLower().StartsWith(k))) continue;
                var t = (double[])c.Transform2.ArrayData;
                double[] bx = c.GetBox(false, false) as double[];
                r.AppendLine(string.Format("{0} | fixe {1} | supprime {2} | {3}", c.Name2, c.IsFixed(), c.IsSuppressed(), c.GetPathName()));
                r.AppendLine("   R " + string.Join(" ", t.Take(9).Select(v => v.ToString("F4"))) + " | T(mm) " + string.Join(" ", t.Skip(9).Take(3).Select(v => (v * 1000).ToString("F2"))));
                if (bx != null) r.AppendLine("   boite(mm) " + string.Join(" ", bx.Select(v => (v * 1000).ToString("F1"))));
            }
            var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") {
                    var m = (Feature)fe.GetFirstSubFeature();
                    while (m != null) {
                        var mm = m.GetSpecificFeature2() as Mate2;
                        if (mm != null) {
                            var noms = Enumerable.Range(0, mm.GetMateEntityCount()).Select(i => { var cc = mm.MateEntity(i).ReferenceComponent; return cc == null ? "?" : cc.Name2; }).ToArray();
                            if (noms.Any(n => cles.Any(k => n.ToLower().StartsWith(k)))) { bool av; int code = m.GetErrorCode2(out av); r.AppendLine("CONTRAINTE " + m.Name + " type " + mm.Type + " err " + code + " : " + string.Join(" / ", noms)); }
                        }
                        m = (Feature)m.GetNextSubFeature();
                    }
                }
                fe = (Feature)fe.GetNextFeature();
            }
        } catch (Exception ex) { r.AppendLine("EXCEPTION " + ex.Message); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r.ToString();
    }
}
'@
[Insp]::Lancer($asm) | Out-File "$PSScriptRoot\fleche_inspecter.txt" -Encoding utf8
Get-Content "$PSScriptRoot\fleche_inspecter.txt"
