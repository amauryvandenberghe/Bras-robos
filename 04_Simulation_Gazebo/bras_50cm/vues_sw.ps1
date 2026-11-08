# Ouvre l'assemblage du bras, colore les servos au niveau des composants et enregistre des vues PNG (ensemble + zooms).
# Usage : powershell -ExecutionPolicy Bypass -File vues_sw.ps1 "<dossier 01_SolidWorks>\" "<dossier de sortie>\"
param([string]$dossier, [string]$sortie)
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks est ouvert : ferme-le puis relance.'; exit 1 }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class VuesBras {
    public static string Lancer(string d, string s) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string j = "";
        try {
            int err = 0, att = 0;
            var doc = (ModelDoc2)sw.OpenDoc6(d + "Bras 50cm.SLDASM", 2, 1, "", ref err, ref att);
            if (doc == null) return "ouverture impossible " + err;
            foreach (Component2 c in (object[])((AssemblyDoc)doc).GetComponents(true)) {
                string n = c.Name2.ToLower();
                if (n.StartsWith("servo")) c.MaterialPropertyValues = new double[] { 0.12, 0.12, 0.16, 1, 1, 0.5, 0.3, 0, 0 };
                else if (n.StartsWith("palonnier")) c.MaterialPropertyValues = new double[] { 0.95, 0.95, 0.95, 1, 1, 0.5, 0.3, 0, 0 };
            }
            doc.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swDisplaySketches, false);
            doc.Save3(1, ref err, ref att);
            // vues : nom, vue nommee, boite de zoom (m, repere SolidWorks : y vers le haut) ou null = tout
            object[][] vues = {
                new object[] { "ensemble", "*Isometric", null },
                new object[] { "zoom_socle_epaule", "*Isometric", new double[] { -0.06, 0.07, -0.06, 0.07, 0.23, 0.06 } },
                new object[] { "zoom_coude", "*Isometric", new double[] { 0.20, 0.13, -0.05, 0.29, 0.21, 0.04 } },
                new object[] { "zoom_poignet_pince", "*Isometric", new double[] { 0.33, 0.06, -0.04, 0.42, 0.17, 0.06 } },
                new object[] { "zoom_poignet_dessous", "*Bottom", new double[] { 0.30, 0.06, -0.05, 0.43, 0.17, 0.07 } },
            };
            foreach (var v in vues) {
                doc.ShowNamedView2((string)v[1], -1);
                if (v[2] == null) doc.ViewZoomtofit2();
                else { var b = (double[])v[2]; doc.ViewZoomTo2(b[0], b[1], b[2], b[3], b[4], b[5]); }
                doc.GraphicsRedraw2();
                bool ok = doc.Extension.SaveAs(s + "sw_" + (string)v[0] + ".png", 0, 1, null, ref err, ref att);
                j += v[0] + (ok ? " ok\n" : " erreur " + err + "\n");
            }
        } catch (Exception ex) { j += "ERREUR " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j;
    }
}
'@
[VuesBras]::Lancer($dossier, $sortie)
