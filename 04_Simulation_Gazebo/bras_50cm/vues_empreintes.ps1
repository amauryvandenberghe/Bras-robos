# Vues PNG des empreintes de palonniers (chaque piece vue du cote du servo) et des articulations montees.
$dossier = "C:\Users\amaur\Desktop\Bras robotisé\01_SolidWorks\1_ASSEMBLAGE_FINAL (a utiliser)\"; $sortie = "C:\Users\amaur\Desktop\Bras robotisé\05_Documentation\images\"
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks est ouvert : ferme-le puis relance.'; exit 1 }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class VuesEmpreintes {
    public static string Lancer(string d, string s) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string j = "";
        try {
            int err = 0, att = 0;
            // 1. chaque piece entrainee, vue du cote du servo (l'empreinte est face a nous)
            string[][] pieces = {
                new string[] { "plateau", "*Front" }, new string[] { "bras", "*Front" },
                new string[] { "avant_bras", "*Front" }, new string[] { "corps_pince", "*Front" } };
            foreach (var p in pieces) {
                var doc = (ModelDoc2)sw.OpenDoc6(d + p[0] + ".SLDPRT", 1, 1, "", ref err, ref att);
                if (doc == null) { j += p[0] + " : ouverture impossible\n"; continue; }
                doc.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swDisplaySketches, false);
                string[] noms = { "*Front", "*Back" };
                int[] ids = { 1, 2 };
                for (int k = 0; k < 2; k++) {
                    string vue = noms[k];
                    doc.ShowNamedView2(vue, ids[k]);
                    doc.ViewZoomtofit2();
                    doc.GraphicsRedraw2();
                    string f = "empreinte_" + p[0] + "_" + vue.Substring(1).ToLower() + ".png";
                    bool ok = doc.Extension.SaveAs(s + f, 0, 1, null, ref err, ref att);
                    j += f + (ok ? " ok\n" : " erreur " + err + "\n");
                }
                sw.CloseDoc(doc.GetTitle());
            }
            // 2. assemblage : couleurs (servos noirs, palonniers blancs), vues des articulations
            var a = (ModelDoc2)sw.OpenDoc6(d + "Bras 50cm.SLDASM", 2, 1, "", ref err, ref att);
            if (a == null) return j + "assemblage : ouverture impossible " + err;
            var comps = (object[])((AssemblyDoc)a).GetComponents(true);
            foreach (Component2 c in comps) {
                string n = c.Name2;
                if (n.StartsWith("Servo MG996R arm01") || n.StartsWith("broche"))
                    c.MaterialPropertyValues = new double[] { 0.97, 0.97, 0.97, 1, 1, 0.5, 0.3, 0, 0 };
                else if (n.StartsWith("Servo"))
                    c.MaterialPropertyValues = new double[] { 0.12, 0.12, 0.16, 1, 1, 0.5, 0.3, 0, 0 };
            }
            a.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swDisplaySketches, false);
            a.Save3(1, ref err, ref att);
            a.ShowNamedView2("*Isometric", 7);
            a.ViewZoomtofit2();
            a.Extension.SaveAs(s + "sw_ensemble.png", 0, 1, null, ref err, ref att);
            string[][] vues = {
                new string[] { "articulation_lacet", "*Isometric", "socle", "Servo MG996R-1", "Servo MG996R arm01-1" },
                new string[] { "articulation_epaule", "*Isometric", "Servo MG996R-2", "Servo MG996R arm01-2" },
                new string[] { "articulation_coude", "*Isometric", "Servo MG90S-1", "broche-1", "bride_coude" },
                new string[] { "articulation_poignet", "*Isometric", "Servo MG90S-2", "broche-2", "bride_poignet" } };
            foreach (var v in vues) {
                a.ShowNamedView2(v[1], 7);
                a.ClearSelection2(true);
                foreach (Component2 c in comps)
                    for (int k = 2; k < v.Length; k++)
                        if (c.Name2.StartsWith(v[k])) c.Select4(true, null, false);
                a.ViewZoomToSelection();
                a.ClearSelection2(true);
                a.ViewZoomout();
                a.GraphicsRedraw2();
                bool ok = a.Extension.SaveAs(s + "sw_" + v[0] + ".png", 0, 1, null, ref err, ref att);
                j += v[0] + (ok ? " ok\n" : " erreur " + err + "\n");
            }
            j += "composants : ";
            foreach (Component2 c in comps) j += c.Name2 + " ; ";
            // 3. detection d'interferences de SolidWorks sur tout l'assemblage (posture d'enregistrement)
            var idm = ((AssemblyDoc)a).InterferenceDetectionManager;
            idm.TreatCoincidenceAsInterference = false;
            idm.IncludeMultibodyPartInterferences = false;
            var inter = (object[])idm.GetInterferences();
            j += "\nINTERFERENCES SolidWorks : " + (inter == null ? 0 : inter.Length) + "\n";
            if (inter != null)
                foreach (IInterference it in inter) {
                    string noms = "";
                    foreach (Component2 c in (object[])it.Components) noms += c.Name2 + " / ";
                    j += string.Format("  {0}: {1:F1} mm3\n", noms, it.Volume * 1e9);
                }
            idm.Done();
        } catch (Exception ex) { j += "ERREUR " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j;
    }
}
'@
[VuesEmpreintes]::Lancer($dossier, $sortie)
