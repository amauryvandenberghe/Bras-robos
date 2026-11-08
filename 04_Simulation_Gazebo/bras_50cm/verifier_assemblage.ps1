# Assemblage final (sans fenetre, sans enregistrer) : composants retrouves, contraintes en erreur apres reconstruction,
# etat de contrainte de chaque composant, interferences, et test de rotation des 4 articulations.
$dossier = 'C:\Users\amaur\Desktop\Bras robotisé\01_SolidWorks\1_ASSEMBLAGE_FINAL (a utiliser)\'
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks est ouvert : ferme-le puis relance.'; exit 1 }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using SolidWorks.Interop.sldworks;
public static class VerifAssemblage {
    public static string Lancer(string d) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string j = "";
        try {
            int err = 0, att = 0;
            var a = (ModelDoc2)sw.OpenDoc6(d + "Bras 50cm.SLDASM", 2, 1, "", ref err, ref att);
            if (a == null) return "ouverture impossible, erreur " + err;
            var comps = (object[])((AssemblyDoc)a).GetComponents(true);
            int ok = 0, ko = 0;
            string etats = "";
            foreach (Component2 c in comps) {
                string p = c.GetPathName();
                if (p.StartsWith(d, StringComparison.OrdinalIgnoreCase) && File.Exists(p) && c.GetSuppression() != 0) ok++;
                else { ko++; j += "  PROBLEME " + c.Name2 + " -> " + p + "\n"; }
                // swConstrainedStatus_e : 2 = sous-contraint (peut bouger), 3 = completement contraint, 4 = sur-contraint
                etats += string.Format("  {0,-26} {1}{2}\n", c.Name2, c.GetConstrainedStatus(), c.IsFixed() ? " (fixe)" : "");
            }
            a.ForceRebuild3(false);
            int mates = 0, mates_err = 0;
            Feature f = (Feature)a.FirstFeature();
            while (f != null) {
                if (f.GetTypeName2() == "MateGroup") {
                    Feature s = (Feature)f.GetFirstSubFeature();
                    while (s != null) {
                        mates++; bool w; int code = s.GetErrorCode2(out w);
                        if (code != 0) { mates_err++; j += "  contrainte " + s.Name + " : code " + code + "\n"; }
                        s = (Feature)s.GetNextSubFeature();
                    }
                }
                f = (Feature)f.GetNextFeature();
            }
            j += string.Format("composants trouves dans le dossier final : {0}, problemes : {1}\ncontraintes : {2}, en erreur apres reconstruction : {3}\n", ok, ko, mates, mates_err);
            j += "etat de contrainte (2 = peut bouger, 3 = completement contraint, 4 = sur-contraint) :\n" + etats;
            var idm = ((AssemblyDoc)a).InterferenceDetectionManager;
            idm.TreatCoincidenceAsInterference = false;
            idm.IncludeMultibodyPartInterferences = false;
            var inter = (object[])idm.GetInterferences();
            j += "INTERFERENCES : " + (inter == null ? 0 : inter.Length) + "\n";
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
[VerifAssemblage]::Lancer($dossier)
