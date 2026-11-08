# FICHIER GENERE par bras_50cm/generer_solidworks.py
# Importe les pieces neuves du bras 50 cm dans SolidWorks (STEP -> SLDPRT) et cree l'assemblage.
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'

using System;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class ImportBras {
    public static string Lancer(string dossier, string[] noms, double[][] placements) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        string journal = "";
        foreach (var nom in noms) {
            if (Array.IndexOf(noms, nom) != Array.LastIndexOf(noms, nom)) continue;
            int err = 0, att = 0;
            object donnees = sw.GetImportFileData(dossier + nom + ".step");
            var doc = (ModelDoc2)sw.LoadFile4(dossier + nom + ".step", "r", donnees, ref err);
            if (doc == null) { journal += nom + " : import impossible (erreur " + err + ")\n"; continue; }
            ((PartDoc)doc).SetMaterialPropertyName2("", "SOLIDWORKS Materials", "PLA");
            bool ok = doc.Extension.SaveAs(dossier + nom + ".SLDPRT", 0, 1, null, ref err, ref att);
            journal += nom + ".SLDPRT : " + (ok ? "enregistre" : "erreur " + err) + "\n";
            sw.CloseDoc(doc.GetTitle());
        }
        string gabarit = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
        var assy = (ModelDoc2)sw.NewDocument(gabarit, 0, 0, 0);
        var ad = (AssemblyDoc)assy;
        var mu = (MathUtility)sw.GetMathUtility();
        for (int i = 0; i < noms.Length; i++) {
            int err = 0, att = 0;
            var piece = (ModelDoc2)sw.OpenDoc6(dossier + noms[i] + ".SLDPRT", 1, 1, "", ref err, ref att);
            var comp = (Component2)ad.AddComponent5(dossier + noms[i] + ".SLDPRT", 0, "", false, "", 0, 0, 0);
            if (comp == null) { journal += noms[i] + " : non insere\n"; continue; }
            comp.Transform2 = (MathTransform)mu.CreateTransform(placements[i]);
        }
        assy.EditRebuild3();
        int e2 = 0, w2 = 0;
        bool ok2 = assy.Extension.SaveAs(dossier + "Bras 50cm.SLDASM", 0, 1, null, ref e2, ref w2);
        journal += "Bras 50cm.SLDASM : " + (ok2 ? "enregistre" : "erreur " + e2) + "\n";
        sw.CloseAllDocuments(true);
        sw.ExitApp();
        return journal;
    }
}

'@
$placements = [double[][]]@(
    [double[]]@(1.000000, 0.000000, 0.000000, 0.000000, 0.000000, -1.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.080000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000),
    [double[]]@(1.000000, 0.000000, 0.000000, 0.000000, 0.000000, -1.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.130000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000),
    [double[]]@(0.479422, 0.877584, 0.000002, 0.877584, -0.479422, 0.000003, 0.000004, 0.000000, -1.000000, 0.000000, 0.173000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000),
    [double[]]@(0.479422, 0.877584, 0.000002, 0.877584, -0.479422, 0.000003, 0.000004, 0.000000, -1.000000, 0.000000, 0.173000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000)
)
[ImportBras]::Lancer('C:\Users\amaur\Desktop\Bras 50cm\', [string[]]@('socle', 'plateau', 'bras', 'bride_coude'), $placements)
