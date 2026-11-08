"""Prepare l'import dans SolidWorks des pieces neuves du bras 50 cm.

Copie les STEP et STL des pieces dans "Bureau\\Bras 50cm" et ecrit `sw_importer.ps1` (PowerShell + C#, via les
bibliotheques officielles de l'API SolidWorks), qui :
  1. importe chaque STEP et l'enregistre en piece SolidWorks (.SLDPRT), matiere PLA ;
  2. cree l'assemblage "Bras 50cm.SLDASM" avec les pieces placees comme dans l'URDF (bras en posture de travail).
L'avant-bras et la pince sont ceux du bras actuel (fichiers SolidWorks d'origine, inchanges).
Usage : ~/venv_cao/bin/python generer_solidworks.py   puis, sous Windows : powershell -File sw_importer.ps1
"""
import os
import shutil
import sys
import numpy as np

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ICI, '..', 'ia_50cm'))
from cinematique import Bras, BRAS  # noqa: E402

DOSSIER_WIN = 'C:\\Users\\amaur\\Desktop\\Bras 50cm\\'
DOSSIER_WSL = '/mnt/c/Users/amaur/Desktop/Bras 50cm'
DLL = 'C:\\Program Files\\SOLIDWORKS Corp\\SOLIDWORKS\\api\\redist\\'
PIECES = {'socle': 'base_link', 'plateau': 'bras un', 'bras': 'bras deux', 'bride_coude': 'bras deux'}
POSTURE = np.array([0.0, 0.5, 1.4, 0.9])      # bras penche vers l'avant, pince vers le bas

CSHARP = r'''
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
'''


def ps1():
    b = Bras()
    q = dict(zip(BRAS, POSTURE))
    placements = []
    # SolidWorks a l'axe Y vers le haut (l'URDF : Z) : on tourne tout de -90 degres autour de X
    vers_sw = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)
    for lien in PIECES.values():
        T = vers_sw @ b.pose(lien, q)
        # MathTransform SolidWorks : rotation (3 x 3, par lignes des axes), translation (m), echelle, 3 inutilises
        placements.append(list(T[:3, :3].T.flatten()) + list(T[:3, 3]) + [1.0, 0.0, 0.0, 0.0])
    tableaux = ',\n    '.join('[double[]]@(' + ', '.join(f'{v:.6f}' for v in p) + ')' for p in placements)
    noms = ', '.join(f"'{n}'" for n in PIECES)
    return f'''# FICHIER GENERE par bras_50cm/generer_solidworks.py
# Importe les pieces neuves du bras 50 cm dans SolidWorks (STEP -> SLDPRT) et cree l'assemblage.
$dll = '{DLL}'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
{CSHARP}
'@
$placements = [double[][]]@(
    {tableaux}
)
[ImportBras]::Lancer('{DOSSIER_WIN}', [string[]]@({noms}), $placements)
'''


if __name__ == '__main__':
    os.makedirs(DOSSIER_WSL, exist_ok=True)
    for nom in PIECES:
        for ext in ('step', 'stl'):
            shutil.copy(os.path.join(ICI, 'pieces', f'{nom}.{ext}'), os.path.join(DOSSIER_WSL, f'{nom}.{ext}'))
    for vieux in ('sw_importer.vbs', 'Bras 50cm.SLDASM'):
        if os.path.exists(os.path.join(DOSSIER_WSL, vieux)):
            os.remove(os.path.join(DOSSIER_WSL, vieux))      # premier essai rate (assemblage vide)
    with open(os.path.join(DOSSIER_WSL, 'sw_importer.ps1'), 'w', encoding='utf-8-sig') as f:
        f.write(ps1())
    print('->', DOSSIER_WIN)
