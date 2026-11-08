$bureau = [Environment]::GetFolderPath('Desktop')
$v2 = Join-Path $bureau 'Assemblage1v2.SLDASM'
$sw_dir = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$fin = Join-Path $sw_dir '1_ASSEMBLAGE_FINAL (a utiliser)'
$bras = Join-Path $fin 'Bras 50cm.SLDASM'
$sortie = Join-Path $fin 'Vues visserie'
New-Item -ItemType Directory -Force $sortie | Out-Null
$png = Join-Path $fin 'Robot complet (Assemblage1v2).png'
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.Linq;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Vues {
    static double[] M(double r, double g, double b, double t) { return new[] { r, g, b, 1, 1, 0.3, 0.3, t, 0 }; }
    public static string Lancer(string v2, string bras, string sortie, string png) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        string r = "";
        int e = 0, w = 0;
        // 1. robot complet : esquisses masquees (reglage du document), image propre, enregistrement du seul document de tete
        sw.Visible = true;
        var d2 = sw.OpenDoc6(v2, 2, 1, "", ref e, ref w) as ModelDoc2;
        if (d2 == null) return "Assemblage1v2 non ouvert, err " + e;
        sw.ActivateDoc3(d2.GetTitle(), false, 0, ref e);
        d2.ShowNamedView2("*Isometric", 7); d2.ViewZoomtofit2();
        d2.GraphicsRedraw2();
        d2.Extension.SaveAs(png, 0, 1, null, ref e, ref w);
        bool ok = d2.Save3(1, ref e, ref w);
        r += "Assemblage1v2 enregistre : " + ok + " err " + e + " avert " + w + "\n";
        sw.CloseDoc(d2.GetTitle());
        // 2. bras seul
        var doc = (ModelDoc2)sw.OpenDoc6(bras, 2, 1, "", ref e, ref w);
        if (doc == null) return r + "ouverture bras impossible " + e;
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        doc = (ModelDoc2)sw.ActiveDoc;
        var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
        int nv = 0, ne = 0, na = 0;
        foreach (var c in enf) {
            string n = c.Name2;
            double[] m;
            if (n.StartsWith("Vis 2x8")) { m = M(0, 0.8, 0.1, 0); na++; }
            else if (n.StartsWith("Vis M3")) { m = M(0.05, 0.3, 1, 0); nv++; }
            else if (n.StartsWith("Ecrou") || n.StartsWith("Rondelle")) { m = M(1, 0.1, 0.1, 0); ne++; }
            else if (n.StartsWith("Servo") ) { m = M(0.25, 0.25, 0.25, 0.75); }
            else if (n.StartsWith("Roulement")) { m = M(0.6, 0.6, 0.65, 0.5); }
            else { m = M(0.9, 0.6, 0.25, 0.8); }
            c.SetMaterialPropertyValues2(m, 1, null);
        }
        r += "vis M3 " + nv + ", ecrous+rondelle " + ne + ", vis 2 mm " + na + "\n";
        doc.GraphicsRedraw2();
        var zones = new[] {
            new { nom = "1_base_lacet", comp = new[] { "socle-1", "patin_a-1", "patin_b-1", "patin_c-1", "plateau-1" }, vues = new[] { 7, 1, 6 } },
            new { nom = "2_epaule", comp = new[] { "plateau-1", "axe_epaule-1" }, vues = new[] { 7, 3, 4 } },
            new { nom = "3_coude", comp = new[] { "palier_coude-1", "bride_coude-1" }, vues = new[] { 7, 5, 6 } },
            new { nom = "4_poignet", comp = new[] { "bride_poignet-1" }, vues = new[] { 7, 5 } },
            new { nom = "5_pince", comp = new[] { "corps_pince-1" }, vues = new[] { 7 } },
        };
        foreach (var z in zones) {
            foreach (var v in z.vues) {
                doc.ClearSelection2(true);
                doc.ShowNamedView2("", v);
                bool prem = true;
                foreach (var nom in z.comp) {
                    var c = enf.FirstOrDefault(x => x.Name2 == nom);
                    if (c == null) { r += "absent : " + nom + "\n"; continue; }
                    c.Select4(!prem, null, false); prem = false;
                }
                doc.ViewZoomToSelection();
                doc.ClearSelection2(true);
                doc.GraphicsRedraw2();
                var f = System.IO.Path.Combine(sortie, z.nom + "_vue" + v + ".png");
                doc.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
            }
        }
        doc.ClearSelection2(true);
        doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
        doc.Extension.SaveAs(System.IO.Path.Combine(sortie, "0_ensemble.png"), 0, 1, null, ref e, ref w);
        // 3. fermer sans enregistrer les couleurs, puis rouvrir le fichier intact pour l'utilisateur
        sw.CloseDoc(doc.GetTitle());
        doc = (ModelDoc2)sw.OpenDoc6(bras, 2, 1, "", ref e, ref w);
        r += "bras rouvert sans couleurs : " + (doc != null) + "\n";
        if (doc != null) { doc.ShowNamedView2("", 7); doc.ViewZoomtofit2(); }
        return r;
    }
}
'@
[Vues]::Lancer($v2, $bras, $sortie, $png)
Get-ChildItem $sortie | ForEach-Object { '{0:HH:mm:ss} {1,9} {2}' -f $_.LastWriteTime, $_.Length, $_.Name }
Get-ChildItem $bras, $v2 | ForEach-Object { '{0:dd/MM HH:mm:ss} {1}' -f $_.LastWriteTime, $_.Name }
