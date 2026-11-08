# Images du PDF depuis l'assemblage actuel. Se connecte au SolidWorks ouvert, n'enregistre rien, ne ferme pas SolidWorks.
$bureau = [Environment]::GetFolderPath('Desktop')
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$fin = Join-Path $racine '1_ASSEMBLAGE_FINAL (a utiliser)'
$img = Join-Path (Split-Path $racine) '05_Documentation\images'
$arch = Join-Path $img 'anciennes analyses\avant 06-11'
New-Item -ItemType Directory -Force $arch | Out-Null
Get-ChildItem $img -File | Where-Object { $_.Name -match '^(sw_|empreinte_)' } | ForEach-Object { if (-not (Test-Path (Join-Path $arch $_.Name))) { Copy-Item $_.FullName $arch } }
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Images {
    public static string Lancer(string fin, string img) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        string r = ""; int e = 0, w = 0;
        var doc = (ModelDoc2)sw.OpenDoc6(Path.Combine(fin, "Bras 50cm.SLDASM"), 2, 1, "", ref e, ref w);
        if (doc == null) return "bras non ouvert " + e;
        doc = (ModelDoc2)sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
        var vues = new[] {
            new { f = "sw_ensemble.png", c = new string[0], v = 7 },
            new { f = "sw_articulation_epaule.png", c = new[] { "plateau-1", "axe_epaule-1" }, v = 7 },
            new { f = "sw_articulation_lacet.png", c = new[] { "socle-1", "patin_a-1", "patin_b-1", "patin_c-1", "plateau-1" }, v = 1 },
            new { f = "sw_articulation_coude.png", c = new[] { "palier_coude-1", "bride_coude-1" }, v = 7 },
            new { f = "sw_articulation_poignet.png", c = new[] { "bride_poignet-1", "corps_pince-1" }, v = 7 },
        };
        foreach (var x in vues) {
            doc.ClearSelection2(true);
            doc.ShowNamedView2("", x.v);
            if (x.c.Length == 0) doc.ViewZoomtofit2();
            else {
                bool prem = true;
                foreach (var n in x.c) { var c = enf.FirstOrDefault(k => k.Name2 == n); if (c != null) { c.Select4(!prem, null, false); prem = false; } else r += "absent " + n + "\n"; }
                doc.ViewZoomToSelection();
                doc.ClearSelection2(true);
            }
            doc.GraphicsRedraw2();
            doc.Extension.SaveAs(Path.Combine(img, x.f), 0, 1, null, ref e, ref w);
        }
        doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
        // pieces seules : empreintes
        var pieces = new[] { new { p = "plateau", v = 6, f = "empreinte_plateau.png" }, new { p = "corps_pince", v = 7, f = "empreinte_corps_pince.png" }, new { p = "avant_bras", v = 6, f = "empreinte_avant_bras.png" } };
        foreach (var x in pieces) {
            var d = sw.OpenDoc6(Path.Combine(fin, x.p + ".SLDPRT"), 1, 1, "", ref e, ref w) as ModelDoc2;
            if (d == null) { r += "piece absente " + x.p + " err " + e + "\n"; continue; }
            d = (ModelDoc2)sw.ActivateDoc3(d.GetTitle(), false, 0, ref e);
            d.ShowNamedView2("", x.v); d.ViewZoomtofit2(); d.GraphicsRedraw2();
            d.Extension.SaveAs(Path.Combine(img, x.f), 0, 1, null, ref e, ref w);
            sw.QuitDoc(d.GetTitle());
        }
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        return r + "ok\n";
    }
}
'@
[Images]::Lancer($fin, $img)
Copy-Item (Join-Path $fin 'Robot complet (Assemblage1v2).png') (Join-Path $img 'sw_robot_complet.png') -Force
foreach ($n in '1_base_lacet_vue7', '1_base_lacet_vue6', '2_epaule_vue7', '3_coude_vue7', '4_poignet_vue7') { Copy-Item (Join-Path $fin "Vues visserie\$n.png") (Join-Path $img "visserie_$n.png") -Force }
Get-ChildItem $img -File | ForEach-Object { '{0:HH:mm:ss} {1,8} {2}' -f $_.LastWriteTime, $_.Length, $_.Name }
Get-ChildItem $fin -Filter 'Bras 50cm.SLDASM' | ForEach-Object { '{0:HH:mm:ss} {1}' -f $_.LastWriteTime, $_.Name }
