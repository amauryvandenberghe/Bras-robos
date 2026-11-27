# 27/11/2026 : capteurs EN FLECHE (4 devant, plus de rangee arriere, elements a 8 mm du sol) + embouts de batteries agrandis.
# Archive les fichiers touches, copie les STEP/STL generes par CadQuery (WSL ~/fleche_out), reimporte les 2 pieces,
# deplace 4 sous-assemblages "capteur", supprime les 2 en trop, controle contraintes et interferences, enregistre.
# NE PAS RELANCER apres usage (l'archive serait ecrasee par l'etat deja modifie).
for ($i = 0; $i -lt 12 -and (Get-Process SLDWORKS -ErrorAction SilentlyContinue); $i++) { Start-Sleep -Seconds 10 }
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks ouvert : abandon'; exit 1 }
$racine = (Resolve-Path (Join-Path ([Environment]::GetFolderPath('Desktop')) '*\01_SolidWorks')).Path
$fin = Join-Path $racine '1_VERSION_FINALE (a ouvrir)'
$pr = Join-Path $fin 'Pieces robot'
$asm = Join-Path $fin 'Assemblage1v2.SLDASM'
$arch = Join-Path $racine '3_Anciennes versions\3u_27-11 avant capteurs en fleche et embouts agrandis'
$impr = Join-Path (Split-Path $racine) '02_Impression_3D'
$src = '\\wsl.localhost\Ubuntu-22.04\home\amaury\fleche_out\'
$moteur = [char]0x00E8   # e accent grave
$nomMoteur = "pi${moteur}ce moteur direction"
if (Test-Path -LiteralPath $arch) { 'archive deja presente : script deja passe, abandon'; exit 1 }
New-Item -ItemType Directory -Force $arch | Out-Null
foreach ($f in @('Assemblage1v2.SLDASM')) { Copy-Item -LiteralPath (Join-Path $fin $f) $arch }
foreach ($f in @("$nomMoteur.SLDPRT", 'piece moteur direction capteurs.step', 'embout batteries.SLDPRT', 'embout batteries.step')) { Copy-Item -LiteralPath (Join-Path $pr $f) $arch }
Copy-Item -LiteralPath (Join-Path $src 'piece_moteur_direction_capteurs.step') (Join-Path $pr 'piece moteur direction capteurs.step') -Force
Copy-Item -LiteralPath (Join-Path $src 'embout_batteries.step') (Join-Path $pr 'embout batteries.step') -Force
Copy-Item -LiteralPath (Join-Path $src 'piece_moteur_direction_capteurs.stl') (Join-Path $impr "$nomMoteur.STL") -Force
Copy-Item -LiteralPath (Join-Path $src 'embout_batteries.stl') (Join-Path $impr 'embout batteries x2.stl') -Force
"archive : $((Get-ChildItem -LiteralPath $arch).Count) fichiers"
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.IO;
using System.Linq;
using System.Text;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Fleche {
    // element (bas, centre) dans le repere propre du sous-assemblage "capteur" (releve sur les 6 anciens capteurs)
    static readonly double[] P_LOCAL = { 12.64, 61.42, 8.39 };
    static double[] Mul(double[] M, double[] p) { return new[] { M[0] * p[0] + M[1] * p[1] + M[2] * p[2], M[3] * p[0] + M[4] * p[1] + M[5] * p[2], M[6] * p[0] + M[7] * p[1] + M[8] * p[2] }; }
    static string Placer(MathUtility mu, AssemblyDoc ad, ModelDoc2 doc, Component2 c, double x, double y, double z) {
        var t = (double[])c.Transform2.ArrayData;
        var m = Mul(t, P_LOCAL);    // rotations symetriques (0/1/-1) : pas d'ambiguite de transposition
        t[9] = (x - m[0]) / 1000; t[10] = (y - m[1]) / 1000; t[11] = (z - m[2]) / 1000;
        doc.ClearSelection2(true); c.Select4(false, null, false); ad.UnfixComponent();
        c.Transform2 = (MathTransform)mu.CreateTransform(t);
        doc.ClearSelection2(true); c.Select4(false, null, false); ad.FixComponent(); doc.ClearSelection2(true);
        var bx = (double[])c.GetBox(false, false);
        return string.Format("  {0} -> element ({1:F1}; {2:F1}; {3:F1}) boite x {4:F1}..{5:F1} y {6:F1}..{7:F1} z {8:F1}..{9:F1}\n", c.Name2, x, y, z,
                             bx[0] * 1000, bx[3] * 1000, bx[1] * 1000, bx[4] * 1000, bx[2] * 1000, bx[5] * 1000);
    }
    public static string Lancer(string asm, string pr, string nomMoteur, string png1, string png2) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false; var mu = (MathUtility)sw.GetMathUtility(); var j = new StringBuilder();
        try {
            int e = 0, w = 0, err = 0;
            sw.CloseAllDocuments(true);
            foreach (var pair in new[] { new[] { nomMoteur, "piece moteur direction capteurs" }, new[] { "embout batteries", "embout batteries" } }) {
                string f = Path.Combine(pr, pair[0] + ".SLDPRT"), st = Path.Combine(pr, pair[1] + ".step");
                File.Delete(f);
                var imp = (ModelDoc2)sw.LoadFile4(st, "r", sw.GetImportFileData(st), ref err);
                var corps = (object[])((PartDoc)imp).GetBodies2((int)swBodyType_e.swSolidBody, true);
                imp.Extension.SaveAs(f, 0, 1, null, ref e, ref w);
                j.AppendFormat("{0} reimporte : {1} corps volumique(s), err {2}\n", pair[0], corps == null ? 0 : corps.Length, e);
                sw.CloseDoc(imp.GetTitle());
            }
            var doc = (ModelDoc2)sw.OpenDoc6(asm, 2, 1, "", ref e, ref w);
            var ad = (AssemblyDoc)doc;
            doc.ForceRebuild3(false);
            var enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            Func<string, Component2> C = n => enf.First(c => c.Name2 == n);
            // sol a y 449,6 ; elements a 8 mm ; pivot z 512,72 ; +z = GAUCHE dans le sens de marche (le robot roule vers -x)
            double y = 449.6 + 8.0, zm = 512.72;
            j.Append("capteurs :\n");
            j.Append(Placer(mu, ad, doc, C("capteur-7"), 5.5, y, zm - 19.5));    // exterieur droit (carte vers -z)
            j.Append(Placer(mu, ad, doc, C("capteur-8"), 5.5, y, zm + 19.5));    // exterieur gauche (carte vers +z)
            j.Append(Placer(mu, ad, doc, C("capteur-9"), -2.5, y, zm - 7.6));    // interieur droit (carte vers +x)
            j.Append(Placer(mu, ad, doc, C("capteur-12"), -2.5, y, zm + 7.6));   // interieur gauche
            doc.ClearSelection2(true);
            C("capteur-10").Select4(false, null, false); C("capteur-11").Select4(true, null, false);
            j.Append("capteurs 10 et 11 (rangee arriere) supprimes : " + doc.Extension.DeleteSelection2(0) + "\n");
            doc.ClearSelection2(true); doc.ForceRebuild3(false);
            int total = 0, mauvais = 0; var fe = (Feature)doc.FirstFeature();
            while (fe != null) {
                if (fe.GetTypeName2() == "MateGroup") { var m = (Feature)fe.GetFirstSubFeature(); while (m != null) { if (m.GetSpecificFeature2() as Mate2 != null) { bool av; int code = m.GetErrorCode2(out av); total++; if (code != 0) { mauvais++; j.Append("ERREUR " + m.Name + " code " + code + "\n"); } } m = (Feature)m.GetNextSubFeature(); } }
                fe = (Feature)fe.GetNextFeature();
            }
            enf = ((object[])((Configuration)doc.GetActiveConfiguration()).GetRootComponent3(true).GetChildren()).Cast<Component2>().ToArray();
            j.AppendFormat("contraintes : {0}, en erreur : {1}, composants : {2}\n", total, mauvais, enf.Length);
            var mgr = ad.InterferenceDetectionManager;
            mgr.TreatCoincidenceAsInterference = false; mgr.TreatSubAssembliesAsComponents = true; mgr.IncludeMultibodyPartInterferences = false; mgr.IgnoreHiddenBodies = true;
            var lst = mgr.GetInterferences() as object[];
            var cles = new[] { "capteur", nomMoteur, "embout", "batterie", "roue arriere", "mcc", "cuve" };
            int nb = 0, nous = 0;
            if (lst != null) foreach (IInterference it in lst) { nb++; var cs = ((object[])it.Components).Cast<Component2>().Select(c => c.Name2).ToArray(); if (cs.Any(s => cles.Any(k => s.StartsWith(k)))) { nous++; j.AppendFormat("  INTERFERENCE {0:F2} mm3 : {1}\n", it.Volume * 1e9, string.Join(" / ", cs)); } }
            mgr.Done();
            j.AppendFormat("interferences totales : {0}, dont {1} avec les pieces touchees\n", nb, nous);
            if (mauvais > 0) { j.Append("ARRET : contraintes en erreur, NON enregistre\n"); return j.ToString(); }
            doc.ShowNamedView2("", 7); doc.ViewZoomtofit2();
            j.Append("enregistre : " + doc.Save3(1, ref e, ref w) + " err " + e + "\n");
            doc.Extension.SaveAs(png1, 0, 1, null, ref e, ref w);
            doc.ShowNamedView2("", 6); doc.ViewZoomtofit2();
            doc.Extension.SaveAs(png2, 0, 1, null, ref e, ref w);
        } catch (Exception ex) { j.Append("EXCEPTION " + ex.Message + "\n" + ex.StackTrace); }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return j.ToString();
    }
}
'@
$docs = Join-Path $fin 'Documents'
[Fleche]::Lancer($asm, $pr, $nomMoteur, (Join-Path $docs 'Robot complet (Assemblage1v2).png'), (Join-Path $docs 'Capteurs en fleche (vue de dessous).png')) | Out-File "$PSScriptRoot\fleche_sw.txt" -Encoding utf8
Get-Content "$PSScriptRoot\fleche_sw.txt"
