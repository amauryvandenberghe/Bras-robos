$bureau = [Environment]::GetFolderPath('Desktop')
$f = Join-Path $bureau 'ICAM\Pieces tipe\piece avec les bites.SLDPRT'
$racine = (Resolve-Path (Join-Path $bureau '*\01_SolidWorks') | Where-Object { Test-Path (Join-Path $_ '1_ASSEMBLAGE_FINAL (a utiliser)') } | Select-Object -First 1).Path
$stl = Join-Path (Split-Path $racine) '02_Impression_3D\Robot (chassis)\piece avec les bites (poutre droite).stl'
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'
using System;
using System.Linq;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class Tp {
    static double Vol(ModelDoc2 d) { return ((MassProperty)d.Extension.CreateMassProperty()).Volume * 1e9; }
    public static string Lancer(string f, string stl) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        string r = ""; int e = 0, w = 0;
        try {
            sw.CloseAllDocuments(true);
            var d = (ModelDoc2)sw.OpenDoc6(f, 1, 1, "", ref e, ref w);
            sw.Visible = true;
            d = (ModelDoc2)sw.ActivateDoc3(d.GetTitle(), false, 0, ref e);
            double v0 = Vol(d);
            // esquisse d'essai precedente : supprimee
            var old = (Feature)((PartDoc)d).FeatureByName("Esquisse11");
            if (old != null) { d.ClearSelection2(true); old.Select2(false, 0); d.Extension.DeleteSelection2(0); r += "ancienne esquisse supprimee\n"; }
            d.ClearSelection2(true);
            var e3 = (Feature)((PartDoc)d).FeatureByName("Esquisse3");
            Feature plan = null;
            foreach (var p in (object[])e3.GetParents()) { var pf = p as Feature; if (pf != null && pf.GetTypeName2() == "RefPlane") plan = pf; }
            bool sel = plan != null && plan.Select2(false, 0);
            r += "plan de la poutre : " + (plan != null ? plan.Name : "?") + "\n";
            d.SketchManager.InsertSketch(true);
            d.SketchManager.AddToDB = true;
            var ska = (Sketch)d.SketchManager.ActiveSketch;
            var mu = (MathUtility)sw.GetMathUtility();
            var m2s = (MathTransform)ska.ModelToSketchTransform;
            object c1 = null, c2 = null;
            foreach (var yy in new[] { 0.020, -0.020 }) {
                var q = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { 0.0, yy, 0.0 })).MultiplyTransform(m2s)).ArrayData;
                var c = d.SketchManager.CreateCircleByRadius(q[0], q[1], 0, 0.00175);
                if (yy > 0) c1 = c; else c2 = c;
            }
            d.SketchManager.AddToDB = false;
            r += "plan " + sel + ", cercles " + (c1 != null) + " " + (c2 != null) + "\n";
            var tr = (MathTransform)ska.ModelToSketchTransform.Inverse();
            foreach (SketchSegment sg in (object[])ska.GetSketchSegments()) {
                var arc = sg as SketchArc;
                if (arc == null) continue;
                var cp = (SketchPoint)arc.GetCenterPoint2();
                var pm = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new[] { cp.X, cp.Y, cp.Z })).MultiplyTransform(tr)).ArrayData;
                r += "  cercle centre modele " + string.Join(",", pm.Select(x => (x * 1000).ToString("F2"))) + " r " + (arc.GetRadius() * 1000).ToString("F2") + "\n";
            }
            d.SketchManager.InsertSketch(true);
            var esq = (Feature)d.FeatureByPositionReverse(0);
            var skf = (Sketch)esq.GetSpecificFeature2();
            r += "esquisse : " + esq.Name + " " + esq.GetTypeName2() + " regions " + skf.GetSketchRegionCount() + " lecture seule " + d.IsOpenedReadOnly() + "\n";
            d.ClearSelection2(true);
            d.Extension.SelectByID2(esq.Name, "SKETCH", 0, 0, 0, false, 0, null, 0);
            Feature cut = null;
            foreach (var dir in new[] { true, false }) {
                d.ClearSelection2(true);
                d.Extension.SelectByID2(esq.Name, "SKETCH", 0, 0, 0, false, 0, null, 0);
                cut = d.FeatureManager.FeatureCut4(true, false, dir, (int)swEndConditions_e.swEndCondThroughAll, (int)swEndConditions_e.swEndCondThroughAll, 0.01, 0.01, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, false);
                r += "coupe a travers tout (2 sens) dir=" + dir + " : " + (cut != null) + "\n";
                if (cut != null) { d.ForceRebuild3(false); r += string.Format("  volume enleve {0:F1} mm3 (attendu 2 x 144 = 289)\n", v0 - Vol(d)); break; }
            }
            for (int essai = 0; essai < 6 && cut == null; essai++) {
                bool flip = essai % 2 == 1;
                d.ClearSelection2(true);
                bool s2 = d.Extension.SelectByID2(esq.Name, "SKETCH", 0, 0, 0, false, 0, null, 0);
                if (essai < 2) cut = d.FeatureManager.FeatureCut3(false, false, flip, (int)swEndConditions_e.swEndCondBlind, 0, 0.016, 0, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false);
                else if (essai < 4) cut = d.FeatureManager.FeatureCut4(false, false, flip, (int)swEndConditions_e.swEndCondBlind, 0, 0.016, 0, false, false, false, false, 0, 0, false, false, false, false, false, true, true, true, true, false, 0, 0, false, true);
                else { d.FeatureCut(false, false, flip, (int)swEndConditions_e.swEndCondBlind, 0, 0.016, 0, false, false, false, false, 0, 0, false, false); cut = (Feature)d.FeatureByPositionReverse(0); if (cut != null && cut.GetTypeName2() != "Cut" && cut.GetTypeName2() != "ICE") cut = null; }
                r += "essai " + essai + " (sel " + s2 + ") flip=" + flip + " : " + (cut != null) + (cut != null ? " " + cut.GetTypeName2() : "") + "\n";
                if (cut != null) {
                    d.ForceRebuild3(false);
                    double dv = v0 - Vol(d);
                    r += string.Format("  volume enleve {0:F1} mm3 (attendu 2 x 144 = 289)\n", dv);
                    if (dv > 250) break;
                    d.EditUndo2(1); cut = null;
                }
            }
            if (cut != null) {
                cut.Name = "Trous vis support capteurs";
                d.Save3(1, ref e, ref w); r += "enregistre err " + e + "\n";
                d.Extension.SaveAs(stl, 0, 1, null, ref e, ref w); r += "stl err " + e + "\n";
            }
            var b = (double[])((PartDoc)d).GetPartBox(true);
            r += "boite " + string.Join(" ", b.Select(x => (x * 1000).ToString("F1"))) + "\n";
            foreach (Body2 bo in (object[])((PartDoc)d).GetBodies2(0, true)) { var bb = (double[])bo.GetBodyBox(); r += "corps " + bo.Name + " " + string.Join(" ", bb.Select(x => (x * 1000).ToString("F2"))) + "\n"; }
        } catch (Exception ex) { r += "EXCEPTION " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
        return r;
    }
}
'@
[Tp]::Lancer($f, $stl)
