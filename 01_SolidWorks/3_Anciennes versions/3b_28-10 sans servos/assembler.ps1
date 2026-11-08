# FICHIER GENERE par bras_50cm/generer_assemblage_sw.py : assemblage contraint du bras 50 cm
$dll = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\'
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.sldworks.dll')
[void][Reflection.Assembly]::LoadFrom($dll + 'SolidWorks.Interop.swconst.dll')
Add-Type -ReferencedAssemblies ($dll + 'SolidWorks.Interop.sldworks.dll'), ($dll + 'SolidWorks.Interop.swconst.dll') -TypeDefinition @'

using System;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
public static class AssemblageBras {
    static string journal = "";
    static Feature DernierePiece(ModelDoc2 doc) { return (Feature)doc.FeatureByPositionReverse(0); }
    static List<Feature> Plans(Component2 c) {
        var l = new List<Feature>();
        var f = (Feature)c.FirstFeature();
        while (f != null && l.Count < 3) { if (f.GetTypeName2() == "RefPlane") l.Add(f); f = (Feature)f.GetNextFeature(); }
        return l;
    }
    static void Contrainte(AssemblyDoc ad, string quoi) {
        int err = 0;
        var m = ad.AddMate5((int)swMateType_e.swMateCOINCIDENT, (int)swMateAlign_e.swMateAlignCLOSEST, false,
                            0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
        journal += quoi + (m == null ? " : ECHEC (" + err + ")\n" : " : ok\n");
        ((ModelDoc2)ad).ClearSelection2(true);
    }
    public static string Lancer(string d) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        try { return Construire(sw, d); }
        catch (Exception ex) { return journal + "ERREUR : " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
    }
    static string Construire(SldWorks sw, string d) {
string[] pieces = { "socle", "plateau", "bras", "bride_coude", "avant_bras", "corps_pince", "pignon_pince", "machoire_a", "machoire_b" };
string[][] esqNoms = {
new string[] { "Axe_plateau" },
new string[] { "Axe_propre", "Axe_bras" },
new string[] { "Axe_propre", "Axe_avant_bras" },
new string[] {  },
new string[] { "Axe_propre", "Axe_corps_pince" },
new string[] { "Axe_propre", "Axe_pignon_pince", "Axe_machoire_a", "Axe_machoire_b" },
new string[] { "Axe_propre" },
new string[] { "Axe_propre" },
new string[] { "Axe_propre" } };
double[][][] esqPts = {
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0500000, 0.0000000, 0.0000000, 0.0800000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0300000 }, new double[] { 0.0000000, 0.0000000, 0.0430000, 0.0000001, 0.0300000, 0.0430000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0300000 }, new double[] { 0.2500000, 0.0000000, 0.0126000, 0.2500000, 0.0000000, 0.0426000 } },
new double[][] {  },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0300000 }, new double[] { -0.1234000, 0.0160000, -0.0236000, -0.1234002, 0.0160000, 0.0064000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0300000 }, new double[] { -0.0226170, 0.0263250, -0.0176400, 0.0051959, 0.0375697, -0.0176399 }, new double[] { -0.0319180, 0.0442630, -0.0222000, -0.0041050, 0.0555073, -0.0222000 }, new double[] { -0.0319180, 0.0442630, 0.0078000, -0.0041050, 0.0555073, 0.0078000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0300000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, -0.0300000, 0.0000000, 0.0000000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, -0.0300000, 0.0000000, 0.0000000 } } };
double[][] placements = {
new double[] { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -1.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0800000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -1.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.1300000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4794223, 0.8775843, 0.0000018, 0.8775843, -0.4794223, 0.0000032, 0.0000037, 0.0000000, -1.0000000, 0.0000000, 0.1730000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4794223, 0.8775843, 0.0000018, 0.8775843, -0.4794223, 0.0000032, 0.0000037, 0.0000000, -1.0000000, 0.0000000, 0.1730000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.8968755, 0.4422831, -0.0000033, 0.4422831, 0.8968755, 0.0000016, 0.0000037, 0.0000000, -1.0000000, 0.1198556, 0.3923961, -0.0125996, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.9774735, -0.2110581, -0.0000093, 0.2110581, -0.9774735, 0.0000053, -0.0000103, 0.0000032, 1.0000000, 0.2376065, 0.3521684, 0.0110009, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4834044, -0.7113640, -0.5101778, -0.2867431, 0.4219713, -0.8600689, 0.8271025, 0.5620511, 0.0000041, 0.2652703, 0.3312098, -0.0066388, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.8271101, 0.5620399, 0.0000067, 0.4147792, -0.6104062, 0.6748055, 0.3792717, -0.5581357, -0.7379956, 0.2781478, 0.3156389, -0.0111986, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.8271101, 0.5620399, 0.0000067, 0.1821705, -0.2680973, 0.9460115, 0.5316980, -0.7824545, -0.3241331, 0.2781475, 0.3156390, 0.0188014, 1.0000000, 0.0000000, 0.0000000, 0.0000000 } };
string[][] liaisons = {
new string[] { "socle", "Axe_plateau", "plateau" },
new string[] { "plateau", "Axe_bras", "bras" },
new string[] { "bras", "Axe_avant_bras", "avant_bras" },
new string[] { "avant_bras", "Axe_corps_pince", "corps_pince" },
new string[] { "corps_pince", "Axe_pignon_pince", "pignon_pince" },
new string[] { "corps_pince", "Axe_machoire_a", "machoire_a" },
new string[] { "corps_pince", "Axe_machoire_b", "machoire_b" } };

        int err = 0, att = 0;
        // 0. import des STEP -> pieces SolidWorks (PLA)
        foreach (var nom in pieces) {
            if (System.IO.File.Exists(d + nom + ".SLDPRT")) System.IO.File.Delete(d + nom + ".SLDPRT");
            object donnees = sw.GetImportFileData(d + nom + ".step");
            var imp = (ModelDoc2)sw.LoadFile4(d + nom + ".step", "r", donnees, ref err);
            if (imp == null) { journal += nom + " : import impossible (" + err + ")\n"; continue; }
            if (imp.GetType() != 1) {
                // maillage en plusieurs morceaux : importe comme assemblage -> enregistre en piece multicorps
                imp.Extension.SaveAs(d + nom + ".SLDPRT", 0, 1, null, ref err, ref att);
                sw.CloseDoc(imp.GetTitle());
                journal += nom + " : assemblage importe enregistre en piece (" + err + ")\n";
                continue;
            }
            ((PartDoc)imp).SetMaterialPropertyName2("", "SOLIDWORKS Materials", "PLA");
            imp.Extension.SaveAs(d + nom + ".SLDPRT", 0, 1, null, ref err, ref att);
            sw.CloseDoc(imp.GetTitle());
        }
        // 1. esquisses d'axes dans chaque piece
        for (int i = 0; i < pieces.Length; i++) {
            var doc = (ModelDoc2)sw.OpenDoc6(d + pieces[i] + ".SLDPRT", 1, 1, "", ref err, ref att);
            if (doc == null) { journal += pieces[i] + " : ouverture impossible\n"; continue; }
            for (int k = 0; k < esqNoms[i].Length; k++) {
                if (((PartDoc)doc).FeatureByName(esqNoms[i][k]) != null) continue;
                doc.ClearSelection2(true);
                doc.SketchManager.Insert3DSketch(true);
                doc.SketchManager.AddToDB = true;            // pas d'accrochage automatique sur la geometrie voisine
                var p = esqPts[i][k];
                var ligne = doc.SketchManager.CreateLine(p[0], p[1], p[2], p[3], p[4], p[5]);
                doc.SketchManager.AddToDB = false;
                doc.SketchManager.Insert3DSketch(true);
                Feature esq = null;
                var f = (Feature)doc.FirstFeature();
                while (f != null) { if (f.GetTypeName2() == "3DProfileFeature") esq = f; f = (Feature)f.GetNextFeature(); }
                if (esq != null) esq.Name = esqNoms[i][k];
                journal += string.Format("{0} : esquisse {1} {2}\n", pieces[i], esqNoms[i][k],
                    (ligne != null && esq != null && ((PartDoc)doc).FeatureByName(esqNoms[i][k]) != null) ? "ok" : "ECHEC");
            }
            doc.Extension.SaveAs(d + pieces[i] + ".SLDPRT", 0, 1, null, ref err, ref att);
        }
        // 2. assemblage
        string gabarit = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
        var assy = (ModelDoc2)sw.NewDocument(gabarit, 0, 0, 0);
        var ad = (AssemblyDoc)assy;
        var mu = (MathUtility)sw.GetMathUtility();
        var comps = new Dictionary<string, Component2>();
        for (int i = 0; i < pieces.Length; i++) {
            var c = (Component2)ad.AddComponent5(d + pieces[i] + ".SLDPRT", 0, "", false, "", 0, 0, 0);
            if (c == null) { journal += pieces[i] + " : non insere\n"; continue; }
            c.Transform2 = (MathTransform)mu.CreateTransform(placements[i]);
            comps[pieces[i]] = c;
        }
        comps["socle"].Select4(false, null, false);
        ad.FixComponent();
        assy.ClearSelection2(true);
        // 3. liaisons pivot : segments colineaires + debuts coincidents
        foreach (var l in liaisons) {
            var fp = (Feature)comps[l[0]].FeatureByName(l[1]);
            var fe = (Feature)comps[l[2]].FeatureByName("Axe_propre");
            if (fp == null || fe == null) { journal += l[0] + " -> " + l[2] + " : esquisse introuvable\n"; continue; }
            var sp = (SketchLine)((object[])((Sketch)fp.GetSpecificFeature2()).GetSketchSegments())[0];
            var se = (SketchLine)((object[])((Sketch)fe.GetSpecificFeature2()).GetSketchSegments())[0];
            ((SketchSegment)sp).Select4(false, null);
            ((SketchSegment)se).Select4(true, null);
            Contrainte(ad, "pivot " + l[0] + " / " + l[2] + " (axe)");
            ((SketchPoint)sp.GetStartPoint2()).Select4(false, null);
            ((SketchPoint)se.GetStartPoint2()).Select4(true, null);
            Contrainte(ad, "pivot " + l[0] + " / " + l[2] + " (centre)");
        }
        // 4. bride du coude : fixee au bras par ses trois plans
        var pb = Plans(comps["bras"]); var pr = Plans(comps["bride_coude"]);
        for (int k = 0; k < 3; k++) {
            pb[k].Select2(false, 0); pr[k].Select2(true, 0);
            Contrainte(ad, "bride / bras (plan " + (k + 1) + ")");
        }
        assy.ForceRebuild3(false);
        foreach (var kv in comps) {
            var t = (double[])kv.Value.Transform2.ArrayData;
            journal += string.Format("  {0} en ({1:F0}, {2:F0}, {3:F0}) mm\n", kv.Key, t[9] * 1000, t[10] * 1000, t[11] * 1000);
        }
        int e2 = 0, w2 = 0;
        bool ok = assy.Extension.SaveAs(d + "Bras 50cm.SLDASM", 0, 1, null, ref e2, ref w2);
        journal += "Bras 50cm.SLDASM : " + (ok ? "enregistre" : "erreur " + e2) + "\n";
        return journal;
    }
}

'@
[AssemblageBras]::Lancer('C:\Users\amaur\Desktop\Bras robotisé\01_SolidWorks\')
