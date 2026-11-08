# FICHIER GENERE par bras_50cm/generer_assemblage_sw.py : assemblage contraint du bras 50 cm (servos compris)
if (Get-Process SLDWORKS -ErrorAction SilentlyContinue) { 'SolidWorks est ouvert : enregistre ton travail, ferme-le, puis relance ce script.'; exit 1 }
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
    static int nOk = 0, nEchec = 0;
    static void Contrainte(AssemblyDoc ad, int type, string quoi) {
        int err = 0;
        var m = ad.AddMate5(type, (int)swMateAlign_e.swMateAlignCLOSEST, false,
                            0, 0, 0, 0, 0, 0, 0, 0, false, false, 0, out err);
        if (m == null) { nEchec++; journal += quoi + " : ECHEC (" + err + ")\n"; } else nOk++;
        ((ModelDoc2)ad).ClearSelection2(true);
    }
    static SketchLine Ligne(Component2 c, string esquisse, int k) {
        var f = (Feature)c.FeatureByName(esquisse);
        if (f == null) return null;
        var segs = (object[])((Sketch)f.GetSpecificFeature2()).GetSketchSegments();
        return k < segs.Length ? (SketchLine)segs[k] : null;
    }
    public static string Lancer(string d) {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = false;
        try { return Construire(sw, d); }
        catch (Exception ex) { return journal + "ERREUR : " + ex.Message; }
        finally { sw.CloseAllDocuments(true); sw.ExitApp(); }
    }
    static string Construire(SldWorks sw, string d) {
string[] pieces = { "socle", "plateau", "bras", "bride_coude", "avant_bras", "bride_poignet", "corps_pince", "pignon_pince", "machoire_a", "machoire_b", "Servo MG996R", "Servo MG996R arm01", "Servo MG90S", "broche" };
string[][] esqNoms = {
new string[] { "Servo_servo_lacet" },
new string[] { "Servo_servo_epaule", "Palonnier_palonnier_lacet" },
new string[] { "Servo_servo_coude", "Palonnier_palonnier_epaule" },
new string[] {  },
new string[] { "Servo_servo_poignet", "Palonnier_broche_coude" },
new string[] {  },
new string[] { "Axe_pignon_pince", "Axe_machoire_a", "Axe_machoire_b", "Palonnier_broche_poignet", "Servo_servo_pince" },
new string[] { "Axe_propre" },
new string[] { "Axe_propre" },
new string[] { "Axe_propre" },
new string[] { "Repere" },
new string[] { "Repere" },
new string[] { "Repere" },
new string[] { "Repere" } };
double[][][] esqPts = {
new double[][] { new double[] { 0.0000000, -0.0000000, 0.0500000, 0.0000000, -0.0000000, 0.0800000, 0.0000000, -0.0000000, 0.0500000, 0.0300000, -0.0000000, 0.0500000 } },
new double[][] { new double[] { 0.0000000, 0.0200000, 0.0430000, 0.0000000, 0.0500000, 0.0430000, 0.0000000, 0.0200000, 0.0430000, 0.0000000, 0.0200000, 0.0130000 }, new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0300000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0300000, 0.0000000 } },
new double[][] { new double[] { 0.2500000, 0.0000000, 0.0126000, 0.2500000, 0.0000000, -0.0174000, 0.2500000, 0.0000000, 0.0126000, 0.2200000, 0.0000000, 0.0126000 }, new double[] { -0.0000000, -0.0000001, 0.0200000, -0.0000000, -0.0000002, 0.0500000, -0.0000000, -0.0000001, 0.0200000, 0.0300000, -0.0000001, 0.0200000 } },
new double[][] {  },
new double[][] { new double[] { -0.1234000, 0.0160000, -0.0216000, -0.1234000, 0.0160000, -0.0516000, -0.1234000, 0.0160000, -0.0216000, -0.0934000, 0.0160000, -0.0216000 }, new double[] { 0.0000000, 0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0300000, 0.0000000, 0.0000000, -0.0000000, -0.0297510, 0.0038575, -0.0000000 } },
new double[][] {  },
new double[][] { new double[] { -0.0226170, 0.0263250, -0.0176400, 0.0051959, 0.0375697, -0.0176399 }, new double[] { -0.0319180, 0.0442630, -0.0222000, -0.0041050, 0.0555073, -0.0222000 }, new double[] { -0.0319180, 0.0442630, 0.0078000, -0.0041050, 0.0555073, 0.0078000 }, new double[] { -0.0000000, -0.0000000, -0.0020000, -0.0000000, 0.0000002, 0.0280000, -0.0000000, -0.0000000, -0.0020000, -0.0112684, 0.0278033, -0.0020002 }, new double[] { -0.0170540, 0.0285744, -0.0176398, -0.0448668, 0.0173294, -0.0176398, -0.0170540, 0.0285744, -0.0176398, -0.0250055, 0.0482410, 0.0035734 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0300000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, -0.0300000, 0.0000000, 0.0000000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0000000, -0.0300000, 0.0000000, 0.0000000 } },
new double[][] { new double[] { -0.0102129, -0.0000047, 0.0394710, -0.0102363, -0.0000094, 0.0694710, -0.0102129, -0.0000047, 0.0394710, 0.0197871, -0.0000016, 0.0394944 } },
new double[][] { new double[] { 0.0000000, 0.0000000, 0.0055000, 0.0000000, 0.0000000, 0.0355000, 0.0000000, 0.0000000, 0.0055000, 0.0300000, 0.0000000, 0.0055000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, -0.0032000, 0.0000000, 0.0000000, 0.0268000, 0.0000000, 0.0000000, -0.0032000, 0.0300000, 0.0000000, -0.0032000 } },
new double[][] { new double[] { 0.0000000, 0.0000000, -0.0030000, 0.0000000, 0.0000000, 0.0270000, 0.0000000, 0.0000000, -0.0030000, 0.0300000, 0.0000000, -0.0030000 } } };
string[] instNoms = { "socle", "plateau", "bras", "bride_coude", "avant_bras", "bride_poignet", "corps_pince", "pignon_pince", "machoire_a", "machoire_b", "servo_lacet", "servo_epaule", "servo_coude", "servo_poignet", "palonnier_lacet", "palonnier_epaule", "broche_coude", "broche_poignet", "servo_pince" };
string[] instPieces = { "socle", "plateau", "bras", "bride_coude", "avant_bras", "bride_poignet", "corps_pince", "pignon_pince", "machoire_a", "machoire_b", "Servo MG996R", "Servo MG996R", "Servo MG90S", "Servo MG90S", "Servo MG996R arm01", "Servo MG996R arm01", "broche", "broche", "Servo MG90S" };
double[][] placements = {
new double[] { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -1.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0800000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, -1.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.1300000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4794223, 0.8775843, 0.0000018, 0.8775843, -0.4794223, 0.0000032, 0.0000037, 0.0000000, -1.0000000, 0.0000000, 0.1730000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4794223, 0.8775843, 0.0000018, 0.8775843, -0.4794223, 0.0000032, 0.0000037, 0.0000000, -1.0000000, 0.0000000, 0.1730000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.8968755, 0.4422831, -0.0000033, 0.4422831, 0.8968755, 0.0000016, 0.0000037, 0.0000000, -1.0000000, 0.1198556, 0.3923961, -0.0125996, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.8968755, 0.4422831, -0.0000033, 0.4422831, 0.8968755, 0.0000016, 0.0000037, 0.0000000, -1.0000000, 0.1198556, 0.3923961, -0.0125996, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.9774735, -0.2110581, -0.0000093, 0.2110581, -0.9774735, 0.0000053, -0.0000103, 0.0000032, 1.0000000, 0.2376065, 0.3521684, 0.0110009, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4834044, -0.7113640, -0.5101778, -0.2867431, 0.4219713, -0.8600689, 0.8271025, 0.5620511, 0.0000041, 0.2652703, 0.3312098, -0.0066388, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.8271101, 0.5620399, 0.0000067, 0.4147792, -0.6104062, 0.6748055, 0.3792717, -0.5581357, -0.7379956, 0.2781478, 0.3156389, -0.0111986, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.8271101, 0.5620399, 0.0000067, 0.1821705, -0.2680973, 0.9460115, 0.5316980, -0.7824545, -0.3241331, 0.2781475, 0.3156390, 0.0188014, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.9999997, -0.0007793, 0.0001052, 0.0001051, -0.0001566, -1.0000000, 0.0007793, 0.9999997, -0.0001565, 0.0101822, 0.0905210, 0.0000025, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.0001052, -0.9999997, 0.0007793, -1.0000000, -0.0001051, 0.0001566, -0.0001565, -0.0007793, -0.9999997, 0.0000025, 0.1628178, 0.0194790, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.4794223, -0.8775843, -0.0000018, 0.8775843, -0.4794223, 0.0000032, -0.0000037, -0.0000000, 1.0000000, 0.1198556, 0.3923961, -0.0093996, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { -0.8968755, 0.4422831, -0.0000033, -0.4422831, -0.8968755, -0.0000016, -0.0000037, -0.0000000, 1.0000000, 0.2376065, 0.3521684, 0.0122009, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.0000000, 0.0000000, -1.0000000, -1.0000000, 0.0000000, 0.0000000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.1245000, 0.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.4794223, 0.8775843, 0.0000018, 0.8775843, -0.4794223, -0.0000004, 0.0000004, 0.0000018, -1.0000000, 0.0000000, 0.1730000, -0.0145000, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.9463005, -0.3232883, 0.0000035, 0.3232883, 0.9463005, 0.0000012, -0.0000037, -0.0000000, 1.0000000, 0.1198556, 0.3923961, -0.0095996, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.5627573, -0.8266222, 0.0000017, 0.8266222, 0.5627573, 0.0000094, -0.0000087, -0.0000039, 1.0000000, 0.2376065, 0.3521684, 0.0120009, 1.0000000, 0.0000000, 0.0000000, 0.0000000 },
new double[] { 0.3974295, -0.5848430, 0.7071128, 0.3974441, -0.5848476, -0.7071008, 0.8270961, 0.5620605, 0.0000067, 0.2629541, 0.3296356, -0.0066386, 1.0000000, 0.0000000, 0.0000000, 0.0000000 } };
string[][] liaisons = {
new string[] { "corps_pince", "Axe_pignon_pince", "pignon_pince" },
new string[] { "corps_pince", "Axe_machoire_a", "machoire_a" },
new string[] { "corps_pince", "Axe_machoire_b", "machoire_b" } };
string[][] fixations = {
new string[] { "socle", "Servo_servo_lacet", "servo_lacet" },
new string[] { "plateau", "Servo_servo_epaule", "servo_epaule" },
new string[] { "bras", "Servo_servo_coude", "servo_coude" },
new string[] { "avant_bras", "Servo_servo_poignet", "servo_poignet" },
new string[] { "plateau", "Palonnier_palonnier_lacet", "palonnier_lacet" },
new string[] { "bras", "Palonnier_palonnier_epaule", "palonnier_epaule" },
new string[] { "avant_bras", "Palonnier_broche_coude", "broche_coude" },
new string[] { "corps_pince", "Palonnier_broche_poignet", "broche_poignet" },
new string[] { "corps_pince", "Servo_servo_pince", "servo_pince" } };
string[][] pivotsServo = {
new string[] { "servo_lacet", "palonnier_lacet" },
new string[] { "servo_epaule", "palonnier_epaule" },
new string[] { "servo_coude", "broche_coude" },
new string[] { "servo_poignet", "broche_poignet" } };
string[][] brides = {
new string[] { "bride_coude", "bras" },
new string[] { "bride_poignet", "avant_bras" } };

        int err = 0, att = 0;
        // 0. import des STEP -> pieces SolidWorks (PLA pour les pieces imprimees) ; les pieces du commerce sont deja
        //    des copies des fichiers SolidWorks d'origine (pas de STEP) : on les garde telles quelles
        foreach (var nom in pieces) {
            if (!System.IO.File.Exists(d + nom + ".step")) continue;
            if (System.IO.File.Exists(d + nom + ".SLDPRT")) System.IO.File.Delete(d + nom + ".SLDPRT");
            object donnees = sw.GetImportFileData(d + nom + ".step");
            var imp = (ModelDoc2)sw.LoadFile4(d + nom + ".step", "r", donnees, ref err);
            if (imp == null) { journal += nom + " : import impossible (" + err + ")\n"; continue; }
            bool commerce = nom.StartsWith("Servo") || nom.StartsWith("Palonnier");
            if (imp.GetType() == 1 && !commerce)
                ((PartDoc)imp).SetMaterialPropertyName2("", "SOLIDWORKS Materials", "PLA");
            // couleurs : PLA orange, servos noirs, palonniers blancs
            double[] c = nom.StartsWith("Servo") ? new double[] { 0.12, 0.12, 0.16 }
                       : nom.StartsWith("Palonnier") ? new double[] { 0.92, 0.92, 0.92 } : new double[] { 0.95, 0.55, 0.15 };
            imp.MaterialPropertyValues = new double[] { c[0], c[1], c[2], 1, 1, 0.5, 0.3, 0, 0 };
            imp.Extension.SaveAs(d + nom + ".SLDPRT", 0, 1, null, ref err, ref att);
            sw.CloseDoc(imp.GetTitle());
        }
        // 1. esquisses 3D (segments) dans chaque piece
        for (int i = 0; i < pieces.Length; i++) {
            var doc = (ModelDoc2)sw.OpenDoc6(d + pieces[i] + ".SLDPRT", 1, 1, "", ref err, ref att);
            if (doc == null) { journal += pieces[i] + " : ouverture impossible\n"; continue; }
            for (int k = 0; k < esqNoms[i].Length; k++) {
                if (((PartDoc)doc).FeatureByName(esqNoms[i][k]) != null) continue;
                doc.ClearSelection2(true);
                doc.SketchManager.Insert3DSketch(true);
                doc.SketchManager.AddToDB = true;            // pas d'accrochage automatique sur la geometrie voisine
                var p = esqPts[i][k];
                bool ok = true;
                for (int s = 0; s + 5 < p.Length; s += 6)
                    ok &= doc.SketchManager.CreateLine(p[s], p[s + 1], p[s + 2], p[s + 3], p[s + 4], p[s + 5]) != null;
                doc.SketchManager.AddToDB = false;
                doc.SketchManager.Insert3DSketch(true);
                Feature esq = null;
                var f = (Feature)doc.FirstFeature();
                while (f != null) { if (f.GetTypeName2() == "3DProfileFeature") esq = f; f = (Feature)f.GetNextFeature(); }
                if (esq != null) esq.Name = esqNoms[i][k];
                if (!ok || esq == null || ((PartDoc)doc).FeatureByName(esqNoms[i][k]) == null)
                    journal += pieces[i] + " : esquisse " + esqNoms[i][k] + " ECHEC\n";
            }
            doc.Extension.SaveAs(d + pieces[i] + ".SLDPRT", 0, 1, null, ref err, ref att);   // reste ouverte : AddComponent5 le demande
        }
        // 2. assemblage
        string gabarit = sw.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateAssembly);
        var assy = (ModelDoc2)sw.NewDocument(gabarit, 0, 0, 0);
        var ad = (AssemblyDoc)assy;
        var mu = (MathUtility)sw.GetMathUtility();
        var comps = new Dictionary<string, Component2>();
        for (int i = 0; i < instNoms.Length; i++) {
            var c = (Component2)ad.AddComponent5(d + instPieces[i] + ".SLDPRT", 0, "", false, "", 0, 0, 0);
            if (c == null) { journal += instNoms[i] + " : non insere\n"; continue; }
            c.Transform2 = (MathTransform)mu.CreateTransform(placements[i]);
            try { c.Name2 = instNoms[i]; } catch { }
            comps[instNoms[i]] = c;
        }
        comps["socle"].Select4(false, null, false);
        ad.FixComponent();
        assy.ClearSelection2(true);
        // 3. liaisons pivot : segments colineaires + debuts coincidents
        foreach (var l in liaisons) {
            var sp = Ligne(comps[l[0]], l[1], 0); var se = Ligne(comps[l[2]], "Axe_propre", 0);
            if (sp == null || se == null) { journal += l[0] + " -> " + l[2] + " : esquisse introuvable\n"; nEchec++; continue; }
            ((SketchSegment)sp).Select4(false, null); ((SketchSegment)se).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "pivot " + l[0] + " / " + l[2] + " (axe)");
            ((SketchPoint)sp.GetStartPoint2()).Select4(false, null); ((SketchPoint)se.GetStartPoint2()).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "pivot " + l[0] + " / " + l[2] + " (centre)");
        }
        // 3 bis. rotation des 4 servos du bras : entre le palonnier et le servo (axes colineaires, centres coincidents)
        foreach (var l in pivotsServo) {
            var s0 = Ligne(comps[l[0]], "Repere", 0); var p0 = Ligne(comps[l[1]], "Repere", 0);
            if (s0 == null || p0 == null) { journal += l[0] + " / " + l[1] + " : esquisse introuvable\n"; nEchec++; continue; }
            ((SketchSegment)s0).Select4(false, null); ((SketchSegment)p0).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "rotation " + l[0] + " / " + l[1] + " (axe)");
            ((SketchPoint)s0.GetStartPoint2()).Select4(false, null); ((SketchPoint)p0.GetStartPoint2()).Select4(true, null);
            Contrainte(ad, (int)swMateType_e.swMateCOINCIDENT, "rotation " + l[0] + " / " + l[1] + " (centre)");
        }
        // 4. servos dans leurs logements, palonniers dans leurs empreintes : UNE contrainte de verrouillage (les deux
        //    pieces sont deja a leur place exacte). Avant le 30/10 : axe + centre + parallele, en partie redondants ->
        //    SolidWorks signalait 18 contraintes sur-contraintes / mal definies a la reconstruction.
        foreach (var l in fixations) {
            comps[l[0]].Select4(false, null, false); comps[l[2]].Select4(true, null, false);
            Contrainte(ad, (int)swMateType_e.swMateLOCK, l[2] + " dans " + l[0] + " (verrouillage)");
        }
        // 5. brides : verrouillees sur leur piece
        foreach (var l in brides) {
            comps[l[0]].Select4(false, null, false); comps[l[1]].Select4(true, null, false);
            Contrainte(ad, (int)swMateType_e.swMateLOCK, l[1] + " sur " + l[0] + " (verrouillage)");
        }
        assy.ForceRebuild3(false);
        journal += string.Format("{0} contraintes ajoutees, {1} echecs\n", nOk, nEchec);
        foreach (var kv in comps) {
            var t = (double[])kv.Value.Transform2.ArrayData;
            journal += string.Format("  {0} en ({1:F0}, {2:F0}, {3:F0}) mm\n", kv.Key, t[9] * 1000, t[10] * 1000, t[11] * 1000);
        }
        assy.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swDisplaySketches, false);   // esquisses d'axes masquees
        assy.ShowNamedView2("*Isometric", 7);
        assy.ViewZoomtofit2();
        int e2 = 0, w2 = 0;
        bool ok2 = assy.Extension.SaveAs(d + "Bras 50cm.SLDASM", 0, 1, null, ref e2, ref w2);
        journal += "Bras 50cm.SLDASM : " + (ok2 ? "enregistre" : "erreur " + e2) + "\n";
        assy.Extension.SaveAs(d + "Bras 50cm.png", 0, 1, null, ref e2, ref w2);
        return journal;
    }
}

'@
[AssemblageBras]::Lancer('C:\Users\amaur\Desktop\Bras robotisé\01_SolidWorks\1_ASSEMBLAGE_FINAL (a utiliser)\')
