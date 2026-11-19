// bras_robot : PROGRAMME UNIQUE du robot (13/11/2026). Trois autres fichiers dans ce dossier :
//   servo.h / servo.cpp  broches, calage et commande des servos (a regler la, et seulement la)
//   reseau.h             reseau de neurones du bras (poids + calcul), inclus ici seulement
// Carte "ESP32 Dev Module", schema de partition "Huge APP (3MB No OTA)" : les poids du reseau font 1,1 Mo.
//
// BLUETOOTH ("Bras_ESP32_Robot", application MIT App Inventor) - aussi accepte au moniteur serie USB (115 200 bauds)
//   L        envoye a la connexion : ignore
//   O<n>     direction : angle du servo 0 a 180, 90 = tout droit (borne a 10..170)
//   V<n>A    avance a la vitesse n (0 a 255) ; V<n>B recule ; V0A coupe le moteur
//   S        arret immediat : moteur coupe, direction droite, suivi de ligne arrete, bras fige sur place
//   H        bras en position Home : replie, centre de gravite bas, pour rouler. Revient par un chemin sur.
//   N        suivi de ligne doux sur les 3 capteurs avant (attend la ligne, puis la suit ; ligne perdue 1,5 s = arret)
//   1        mission (depuis Home) : balle et panier tires au hasard a portee du bras, annonces a l'application ;
//            le reseau prend la balle au sol et la lache au-dessus du panier (10 cm de haut), puis le bras revient en Home.
//            Sans servo de poignet (ACTIF dans servo.cpp) : sequence fixe a la place, le reseau ne sait pas faire sans.
//   2 3      ignores
//   Pendant une mission, V, O et N sont refuses : S pour tout arreter, ou H pour ramener le bras.
// USB EN PLUS (une ligne par commande) :
//   J i d    tourne le servo i (0 base, 1 epaule, 2 coude, 3 poignet, 4 pince) de d degres
//   D i d    essai : active le servo i et l'envoie a d degres servo
//   C        bras vertical et droit (posture de calage des palonniers), lentement
//   X        relache tous les servos du bras (tenir le bras !)
//   L        valeurs brutes des 3 capteurs de ligne avant
//   P        etat et diagnostic des servos

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "servo.h"
#include "reseau.h"

BluetoothSerial SerialBT;

// ---- PROPULSION (L298N) ----
const int PIN_ENA = 25, PIN_IN1 = 33, PIN_IN2 = 32;
const uint32_t FREQ_MOTEUR_HZ = 5000;   // 1000 -> 5000 le 19/11 : plus de sifflement a bas regime
const char SENS_AVANCE = 'A';  // essai du 12/11 : l'application avance avec A
const int CENTRE_DIR = 90;

// ---- SUIVI DE LIGNE (3 capteurs avant) : REECRIT le 19/11 ----
// Probleme constate : trop lent, reagit tard, quitte la ligne ; moteur qui siffle sans avancer a pleine consigne (alimentation 5 V :
// l'appel de courant au demarrage fait chuter la tension). Maintenant :
//  - position de la ligne en continu (-1 capteur gauche ... +1 capteur droit), derivee mesuree avec le vrai pas de temps ;
//  - braquage = proportionnel + derive, applique tout de suite (le servo est deja le facteur limitant), jusqu'a 32 degres ;
//  - vitesse : pleine en ligne droite, reduite selon l'ecart ET la vitesse de l'ecart ; montee en rampe (pas de calage) ;
//  - ligne perdue : on braque a fond du dernier cote vu, plus lentement, 1,5 s au plus ;
//  - calibration noir / blanc continue (glissante), contraste minimal abaisse a 120.
const int CAPT[3] = {27, 15, 34};          // gauche, milieu, droite
const bool NOIR_ELEVE = true;              // la ligne noire donne une valeur haute
const int CONTRASTE_MIN = 120;             // ecart noir / blanc minimal (sur 4095) pour croire a une ligne
const int V_LIGNE_DROITE = 220;            // vitesse ligne bien centree (0 a 255)
const int V_LIGNE_VIRAGE = 160;            // vitesse mini dans un virage serre
const int V_PERDU = 140;                   // vitesse quand la ligne est perdue
const int V_DEMARRAGE = 190;               // consigne de depart (puis rampe)
const float ACCEL_PAR_S = 500.0f;          // montee de la vitesse (unites PWM par seconde)
const float KP_DEG = 40.0f;                // degres de braquage pour un ecart de 1 (ligne sous un capteur exterieur)
const float KD_DEFAUT = 60.0f;             // terme derive, par pas de 10 ms
const float BRAQUAGE_MAX_DEG = 32.0f;
const unsigned long PERIODE_SUIVI_MS = 5;  // 200 Hz (le servo ne se rafraichit qu'a 50 Hz)
const int SIGNE_BRAQUAGE = 1;              // -1 si le robot s'ecarte de la ligne au lieu d'y revenir
// Reglables sans reteleverser (USB : T kp kd vdroite vvirage, ex. T 40 60 220 160).
float kp = KP_DEG, kd = KD_DEFAUT, v_droite = V_LIGNE_DROITE, v_virage = V_LIGNE_VIRAGE;
const unsigned long PERTE_MAX_MS = 1500;   // ligne perdue plus longtemps : arret

// Une rangee de 3 capteurs (gauche, milieu, droite) avec son etalonnage noir / blanc automatique.
struct Rangee {
    const int *pin;
    int cmin[3], cmax[3], b[3];
    bool vu;
    float p;   // -1 ligne sous le capteur gauche, 0 au milieu, +1 sous le capteur droit
};
// ---- BRAS ----
// Postures en radians (convention du reseau ; pince : SERVO_OUVERT / SERVO_FERME). Entre parentheses : degres servo.
// Home : epaule penchee de 40 degres, coude replie : centre de gravite 6,5 cm plus bas qu'au repos du reseau.
// Trajets repos <-> Home <-> vertical verifies (ni sol, ni robot) en simulation le 13/11.
const float Q_HOME[N_BRAS] = {0.0f, 0.69f, 2.46f, 1.1704f, SERVO_OUVERT};     // (90, 93.9, 155.9, 39.9, 110)
const float Q_VERTICAL[N_BRAS] = {0.0f, 0.0f, 0.0f, 1.1704f, SERVO_OUVERT};   // (90, 54.4, 15, 39.9, 110)
float Q_DEPART_RESEAU[N_BRAS];                                                // Q_REPOS du reseau + pince ouverte
// Mission : balle a 21-29 cm de l'axe du bras et a moins de 50 degres de l'axe du robot ; panier a 22-28 cm, a moins
// de 70 degres, a plus de 45 degres de la balle. Lacher a 18 cm (hauteur apprise pour un petit receptacle) :
// la balle tombe de 8 cm dans le panier de 10 cm. Simulation du 13/11 : 99 missions reussies sur 100, ~15 s chacune.
const float Z_BALLE_SOL = 0.0148f, Z_LACHER = 0.18f;
const float R_TENUE = 0.02f;               // pince fermee a moins de 2 cm de la balle = balle tenue (pas de capteur)
const unsigned long PERIODE_RESEAU_MS = 100, PERIODE_BRAS_MS = 20, PAUSE_DEPOT_MS = 600;
const int PAS_MAX = 700;                   // 70 s au plus
// Sequence fixe (sans poignet), en degres servo : celle du 12/11, vue a la camera.
struct PasFixe { int servo; float deg; unsigned long ms; };
const PasFixe SEQ_FIXE[] = {{BASE, 115, 2000}, {EPAULE, 85, 2500}, {COUDE, 105, 2500}, {EPAULE, 115, 3000},
                            {PINCE, 60, 1000}, {EPAULE, 80, 3000}, {COUDE, 15, 2500}, {EPAULE, 54.4f, 2500},
                            {PINCE, 110, 1000}, {BASE, 90, 2000}};

ESP32PWM moteur;

static void signaler(const char *m) { Serial.println(m); SerialBT.println(m); }

// ======================================================================== CONDUITE
enum Suivi { SUIVI_OFF, SUIVI_ATTENTE, SUIVI_ROULE };
Suivi suivi = SUIVI_OFF;
int cmin[3], cmax[3], dernier_cote = 1;

unsigned long debut_roule = 0, derniere_vue = 0, prochain_suivi = 0, prochain_aff = 0;

static void moteur_commande(int v, char sens) {
    v = constrain(v, 0, 255);
    digitalWrite(PIN_IN1, (v > 0 && sens == 'B') ? HIGH : LOW);
    digitalWrite(PIN_IN2, (v > 0 && sens == 'A') ? HIGH : LOW);
    moteur.write(v);
}
static void arreter_conduite() { suivi = SUIVI_OFF; moteur_commande(0, SENS_AVANCE); direction_ecrire(CENTRE_DIR); }
const int CAPT_AR[3] = {26, 14, 35};       // rangee arriere (ajoutee le 19/11) : gauche, milieu, droite
Rangee AV = {CAPT}, AR = {CAPT_AR};
const float KH_DEFAUT = 25.0f;             // degres de braquage pour un cap de 1 (ecart avant - arriere)
float kh = KH_DEFAUT;

static void lire_rangee(Rangee &r) {
    for (int i = 0; i < 3; i++) { int s = 0; for (int k = 0; k < 4; k++) s += analogRead(r.pin[i]); r.b[i] = s / 4; }
}
static void lire_capteurs(int b[3]) { lire_rangee(AV); for (int i = 0; i < 3; i++) b[i] = AV.b[i]; }
static void init_rangee(Rangee &r) { lire_rangee(r); for (int i = 0; i < 3; i++) r.cmin[i] = r.cmax[i] = r.b[i]; r.vu = false; r.p = 0; }
static void mesurer(Rangee &r) {
    lire_rangee(r);
    int gmin = 4095, gmax = 0;
    for (int i = 0; i < 3; i++) {
        r.cmin[i] = min(r.cmin[i], r.b[i]); r.cmax[i] = max(r.cmax[i], r.b[i]);
        gmin = min(gmin, r.cmin[i]); gmax = max(gmax, r.cmax[i]);
    }
    r.vu = false; r.p = 0;
    if (gmax - gmin < CONTRASTE_MIN) return;
    float n[3];
    for (int i = 0; i < 3; i++) {
        bool propre = r.cmax[i] - r.cmin[i] >= CONTRASTE_MIN;
        float bas = propre ? r.cmin[i] : gmin, haut = propre ? r.cmax[i] : gmax;
        float x = constrain((r.b[i] - bas) / (haut - bas), 0.0f, 1.0f);
        n[i] = NOIR_ELEVE ? x : 1.0f - x;
    }
    float t = n[0] + n[1] + n[2];
    if (t >= 0.35f) { r.vu = true; r.p = (n[2] - n[0]) / t; }
}

float pos_prec = 0, derive_filtree = 0, v_actuelle = 0, braquage = 0;
unsigned long dernier_us = 0;

static void lancer_suivi() {
    arreter_conduite();
    init_rangee(AV); init_rangee(AR);
    pos_prec = derive_filtree = v_actuelle = braquage = 0;
    dernier_us = 0;
    suivi = SUIVI_ATTENTE;
    signaler("N suivi de ligne : attente de la ligne");
}

// Toutes les PERIODE_SUIVI_MS. Avant : p_av (regarde devant). Arriere : p_ar (86 mm derriere). Cap = p_av - p_ar :
// si la ligne penche, l'avant la voit s'ecarter avant que l'arriere ne bouge : on braque plus tot et plus juste.
static void pas_suivi() {
    unsigned long us = micros();
    float dt = dernier_us ? (us - dernier_us) * 1e-6f : PERIODE_SUIVI_MS * 1e-3f;
    dernier_us = us;
    if (dt <= 0 || dt > 0.05f) dt = PERIODE_SUIVI_MS * 1e-3f;

    mesurer(AV); mesurer(AR);
    bool vu = AV.vu || AR.vu;
    float p = AV.p, cap = 0;
    if (AV.vu && AR.vu) cap = AV.p - AR.p;
    else if (!AV.vu && AR.vu) p = 1.3f * AR.p;   // l'avant a depasse la ligne, l'arriere la voit encore : on la rattrape

    unsigned long t = millis();
    if (suivi == SUIVI_ATTENTE) {
        if (!AV.vu) return;
        suivi = SUIVI_ROULE;
        debut_roule = derniere_vue = t;
        pos_prec = p;
        v_actuelle = V_DEMARRAGE;
        signaler("N ligne trouvee");
    }
    if (vu) {
        derniere_vue = t;
        if (fabsf(p) > 0.15f) dernier_cote = p > 0 ? 1 : -1;
    } else {
        if (t - derniere_vue > PERTE_MAX_MS) { arreter_conduite(); signaler("N ligne perdue : arret (renvoyer N)"); return; }
        p = 1.5f * dernier_cote;  // ligne sortie d'un cote : braquage a fond de ce cote
    }
    float derive = (p - pos_prec) / dt;           // 1/s
    pos_prec = p;
    derive_filtree += 0.5f * (derive - derive_filtree);
    float but = constrain(-(kp * p + kd * 0.01f * derive_filtree + kh * cap), -BRAQUAGE_MAX_DEG, BRAQUAGE_MAX_DEG);
    braquage += 0.7f * (but - braquage);          // lissage leger contre le bruit des capteurs
    direction_ecrire(CENTRE_DIR + SIGNE_BRAQUAGE * braquage);

    // vitesse : pleine si l'avant et l'arriere sont sur la ligne et alignes ; on ralentit des que la ligne s'ecarte ou tourne
    float gene = min(1.0f, 1.2f * fabsf(p) + 0.06f * fabsf(derive_filtree) + 0.8f * fabsf(cap));
    float v_cible = vu ? v_droite - (v_droite - v_virage) * gene : (float)V_PERDU;
    if (v_cible > v_actuelle) v_actuelle = min(v_cible, v_actuelle + ACCEL_PAR_S * dt);
    else v_actuelle = v_cible;
    moteur_commande((int)lroundf(v_actuelle), SENS_AVANCE);

    static unsigned long dernier_pas = 0, pire = 0;
    if (dernier_pas) pire = max(pire, t - dernier_pas);
    dernier_pas = t;
    if ((long)(t - prochain_aff) >= 0) {
        prochain_aff = t + 200;
        Serial.printf("N av %+.2f%s ar %+.2f%s cap %+.2f brq %+.1f v %d | AV %d %d %d AR %d %d %d | pas max %lu ms\n", AV.p, AV.vu ? "" : "(-)", AR.p,
                      AR.vu ? "" : "(-)", cap, braquage, (int)v_actuelle, AV.b[0], AV.b[1], AV.b[2], AR.b[0], AR.b[1], AR.b[2], pire);
        pire = 0;
    }
}

// ======================================================================== BRAS
enum Mode { LIBRE, TRAJET, RESEAU, DEPOT, REBOURS };
enum Suite { FIN_RIEN, FIN_HOME, FIN_RESEAU };
Mode mode = LIBRE;
bool au_home = false;      // bras immobile en Home
bool mission = false;      // mission (bouton 1) en cours, jusqu'au retour en Home
bool histo_valide = false; // le bras est sur le chemin du reseau : le retour sur passe par l'historique

// Trajet : suite de postures rejointes en douceur (tous les servos ensemble, ou un seul si servo >= 0).
struct Etape { float q[N_BRAS]; unsigned long ms; int servo; };
const int N_ETAPES_MAX = 16;
Etape etapes[N_ETAPES_MAX];
int n_etapes = 0, etape_i = 0;
Suite suite = FIN_RIEN;
float depart[N_BRAS], fin_file[N_BRAS];
unsigned long debut_etape = 0, prochain_bras = 0;

static void vider_file() {
    n_etapes = 0;
    for (int i = 0; i < N_BRAS; i++) fin_file[i] = bras_consigne(i);
}
static void ajouter(const float q[N_BRAS], unsigned long ms) {
    if (n_etapes >= N_ETAPES_MAX) return;
    Etape &E = etapes[n_etapes++];
    for (int i = 0; i < N_BRAS; i++) E.q[i] = fin_file[i] = servo_borne(i, q[i]);
    E.ms = ms;
    E.servo = -1;
}
static void ajouter_un(int servo, float deg, unsigned long ms) {
    float q[N_BRAS];
    memcpy(q, fin_file, sizeof(q));
    q[servo] = servo_q(servo, deg);
    ajouter(q, ms);
    etapes[n_etapes - 1].servo = servo;
}
static void lancer_trajet(int s) {  // s : une valeur de Suite
    etape_i = 0;
    for (int i = 0; i < N_BRAS; i++) depart[i] = bras_consigne(i);
    debut_etape = prochain_bras = millis();
    suite = (Suite)s;
    au_home = false;
    mode = TRAJET;
}

// Reseau
EtatBras etat;
float balle[3], cible[3], prec[N_BRAS], suiv[N_BRAS], bas_reseau[4], haut_reseau[4];
bool tenue = false;
int pas_n = 0, n_histo = 0;
float histo[PAS_MAX][N_BRAS];  // consignes du reseau, pour revenir par le meme chemin (14 Ko)
unsigned long t_pas = 0, prochain_reseau = 0, debut_depot = 0;

static void finir_home() {
    mode = LIBRE;
    au_home = true;
    histo_valide = false;
    if (mission) { mission = false; signaler("Mission terminee : bras en Home"); }
    else signaler("Bras en Home");
}

static void aller_home_direct() {
    histo_valide = false;
    vider_file();
    ajouter(Q_HOME, 3000);
    lancer_trajet(FIN_HOME);
}

static void lancer_rebours() {
    mode = REBOURS;
    au_home = false;
    for (int i = 0; i < N_BRAS; i++) prec[i] = suiv[i] = bras_consigne(i);
    t_pas = prochain_reseau = millis();
    signaler("Retour par le chemin du reseau");
}

static void aller_home() {
    if (mode == RESEAU || (mode == LIBRE && histo_valide && n_histo > 0)) { lancer_rebours(); return; }
    if (mode == REBOURS || mode == DEPOT) return;                                       // deja en route vers Home
    if (mode == TRAJET && suite == FIN_HOME && etape_i == n_etapes - 1) return;          // derniere etape : Home
    if (au_home && mode == LIBRE) { signaler("Deja en Home"); return; }
    signaler("H : retour en Home");
    aller_home_direct();
}

static float hasard(float a, float b) { return a + (b - a) * (float)(esp_random() / 4294967296.0); }

static void lancer_mission() {
    if (mode != LIBRE || !au_home) { signaler("1 refuse : bras pas en Home (appuyer sur H, puis 1)"); return; }
    arreter_conduite();
    mission = true;
    if (!ACTIF[POIGNET]) {
        signaler("Poignet hors service : sequence fixe (le reseau a besoin du poignet)");
        vider_file();
        ajouter(Q_VERTICAL, 3000);
        for (const PasFixe &p : SEQ_FIXE) ajouter_un(p.servo, p.deg, p.ms);
        ajouter(Q_HOME, 3000);
        lancer_trajet(FIN_HOME);
        return;
    }
    const float D = (float)M_PI / 180.0f;
    float rb, ab, rp, ap;
    do {
        rb = hasard(0.21f, 0.29f); ab = hasard(-50, 50) * D;
        rp = hasard(0.22f, 0.28f); ap = hasard(-70, 70) * D;
    } while (fabsf(ap - ab) < 45 * D);
    balle[0] = rb * cosf(ab); balle[1] = rb * sinf(ab); balle[2] = Z_BALLE_SOL;
    cible[0] = rp * cosf(ap); cible[1] = rp * sinf(ap); cible[2] = Z_LACHER;
    char m[160];
    snprintf(m, sizeof(m), "Mission : balle a %.0f cm, %s de %.0f deg (x %.1f y %.1f cm) ; panier a %.0f cm, %s de %.0f deg "
             "(x %.1f y %.1f cm)", rb * 100, ab >= 0 ? "gauche" : "droite", fabsf(ab) / D, balle[0] * 100, balle[1] * 100,
             rp * 100, ap >= 0 ? "gauche" : "droite", fabsf(ap) / D, cible[0] * 100, cible[1] * 100);
    signaler(m);
    vider_file();
    ajouter(Q_DEPART_RESEAU, 2500);
    lancer_trajet(FIN_RESEAU);
}

static void demarrer_reseau() {
    memcpy(etat.q, Q_REPOS, sizeof(etat.q));
    etat.servo = SERVO_OUVERT;
    tenue = false;
    pas_n = n_histo = 0;
    histo_valide = true;
    for (int i = 0; i < N_BRAS; i++) prec[i] = suiv[i] = Q_DEPART_RESEAU[i];
    t_pas = prochain_reseau = millis();
    mode = RESEAU;
    signaler("Reseau : prise de la balle");
}

static void pas_reseau() {  // toutes les 100 ms
    float o[3];
    reseau_outil(etat.q, o);
    if (etat.servo > SERVO_CONTACT) {
        if (tenue) { tenue = false; signaler("Balle lachee"); mode = DEPOT; debut_depot = millis(); return; }
    } else {
        float dx = o[0] - balle[0], dy = o[1] - balle[1], dz = o[2] - balle[2];
        if (!tenue && sqrtf(dx * dx + dy * dy + dz * dz) < R_TENUE) { tenue = true; signaler("Balle saisie"); }
    }
    if (pas_n >= PAS_MAX) { signaler("Mission trop longue : abandon"); lancer_rebours(); return; }
    reseau_pas(&etat, tenue ? o : balle, cible, 0, bas_reseau, haut_reseau);
    pas_n++;
    memcpy(prec, suiv, sizeof(prec));
    for (int i = 0; i < 4; i++) suiv[i] = etat.q[i];
    suiv[PINCE] = etat.servo;
    memcpy(histo[n_histo++], suiv, sizeof(suiv));
    t_pas = millis();
}

static void pas_rebours() {  // toutes les 100 ms : l'historique a l'envers
    memcpy(prec, suiv, sizeof(prec));
    if (n_histo > 0) { memcpy(suiv, histo[--n_histo], sizeof(suiv)); t_pas = millis(); return; }
    histo_valide = false;
    vider_file();
    ajouter(Q_DEPART_RESEAU, 1000);
    ajouter(Q_HOME, 2500);
    lancer_trajet(FIN_HOME);
}

static void pas_trajet() {  // toutes les 20 ms
    const Etape &E = etapes[etape_i];
    unsigned long t = millis() - debut_etape;
    float s = t >= E.ms ? 1.0f : (float)t / E.ms;
    s = s * s * (3.0f - 2.0f * s);
    for (int i = 0; i < N_BRAS; i++)
        if (E.servo < 0 || E.servo == i) bras_poser(i, depart[i] + (E.q[i] - depart[i]) * s);
    if (t < E.ms) return;
    if (++etape_i < n_etapes) {
        for (int i = 0; i < N_BRAS; i++) depart[i] = bras_consigne(i);
        debut_etape = millis();
        return;
    }
    mode = LIBRE;
    if (suite == FIN_HOME) finir_home();
    else if (suite == FIN_RESEAU) demarrer_reseau();
}

static void interpoler() {  // toutes les 20 ms en RESEAU et REBOURS : pas de saut de 3 degres tous les 100 ms
    float s = min(1.0f, (millis() - t_pas) / (float)PERIODE_RESEAU_MS);
    for (int i = 0; i < N_BRAS; i++) bras_poser(i, prec[i] + (suiv[i] - prec[i]) * s);
}

static void tout_arreter(const char *m) {
    arreter_conduite();
    if (mode != LIBRE) { mode = LIBRE; au_home = false; }  // bras fige sur place
    mission = false;
    signaler(m);
}

// ======================================================================== COMMANDES
char trame = 0;
int valeur = 0, nb_chiffres = 0;
unsigned long debut_trame = 0, dernier_refus = 0;

static bool conduite_refusee() {
    if (!mission) return false;
    if (millis() - dernier_refus > 1000) { dernier_refus = millis(); signaler("Mission en cours : S pour tout arreter, H pour ramener le bras"); }
    return true;
}
static void commande_direction(int n) {
    if (conduite_refusee()) return;
    suivi = SUIVI_OFF;
    direction_ecrire(n);
}

static void traiter_octet(char c) {
    c = toupper(c);
    if (c == '\r' || c == '\n' || c == ' ') return;
    if (c >= '0' && c <= '9') {
        if (trame && nb_chiffres < 4) { valeur = valeur * 10 + (c - '0'); nb_chiffres++; debut_trame = millis(); }
        else if (c == '1') lancer_mission();
        return;
    }
    char en_cours = trame;
    int n = nb_chiffres > 0 ? valeur : -1;
    trame = 0;
    if (en_cours == 'O' && n >= 0) commande_direction(n);
    switch (c) {
        case 'S': tout_arreter("S arret"); break;
        case 'N': if (!conduite_refusee()) lancer_suivi(); break;
        case 'H': aller_home(); break;
        case 'V': case 'O': trame = c; valeur = nb_chiffres = 0; debut_trame = millis(); break;
        case 'A': case 'B':
            if (en_cours == 'V' && n >= 0 && !conduite_refusee()) { suivi = SUIVI_OFF; moteur_commande(n, c); }
            break;
        default: break;  // L (connexion) et le reste : ignores
    }
}

static void etat_complet() {
    int b[3];
    lire_capteurs(b);
    Serial.printf("P %lu ms, mode %d, mission %d, home %d, suivi %d, capteurs %d %d %d\n", millis(), (int)mode, mission, au_home,
                  (int)suivi, b[0], b[1], b[2]);
    servos_diagnostic(Serial);
}

static void ligne_usb(char *l) {
    int i;
    float d;
    if (l[0] == 'J' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i < N_BRAS) {
        tout_arreter("J : bras fige");
        au_home = false;  // bouge a la main : H doit pouvoir le ramener
        bras_poser(i, servo_q(i, servo_degres(i, bras_consigne(i)) + d));
        Serial.printf("J %s : %.1f degres servo\n", NOM_SERVO[i], servo_degres(i, bras_consigne(i)));
    } else if (l[0] == 'D' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i < N_BRAS) {
        tout_arreter("D : bras fige");   // essai : active le servo i et l'envoie a d degres servo (10 a 170)
        au_home = false;
        ACTIF[i] = true;
        bras_poser(i, servo_q(i, constrain(d, ANGLE_MIN, ANGLE_MAX)));
        Serial.printf("D %s actif : %.1f degres servo\n", NOM_SERVO[i], servo_degres(i, bras_consigne(i)));
    } else if (l[0] == 'T') {
        float a, b2, c, e2;
        float h2 = kh;
        int nb = sscanf(l + 1, "%f %f %f %f %f", &a, &b2, &c, &e2, &h2);
        if (nb >= 4) { kp = constrain(a, 0, 100); kd = constrain(b2, 0, 150); v_droite = constrain(c, 40, 255); v_virage = constrain(e2, 40, v_droite); if (nb >= 5) kh = constrain(h2, 0, 80); }
        Serial.printf("T kp %.1f kd %.1f vitesse droite %.0f virage %.0f cap %.1f\n", kp, kd, v_droite, v_virage, kh);
    } else if (l[0] == 'C' && !l[1]) {
        tout_arreter("C : bras vertical et droit (calage)");
        vider_file();
        ajouter(Q_VERTICAL, 3000);
        lancer_trajet(FIN_RIEN);
    } else if (l[0] == 'X' && !l[1]) {
        tout_arreter("X : bras relache");
        bras_relacher();
    } else if (l[0] == 'P' && !l[1]) etat_complet();
    else if (l[0] == 'L' && !l[1]) { int b[3]; lire_capteurs(b); lire_rangee(AR); Serial.printf("L avant %d %d %d arriere %d %d %d\n", b[0], b[1], b[2], AR.b[0], AR.b[1], AR.b[2]); }
    else for (char *p = l; *p; p++) traiter_octet(*p);
}

static void lire_usb() {
    static char l[64];
    static int n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { l[n] = 0; n = 0; if (l[0]) ligne_usb(l); }
        else if (n < 63) l[n++] = (char)toupper(c);
    }
}

// ======================================================================== DEMARRAGE ET BOUCLE
void setup() {
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    Serial.begin(115200);
    servos_demarrer();
    moteur.attachPin(PIN_ENA, FREQ_MOTEUR_HZ, 8);
    moteur.write(0);
    for (int j = 0; j < 4; j++) {
        Q_DEPART_RESEAU[j] = Q_REPOS[j];
        bas_reseau[j] = servo_borne(j, -10.0f);
        haut_reseau[j] = servo_borne(j, 10.0f);
    }
    Q_DEPART_RESEAU[PINCE] = SERVO_OUVERT;
    const int ordre[N_BRAS] = {PINCE, POIGNET, COUDE, EPAULE, BASE};  // un servo toutes les 0,4 s, droit en Home
    for (int k = 0; k < N_BRAS; k++) {
        bras_poser(ordre[k], Q_HOME[ordre[k]]);
        if (ACTIF[ordre[k]]) delay(400);
    }
    au_home = true;
    SerialBT.begin("Bras_ESP32_Robot");
    Serial.println("Robot pret : O, V..A/B, S, H, N, 1 (USB : J, C, X, L, P)");
}

void loop() {
    while (SerialBT.available() > 0) traiter_octet((char)SerialBT.read());
    lire_usb();
    if (trame == 'O' && nb_chiffres > 0 && millis() - debut_trame > 30) { commande_direction(valeur); trame = 0; }
    if (trame && millis() - debut_trame > 200) trame = 0;

    static bool connecte_avant = false;
    bool connecte = SerialBT.hasClient();
    if (connecte_avant && !connecte) tout_arreter("Bluetooth perdu : arret");
    connecte_avant = connecte;

    unsigned long t = millis();
    if ((mode == RESEAU || mode == REBOURS) && (long)(t - prochain_reseau) >= 0) {
        prochain_reseau = t + PERIODE_RESEAU_MS;
        if (mode == RESEAU) pas_reseau(); else pas_rebours();
    }
    if (mode == DEPOT && t - debut_depot >= PAUSE_DEPOT_MS) {  // la balle est tombee : retour direct (sur, verifie)
        histo_valide = false;
        vider_file();
        ajouter(Q_DEPART_RESEAU, 2500);
        ajouter(Q_HOME, 2500);
        lancer_trajet(FIN_HOME);
    }
    if (mode != LIBRE && (long)(t - prochain_bras) >= 0) {
        prochain_bras = t + PERIODE_BRAS_MS;
        if (mode == TRAJET) pas_trajet();
        else if (mode == RESEAU || mode == REBOURS) interpoler();
    }
    if (suivi != SUIVI_OFF && (long)(t - prochain_suivi) >= 0) { prochain_suivi = t + PERIODE_SUIVI_MS; pas_suivi(); }
}
