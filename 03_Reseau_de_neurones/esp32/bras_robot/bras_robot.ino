// bras_robot : PROGRAMME UNIQUE du robot (13/11/2026). Trois autres fichiers dans ce dossier :
//   servo.h / servo.cpp  broches, calage et commande des servos (a regler la, et seulement la)
//   reseau.h             reseau de neurones du bras (poids + calcul), inclus ici seulement
// Carte "ESP32 Dev Module", schema de partition "Huge APP (3MB No OTA)" : les poids du reseau font 1,1 Mo.
//
// BLUETOOTH ("Bras_ESP32_Robot", application MIT App Inventor) - aussi accepte au moniteur serie USB (115 200 bauds)
//   L        envoye a la connexion : ignore
//   O<n>     direction : angle du servo 0 a 180, 90 = tout droit (borne a 10..170)
//   V<n>A    avance a la vitesse n (0 a 255, plafonnee a PWM_MAX = 170 depuis le passage en 14 V) ; V<n>B recule ; V0A coupe
//   S        arret immediat : moteur coupe, direction droite, suivi de ligne arrete, bras fige sur place
//   H        bras en position Home : replie, centre de gravite bas, pour rouler. Revient par un chemin sur.
//   N        suivi de ligne (26/11, barycentre des 3 capteurs avant + PID) : la roue balaie pour etalonner, puis suit la ligne
//            (perdue 1,5 s = arret)
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
//   L        valeurs brutes des 3 capteurs de ligne (27 15 34) et position estimee
//   M        test de la direction, moteur coupe : la roue doit suivre une bande noire deplacee sous les capteurs (S pour finir)
//   G p i d  reglage du PID de direction : kp ki kd (et en option tau vit_servo)      T a b c d  vitesses : v_max v_min (m/s) braq_lent (deg) ecart_lent (mm)
//   K v      vitesse reelle du robot a PWM 255 (m/s) : sert a convertir les vitesses en PWM
//   P        etat et diagnostic des servos

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "servo.h"
#include "reseau.h"

BluetoothSerial SerialBT;

// ---- PROPULSION (L298N) ----
const int PIN_ENA = 25, PIN_IN1 = 33, PIN_IN2 = 32;
const uint32_t FREQ_MOTEUR_HZ = 1000;   // 5 kHz le 19/11 : le moteur ne partait plus, retour a 1 kHz
// 19/11 : le robot roule du cote du moteur de direction (la direction est a l'AVANT). true : le sens A de l'application
// et du suivi de ligne est un recul du point de vue de l'ancien robot, et la direction manuelle (O) est en miroir.
const bool INVERSER_AVANT_ARRIERE = true;
const char SENS_AVANCE = INVERSER_AVANT_ARRIERE ? 'B' : 'A';
const int CENTRE_DIR = 90;

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

// ======================================================================== CONDUITE ET SUIVI DE LIGNE (barycentre, 26/11/2026)
// 3 capteurs TCRT5000 analogiques devant la roue directrice, fixes sur le bloc moteur de direction : ils TOURNENT AVEC LA ROUE.
//   27 gauche / 15 milieu / 34 droite (dans le sens de marche, l'avant etant le cote de la direction), 13 mm d'ecart.
//   La 2e rangee (26 / 14 / 35) n'est pas branchee : elle n'est pas lue.
// Position de la ligne = BARYCENTRE des 3 capteurs : e = (-13 x n_gauche + 0 x n_milieu + 13 x n_droite) / (somme des n),
//   n = 0 sur le blanc ... 1 sur le noir, apres etalonnage. Avec 13 mm d'ecart et une ligne de 19 mm, la mesure varie
//   en continu de -13 a +13 mm (pas de zone morte au centre, contrairement a l'ancien ecart de 20 mm).
// Direction par PID (26/11) sur l'ecart e (mm, + = ligne a droite), recalcule toutes les 5 ms :
//   braquage vise (degres) = KP x e + KI x integrale de e + KD x derivee de e (derivee filtree sur 20 ms).
//   KP : tourne d'autant plus que la ligne est loin du centre. KI : corrige l'ecart qui reste dans les longs virages.
//   KD : freine la correction quand l'ecart diminue vite (amortit les oscillations).
//   L'integrale ne grossit plus quand la roue est en butee, et elle est bornee (KI x integrale <= BRAQ_MAX).
//   Puis lissage (tau_dir, vit_servo) : pas d'a-coups ; le servo recoit une consigne neuve a chaque impulsion (50 Hz).
// Vitesse (en m/s, convertie en PWM avec v255 et SEUIL_PWM) : v_max quand la roue est droite et la ligne centree,
//   descend vers v_min avec le braquage et l'ecart ; frein moteur (L298N : IN1 = IN2) quand elle doit baisser vite.
//   Ligne perdue : braquage a fond du dernier cote vu, v_perdu, arret apres 1,5 s.
// Lancement (N) : la roue balaie +-35 degres a l'arret (1,2 s) : chaque capteur voit le noir et le blanc (etalonnage).
// Simulation (geometrie SolidWorks, servo a 50 Hz) : esp32/simulation_ligne/sim_ligne2.py pid (reglages_pid.log : Kp 2,5 Ki 5,3 Kd 0,04 ;
//   a 0,6 m/s : ecart RMS 3 mm, max 7 mm, ligne jamais perdue).
const int N_CAPT = 3;
const int CAPT_TETE[N_CAPT] = {27, 15, 34};   // gauche, milieu, droite
const float ECART_CAPTEURS_MM = 13.0f;        // entre 2 capteurs voisins
const bool NOIR_ELEVE = true;                 // la ligne noire donne une valeur haute
const int CONTRASTE_MIN = 150;                // ecart noir / blanc minimal (sur 4095) pour qu'un capteur serve
const int SIGNE_BRAQUAGE = 1;                 // -1 si, au test M, la roue s'ecarte de la ligne au lieu d'aller vers elle
const float BRAQ_MAX = 45.0f;
// L298N alimente par la batterie (14 V, ~12 V au moteur) depuis le 26/11. SEUIL_PWM et v255 sont des estimations :
// mesurer le temps sur 1 m de ligne droite puis corriger avec K (et T pour les vitesses).
const int SEUIL_PWM = 35;                     // en dessous, le moteur ne tourne pas (estimation pour ~12 V)
const int PWM_MAX = 170;                      // plafond pour TOUT (suivi et commande V de l'application) : ~9 V moyens au moteur
const unsigned long PERIODE_SUIVI_MS = 5, PERTE_MAX_MS = 1500, DUREE_BALAYAGE_MS = 1200;
// Reglables en USB sans reteleverser : G kp ki kd [tau vit_servo] ; T v_max v_min braq_lent ecart_lent (m/s, m/s, deg, mm) ; K v255.
float kp = 2.5f, ki = 5.3f, kd = 0.04f;   // degres / mm, degres / (mm.s), degres.s / mm
float tau_dir = 0.015f, vit_servo = 257.0f;
float v_max = 0.50f, v_min = 0.29f, braq_lent = 63.5f, ecart_lent = 33.6f;   // vitesses en m/s
float v_perdu = 0.15f, accel = 2.5f;          // m/s et m/s2
float v255 = 1.5f;                            // vitesse (m/s) a PWM 255 : ESTIMATION pour 12 V, a mesurer puis K

// Une rangee de capteurs (declaree avant toute fonction : l'IDE Arduino place ses prototypes devant la 1re fonction)
struct Rangee {
    const int *pin;
    int brut[N_CAPT], cmin[N_CAPT], cmax[N_CAPT];
    float n[N_CAPT];      // 0 blanc ... 1 noir
    bool vu;
    float e;              // position de la ligne (mm), + = a DROITE du bloc
};
Rangee TETE = {CAPT_TETE};

ESP32PWM moteur;

static void signaler(const char *m) { Serial.println(m); SerialBT.println(m); }

enum Suivi { SUIVI_OFF, SUIVI_BALAYAGE, SUIVI_ATTENTE, SUIVI_ROULE };
Suivi suivi = SUIVI_OFF;
bool sans_moteur = false;   // test M : la roue suit la ligne, le moteur reste coupe

static void moteur_commande(int v, char sens) {
    v = constrain(v, 0, PWM_MAX);
    digitalWrite(PIN_IN1, (v > 0 && sens == 'B') ? HIGH : LOW);
    digitalWrite(PIN_IN2, (v > 0 && sens == 'A') ? HIGH : LOW);
    moteur.write(v);
}
static void moteur_freiner() { digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH); moteur.write(255); }   // L298N : arret rapide
static void arreter_conduite() { suivi = SUIVI_OFF; moteur_commande(0, SENS_AVANCE); direction_ecrire(CENTRE_DIR); }
static void ecrire_braquage(float b) { direction_ecrire(CENTRE_DIR + SIGNE_BRAQUAGE * b); }   // b > 0 : vers la droite
static int pwm_pour(float v_ms) {             // vitesse voulue (m/s) -> PWM du L298N
    if (v_ms <= 0.0f) return 0;
    return min(PWM_MAX, (int)lroundf(SEUIL_PWM + v_ms / v255 * (255 - SEUIL_PWM)));
}

static void lire_rangee(Rangee &r) {
    for (int i = 0; i < N_CAPT; i++) { int s = 0; for (int k = 0; k < 4; k++) s += analogRead(r.pin[i]); r.brut[i] = s / 4; }
}
static void etalonner_debut(Rangee &r) { lire_rangee(r); for (int i = 0; i < N_CAPT; i++) r.cmin[i] = r.cmax[i] = r.brut[i]; r.vu = false; r.e = 0; }
static void etalonner(Rangee &r) { for (int i = 0; i < N_CAPT; i++) { r.cmin[i] = min(r.cmin[i], r.brut[i]); r.cmax[i] = max(r.cmax[i], r.brut[i]); } }
static int capteurs_valides(const Rangee &r) { int k = 0; for (int i = 0; i < N_CAPT; i++) if (r.cmax[i] - r.cmin[i] >= CONTRASTE_MIN) k++; return k; }
static void estimer(Rangee &r) {
    lire_rangee(r);
    etalonner(r);                      // l'etalonnage continue en roulant
    float mx = 0, s = 0, se = 0;
    for (int i = 0; i < N_CAPT; i++) {
        int d = r.cmax[i] - r.cmin[i];
        float x = d >= CONTRASTE_MIN ? constrain((r.brut[i] - r.cmin[i]) / (float)d, 0.0f, 1.0f) : (NOIR_ELEVE ? 0.0f : 1.0f);
        r.n[i] = NOIR_ELEVE ? x : 1.0f - x;
        mx = max(mx, r.n[i]);
        s += r.n[i];
        se += r.n[i] * (i - (N_CAPT - 1) / 2.0f) * ECART_CAPTEURS_MM;   // gauche -13, milieu 0, droite +13
    }
    r.vu = mx >= 0.25f;
    if (r.vu) r.e = se / s;            // barycentre
}

// ---- etat du suivi
float braq = 0, braq_envoye = 0, braq_est = 0, braq_f = 0, v_cmd = 0, v_est = 0;
float integ = 0, e_prec = 0, deriv = 0;   // PID
int cote = 1, pertes = 0;
bool vu_prec = true, frein = false;
unsigned long t_vue = 0, t_servo = 0, t_debut = 0, dernier_us = 0, prochain_suivi = 0, prochain_aff = 0;

static void lancer_suivi(bool moteur_coupe) {
    arreter_conduite();
    sans_moteur = moteur_coupe;
    etalonner_debut(TETE);
    braq = braq_envoye = braq_est = braq_f = v_cmd = v_est = 0;
    integ = e_prec = deriv = 0;
    pertes = 0; vu_prec = true; dernier_us = 0;
    t_debut = millis();
    suivi = SUIVI_BALAYAGE;
    signaler(moteur_coupe ? "M test de la direction (moteur coupe) : etalonnage, la roue balaie" : "N suivi de ligne : etalonnage, la roue balaie");
}

// Toutes les PERIODE_SUIVI_MS.
static void pas_suivi() {
    unsigned long t = millis();
    unsigned long us = micros();
    float dt = dernier_us ? (us - dernier_us) * 1e-6f : PERIODE_SUIVI_MS * 1e-3f;
    dernier_us = us;
    if (dt <= 0 || dt > 0.05f) dt = PERIODE_SUIVI_MS * 1e-3f;

    if (suivi == SUIVI_BALAYAGE) {   // aller-retour +-35 degres : chaque capteur passe sur le noir et sur le blanc
        float s = (t - t_debut) / (float)DUREE_BALAYAGE_MS;
        lire_rangee(TETE); etalonner(TETE);
        if (s < 1.0f) { ecrire_braquage(35.0f * sinf(2.0f * (float)M_PI * s)); return; }
        ecrire_braquage(0);
        int vt = capteurs_valides(TETE);
        char m[120];
        snprintf(m, sizeof(m), "N etalonnage : %d/3 capteurs voient la ligne (noir/blanc : %d-%d %d-%d %d-%d)", vt,
                 TETE.cmin[0], TETE.cmax[0], TETE.cmin[1], TETE.cmax[1], TETE.cmin[2], TETE.cmax[2]);
        signaler(m);
        if (vt < 2) { arreter_conduite(); signaler("N pas assez de contraste : poser la roue sur la ligne et renvoyer N"); return; }
        suivi = SUIVI_ATTENTE;
        return;
    }
    estimer(TETE);
    if (suivi == SUIVI_ATTENTE) {
        if (!TETE.vu) return;
        suivi = SUIVI_ROULE; t_vue = t;
        signaler(sans_moteur ? "M ligne vue : deplacer la ligne sous les capteurs, la roue doit la suivre (S pour finir)" : "N ligne trouvee : depart");
    }
    float cible;
    if (TETE.vu) {
        t_vue = t;
        if (fabsf(TETE.e) > 4.0f) cote = TETE.e > 0 ? 1 : -1;
        if (!vu_prec) e_prec = TETE.e;                            // ligne retrouvee : pas de pic de derivee
        deriv += ((TETE.e - e_prec) / dt - deriv) * (1.0f - expf(-dt / 0.02f));
        e_prec = TETE.e;
        float pid = kp * TETE.e + ki * integ + kd * deriv;
        if (fabsf(pid) < BRAQ_MAX || pid * TETE.e < 0) integ += TETE.e * dt;   // pas d'integration en butee
        if (ki > 0) integ = constrain(integ, -BRAQ_MAX / ki, BRAQ_MAX / ki);
        cible = kp * TETE.e + ki * integ + kd * deriv;
    } else {
        cible = BRAQ_MAX * cote;   // la ligne est sortie par un cote : braquage a fond de ce cote
        if (!sans_moteur && t - t_vue > PERTE_MAX_MS) { arreter_conduite(); signaler("N ligne perdue : arret (renvoyer N)"); return; }
        if (sans_moteur) cible = braq;  // test : on attend que la ligne revienne
    }
    if (!TETE.vu && vu_prec) pertes++;
    vu_prec = TETE.vu;
    cible = constrain(cible, -BRAQ_MAX, BRAQ_MAX);
    float d = (cible - braq) * (1.0f - expf(-dt / tau_dir));
    braq += constrain(d, -vit_servo * dt, vit_servo * dt);
    if (t - t_servo >= 20) { t_servo = t; braq_envoye = braq; ecrire_braquage(braq_envoye); }   // une consigne par impulsion (50 Hz)
    // position estimee du servo (retard ~50 ms, 350 degres/s au plus) : c'est la que sont les capteurs
    float de = (braq_envoye - braq_est) * (1.0f - expf(-dt / 0.05f));
    braq_est += constrain(de, -350.0f * dt, 350.0f * dt);
    braq_f += (braq_est - braq_f) * (1.0f - expf(-dt / 0.08f));

    // vitesse variable (m/s)
    float vc;
    if (TETE.vu) {
        float gene = fabsf(braq_f) / braq_lent + fabsf(TETE.e) / ecart_lent + 0.3f;
        vc = v_max - (v_max - v_min) * min(1.0f, gene);
    } else vc = v_perdu;
    if (vc > v_cmd) v_cmd = min(vc, v_cmd + accel * dt); else v_cmd = vc;
    frein = v_est > v_cmd + 0.08f;
    v_est += (v_cmd - v_est) * (1.0f - expf(-dt / (frein ? 0.05f : 0.15f)));
    if (sans_moteur) { v_est = 0; frein = false; moteur_commande(0, SENS_AVANCE); }
    else if (frein) moteur_freiner();
    else moteur_commande(pwm_pour(v_cmd), SENS_AVANCE);

    static unsigned long dernier_pas = 0, pire = 0;
    if (dernier_pas) pire = max(pire, t - dernier_pas);
    dernier_pas = t;
    if ((long)(t - prochain_aff) >= 0) {
        prochain_aff = t + 200;
        Serial.printf("N ligne %+5.1f mm%s brq %+5.1f (P %+5.1f I %+5.1f D %+5.1f) v %.2f m/s (PWM %d)%s | capteurs %d %d %d | pertes %d | pas max %lu ms\n",
                      TETE.e, TETE.vu ? "" : "(-)", braq, kp * TETE.e, ki * integ, kd * deriv,
                      v_cmd, sans_moteur ? 0 : pwm_pour(v_cmd), frein ? " FREIN" : "",
                      TETE.brut[0], TETE.brut[1], TETE.brut[2], pertes, pire);
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
    direction_ecrire(INVERSER_AVANT_ARRIERE ? 180 - n : n);
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
        case 'N': if (!conduite_refusee()) lancer_suivi(false); break;
        case 'H': aller_home(); break;
        case 'V': case 'O': trame = c; valeur = nb_chiffres = 0; debut_trame = millis(); break;
        case 'A': case 'B':
            if (en_cours == 'V' && n >= 0 && !conduite_refusee()) { suivi = SUIVI_OFF; moteur_commande(n, INVERSER_AVANT_ARRIERE ? (c == 'A' ? 'B' : 'A') : c); }
            break;
        default: break;  // L (connexion) et le reste : ignores
    }
}

static void etat_complet() {
    lire_rangee(TETE);
    Serial.printf("P %lu ms, mode %d, mission %d, home %d, suivi %d, capteurs de tete %d %d %d\n", millis(), (int)mode, mission, au_home,
                  (int)suivi, TETE.brut[0], TETE.brut[1], TETE.brut[2]);
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
    } else if (l[0] == 'G') {
        sscanf(l + 1, "%f %f %f %f %f", &kp, &ki, &kd, &tau_dir, &vit_servo);
        kp = constrain(kp, 0.0f, 20.0f); ki = constrain(ki, 0.0f, 100.0f); kd = constrain(kd, 0.0f, 2.0f);
        tau_dir = constrain(tau_dir, 0.005f, 0.5f); vit_servo = constrain(vit_servo, 50.0f, 800.0f);
        integ = 0;
        Serial.printf("G kp %.2f ki %.2f kd %.3f tau %.3f s vit_servo %.0f deg/s\n", kp, ki, kd, tau_dir, vit_servo);
    } else if (l[0] == 'T') {
        sscanf(l + 1, "%f %f %f %f", &v_max, &v_min, &braq_lent, &ecart_lent);
        v_max = constrain(v_max, 0.05f, 2.0f); v_min = constrain(v_min, 0.05f, v_max);
        braq_lent = constrain(braq_lent, 2.0f, 90.0f); ecart_lent = constrain(ecart_lent, 2.0f, 90.0f);
        Serial.printf("T v_max %.2f m/s (PWM %d) v_min %.2f m/s (PWM %d) braq_lent %.0f ecart_lent %.0f\n", v_max, pwm_pour(v_max), v_min, pwm_pour(v_min), braq_lent, ecart_lent);
    } else if (l[0] == 'K') {
        float a = v255; sscanf(l + 1, "%f", &a); v255 = constrain(a, 0.1f, 3.0f);
        Serial.printf("K vitesse a PWM 255 : %.2f m/s -> v_max = PWM %d, v_min = PWM %d\n", v255, pwm_pour(v_max), pwm_pour(v_min));
    } else if (l[0] == 'M' && !l[1]) {
        if (!conduite_refusee()) lancer_suivi(true);
    } else if (l[0] == 'C' && !l[1]) {
        tout_arreter("C : bras vertical et droit (calage)");
        vider_file();
        ajouter(Q_VERTICAL, 3000);
        lancer_trajet(FIN_RIEN);
    } else if (l[0] == 'X' && !l[1]) {
        tout_arreter("X : bras relache");
        bras_relacher();
    } else if (l[0] == 'P' && !l[1]) etat_complet();
    else if (l[0] == 'L' && !l[1]) {
        if (suivi == SUIVI_OFF) lire_rangee(TETE);
        Serial.printf("L capteurs gauche 27 : %d | milieu 15 : %d | droite 34 : %d", TETE.brut[0], TETE.brut[1], TETE.brut[2]);
        if (suivi != SUIVI_OFF) Serial.printf(" | ligne %+.1f mm%s", TETE.e, TETE.vu ? "" : " (pas vue)");
        Serial.println();
    }
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
    Serial.println("Robot pret : O, V..A/B, S, H, N, 1 (USB : J, D, C, X, L, M, G, T, K, P)");
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
