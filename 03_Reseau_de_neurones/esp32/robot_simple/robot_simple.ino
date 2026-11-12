// Robot : programme simple, reparti de zero le 12/11 (sans reseau de neurones). L'ancien programme complet reste dans
// bras_robot/ (reseau, sequences 1 2 3).
//
// BLUETOOTH ("Bras_ESP32_Robot", application MIT App Inventor) - aussi accepte au moniteur serie USB (115 200 bauds)
//   L        envoye a la connexion : ignore
//   O<n>     direction : angle du servo 0 a 180 (curseur), 90 = tout droit
//   V<n>A    avance a la vitesse n (0 a 255) ; V<n>B recule ; V0A / V0B coupe le moteur
//   S        arret : moteur coupe, direction droite, suivi de ligne arrete, bras arrete sur place
//   N        suivi de ligne (3 capteurs avant) : attend la ligne, puis avance et la suit sans s'arreter
//   H        sequence du bras (un servo apres l'autre, voir SEQ) puis retour au repos
// USB EN PLUS (une ligne par commande) :
//   Q        sequence du bras (comme H)
//   J i d    bouge le servo i (0 base, 1 epaule, 2 coude, 3 poignet, 4 pince) de d degres
//   L        valeurs brutes des 3 capteurs de ligne avant
//   P        diagnostic : impulsions mesurees sur chaque broche
//
// Arduino IDE : carte "ESP32 Dev Module", bibliotheque ESP32Servo. Servos sur un abaisseur 5-6 V, masses reliees.

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "driver/gpio.h"

BluetoothSerial SerialBT;

// ---- BROCHES (12/11 soir) ----
const int PIN_DIR = 17;
const int PIN_ENA = 25, PIN_IN1 = 33, PIN_IN2 = 32;
const char *NOM[5] = {"base", "epaule", "coude", "poignet", "pince"};
const int BROCHE[5] = {21, 4, 13, 22, 16};
const bool ACTIF[5] = {true, true, true, false, true};  // poignet coupe : il ne bouge plus (servo a verifier)
const int CAPT[3] = {27, 15, 34};                       // capteurs avant gauche, milieu, droite

// ---- DIRECTION ET MOTEUR ----
const int CENTRE_DIR = 90;
const uint32_t FREQ_MOTEUR_HZ = 1000;
const char SENS_AVANCE = 'A';          // l'application avance avec A (balle vers le haut)

// ---- SUIVI DE LIGNE ----
const bool NOIR_ELEVE = true;          // TCRT5000 : valeur haute sur le noir
const int VITESSE_SUIVI = 115;
const int SIGNE_BRAQUAGE = 1;          // -1 si le robot tourne du mauvais cote
const float BRAQUAGE_MAX_DEG = 20.0f, VITESSE_BRAQUAGE_DEG_S = 60.0f, FILTRE_ECART = 0.2f;
const float ECART_CAPTEURS_MM = 13.0f, K_SUIVI = 4.0e-4f, EMPATTEMENT_MM = 140.0f;
const int CONTRASTE_MIN = 300;

// ---- BRAS : conversion angle articulaire (rad) -> degre servo, palonniers cales le 12/11 (bras vertical a q = 0) ----
const float BUTEE_BAS[4] = {-1.5708f, -0.95f, -0.05f, -1.2f};
const float BUTEE_HAUT[4] = {1.5708f, 2.1916f, 3.0916f, 1.9416f};
const int SENS[4] = {1, 1, 1, -1};
const float DECALAGE_DEG[4] = {0, 0, 0, -42.5f};
const float Q_REPOS[4] = {-0.0002f, -0.1895f, 2.2860f, 1.1704f};  // home : bras replie au-dessus du robot
const float PINCE_OUVERTE = 104.3f, PINCE_FERMEE = 82.6f;        // degres servo

// Sequence : un seul servo bouge a la fois (moins de courant). Verifiee (sequence_simple.py) : pince jamais sous 10 cm.
struct Etape { int servo; float but; unsigned long ms; };   // servo 4 : but en degres servo ; sinon q en rad
const Etape SEQ[] = {
    {0, 0.50f, 2000},     // base : tourne vers la gauche
    {1, 0.60f, 2000},     // epaule : penche en avant
    {2, 1.60f, 2000},     // coude : ouvre l'avant-bras
    {1, 1.00f, 2000},     // epaule : descend (pince a ~11 cm du sol)
    {4, PINCE_FERMEE, 1000},
    {1, 0.50f, 2000},     // epaule : remonte
    {2, 1.00f, 2000},     // coude : tend le bras
    {1, -0.10f, 2000},    // epaule : bras vertical, pince en haut (~48 cm)
    {4, PINCE_OUVERTE, 1000},
    {1, -0.19f, 1000},    // retour au repos
    {2, 2.286f, 2000},
    {0, 0.0f, 2000}};
const int SEQ_N = sizeof(SEQ) / sizeof(SEQ[0]);

ESP32PWM servos[5], servo_dir, moteur;
float q[4], pince_deg = PINCE_OUVERTE;
bool seq_en_cours = false;
int seq_i = 0;
float seq_depart = 0;
unsigned long seq_debut = 0, prochain_seq = 0;

static void signaler(const char *m) { Serial.println(m); SerialBT.println(m); }
static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }
static void ecrire_us(ESP32PWM &s, int us) {
    if (s.attached()) s.write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}
static float vers_degres(int i, float qi) {
    float d = (qi - BUTEE_BAS[i]) / (BUTEE_HAUT[i] - BUTEE_BAS[i]) * 180.0f;
    if (SENS[i] < 0) d = 180.0f - d;
    d += DECALAGE_DEG[i];
    return d < 0 ? 0 : (d > 180 ? 180 : d);
}
static void commander_bras() {
    for (int i = 0; i < 4; i++) if (ACTIF[i]) ecrire_us(servos[i], vers_us(vers_degres(i, q[i])));
    if (ACTIF[4]) ecrire_us(servos[4], vers_us(pince_deg));
}

// ---- CONDUITE ----
enum Suivi { SUIVI_OFF, SUIVI_ATTENTE, SUIVI_ROULE };
Suivi suivi = SUIVI_OFF;
int cmin[3], cmax[3];
float dernier_ecart = 0, ecart_filtre = 0, braquage = 0;
unsigned long prochain_suivi = 0, prochain_aff = 0;

static void moteur_commande(int v, char sens) {
    v = constrain(v, 0, 255);
    digitalWrite(PIN_IN1, (v > 0 && sens == 'B') ? HIGH : LOW);
    digitalWrite(PIN_IN2, (v > 0 && sens == 'A') ? HIGH : LOW);
    moteur.write(v);
}
static void direction(float angle) { ecrire_us(servo_dir, vers_us(constrain(angle, 0.0f, 180.0f))); }
static void arreter_conduite() { suivi = SUIVI_OFF; moteur_commande(0, 'A'); direction(CENTRE_DIR); }

static void lire_capteurs(int b[3]) { for (int i = 0; i < 3; i++) b[i] = (analogRead(CAPT[i]) + analogRead(CAPT[i])) / 2; }

static void lancer_suivi() {
    arreter_conduite();
    int b[3];
    lire_capteurs(b);
    for (int i = 0; i < 3; i++) cmin[i] = cmax[i] = b[i];
    dernier_ecart = ecart_filtre = braquage = 0;
    suivi = SUIVI_ATTENTE;
    signaler("N suivi de ligne : attente de la ligne");
}

static void pas_suivi() {
    int b[3];
    float n[3];
    lire_capteurs(b);
    int gmin = 4095, gmax = 0;
    for (int i = 0; i < 3; i++) {
        cmin[i] = min(cmin[i], b[i]); cmax[i] = max(cmax[i], b[i]);
        gmin = min(gmin, cmin[i]); gmax = max(gmax, cmax[i]);
    }
    bool vu = false;
    float e = 0;
    if (gmax - gmin >= CONTRASTE_MIN) {
        for (int i = 0; i < 3; i++) {
            bool propre = cmax[i] - cmin[i] >= CONTRASTE_MIN;
            float bas = propre ? cmin[i] : gmin, haut = propre ? cmax[i] : gmax;
            float x = constrain((b[i] - bas) / (haut - bas), 0.0f, 1.0f);
            n[i] = NOIR_ELEVE ? x : 1.0f - x;
        }
        float t = n[0] + n[1] + n[2];
        if (t >= 0.5f) { vu = true; e = ECART_CAPTEURS_MM * (n[2] - n[0]) / t; }
    }
    if (suivi == SUIVI_ATTENTE) {
        if (!vu) return;
        suivi = SUIVI_ROULE;
        signaler("N ligne trouvee : on avance");
    }
    if (vu) dernier_ecart = e;
    else e = (dernier_ecart >= 0 ? 1.5f : -1.5f) * ECART_CAPTEURS_MM;  // perdue : braquer du cote ou on l'a vue
    ecart_filtre += FILTRE_ECART * (e - ecart_filtre);
    float but = constrain(atanf(-K_SUIVI * ecart_filtre * EMPATTEMENT_MM) * 57.2958f, -BRAQUAGE_MAX_DEG, BRAQUAGE_MAX_DEG);
    float pas = VITESSE_BRAQUAGE_DEG_S * 0.010f;
    braquage += constrain(but - braquage, -pas, pas);
    direction(CENTRE_DIR + SIGNE_BRAQUAGE * braquage);
    moteur_commande(VITESSE_SUIVI, SENS_AVANCE);
    if ((long)(millis() - prochain_aff) >= 0) {
        prochain_aff = millis() + 200;
        Serial.printf("N ecart %.1f mm (vu %d) braquage %.1f\n", e, vu, braquage);
    }
}

// ---- SEQUENCE DU BRAS ----
static void lancer_sequence() {
    arreter_conduite();
    seq_i = 0;
    seq_depart = SEQ[0].servo == 4 ? pince_deg : q[SEQ[0].servo];
    seq_debut = prochain_seq = millis();
    seq_en_cours = true;
    signaler("Sequence du bras");
}

static void pas_sequence() {
    const Etape &E = SEQ[seq_i];
    unsigned long t = millis() - seq_debut;
    float s = t >= E.ms ? 1.0f : (float)t / E.ms;
    s = s * s * (3.0f - 2.0f * s);
    float v = seq_depart + (E.but - seq_depart) * s;
    if (E.servo == 4) pince_deg = v; else q[E.servo] = v;
    commander_bras();
    if (t < E.ms) return;
    if (++seq_i >= SEQ_N) {
        seq_en_cours = false;
        signaler("Sequence terminee");
        return;
    }
    char m[48];
    snprintf(m, sizeof(m), "etape %d : %s", seq_i + 1, NOM[SEQ[seq_i].servo]);
    Serial.println(m);
    seq_depart = SEQ[seq_i].servo == 4 ? pince_deg : q[SEQ[seq_i].servo];
    seq_debut = millis();
}

// ---- COMMANDES ----
char trame = 0;
int valeur = 0, nb_chiffres = 0;
unsigned long debut_trame = 0;

static void traiter_octet(char c) {
    c = toupper(c);
    if (c == '\r' || c == '\n' || c == ' ') return;
    if (c >= '0' && c <= '9') {
        if (trame && nb_chiffres < 4) { valeur = valeur * 10 + (c - '0'); nb_chiffres++; debut_trame = millis(); }
        return;
    }
    char en_cours = trame;
    int n = nb_chiffres > 0 ? valeur : -1;
    trame = 0;
    if (en_cours == 'O' && n >= 0) { suivi = SUIVI_OFF; direction(n); }
    switch (c) {
        case 'S': arreter_conduite(); seq_en_cours = false; signaler("S arret"); break;
        case 'N': seq_en_cours = false; lancer_suivi(); break;
        case 'H': lancer_sequence(); break;
        case 'V': case 'O': trame = c; valeur = nb_chiffres = 0; debut_trame = millis(); break;
        case 'A': case 'B':
            if (en_cours == 'V' && n >= 0) { suivi = SUIVI_OFF; moteur_commande(n, c); }
            break;
        default: break;  // L a la connexion, 1 2 3 : ignores
    }
}

static void diagnostic() {
    int b[3];
    lire_capteurs(b);
    Serial.printf("P %lu ms, sequence %d, suivi %d, capteurs %d %d %d\n", millis(), seq_en_cours, (int)suivi, b[0], b[1], b[2]);
    for (int i = 0; i < 5; i++) {
        gpio_input_enable((gpio_num_t)BROCHE[i]);
        Serial.printf("  %-8s broche %2d actif %d impulsion %4lu us\n", NOM[i], BROCHE[i], ACTIF[i], pulseIn(BROCHE[i], HIGH, 30000));
    }
}

static void ligne_usb(char *l) {
    int i; float d;
    if (l[0] == 'J' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i <= 4 && ACTIF[i]) {
        if (i == 4) pince_deg = constrain(pince_deg + d, 0.0f, 180.0f);
        else q[i] = constrain(q[i] + d * 0.0174533f, BUTEE_BAS[i], BUTEE_HAUT[i]);
        commander_bras();
        Serial.printf("J %s : %.1f degres servo\n", NOM[i], i == 4 ? pince_deg : vers_degres(i, q[i]));
    } else if (l[0] == 'Q' && !l[1]) lancer_sequence();
    else if (l[0] == 'P' && !l[1]) diagnostic();
    else if (l[0] == 'L' && !l[1]) { int b[3]; lire_capteurs(b); Serial.printf("L %d %d %d\n", b[0], b[1], b[2]); }
    else for (char *p = l; *p; p++) traiter_octet(*p);
}

static void lire_usb() {
    static char l[64];
    static int n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { l[n] = 0; n = 0; if (l[0]) ligne_usb(l); }
        else if (n < 63) l[n++] = c;
    }
}

void setup() {
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    Serial.begin(115200);
    for (int t = 0; t < 4; t++) ESP32PWM::allocateTimer(t);
    moteur.attachPin(PIN_ENA, FREQ_MOTEUR_HZ, 8);
    moteur.write(0);
    servo_dir.attachPin(PIN_DIR, 50.0, 16);
    direction(CENTRE_DIR);
    memcpy(q, Q_REPOS, sizeof(q));
    for (int i = 0; i < 5; i++) {        // un servo toutes les 0,5 s, vers le repos
        if (!ACTIF[i]) { pinMode(BROCHE[i], OUTPUT); digitalWrite(BROCHE[i], LOW); continue; }
        servos[i].attachPin(BROCHE[i], 50.0, 16);
        commander_bras();
        delay(500);
    }
    SerialBT.begin("Bras_ESP32_Robot");
    Serial.println("Robot pret (programme simple) : O, V..A/B, S, N, H = sequence du bras");
}

void loop() {
    while (SerialBT.available() > 0) traiter_octet((char)SerialBT.read());
    lire_usb();
    if (trame == 'O' && nb_chiffres > 0 && millis() - debut_trame > 30) { suivi = SUIVI_OFF; direction(valeur); trame = 0; }
    if (trame && millis() - debut_trame > 200) trame = 0;
    static bool connecte_avant = false;
    bool connecte = SerialBT.hasClient();
    if (connecte_avant && !connecte) { arreter_conduite(); seq_en_cours = false; Serial.println("Bluetooth perdu : arret"); }
    connecte_avant = connecte;
    if (seq_en_cours && (long)(millis() - prochain_seq) >= 0) { prochain_seq = millis() + 20; pas_sequence(); }
    if (suivi != SUIVI_OFF && (long)(millis() - prochain_suivi) >= 0) { prochain_suivi = millis() + 10; pas_suivi(); }
}
