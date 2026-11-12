// Robot : programme simple (12/11 soir), sans reseau de neurones. Le programme complet reste dans bras_robot/.
// Les servos du bras sont commandes en DEGRES SERVO directement (les palonniers ne sont pas encore recales),
// toujours entre ANGLE_MIN et ANGLE_MAX pour ne jamais forcer un servo en butee (un servo bloque chauffe et grille).
//
// BLUETOOTH ("Bras_ESP32_Robot", application MIT App Inventor) - aussi accepte au moniteur serie USB (115 200 bauds)
//   L        envoye a la connexion : ignore
//   O<n>     direction : angle du servo 0 a 180 (curseur), 90 = tout droit
//   V<n>A    avance a la vitesse n (0 a 255) ; V<n>B recule ; V0A / V0B coupe le moteur
//   S        arret : moteur coupe, direction droite, suivi de ligne arrete, bras arrete sur place
//   N        suivi de ligne (3 capteurs avant) : attend la ligne, puis avance et la suit sans s'arreter
//   1 (ou H) sequence du bras (un servo apres l'autre, voir SEQ) : descend vers l'avant, ferme la pince, remonte, rouvre
//   2 3      ignores pour l'instant
// USB EN PLUS (une ligne par commande) :
//   J i d    bouge le servo i (0 base, 1 epaule, 2 coude, 3 poignet, 4 pince) de d degres
//   X        relache tous les servos du bras (ils ne chauffent plus ; tenir le bras)
//   L        valeurs brutes des 3 capteurs de ligne avant
//   P        diagnostic : angle demande et impulsion mesuree sur chaque broche

#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include "driver/gpio.h"

BluetoothSerial SerialBT;

// ---- BROCHES (12/11 soir) ----
const int PIN_DIR = 17;
const int PIN_ENA = 25, PIN_IN1 = 33, PIN_IN2 = 32;
const char *NOM[5] = {"base", "epaule", "coude", "poignet", "pince"};
const int BROCHE[5] = {21, 4, 13, 22, 16};
const bool ACTIF[5] = {true, true, true, false, true};  // poignet coupe : probablement grille (tres chaud, ne bouge plus)
const int CAPT[3] = {27, 15, 34};                       // capteurs avant gauche, milieu, droite

// ---- BRAS ----
const float ANGLE_MIN = 10.0f, ANGLE_MAX = 170.0f;      // jamais au-dela : butee interne du servo = blocage = surchauffe
// Posture de depart (bras vertical au calage du 12/11 matin : base 90, epaule 54, coude 3 = droit, pince 90).
// Le coude neuf n'est pas recale (droit vers 85 au lieu de 3) : on part de 100 et on ne descend pas sous 60.
const float DEPART[5] = {90.0f, 54.0f, 100.0f, 90.0f, 90.0f};
struct Etape { int servo; float but; unsigned long ms; };
const Etape SEQ[] = {
    {0, 115.0f, 2000},   // base : tourne
    {1, 85.0f, 2500},    // epaule : penche vers l'avant
    {2, 130.0f, 2000},   // coude : replie l'avant-bras vers le bas
    {1, 112.0f, 2500},   // epaule : descend (la pince s'approche du sol)
    {4, 60.0f, 1000},    // pince : ferme
    {1, 80.0f, 2500},    // epaule : remonte
    {2, 100.0f, 2000},   // coude : tend
    {1, 54.0f, 2500},    // epaule : bras vertical
    {4, 90.0f, 1000},    // pince : rouvre
    {0, 90.0f, 2000}};   // base : retour
const int SEQ_N = sizeof(SEQ) / sizeof(SEQ[0]);

// ---- DIRECTION, MOTEUR, SUIVI ----
const int CENTRE_DIR = 90;
const uint32_t FREQ_MOTEUR_HZ = 1000;
const char SENS_AVANCE = 'A';
const bool NOIR_ELEVE = true;
const int VITESSE_SUIVI = 115;
const int SIGNE_BRAQUAGE = 1;
const float BRAQUAGE_MAX_DEG = 20.0f, VITESSE_BRAQUAGE_DEG_S = 60.0f, FILTRE_ECART = 0.2f;
const float ECART_CAPTEURS_MM = 13.0f, K_SUIVI = 4.0e-4f, EMPATTEMENT_MM = 140.0f;
const int CONTRASTE_MIN = 300;

ESP32PWM servos[5], servo_dir, moteur;
float angle[5];
bool seq_en_cours = false;
int seq_i = 0;
float seq_depart = 0;
unsigned long seq_debut = 0, prochain_seq = 0;

static void signaler(const char *m) { Serial.println(m); SerialBT.println(m); }
static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }
static void ecrire_us(ESP32PWM &s, int us) {
    if (s.attached()) s.write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}
static void poser(int i, float deg) {
    if (!ACTIF[i]) return;
    angle[i] = constrain(deg, ANGLE_MIN, ANGLE_MAX);
    if (!servos[i].attached()) servos[i].attachPin(BROCHE[i], 50.0, 16);
    ecrire_us(servos[i], vers_us(angle[i]));
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
static void direction(float a) { ecrire_us(servo_dir, vers_us(constrain(a, 0.0f, 180.0f))); }
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
    else e = (dernier_ecart >= 0 ? 1.5f : -1.5f) * ECART_CAPTEURS_MM;
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
    seq_depart = angle[SEQ[0].servo];
    seq_debut = prochain_seq = millis();
    seq_en_cours = true;
    signaler("Sequence du bras");
}

static void pas_sequence() {
    const Etape &E = SEQ[seq_i];
    unsigned long t = millis() - seq_debut;
    float s = t >= E.ms ? 1.0f : (float)t / E.ms;
    s = s * s * (3.0f - 2.0f * s);
    poser(E.servo, seq_depart + (E.but - seq_depart) * s);
    if (t < E.ms) return;
    if (++seq_i >= SEQ_N) {
        seq_en_cours = false;
        signaler("Sequence terminee");
        return;
    }
    char m[48];
    snprintf(m, sizeof(m), "etape %d : %s -> %.0f", seq_i + 1, NOM[SEQ[seq_i].servo], SEQ[seq_i].but);
    Serial.println(m);
    seq_depart = angle[SEQ[seq_i].servo];
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
        else if (c == '1' && !seq_en_cours) lancer_sequence();
        return;
    }
    char en_cours = trame;
    int n = nb_chiffres > 0 ? valeur : -1;
    trame = 0;
    if (en_cours == 'O' && n >= 0) { suivi = SUIVI_OFF; direction(n); }
    switch (c) {
        case 'S': arreter_conduite(); seq_en_cours = false; signaler("S arret"); break;
        case 'N': seq_en_cours = false; lancer_suivi(); break;
        case 'H': if (!seq_en_cours) lancer_sequence(); break;
        case 'V': case 'O': trame = c; valeur = nb_chiffres = 0; debut_trame = millis(); break;
        case 'A': case 'B':
            if (en_cours == 'V' && n >= 0) { suivi = SUIVI_OFF; moteur_commande(n, c); }
            break;
        default: break;
    }
}

static void diagnostic() {
    int b[3];
    lire_capteurs(b);
    Serial.printf("P %lu ms, sequence %d, suivi %d, capteurs %d %d %d\n", millis(), seq_en_cours, (int)suivi, b[0], b[1], b[2]);
    for (int i = 0; i < 5; i++) {
        gpio_input_enable((gpio_num_t)BROCHE[i]);
        Serial.printf("  %-8s broche %2d actif %d angle %5.1f impulsion %4lu us\n", NOM[i], BROCHE[i], servos[i].attached(), angle[i],
                      pulseIn(BROCHE[i], HIGH, 30000));
    }
}

static void ligne_usb(char *l) {
    int i; float d;
    if (l[0] == 'J' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i <= 4) {
        poser(i, angle[i] + d);
        Serial.printf("J %s : %.1f degres\n", NOM[i], angle[i]);
    } else if (l[0] == 'X' && !l[1]) {
        seq_en_cours = false;
        for (int k = 0; k < 5; k++) if (servos[k].attached()) { servos[k].detachPin(BROCHE[k]); pinMode(BROCHE[k], OUTPUT); digitalWrite(BROCHE[k], LOW); }
        Serial.println("X bras relache");
    } else if (l[0] == 'P' && !l[1]) diagnostic();
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
    for (int i = 0; i < 5; i++) {        // un servo toutes les 0,5 s, vers la posture de depart
        angle[i] = DEPART[i];
        if (!ACTIF[i]) { pinMode(BROCHE[i], OUTPUT); digitalWrite(BROCHE[i], LOW); continue; }
        poser(i, DEPART[i]);
        delay(500);
    }
    SerialBT.begin("Bras_ESP32_Robot");
    Serial.println("Robot pret (programme simple) : O, V..A/B, S, N, 1/H = sequence du bras, X = relacher");
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
