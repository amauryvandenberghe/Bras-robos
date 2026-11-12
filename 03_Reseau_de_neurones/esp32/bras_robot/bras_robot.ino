// Robot complet sur une seule ESP32 : conduite par Bluetooth (application MIT App Inventor)
// et bras 50 cm pilote par le reseau de neurones (reseau_final.pt, poids dans reseau_poids.h).
//
// Arduino IDE : carte "ESP32 Dev Module", schema de partition "Huge APP (3MB No OTA)" (obligatoire : poids ~1,1 Mo).
// Bibliotheque : ESP32Servo. Dans ce dossier : bras_robot.ino, cerveau.cpp, cerveau.h, reseau_poids.h.
// Cablage : voir le manuel technique. L'ESP32 a sa propre alimentation, masses reliees.
//
// COMMANDES BLUETOOTH (application MIT App Inventor, nom Bluetooth "Bras_ESP32_Robot") - aussi acceptees en USB
//   L        envoye par l'application a la connexion : ignore
//   S        arret (bouton S, ou balle de conduite relachee) : propulsion coupee, direction au centre,
//            le bras s'immobilise sur place (sans se relacher) et une prise en cours est abandonnee
//   H        position home. Si le bras y est deja : rouvre seulement la pince
//   1 2 3    depuis le home : prend la balle 1, 2 ou 3 (DEMO_BALLES), la depose dans le panier (DEMO_PANIER),
//            puis revient seul en home (renvoie D au lacher, H a l'arrivee)
//   V<n>A    recule a la vitesse n (0 a 255, l'application envoie au plus 126) ; V<n>B avance ; V0A ou V0B coupe le moteur
//   O<n>     direction : angle absolu du servo, de 0 a 180 (curseur de l'application), 90 = centre (R<n> accepte aussi)
//   N        suivi de ligne : attend qu'un capteur voie la ligne noire, puis avance et la suit sans s'arreter.
//            S, V ou O l'arretent (la conduite a la main reprend). Lancer une prise (1, 2, 3) l'arrete aussi ; H non.
//   V ou O pendant une prise annule la prise.
//
// COMMANDES USB EN PLUS (115 200 bauds, une ligne par commande, fin de ligne "New Line"), positions en metres,
// repere du robot : origine sur l'axe du bras au sol, x vers l'avant, z vers le haut
//   C x y z g   point de lacher (g = 1 : grand receptacle, 0 : petit)
//   B x y z     balle visee. Le reseau demarre quand C et B sont connus
//   R           position home (seul sur la ligne)
//   J i d       reglage : bouge le servo i (0 base, 1 epaule, 2 coude, 3 poignet, 4 pince) de d degres, puis H pour revenir
//   T           test : depuis le home, la base tourne de +-30 degres puis l'epaule de 15, tous les servos tenus
//   L           valeurs brutes des 6 capteurs de ligne (0 a 4095)
//   P           diagnostic : temps depuis le demarrage, cause du dernier demarrage, impulsions mesurees sur les broches
//
// MESSAGES RENVOYES
//   E q1 q2 q3 q4 pince (us)  un pas du reseau et son temps de calcul (USB seulement)
//   D                         balle deposee (pince rouverte en hauteur apres une prise)
//   H                         bras arrive en home

#include <ESP32Servo.h>
#include "cerveau.h"
#include "reseau_poids.h"
#include "BluetoothSerial.h"
#include "driver/gpio.h"
#include "esp_system.h"

BluetoothSerial SerialBT;

// ---- BROCHES (cablage refait le 11/11, remplace le manuel technique) ----
const int PIN_DIR = 17;
const int PIN_ENA = 25;
const int PIN_IN1 = 33;
const int PIN_IN2 = 32;
//                     base, epaule, coude, poignet, pince (sorties seulement : pas 34, 35, 36, 39)
const int BROCHE[5] = {19, 18, 23, 22, 16};

// ---- A REGLER SUR LE VRAI ROBOT ----
// Chaque servo couvre la fenetre [BUTEE_BAS, BUTEE_HAUT] de son axe sur 180 degres.
// Home sans correction : 90 ; 43,6 ; 133,8 ; 135,8 ; pince 104,3 degres.
const int SENS[5] = {1, 1, 1, 1, 1};            // -1 si le servo est monte a l'envers
const float DECALAGE_DEG[5] = {0, 0, 0, 0, 0};  // corrige le calage d'un palonnier
const float PINCE_CENTRE = 0.0f;                // angle du pignon (rad) quand le servo de pince est a 90 degres
const int CENTRE_DIR = 90;                      // roues droites
const int DIR_MIN_DEG = 0, DIR_MAX_DEG = 180;   // a reduire si la direction arrive en butee mecanique

// ---- CONSTANTES ----
const unsigned long PERIODE_MS = 100;           // 10 decisions par seconde, comme a l'entrainement
const unsigned long ECART_ATTACHE_MS = 500;     // un servo apres l'autre, pour limiter l'appel de courant
const unsigned long PAUSE_DEPOT_MS = 500;       // la balle tombe dans le receptacle
const unsigned long DELAI_TRAME_MS = 200;       // une trame V incomplete est oubliee apres ce delai
// ---- SUIVI DE LIGNE (commande N) : 6 capteurs TCRT5000 analogiques, ecart 13 mm (voir support_capteurs.py) ----
// Gauche / droite vus depuis l'arriere du robot, en regardant vers l'avant (la direction est a l'arriere).
const int CAPT_AV[3] = {27, 15, 34};            // avant gauche, avant milieu, avant droite
const int CAPT_AR[3] = {26, 14, 35};            // arriere gauche, arriere milieu, arriere droite
const bool NOIR_ELEVE = true;                   // TCRT5000 : valeur haute sur le noir. A inverser si le suivi fuit la ligne
const int VITESSE_SUIVI = 115;                  // 0 a 255, a regler sur la piste (12/11 : 130 trop vite, on commence plus doucement)
const char SENS_AVANCE = 'A';                   // essai du 12/11 : avec 'B' le robot partait en arriere (l'application avance avec A)
const int SIGNE_BRAQUAGE = 1;                   // -1 si le robot tourne du mauvais cote
const float BRAQUAGE_MAX_DEG = 20.0f;           // angle maxi du servo de direction autour de 90 (12/11 : +-30 = trop d'a-coups)
const float VITESSE_BRAQUAGE_DEG_S = 60.0f;     // la direction tourne au plus de 60 degres par seconde (0,6 degre tous les 10 ms)
const float FILTRE_ECART = 0.2f;                // lissage de l'ecart mesure (0 a 1 : plus petit = plus doux)
const float ECART_CAPTEURS_MM = 13.0f;
const float K_SUIVI = 4.0e-4f;                  // courbure (1/mm) par mm d'ecart ; simulation 1,3e-3, divise par 3 le 12/11 (a-coups)
const float EMPATTEMENT_MM = 140.0f;
const int CONTRASTE_MIN = 300;                  // ecart noir / blanc minimal (sur 4095) pour croire a une ligne
const unsigned long PERIODE_SUIVI_MS = 10;
enum Suivi { SUIVI_OFF, SUIVI_ATTENTE, SUIVI_ROULE };
Suivi suivi = SUIVI_OFF;
int capt_min[6], capt_max[6];
float dernier_ecart = 0, ecart_filtre = 0, braquage_actuel = 0;
unsigned long prochain_suivi = 0, prochain_affichage = 0;
const unsigned long FIN_ANGLE_MS = 30;          // R<n> n'a pas de lettre de fin : applique apres ce silence
const uint32_t FREQ_MOTEUR_HZ = 1000;
const float Z_BALLE = 0.0148f;                  // centre d'une balle posee sur la reglette
const float Z_DEPOT_MINI = 0.12f;               // rouvrir plus haut que ca = lacher dans un receptacle
const float R_RENTRE = 0.10f, Z_RENTRE = 0.31f; // passage interieur bas : pince a < 10 cm de l'axe, < 31 cm
const int ORDRE_ATTACHE[5] = {4, 3, 2, 1, 0};   // pince, poignet, coude, epaule, puis base
const float R_BALLE_TENUE = 0.02f;              // pince fermee a moins de 2 cm de la balle = balle consideree tenue

// Positions de la demo, en metres (x vers l'avant, y vers la gauche, depuis l'axe du bras). Tirees au hasard puis
// retenues apres 48 essais reussis sur 48 au simulateur, avec jusqu'a 3 mm d'erreur de pose.
const float DEMO_BALLES[3][2] = {{0.2045f, 0.1602f}, {0.2327f, 0.0547f}, {0.2600f, -0.0500f}};  // balle 3 : 12/12 (11/11)
const float DEMO_PANIER[3] = {0.1204f, -0.2492f, 0.18f};  // z = hauteur de lacher apprise pour un petit receptacle

// Memoire des consignes du reseau : le retour au repos refait ce chemin a l'envers, car un retour direct
// heurte le bord du grand receptacle une fois sur quatre (essai_retour_repos.py).
const int N_HISTO = 300;                        // 30 s de consignes
float histo[N_HISTO][4];
int n_histo = 0, debut_histo = 0;

enum Mode { LIBRE, ATTACHE, RESEAU, PAUSE_DEPOT, REBOURS, REPLI_BRAS, REPLI_BASE, TEST_BRAS };
int test_pas = 0;               // avancement du test de la base et de l'epaule (bouton 3)
Mode mode = LIBRE;
bool au_home = false;           // bras alimente et dans la posture de repos
bool mission_attente = false;   // 1, 2 ou 3 demande avant le premier home : le reseau part apres l'attache
unsigned long prochain = 0;

ESP32PWM servos[5];
ESP32PWM servo_dir;
ESP32PWM moteur;  // ESP32PWM garde chaque frequence sur son timer, contrairement a analogWrite

EtatBras etat;
float balle[3] = {0, 0, 0}, cible[3] = {0, 0, 0};
int grand = 0, balle_vue = 0, cible_connue = 0, serree = 0;
bool balle_tenue = false;  // estimation, faute de capteur : voir pas_reseau()

char trame = 0;   // 'V' ou 'R' quand une commande est en cours de lecture
int valeur = 0, nb_chiffres = 0;
unsigned long debut_trame = 0;
bool client_connecte = false;

static void signaler(const char *message) {
    Serial.println(message);
    SerialBT.println(message);
}

// ---- BRAS : conversions et commande ----
static float vers_degres(int i, float q) {
    float deg;
    if (i < 4) deg = (q - BUTEE_BAS[i]) / (BUTEE_HAUT[i] - BUTEE_BAS[i]) * 180.0f;
    else deg = 90.0f + (q - PINCE_CENTRE) * 180.0f / (float)M_PI;
    if (SENS[i] < 0) deg = 180.0f - deg;
    deg += DECALAGE_DEG[i];
    return deg < 0 ? 0 : (deg > 180 ? 180 : deg);
}

static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }

// Servo de la bibliotheque : 10 bits par defaut (pas de 1,8 degre), et Servo::setTimerWidth() detache puis rattache
// le canal, ce qui bloque l'ESP32 (plantage constate). On pilote donc le PWM directement, en 16 bits des l'attache :
// pas de 0,03 degre. Tous les servos partagent ainsi le meme timer a 50 Hz.
static void attacher_servo(ESP32PWM &servo, int broche) { servo.attachPin(broche, 50.0, 16); }

static void ecrire_us(ESP32PWM &servo, int us) {
    if (servo.attached()) servo.write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}

static void commander() {  // un servo pas encore attache ignore la commande
    for (int i = 0; i < 4; i++) ecrire_us(servos[i], vers_us(vers_degres(i, etat.q[i])));
    ecrire_us(servos[4], vers_us(vers_degres(4, etat.servo)));
}

static void memoriser() {
    int i = (debut_histo + n_histo) % N_HISTO;
    memcpy(histo[i], etat.q, sizeof(etat.q));
    if (n_histo < N_HISTO) n_histo++;
    else debut_histo = (debut_histo + 1) % N_HISTO;
}

static bool bras_attache() {
    for (int i = 0; i < 5; i++) if (!servos[i].attached()) return false;
    return true;
}

// ---- CONDUITE ----
static void commander_moteur(int vitesse, char sens) {
    vitesse = constrain(vitesse, 0, 255);
    digitalWrite(PIN_IN1, (vitesse > 0 && sens == 'B') ? HIGH : LOW);  // B = avancer
    digitalWrite(PIN_IN2, (vitesse > 0 && sens == 'A') ? HIGH : LOW);  // A = reculer
    moteur.write(vitesse);
}

static void commander_direction(int angle) {
    ecrire_us(servo_dir, vers_us(constrain(angle, DIR_MIN_DEG, DIR_MAX_DEG)));
}

static void arreter_conduite() {
    suivi = SUIVI_OFF;
    commander_moteur(0, 'A');
    ecrire_us(servo_dir, vers_us(CENTRE_DIR));
}

// ---- SUIVI DE LIGNE ----
static void lire_capteurs(int brut[6]) {
    for (int i = 0; i < 3; i++) {
        brut[i] = (analogRead(CAPT_AV[i]) + analogRead(CAPT_AV[i])) / 2;
        brut[3 + i] = (analogRead(CAPT_AR[i]) + analogRead(CAPT_AR[i])) / 2;
    }
}

static void lancer_suivi() {
    oublier_mission();
    if (mode == RESEAU || mode == PAUSE_DEPOT) mode = LIBRE;
    commander_moteur(0, SENS_AVANCE);
    ecrire_us(servo_dir, vers_us(CENTRE_DIR));
    int brut[6];
    lire_capteurs(brut);
    for (int i = 0; i < 6; i++) capt_min[i] = capt_max[i] = brut[i];
    suivi = SUIVI_ATTENTE;
    dernier_ecart = ecart_filtre = braquage_actuel = 0;
    prochain_suivi = millis();
    signaler("N suivi de ligne : attente de la ligne");
}

// Noirceur 0 (blanc) a 1 (noir) de chaque capteur. Etalonnage continu : chaque capteur garde le plus clair et le plus
// sombre qu'il a vus ; tant qu'il n'a pas vu les deux, on prend le blanc et le noir vus par les 6 capteurs ensemble.
static bool noirceurs(const int brut[6], float n[6]) {
    int gmin = 4095, gmax = 0;
    for (int i = 0; i < 6; i++) {
        if (brut[i] < capt_min[i]) capt_min[i] = brut[i];
        if (brut[i] > capt_max[i]) capt_max[i] = brut[i];
        if (capt_min[i] < gmin) gmin = capt_min[i];
        if (capt_max[i] > gmax) gmax = capt_max[i];
    }
    if (gmax - gmin < CONTRASTE_MIN) return false;  // tout est de la meme couleur : pas de ligne en vue
    for (int i = 0; i < 6; i++) {
        bool propre = capt_max[i] - capt_min[i] >= CONTRASTE_MIN;
        float bas = propre ? capt_min[i] : gmin, haut = propre ? capt_max[i] : gmax;
        float x = (brut[i] - bas) / (haut - bas);
        x = x < 0 ? 0 : (x > 1 ? 1 : x);
        n[i] = NOIR_ELEVE ? x : 1.0f - x;
    }
    return true;
}

// Position de la ligne sous une rangee (mm, > 0 = ligne a droite), par barycentre ; faux si la rangee ne la voit pas.
static bool position_ligne(const float *n, float &e) {
    float t = n[0] + n[1] + n[2];
    if (t < 0.5f) return false;
    e = ECART_CAPTEURS_MM * (n[2] - n[0]) / t;
    return true;
}

static void pas_suivi() {
    int brut[6];
    float n[6], e_av = 0, e_ar = 0;
    lire_capteurs(brut);
    bool contraste = noirceurs(brut, n);
    bool vu_av = contraste && position_ligne(n, e_av);
    bool vu_ar = contraste && position_ligne(n + 3, e_ar);
    if (suivi == SUIVI_ATTENTE) {
        if (!vu_av && !vu_ar) return;
        suivi = SUIVI_ROULE;
        signaler("N ligne trouvee : on avance");
    }
    float e;
    if (vu_av) e = e_av;
    else if (vu_ar) e = e_ar;                                    // l'avant a decroche : la rangee arriere guide
    else e = (dernier_ecart >= 0 ? 1.5f : -1.5f) * ECART_CAPTEURS_MM;  // perdue : braquer du cote ou on l'a vue
    if (vu_av || vu_ar) dernier_ecart = e;
    // Ligne a droite (e > 0) : tourner a droite, courbure negative. Roue directrice a l'arriere : SIGNE_BRAQUAGE.
    ecart_filtre += FILTRE_ECART * (e - ecart_filtre);
    float braquage = atanf(-K_SUIVI * ecart_filtre * EMPATTEMENT_MM) * 180.0f / (float)M_PI;
    braquage = constrain(braquage, -BRAQUAGE_MAX_DEG, BRAQUAGE_MAX_DEG);
    float pas = VITESSE_BRAQUAGE_DEG_S * PERIODE_SUIVI_MS / 1000.0f;
    braquage_actuel += constrain(braquage - braquage_actuel, -pas, pas);
    ecrire_us(servo_dir, vers_us(constrain(CENTRE_DIR + SIGNE_BRAQUAGE * braquage_actuel, (float)DIR_MIN_DEG, (float)DIR_MAX_DEG)));
    commander_moteur(VITESSE_SUIVI, SENS_AVANCE);                        // ne s'arrete jamais : seul S (ou V, O) l'arrete
    if ((long)(millis() - prochain_affichage) >= 0) {
        prochain_affichage = millis() + 200;
        Serial.printf("N av %.1f mm (%d) ar %.1f mm (%d) braquage %.1f\n", e_av, vu_av, e_ar, vu_ar, braquage_actuel);
    }
}

static void afficher_capteurs() {
    int brut[6];
    lire_capteurs(brut);
    Serial.printf("L avant G %d M %d D %d | arriere G %d M %d D %d\n", brut[0], brut[1], brut[2], brut[3], brut[4], brut[5]);
}
static void oublier_mission() {
    balle_vue = cible_connue = serree = 0;
    balle_tenue = false;
    mission_attente = false;
}

static void arreter_tout() {
    arreter_conduite();
    oublier_mission();
    mode = LIBRE;  // les servos deja attaches gardent leur position
}

// ---- ENCHAINEMENT DU BRAS ----
static void lancer_home() {
    oublier_mission();
    if (au_home && mode == LIBRE) {  // deja en home : une balle restee dans la pince peut etre relachee
        etat.servo = SERVO_OUVERT;
        commander();
        signaler("H");
        return;
    }
    if (mode == ATTACHE || mode == REBOURS || mode == REPLI_BRAS || mode == REPLI_BASE) return;  // deja en route
    mode = bras_attache() ? REBOURS : ATTACHE;
    prochain = millis();
}

static void demarrer_reseau() {
    arreter_conduite();
    etat.servo = SERVO_OUVERT;  // apres une prise interrompue, le home garde la pince fermee : faux "D" sinon
    commander();
    mode = RESEAU;
    au_home = false;
    serree = 0;
    balle_tenue = false;
    n_histo = debut_histo = 0;
    prochain = millis();
}

// Le reseau a appris a partir de la posture de repos : il ne demarre que depuis le home.
static void demarrer_si_pret() {
    if (!balle_vue || !cible_connue || mode != LIBRE) return;  // en plein reseau, la nouvelle position est gardee
    if (au_home) {
        demarrer_reseau();
    } else if (!bras_attache()) {
        arreter_conduite();
        mission_attente = true;
        mode = ATTACHE;
        prochain = millis();
    } else {
        oublier_mission();
        signaler("Bras hors de la position home : envoyer H d'abord");
    }
}

static void choisir_demo(int numero) {  // numero : 1, 2 ou 3
    balle[0] = DEMO_BALLES[numero - 1][0];
    balle[1] = DEMO_BALLES[numero - 1][1];
    balle[2] = Z_BALLE;
    memcpy(cible, DEMO_PANIER, sizeof(cible));
    grand = 0;
    balle_vue = cible_connue = 1;
}

// ---- TEST DE LA BASE ET DE L'EPAULE (T en USB), tous les servos du bras tenus ----
// Base : +30, -30, retour (10 degres/s) ; epaule : +15 puis retour ; puis retour en home.
// Si la base et l'epaule bougent seules (test_servos) mais pas ici : l'alimentation ne tient pas 5 servos a la fois.
static void lancer_test() {
    if (mode != LIBRE || !bras_attache()) {
        signaler("3 refuse : bras pas encore en home ou en mouvement");
        return;
    }
    arreter_conduite();
    oublier_mission();
    test_pas = 0;
    au_home = false;
    n_histo = debut_histo = 0;
    mode = TEST_BRAS;
    prochain = millis();
    signaler("T base +-30 puis epaule +15");
}

static void pas_test() {
    const float D = (float)M_PI / 180.0f;
    int k = test_pas++;
    int i = (k < 120) ? 0 : 1;
    float d;
    if (k < 30) d = D; else if (k < 90) d = -D; else if (k < 120) d = D;
    else if (k < 135) d = D; else if (k < 150) d = -D;
    else { mode = REPLI_BRAS; return; }  // retour en home, qui renvoie H
    float q = etat.q[i] + d;
    etat.q[i] = q < BUTEE_BAS[i] ? BUTEE_BAS[i] : (q > BUTEE_HAUT[i] ? BUTEE_HAUT[i] : q);
    commander();
    if (k % 10 == 0) Serial.printf("T base %.1f epaule %.1f degres servo\n", vers_degres(0, etat.q[0]), vers_degres(1, etat.q[1]));
}

// ---- LECTURE DES COMMANDES ----
static void annuler_prise() {  // reprendre la conduite annule la prise
    if (mode == RESEAU || mode == PAUSE_DEPOT) {
        oublier_mission();
        mode = LIBRE;
    }
}

static void appliquer_angle(int angle) {
    suivi = SUIVI_OFF;  // la conduite a la main reprend la main sur le suivi de ligne
    commander_direction(angle);
    annuler_prise();
}

static void diagnostic_broche(const char *nom, int broche, bool attache) {
    gpio_input_enable((gpio_num_t)broche);  // lit le signal sans couper la sortie PWM
    unsigned long impulsion = pulseIn(broche, HIGH, 30000);
    Serial.printf("  %-9s broche %2d attache %d impulsion %4lu us frequence %lu Hz\n", nom, broche, attache ? 1 : 0, impulsion,
                  attache ? (unsigned long)ledcReadFreq(broche) : 0UL);
}

static void diagnostic() {
    static const char *NOMS[5] = {"base", "epaule", "coude", "poignet", "pince"};
    Serial.printf("P depuis le demarrage %lu ms, cause du demarrage %d, mode %d, home %d\n", millis(), (int)esp_reset_reason(),
                  (int)mode, au_home ? 1 : 0);
    diagnostic_broche("direction", PIN_DIR, servo_dir.attached());
    for (int i = 0; i < 5; i++) diagnostic_broche(NOMS[i], BROCHE[i], servos[i].attached());
}

static void traiter_octet(char c) {
    c = toupper(c);
    if (c == '\r' || c == '\n' || c == ' ') return;
    if (c == 'O') c = 'R';  // l'application (curseur "Ascenseur1") envoie O<n> pour la direction

    if (c >= '0' && c <= '9') {
        if (trame) {
            if (nb_chiffres < 4) {
                valeur = valeur * 10 + (c - '0');
                nb_chiffres++;
                debut_trame = millis();
            }
        } else if (c >= '1' && c <= '3' && mode == LIBRE) {
            choisir_demo(c - '0');
            demarrer_si_pret();
        }
        return;
    }

    char en_cours = trame;
    int n = nb_chiffres > 0 ? valeur : -1;
    trame = 0;
    if (en_cours == 'R' && n >= 0) appliquer_angle(n);  // la lettre suivante termine aussi un angle

    switch (c) {
        case 'S':
            arreter_tout();
            break;
        case 'H':
            lancer_home();
            break;
        case 'V':
        case 'R':
            trame = c;
            valeur = nb_chiffres = 0;
            debut_trame = millis();
            break;
        case 'A':
        case 'B':
            if (en_cours == 'V' && n >= 0) {
                suivi = SUIVI_OFF;
                commander_moteur(n, c);
                annuler_prise();
            }
            break;
        case 'N':
            lancer_suivi();
            break;
        default:  // par exemple le L envoye par l'application a la connexion
            break;
    }
}

// Pour regler SENS et DECALAGE_DEG en regardant le bras. Passe par etat : H revient ensuite en douceur.
static void reglage(int i, float d) {
    if (mode != LIBRE || !bras_attache() || i < 0 || i > 4) {
        Serial.println("J refuse : bras pas encore actif ou en mouvement");
        return;
    }
    float r = d * (float)M_PI / 180.0f;
    if (i == 4) {
        etat.servo += r;
    } else {
        float q = etat.q[i] + r;
        etat.q[i] = q < BUTEE_BAS[i] ? BUTEE_BAS[i] : (q > BUTEE_HAUT[i] ? BUTEE_HAUT[i] : q);
    }
    au_home = false;
    commander();
    Serial.printf("J servo %d : %.1f degres, %d us\n", i, i == 4 ? vers_degres(4, etat.servo) : vers_degres(i, etat.q[i]),
                  vers_us(i == 4 ? vers_degres(4, etat.servo) : vers_degres(i, etat.q[i])));
}

static void traiter_ligne_usb(char *ligne) {
    int j_i;
    float j_d;
    if (ligne[0] == 'J' && sscanf(ligne + 1, "%d %f", &j_i, &j_d) == 2) {
        reglage(j_i, j_d);
        return;
    }
    if (ligne[0] == 'B' && sscanf(ligne + 1, "%f %f %f", &balle[0], &balle[1], &balle[2]) == 3) {
        balle_vue = 1;
        demarrer_si_pret();
    } else if (ligne[0] == 'C' && sscanf(ligne + 1, "%f %f %f %d", &cible[0], &cible[1], &cible[2], &grand) == 4) {
        cible_connue = 1;
        demarrer_si_pret();
    } else if (ligne[0] == 'R' && ligne[1] == 0) {
        lancer_home();
    } else if (ligne[0] == 'L' && ligne[1] == 0) {
        afficher_capteurs();
    } else if (ligne[0] == 'T' && ligne[1] == 0) {
        lancer_test();
    } else if (ligne[0] == 'P' && ligne[1] == 0) {
        diagnostic();
    } else {
        for (char *p = ligne; *p; p++) traiter_octet(*p);
    }
}

static void lire_usb() {
    static char ligne[96];
    static int n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            ligne[n] = 0;
            n = 0;
            if (ligne[0]) traiter_ligne_usb(ligne);
        } else if (n < (int)sizeof(ligne) - 1) {
            ligne[n++] = c;
        }
    }
}

// ---- UN PAS DU BRAS (toutes les 100 ms) : rien ne bloque, S reste lu a tout moment ----
static void pas_attache() {
    for (int k = 0; k < 5; k++) {
        int i = ORDRE_ATTACHE[k];
        if (!servos[i].attached()) {
            attacher_servo(servos[i], BROCHE[i]);
            commander();
            prochain = millis() + ECART_ATTACHE_MS;
            return;
        }
    }
    au_home = true;
    mode = LIBRE;
    signaler("H");
    if (mission_attente) {
        mission_attente = false;
        demarrer_reseau();
    }
}

static void pas_reseau() {
    float action[N_ACT], outil[3];
    unsigned long t0 = micros();
    // Le reseau a appris en voyant la balle suivre la pince une fois saisie. Sans capteur, il faut le reproduire :
    // avec la position fixe du plan, il croit la prise ratee et redescend (simulateur : 0/60, contre 60/60 ainsi).
    cerveau_outil(etat.q, outil);
    float dx = outil[0] - balle[0], dy = outil[1] - balle[1], dz = outil[2] - balle[2];
    if (etat.servo > SERVO_CONTACT) balle_tenue = false;
    else if (sqrtf(dx * dx + dy * dy + dz * dz) < R_BALLE_TENUE) balle_tenue = true;
    cerveau_pas(&etat, balle_tenue ? outil : balle, cible, grand, action);
    commander();
    memoriser();
    Serial.printf("E %.3f %.3f %.3f %.3f %.3f (%lu us)\n", etat.q[0], etat.q[1], etat.q[2], etat.q[3], etat.servo,
                  micros() - t0);
    // Fin de la depose : pince rouverte en hauteur apres une prise (rouverte pres du sol = nouvel essai de prise).
    cerveau_outil(etat.q, outil);
    if (etat.servo <= SERVO_CONTACT) {
        serree = 1;
    } else if (serree && outil[2] > Z_DEPOT_MINI) {
        oublier_mission();
        signaler("D");
        mode = PAUSE_DEPOT;
        prochain = millis() + PAUSE_DEPOT_MS;
    }
}

// Refait a l'envers le chemin du reseau jusqu'au passage interieur bas.
static void pas_rebours() {
    if (n_histo == 0) {
        mode = REPLI_BRAS;
        return;
    }
    n_histo--;
    memcpy(etat.q, histo[(debut_histo + n_histo) % N_HISTO], sizeof(etat.q));
    commander();
    float outil[3];
    cerveau_outil(etat.q, outil);
    if (hypotf(outil[0], outil[1]) < R_RENTRE && outil[2] < Z_RENTRE) mode = REPLI_BRAS;
}

// Va vers le repos, au plus vite du reseau ; renvoie vrai a l'arrivee. La pince garde son etat (balle non lachee).
static bool pas_repli(int premier, int dernier) {
    bool fini = true;
    for (int j = premier; j <= dernier; j++) {
        float d = Q_REPOS[j] - etat.q[j];
        if (fabsf(d) > 1e-4f) fini = false;
        etat.q[j] += d > PAS_SERVO ? PAS_SERVO : (d < -PAS_SERVO ? -PAS_SERVO : d);
    }
    commander();
    return fini;
}

void setup() {
    // Le moteur doit rester a l'arret avant tout le reste.
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);

    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    moteur.attachPin(PIN_ENA, FREQ_MOTEUR_HZ, 8);
    moteur.write(0);

    attacher_servo(servo_dir, PIN_DIR);
    ecrire_us(servo_dir, vers_us(CENTRE_DIR));

    // Le bras s'active tout seul, un servo toutes les 500 ms, vers la position home. Son premier mouvement se fait a
    // pleine vitesse : les servos ne donnent pas leur position. BROCHES 34 A 39 INTERDITES : l'ESP32 ne peut qu'y lire.
    cerveau_repos(&etat);

    SerialBT.begin("Bras_ESP32_Robot");
    Serial.println("Robot pret : le bras va en position home");
    mode = ATTACHE;
    prochain = millis();
}

void loop() {
    while (SerialBT.available() > 0) traiter_octet((char)SerialBT.read());
    lire_usb();

    if (trame == 'R' && nb_chiffres > 0 && millis() - debut_trame > FIN_ANGLE_MS) {
        appliquer_angle(valeur);
        trame = 0;
    }
    if (trame && millis() - debut_trame > DELAI_TRAME_MS) trame = 0;

    // Telephone deconnecte : plus personne ne peut envoyer S, donc on arrete tout.
    bool connecte = SerialBT.hasClient();
    if (client_connecte && !connecte) {
        arreter_tout();
        trame = 0;
        Serial.println("Bluetooth perdu : robot arrete");
    }
    client_connecte = connecte;

    if (suivi != SUIVI_OFF && (long)(millis() - prochain_suivi) >= 0) {
        prochain_suivi = millis() + PERIODE_SUIVI_MS;
        pas_suivi();
    }

    if (mode == LIBRE || (long)(millis() - prochain) < 0) return;
    prochain = millis() + PERIODE_MS;

    switch (mode) {
        case ATTACHE:
            pas_attache();
            break;
        case RESEAU:
            pas_reseau();
            break;
        case PAUSE_DEPOT:
            mode = REBOURS;
            break;
        case REBOURS:
            pas_rebours();
            break;
        case TEST_BRAS:
            pas_test();
            break;
        case REPLI_BRAS:
            if (pas_repli(1, 3)) mode = REPLI_BASE;
            break;
        case REPLI_BASE:
            if (pas_repli(0, 0)) {
                n_histo = debut_histo = 0;
                au_home = true;
                mode = LIBRE;
                signaler("H");
            }
            break;
        default:
            break;
    }
}
