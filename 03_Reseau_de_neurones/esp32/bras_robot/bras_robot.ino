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
//   1 2      demo : prend la balle 1 ou 2 et la depose dans le panier (positions : DEMO_BALLES, DEMO_PANIER) ; 3 ignore
//   V<n>A    recule a la vitesse n (0 a 255, l'application envoie au plus 126) ; V<n>B avance ; V0A ou V0B coupe le moteur
//   O<n>     direction : angle absolu du servo, de 0 a 180 (curseur de l'application), 90 = centre (R<n> accepte aussi)
//   V ou O pendant une prise annule la prise.
//
// COMMANDES USB EN PLUS (115 200 bauds, une ligne par commande, fin de ligne "New Line"), positions en metres,
// repere du robot : origine sur l'axe du bras au sol, x vers l'avant, z vers le haut
//   C x y z g   point de lacher (g = 1 : grand receptacle, 0 : petit)
//   B x y z     balle visee. Le reseau demarre quand C et B sont connus
//   R           position home (seul sur la ligne)
//   J i d       reglage : bouge le servo i (0 base, 1 epaule, 2 coude, 3 poignet, 4 pince) de d degres, puis H pour revenir
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
const unsigned long FIN_ANGLE_MS = 30;          // R<n> n'a pas de lettre de fin : applique apres ce silence
const uint32_t FREQ_MOTEUR_HZ = 1000;
const float Z_BALLE = 0.0148f;                  // centre d'une balle posee sur la reglette
const float Z_DEPOT_MINI = 0.12f;               // rouvrir plus haut que ca = lacher dans un receptacle
const float R_RENTRE = 0.10f, Z_RENTRE = 0.31f; // passage interieur bas : pince a < 10 cm de l'axe, < 31 cm
const int ORDRE_ATTACHE[5] = {4, 3, 2, 1, 0};   // pince, poignet, coude, epaule, puis base
const float R_BALLE_TENUE = 0.02f;              // pince fermee a moins de 2 cm de la balle = balle consideree tenue

// Positions de la demo, en metres (x vers l'avant, y vers la gauche, depuis l'axe du bras). Tirees au hasard puis
// retenues apres 48 essais reussis sur 48 au simulateur, avec jusqu'a 3 mm d'erreur de pose.
const float DEMO_BALLES[2][2] = {{0.2045f, 0.1602f}, {0.2327f, 0.0547f}};
const float DEMO_PANIER[3] = {0.1204f, -0.2492f, 0.18f};  // z = hauteur de lacher apprise pour un petit receptacle

// Memoire des consignes du reseau : le retour au repos refait ce chemin a l'envers, car un retour direct
// heurte le bord du grand receptacle une fois sur quatre (essai_retour_repos.py).
const int N_HISTO = 300;                        // 30 s de consignes
float histo[N_HISTO][4];
int n_histo = 0, debut_histo = 0;

enum Mode { LIBRE, ATTACHE, RESEAU, PAUSE_DEPOT, REBOURS, REPLI_BRAS, REPLI_BASE };
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
    commander_moteur(0, 'A');
    ecrire_us(servo_dir, vers_us(CENTRE_DIR));
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

static void choisir_demo(int numero) {  // numero : 1 ou 2
    balle[0] = DEMO_BALLES[numero - 1][0];
    balle[1] = DEMO_BALLES[numero - 1][1];
    balle[2] = Z_BALLE;
    memcpy(cible, DEMO_PANIER, sizeof(cible));
    grand = 0;
    balle_vue = cible_connue = 1;
}

// ---- LECTURE DES COMMANDES ----
static void annuler_prise() {  // reprendre la conduite annule la prise
    if (mode == RESEAU || mode == PAUSE_DEPOT) {
        oublier_mission();
        mode = LIBRE;
    }
}

static void appliquer_angle(int angle) {
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
        } else if ((c == '1' || c == '2') && mode == LIBRE) {
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
                commander_moteur(n, c);
                annuler_prise();
            }
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
