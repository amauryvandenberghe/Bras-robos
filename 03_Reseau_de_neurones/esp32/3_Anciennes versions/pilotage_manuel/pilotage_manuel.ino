// Pilotage manuel du robot par Bluetooth (application MIT App Inventor) : avancer, tourner, bras en position home.
// Pas de reseau de neurones dans ce programme : les boutons 1, 2 et 3 sont ignores pour le moment.
//
// Carte : "ESP32 Dev Module". Bibliotheque : ESP32Servo. Le test USB (115 200 bauds) accepte les memes commandes.
//
// Commandes :
//   S          arrete la propulsion et remet la direction au centre (le bras ne bouge pas)
//   H          met le bras en position home
//   V<n>A      recule a la vitesse n (0 a 255) ; V<n>B avance ; V0A ou V0B arrete le moteur seul
//   R<n>D      tourne a gauche de n degres ; R<n>C tourne a droite ; R0C ou R0D remet au centre
// "R45D" peut arriver d'un bloc ou en deux envois ("R45" puis "D") : les deux marchent.

#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---- BROCHES (manuel technique) ----
const int PIN_DIR = 18;
const int PIN_ENA = 25;
const int PIN_IN1 = 26;
const int PIN_IN2 = 27;
//                     base, epaule, coude, poignet, pince
const int BROCHE[5] = {17, 16, 33, 21, 32};

// ---- REGLAGES A VERIFIER SUR LE VRAI ROBOT ----
// Position home en degres de servo (0 a 180), calculee depuis les butees et la posture de repos du reseau.
// A corriger ici si un palonnier n'est pas monte au bon cran.
const float HOME_DEG[5] = {90.0f, 43.6f, 133.8f, 135.8f, 104.3f};
const int CENTRE_DIR = 90;           // roues droites
const int BRAQUAGE_MAX_DEG = 45;     // limite de la direction : a ajuster selon la butee mecanique

// ---- AUTRES CONSTANTES ----
const unsigned long ECART_HOME_MS = 500;   // un servo apres l'autre, pour limiter l'appel de courant
const unsigned long DELAI_TRAME_MS = 200;  // une trame incomplete est oubliee apres ce delai
const uint32_t FREQ_MOTEUR_HZ = 1000;
const int ORDRE_HOME[5] = {4, 3, 2, 1, 0}; // pince, poignet, coude, epaule, puis base (comme aller_repos)

Servo servos[5];
Servo servo_dir;
ESP32PWM moteur;  // passe par ESP32Servo : analogWrite pourrait prendre un timer utilise par les servos

char trame = 0;   // 'V' ou 'R' quand une commande est en cours de lecture
int valeur = 0;
int nb_chiffres = 0;
unsigned long debut_trame = 0;

int etape_home = -1;  // -1 : aucune sequence en cours
unsigned long prochain_home = 0;

bool client_connecte = false;

static int vers_us(float deg) { return (int)(500.0f + deg * 2000.0f / 180.0f + 0.5f); }

static void commander_moteur(int vitesse, char sens) {
    vitesse = constrain(vitesse, 0, 255);
    // Sens repris du programme codebrasrobot : B = avancer, A = reculer.
    digitalWrite(PIN_IN1, (vitesse > 0 && sens == 'B') ? HIGH : LOW);
    digitalWrite(PIN_IN2, (vitesse > 0 && sens == 'A') ? HIGH : LOW);
    moteur.write(vitesse);
}

static void commander_direction(int ecart, char sens) {
    ecart = constrain(ecart, 0, BRAQUAGE_MAX_DEG);
    // Sens repris du manuel (probleme 5) : C = droite = plus, D = gauche = moins.
    servo_dir.write(sens == 'C' ? CENTRE_DIR + ecart : CENTRE_DIR - ecart);
}

static void arreter_conduite() {
    commander_moteur(0, 'A');
    servo_dir.write(CENTRE_DIR);
}

// Sans blocage : S et la conduite restent actifs pendant la sequence.
static void avancer_home() {
    if (etape_home < 0 || (long)(millis() - prochain_home) < 0) return;
    int i = ORDRE_HOME[etape_home];
    if (!servos[i].attached()) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(BROCHE[i], 500, 2500);
    }
    servos[i].writeMicroseconds(vers_us(HOME_DEG[i]));
    prochain_home = millis() + ECART_HOME_MS;
    etape_home = etape_home < 4 ? etape_home + 1 : -1;
    if (etape_home < 0) Serial.println("Bras en home");
}

static void traiter_octet(char c) {
    c = toupper(c);
    if (c == '\r' || c == '\n' || c == ' ') return;

    if (c >= '0' && c <= '9') {
        // Hors d'une trame V ou R, un chiffre est un bouton 1, 2 ou 3 : ignore pour le moment.
        if (trame && nb_chiffres < 4) {
            valeur = valeur * 10 + (c - '0');
            nb_chiffres++;
        }
        return;
    }

    char en_cours = trame;
    int n = nb_chiffres > 0 ? valeur : -1;
    trame = 0;

    switch (c) {
        case 'S':
            arreter_conduite();
            break;
        case 'H':
            if (etape_home < 0) {
                etape_home = 0;
                prochain_home = millis();
            }
            break;
        case 'V':
        case 'R':
            trame = c;
            valeur = 0;
            nb_chiffres = 0;
            debut_trame = millis();
            break;
        case 'A':
        case 'B':
            if (en_cours == 'V' && n >= 0) commander_moteur(n, c);
            break;
        case 'C':
        case 'D':
            if (en_cours == 'R' && n >= 0) commander_direction(n, c);
            break;
        default:  // par exemple le L envoye par l'application a la connexion
            break;
    }
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

    servo_dir.setPeriodHertz(50);
    servo_dir.attach(PIN_DIR, 500, 2500);
    servo_dir.write(CENTRE_DIR);

    // Les servos du bras restent mous jusqu'au premier H : c'est voulu, le bras ne saute pas a l'allumage.
    SerialBT.begin("Bras_ESP32_Robot");
    Serial.println("Robot pret : bras libre jusqu'a la commande H");
}

void loop() {
    while (SerialBT.available() > 0) traiter_octet((char)SerialBT.read());
    while (Serial.available() > 0) traiter_octet((char)Serial.read());

    if (trame && millis() - debut_trame > DELAI_TRAME_MS) trame = 0;

    // Telephone deconnecte : plus personne ne peut envoyer S, donc on arrete le robot.
    bool connecte = SerialBT.hasClient();
    if (client_connecte && !connecte) {
        arreter_conduite();
        trame = 0;
        Serial.println("Bluetooth perdu : robot arrete");
    }
    client_connecte = connecte;

    avancer_home();
}
