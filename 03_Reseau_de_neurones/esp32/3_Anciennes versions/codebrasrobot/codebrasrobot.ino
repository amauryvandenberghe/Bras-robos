#include <ESP32Servo.h>
#include "cerveau.h"
#include "reseau_poids.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---- PINS MOTEUR DC (MCC) & DIRECTION ----
const int PIN_DIR = 18;  
const int PIN_ENA = 25;  
const int PIN_IN1 = 26;  
const int PIN_IN2 = 27;  

// ---- PINS BRAS ROBOTISE ----
const int BROCHE[5] = {17, 16, 33, 21, 32};              
const int SENS[5] = {1, 1, 1, 1, 1};
const float DECALAGE_DEG[5] = {0, 0, 0, 0, 0};
const float PINCE_CENTRE = 0.0f;

Servo servos[5];
Servo servo_dir;

// ---- VARIABLES IA ----
EtatBras etat;
float balle[3] = {0, 0, 0}, cible[3] = {0, 0, 0};
int grand = 0, balle_vue = 0, cible_connue = 0;
int serree = 0;
const float Z_DEPOT_MINI = 0.12f;
unsigned long prochain = 0;
const unsigned long PERIODE_MS = 100;

const int N_HISTO = 300;
const float R_RENTRE = 0.10f, Z_RENTRE = 0.31f;
float histo[N_HISTO][4];
int n_histo = 0, debut_histo = 0;

// ---- FONCTIONS IA ----
float generer_aleatoire(float min, float max) {
    return min + (float)random(1000) / 1000.0f * (max - min);
}

static float vers_degres(int i, float q) {
    float deg;
    if (i < 4) deg = (q - BUTEE_BAS[i]) / (BUTEE_HAUT[i] - BUTEE_BAS[i]) * 180.0f;
    else deg = 90.0f + (q - PINCE_CENTRE) * 180.0f / (float)M_PI;
    if (SENS[i] < 0) deg = 180.0f - deg;
    deg += DECALAGE_DEG[i];
    return deg < 0 ? 0 : (deg > 180 ? 180 : deg);
}

static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }

static void commander() {
    for (int i = 0; i < 4; i++) servos[i].writeMicroseconds(vers_us(vers_degres(i, etat.q[i])));
    servos[4].writeMicroseconds(vers_us(vers_degres(4, etat.servo)));
}

static void memoriser() {
    int i = (debut_histo + n_histo) % N_HISTO;
    memcpy(histo[i], etat.q, sizeof(etat.q));
    if (n_histo < N_HISTO) n_histo++;
    else debut_histo = (debut_histo + 1) % N_HISTO;
}

void aller_repos() {
    float outil[3];
    while (n_histo > 0) {
        n_histo--;
        memcpy(etat.q, histo[(debut_histo + n_histo) % N_HISTO], sizeof(etat.q));
        commander();
        delay(PERIODE_MS);
        cerveau_outil(etat.q, outil);
        if (hypotf(outil[0], outil[1]) < R_RENTRE && outil[2] < Z_RENTRE) break;
    }
    const int groupes[2][4] = {{1, 2, 3, -1}, {0, -1, -1, -1}};
    for (int g = 0; g < 2; g++) {
        bool fini = false;
        while (!fini) {
            fini = true;
            for (int k = 0; k < 4 && groupes[g][k] >= 0; k++) {
                int j = groupes[g][k];
                float d = Q_REPOS[j] - etat.q[j];
                if (fabsf(d) > 1e-4f) fini = false;
                etat.q[j] += d > PAS_SERVO ? PAS_SERVO : (d < -PAS_SERVO ? -PAS_SERVO : d);
            }
            commander();
            delay(PERIODE_MS);
        }
    }
    n_histo = debut_histo = 0;
}

void setup() {
    Serial.begin(115200);
    
    // --- ALLOCATION DES HORLOGES POUR LES 6 SERVOS ---
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    SerialBT.begin("Bras_ESP32_Robot"); 
    SerialBT.setTimeout(20);

    // Initialisation MCC
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);

    // Initialisation Direction
    servo_dir.setPeriodHertz(50);
    servo_dir.attach(PIN_DIR, 500, 2500);
    servo_dir.write(90);

    // Initialisation Bras
    for (int i = 0; i < 5; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(BROCHE[i], 500, 2500);
        if (i == 4) servos[i].write(0);
        else servos[i].write(90); 
    }
    
    cerveau_repos(&etat);
    commander();
    
    Serial.println("Robot prêt. Moteurs sous tension.");
}

void loop() {
    if (SerialBT.available() > 0) {
        char cmd = SerialBT.read();

        if (cmd == 'S' || cmd == 's') {
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, LOW);
            analogWrite(PIN_ENA, 0);
            servo_dir.write(90);
            balle_vue = cible_connue = 0;
        }
        else if (cmd == 'H' || cmd == 'h') {
            aller_repos();
            balle_vue = cible_connue = serree = 0;
        }
        else if (cmd == '1' || cmd == '2' || cmd == '3') {
            balle[0] = generer_aleatoire(0.15, 0.25); 
            balle[1] = generer_aleatoire(-0.15, 0.15); 
            balle[2] = 0.02; 
            cible[0] = generer_aleatoire(0.15, 0.25); 
            cible[1] = generer_aleatoire(-0.15, 0.15); 
            cible[2] = generer_aleatoire(0.18, 0.25); 
            grand = random(0, 2); 
            cible_connue = 1; balle_vue = 1; 
        }
        else if (cmd == 'V' || cmd == 'v') {
            int vitesse = SerialBT.parseInt(); 
            vitesse = constrain(vitesse, 0, 255);
            
            char sens = 'X';
            long t = millis();
            while(millis() - t < 150) {
                if(SerialBT.available()) {
                    char c = toupper(SerialBT.read());
                    if(c == 'A' || c == 'B') { sens = c; break; }
                }
            }
            
            if (sens == 'B') { 
                digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
                analogWrite(PIN_ENA, vitesse);
            } 
            else if (sens == 'A') { 
                digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
                analogWrite(PIN_ENA, vitesse);
            }
            balle_vue = cible_connue = 0;
        }
        else if (cmd == 'R' || cmd == 'r') {
            int ecart = SerialBT.parseInt(); 
            
            char sens_dir = 'X';
            long t = millis();
            while(millis() - t < 150) {
                if(SerialBT.available()) {
                    char c = toupper(SerialBT.read());
                    if(c == 'C' || c == 'D') { sens_dir = c; break; }
                }
            }
            
            int angle_final = 90; 
            if (sens_dir == 'C') angle_final = 90 + ecart; 
            else if (sens_dir == 'D') angle_final = 90 - ecart; 
            
            servo_dir.write(constrain(angle_final, 0, 180));
            balle_vue = cible_connue = 0;
        }
    }

    // ---- BOUCLE INTELLIGENCE ARTIFICIELLE ----
    if (millis() >= prochain) {
        prochain = millis() + PERIODE_MS;

        if (balle_vue && cible_connue) {
            float action[N_ACT];
            cerveau_pas(&etat, balle, cible, grand, action);
            commander();
            memoriser();

            float outil[3];
            cerveau_outil(etat.q, outil);
            if (etat.servo <= SERVO_CONTACT) {
                serree = 1;
            }
            else if (serree && outil[2] > Z_DEPOT_MINI) {
                serree = balle_vue = cible_connue = 0;
                delay(500); 
                aller_repos(); 
            }
        }
    }
}