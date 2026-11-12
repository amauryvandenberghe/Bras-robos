// PROGRAMME PROVISOIRE : calage des palonniers du bras. Le vrai programme reste bras_robot (a retéléverser ensuite).
//
// Chaque servo est envoye a l'angle qui correspond, pour le reseau de neurones, a la posture de reference
// (meme conversion que bras_robot : degre servo = (q - BUTEE_BAS) / (BUTEE_HAUT - BUTEE_BAS) x 180) :
//   0 base     q = 0 : bras dirige droit vers l'avant du robot          -> 90,0 degres servo
//   1 epaule   q = 0 : bras de 25 cm vertical                           -> 54,4 degres servo
//   2 coude    q = 0 : avant-bras dans le prolongement du bras          ->  2,9 degres servo
//   3 poignet  q = 0 : pince dans le prolongement de l'avant-bras       -> 68,8 degres servo
//   4 pince    milieu de course du servo de pince                      -> 90,0 degres servo
// Avec l'epaule, le coude et le poignet a q = 0, tout le bras est vertical, pince vers le haut.
// Il suffit alors d'emboiter chaque palonnier pour que la piece soit dans cette position.
//
// Moniteur serie 115 200 bauds (ou Bluetooth "Bras_ESP32_Calage") :
//   0 a 4   active ce servo a son angle de calage (les servos deja cales restent tenus)
//   + / -   corrige le dernier servo de 1 degre (pour voir de combien il faudrait decaler)
//   X       relache tous les servos
// Au demarrage, seule la base est active.

#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const char *NOM[5] = {"base", "epaule", "coude", "poignet", "pince"};
const int BROCHE[5] = {19, 18, 23, 22, 16};
const float CALAGE_DEG[5] = {90.0f, 54.43f, 2.86f, 68.75f, 90.0f};
const int PIN_IN1 = 33, PIN_IN2 = 32, PIN_ENA = 25, PIN_DIR = 17;

ESP32PWM servos[5];
float angle[5];
int dernier = 0;

static void ecrire(int i) {
    int us = (int)(500.0f + angle[i] * 2000.0f / 180.0f + 0.5f);
    servos[i].write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}

static void signaler(const char *m) {
    Serial.println(m);
    SerialBT.println(m);
}

static void activer(int i) {
    angle[i] = CALAGE_DEG[i];
    if (!servos[i].attached()) servos[i].attachPin(BROCHE[i], 50.0, 16);
    ecrire(i);
    dernier = i;
    char m[96];
    snprintf(m, sizeof(m), "%s (broche %d) tenu a %.1f degres servo : caler le palonnier", NOM[i], BROCHE[i], angle[i]);
    signaler(m);
}

static void traiter(char c) {
    if (c >= '0' && c <= '4') {
        activer(c - '0');
    } else if ((c == '+' || c == '-') && servos[dernier].attached()) {
        angle[dernier] = constrain(angle[dernier] + (c == '+' ? 1.0f : -1.0f), 0.0f, 180.0f);
        ecrire(dernier);
        char m[96];
        snprintf(m, sizeof(m), "%s a %.1f degres (calage %+.1f)", NOM[dernier], angle[dernier], angle[dernier] - CALAGE_DEG[dernier]);
        signaler(m);
    } else if (c == 'X' || c == 'x') {
        for (int i = 0; i < 5; i++) if (servos[i].attached()) servos[i].detachPin(BROCHE[i]);
        signaler("Tous les servos relaches");
    }
}

void setup() {
    int sorties[] = {PIN_IN1, PIN_IN2, PIN_ENA};
    for (int p : sorties) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }  // moteur coupe
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    SerialBT.begin("Bras_ESP32_Calage");
    delay(500);
    signaler("Calage des palonniers : 0 base, 1 epaule, 2 coude, 3 poignet, 4 pince, +/- 1 degre, X relacher");
    activer(0);
}

void loop() {
    while (Serial.available() > 0) traiter((char)Serial.read());
    while (SerialBT.available() > 0) traiter((char)SerialBT.read());
}
