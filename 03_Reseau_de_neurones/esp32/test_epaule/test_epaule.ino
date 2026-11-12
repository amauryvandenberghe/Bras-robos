// PROVISOIRE : l'epaule bouge-t-elle encore quand les 5 servos du bras sont alimentes ?
// Boucle sans fin, bras vertical (angles de calage) :
//   phase A (10 s) : seuls la base et l'epaule sont actives ; l'epaule oscille de +-20 degres autour de la verticale
//   phase B (10 s) : les 5 servos sont actifs (coude, poignet, pince tenus) ; l'epaule fait la meme oscillation
// Si l'epaule bouge en A mais pas en B : l'alimentation ne tient pas les 5 servos. Remettre ensuite bras_robot.
#include <ESP32Servo.h>

const int BROCHE[5] = {19, 18, 23, 22, 16};
const float CALAGE_DEG[5] = {90.0f, 54.43f, 2.86f, 68.75f, 90.0f};
ESP32PWM servos[5];

static void ecrire(int i, float deg) {
    int us = (int)(500.0f + deg * 2000.0f / 180.0f + 0.5f);
    servos[i].write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}
static void activer(int i) {
    if (!servos[i].attached()) servos[i].attachPin(BROCHE[i], 50.0, 16);
    ecrire(i, CALAGE_DEG[i]);
}
static void relacher(int i) {
    if (servos[i].attached()) servos[i].detachPin(BROCHE[i]);
    pinMode(BROCHE[i], OUTPUT);
    digitalWrite(BROCHE[i], LOW);
}
static void osciller() {  // environ 8 s : +20, -20, retour, deux fois
    for (int k = 0; k < 2; k++) {
        for (int d = 0; d <= 20; d++) { ecrire(1, CALAGE_DEG[1] + d); delay(50); }
        for (int d = 20; d >= -20; d--) { ecrire(1, CALAGE_DEG[1] + d); delay(50); }
        for (int d = -20; d <= 0; d++) { ecrire(1, CALAGE_DEG[1] + d); delay(50); }
    }
}

void setup() {
    int sorties[] = {33, 32, 25};
    for (int p : sorties) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    Serial.begin(115200);
    for (int t = 0; t < 4; t++) ESP32PWM::allocateTimer(t);
    for (int i = 0; i < 5; i++) relacher(i);
}

void loop() {
    Serial.println("PHASE A : base + epaule seulement, l'epaule oscille");
    for (int i = 2; i < 5; i++) relacher(i);
    activer(0); activer(1);
    delay(1000);
    osciller();
    Serial.println("PHASE B : les 5 servos actifs, l'epaule oscille");
    for (int i = 2; i < 5; i++) { activer(i); delay(300); }
    delay(1000);
    osciller();
    delay(1000);
}
