// DIAGNOSTIC (12/11) : commande directe de chaque servo par l'USB, un par un, tous libres au depart.
// Moniteur serie 115 200 bauds, une commande par ligne :
//   A i deg   active le servo i et l'envoie a deg (0 a 180) d'un coup
//   M i deg   amene le servo i a deg lentement (1 degre toutes les 20 ms), l'active s'il ne l'est pas
//   R i       relache le servo i (plus de signal)        R seul : relache tout
//   P         etat : actif, angle demande, impulsion mesuree sur chaque broche
// i : 0 base (21), 1 epaule (4), 2 coude (13), 3 poignet (22), 4 pince (16), 5 direction (17)
// Moteur de propulsion coupe. Pas de Bluetooth. Remettre ensuite robot_simple ou bras_robot.

#include <ESP32Servo.h>
#include "driver/gpio.h"

const int N = 6;
const char *NOM[N] = {"base", "epaule", "coude", "poignet", "pince", "direction"};
const int BROCHE[N] = {21, 4, 13, 22, 16, 17};
ESP32PWM servos[N];
float angle[N];

static void relacher(int i) {
    if (servos[i].attached()) servos[i].detachPin(BROCHE[i]);
    pinMode(BROCHE[i], OUTPUT);
    digitalWrite(BROCHE[i], LOW);
}
static void ecrire(int i, float deg) {
    angle[i] = constrain(deg, 0.0f, 180.0f);
    if (!servos[i].attached()) servos[i].attachPin(BROCHE[i], 50.0, 16);
    int us = (int)(500.0f + angle[i] * 2000.0f / 180.0f + 0.5f);
    servos[i].write((uint32_t)(us * 65536.0f / 20000.0f + 0.5f));
}
static void etat() {
    for (int i = 0; i < N; i++) {
        gpio_input_enable((gpio_num_t)BROCHE[i]);
        Serial.printf("  %-9s broche %2d actif %d angle %5.1f impulsion %4lu us\n", NOM[i], BROCHE[i], servos[i].attached(),
                      angle[i], pulseIn(BROCHE[i], HIGH, 30000));
    }
    Serial.println("fin");
}
static void ligne(char *l) {
    int i; float d;
    if (l[0] == 'A' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i < N) {
        ecrire(i, d);
        Serial.printf("A %s -> %.1f\n", NOM[i], angle[i]);
    } else if (l[0] == 'M' && sscanf(l + 1, "%d %f", &i, &d) == 2 && i >= 0 && i < N) {
        if (!servos[i].attached()) ecrire(i, d);
        float de = angle[i], pas = d > de ? 1.0f : -1.0f;
        for (float a = de; (pas > 0) ? a <= d : a >= d; a += pas) { ecrire(i, a); delay(20); }
        ecrire(i, d);
        Serial.printf("M %s %.1f -> %.1f\n", NOM[i], de, angle[i]);
    } else if (l[0] == 'R' && sscanf(l + 1, "%d", &i) == 1 && i >= 0 && i < N) {
        relacher(i);
        Serial.printf("R %s libre\n", NOM[i]);
    } else if (l[0] == 'R') {
        for (int k = 0; k < N; k++) relacher(k);
        Serial.println("R tout libre");
    } else if (l[0] == 'P') {
        etat();
    } else if (l[0] == 'B') {  // LED bleue de la carte (GPIO 2) allumee 4 s : repere visuel a la camera
        pinMode(2, OUTPUT); digitalWrite(2, HIGH); delay(4000); digitalWrite(2, LOW);
        Serial.println("B LED eteinte");
    } else {
        Serial.println("? A i deg | M i deg | R i | R | P");
    }
}

void setup() {
    int sorties[] = {33, 32, 25};
    for (int p : sorties) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
    Serial.begin(115200);
    for (int t = 0; t < 4; t++) ESP32PWM::allocateTimer(t);
    for (int i = 0; i < N; i++) { angle[i] = 90; relacher(i); }
    Serial.println("diag_bras pret : tous les servos libres. A i deg | M i deg | R i | R | P");
}

void loop() {
    static char l[48];
    static int n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { l[n] = 0; n = 0; if (l[0]) ligne(l); }
        else if (n < 47) l[n++] = (char)toupper(c);
    }
}
