// Test des 6 servos (cablage du 11/11) : chacun fait une course de 90 degres, un seul a la fois.
// Au demarrage, le test passe tout seul sur les 6 servos, dans l'ordre :
//   direction (17), base (21), epaule (4), coude (13), poignet (22), pince (16).
// Pour chaque servo : va a 90 (milieu), descend lentement a 45, monte a 135 (= 90 degres de course),
// revient a 90, puis redevient libre avant de passer au suivant.
// Moniteur serie 115 200 bauds : "T" relance les 6, "0" a "5" teste un seul servo (0 = direction ... 5 = pince).
// TENIR LE BRAS A LA MAIN : les servos non testes sont libres. Moteur de propulsion coupe.
// Remettre ensuite bras_robot.

#include <ESP32Servo.h>

const int N = 6;
const char *NOM[N] = {"direction", "base", "epaule", "coude", "poignet", "pince"};
const int BROCHE[N] = {17, 21, 4, 13, 22, 16};
const int PIN_IN1 = 33, PIN_IN2 = 32, PIN_ENA = 25;  // pont en H L298N
const float MILIEU = 90.0f, BAS = 45.0f, HAUT = 135.0f;

ESP32PWM servo;  // pas la classe Servo : voir bras_robot

static int vers_us(float deg) { return (int)(500.0f + deg * 2000.0f / 180.0f + 0.5f); }

static void ecrire(float deg) {
    servo.write((uint32_t)(constrain(vers_us(deg), 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}

static void aller(float de, float a) {
    float pas = (a > de) ? 1.0f : -1.0f;
    for (float d = de; (pas > 0) ? d <= a : d >= a; d += pas) { ecrire(d); delay(20); }
}

static void tester(int i) {
    Serial.printf("[%d] %s, broche %d : 90 -> 45 -> 135 -> 90\n", i, NOM[i], BROCHE[i]);
    servo.attachPin(BROCHE[i], 50.0, 16);
    ecrire(MILIEU);
    delay(1500);
    aller(MILIEU, BAS);
    delay(500);
    aller(BAS, HAUT);
    delay(500);
    aller(HAUT, MILIEU);
    delay(1000);
    servo.detachPin(BROCHE[i]);
    pinMode(BROCHE[i], OUTPUT);
    digitalWrite(BROCHE[i], LOW);
    Serial.printf("[%d] %s : fini, servo libre\n", i, NOM[i]);
    delay(1000);
}

static void tester_tous() {
    for (int i = 0; i < N; i++) tester(i);
    Serial.println("Test termine. T = recommencer, 0 a 5 = un seul servo");
}

void setup() {
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_ENA, LOW);
    for (int i = 0; i < N; i++) { pinMode(BROCHE[i], OUTPUT); digitalWrite(BROCHE[i], LOW); }
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    delay(2000);
    Serial.println("Test des 6 servos, un par un");
    tester_tous();
}

void loop() {
    if (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == 'T' || c == 't') tester_tous();
        else if (c >= '0' && c <= '5') tester(c - '0');
    }
}
