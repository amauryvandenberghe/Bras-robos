// Test d'un seul servo a la fois, pour separer un probleme de signal d'un probleme d'alimentation.
// Moniteur serie a 115 200 bauds. Envoyer 0, 1, 2, 3 ou 4 :
//   le servo choisi va a sa position home, oscille de +-15 degres pendant environ 10 s, puis redevient libre.
// Les autres servos restent libres pendant ce temps : TENIR LE BRAS A LA MAIN avant de lancer le test.
// Le moteur de la voiture reste coupe. Remettre ensuite bras_robot.

#include <ESP32Servo.h>

const char *NOM[5] = {"base", "epaule", "coude", "poignet", "pince"};
const int BROCHE[5] = {22, 17, 33, 21, 32};
const float HOME_DEG[5] = {90.0f, 43.6f, 133.8f, 135.8f, 104.3f};
const int PIN_IN1 = 26, PIN_IN2 = 27, PIN_ENA = 25;
// Ce qu'on doit voir, calcule depuis l'URDF. Si le mouvement est inverse : SENS = -1 pour ce servo dans bras_robot.
const char *HOME_ATTENDU[5] = {
    "pince droit devant le robot",
    "bras presque vertical, penche de 11 degres vers l'arriere",
    "", "", ""};
const char *SENS_ATTENDU[5] = {
    "la base tourne vers la gauche du robot (vue de dessus : sens inverse des aiguilles)",
    "le bras bascule vers l'avant du robot",
    "", "", ""};

ESP32PWM servo;  // pas la classe Servo : voir bras_robot

static int vers_us(float deg) { return (int)(500.0f + deg * 2000.0f / 180.0f + 0.5f); }

static void ecrire_us(float deg) {
    servo.write((uint32_t)(constrain(vers_us(deg), 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}

static void tester(int i) {
    Serial.printf("Test %s, broche %d\n", NOM[i], BROCHE[i]);
    servo.attachPin(BROCHE[i], 50.0, 16);
    ecrire_us(HOME_DEG[i]);
    if (HOME_ATTENDU[i][0]) Serial.printf("Position home attendue : %s\n", HOME_ATTENDU[i]);
    delay(3000);
    if (SENS_ATTENDU[i][0]) Serial.printf("Premier mouvement attendu : %s\n", SENS_ATTENDU[i]);
    for (int cycle = 0; cycle < 2; cycle++) {
        for (float d = 0; d <= 15; d += 0.5f) { ecrire_us(HOME_DEG[i] + d); delay(40); }
        for (float d = 15; d >= -15; d -= 0.5f) { ecrire_us(HOME_DEG[i] + d); delay(40); }
        for (float d = -15; d <= 0; d += 0.5f) { ecrire_us(HOME_DEG[i] + d); delay(40); }
    }
    delay(1000);
    servo.detachPin(BROCHE[i]);
    Serial.println("Fin du test : servo libre");
}

void setup() {
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENA, OUTPUT);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_ENA, LOW);
    Serial.begin(115200);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.println("Envoyer 0 base (22), 1 epaule (17), 2 coude (33), 3 poignet (21), 4 pince (32)");
}

void loop() {
    if (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c >= '0' && c <= '4') tester(c - '0');
    }
}
