// Servos du robot : broches, calage et commande (voir servo.h).
#include "servo.h"
#include <ESP32Servo.h>
#include "driver/gpio.h"

// ================= REGLAGES (a modifier ici seulement) =================
// Broches (cablage du 12/11 soir).
const char *NOM_SERVO[N_BRAS] = {"base", "epaule", "coude", "poignet", "pince"};
static const int BROCHE[N_BRAS] = {21, 4, 13, 22, 16};
static const int BROCHE_DIRECTION = 17;
// Servo hors service = false. Poignet coupe le 12/11 (tres chaud, ne bougeait plus) : a remettre a true une fois
// le servo remplace ET cale (voir CENTRE_DEG). Sans poignet, le bouton 1 fait la sequence fixe, pas le reseau.
const bool ACTIF[N_BRAS] = {true, true, true, false, true};
// Calage des palonniers : angle du servo (degres) quand q = 0, c'est-a-dire bras vertical et droit (calage du 12/11 :
// base 90 droit devant, epaule 54,4, coude 15). GAIN = degres de servo par radian (negatif si le servo tourne a l'envers).
// Poignet (a caler sur le servo neuf) : servo a 107 degres quand la pince est dans le prolongement de l'avant-bras.
// Il couvre alors -1,10 a +1,69 rad, ce qu'utilise le reseau. L'ancien calage mettait le poignet a 2 degres au repos,
// colle a sa butee : c'est probablement ce qui l'a fait chauffer puis griller.
// Pince : le reseau commande l'angle du pignon de SERVO_FERME (-0,13) a SERVO_OUVERT (+0,25). Sur la vraie pince,
// fermee = 60 degres, a demi ouverte = 90, ouverte = 110 : d'ou 77,1 degres a q = 0 et 131,6 degres par radian.
static const float CENTRE_DEG[N_BRAS] = {90.0f, 54.4f, 15.0f, 107.0f, 77.1f};
static const float GAIN[N_BRAS] = {57.29578f, 57.29578f, 57.29578f, -57.29578f, 131.6f};
// Plage du reseau (butees de l'URDF ; pince : SERVO_FERME a SERVO_OUVERT).
static const float Q_BAS[N_BRAS] = {-1.5708f, -0.95f, -0.05f, -1.2f, -0.13f};
static const float Q_HAUT[N_BRAS] = {1.5708f, 2.1916f, 3.0916f, 1.9416f, 0.25f};
// =======================================================================

static ESP32PWM pwm[N_BRAS], pwm_direction;
static float consigne[N_BRAS];

static void ecrire_us(ESP32PWM &s, int us) {
    if (s.attached()) s.write((uint32_t)(constrain(us, 500, 2500) * 65536.0f / 20000.0f + 0.5f));
}
static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }

float servo_degres(int i, float q) { return CENTRE_DEG[i] + GAIN[i] * q; }
float servo_q(int i, float deg) { return (deg - CENTRE_DEG[i]) / GAIN[i]; }

float servo_borne(int i, float q) {
    float a = servo_q(i, ANGLE_MIN), b = servo_q(i, ANGLE_MAX);
    float bas = max(Q_BAS[i], min(a, b)), haut = min(Q_HAUT[i], max(a, b));
    return constrain(q, bas, haut);
}

void servos_demarrer() {
    for (int t = 0; t < 4; t++) ESP32PWM::allocateTimer(t);
    pwm_direction.attachPin(BROCHE_DIRECTION, 50.0, 16);  // 16 bits : pas de 0,03 degre
    direction_ecrire(90);
    for (int i = 0; i < N_BRAS; i++) {
        consigne[i] = 0;
        pinMode(BROCHE[i], OUTPUT);
        digitalWrite(BROCHE[i], LOW);  // pas de signal tant que le servo n'est pas commande
    }
}

void bras_poser(int i, float q) {
    consigne[i] = servo_borne(i, q);
    if (!ACTIF[i]) return;
    if (!pwm[i].attached()) pwm[i].attachPin(BROCHE[i], 50.0, 16);
    ecrire_us(pwm[i], vers_us(servo_degres(i, consigne[i])));
}

void bras_poser_tout(const float q[N_BRAS]) {
    for (int i = 0; i < N_BRAS; i++) bras_poser(i, q[i]);
}

float bras_consigne(int i) { return consigne[i]; }

void bras_relacher() {
    for (int i = 0; i < N_BRAS; i++) {
        if (pwm[i].attached()) pwm[i].detachPin(BROCHE[i]);
        pinMode(BROCHE[i], OUTPUT);
        digitalWrite(BROCHE[i], LOW);
    }
}

void direction_ecrire(float deg) { ecrire_us(pwm_direction, vers_us(constrain(deg, ANGLE_MIN, ANGLE_MAX))); }

void servos_diagnostic(Print &sortie) {
    for (int i = 0; i < N_BRAS; i++) {
        gpio_input_enable((gpio_num_t)BROCHE[i]);  // permet de relire l'impulsion sur une broche de sortie
        sortie.printf("  %-8s broche %2d actif %d consigne %+.3f rad = %5.1f deg, impulsion mesuree %4lu us\n", NOM_SERVO[i],
                      BROCHE[i], ACTIF[i] && pwm[i].attached(), consigne[i], servo_degres(i, consigne[i]),
                      pulseIn(BROCHE[i], HIGH, 30000));
    }
}
