// Servos du robot : 5 servos du bras + servo de direction (voir servo.cpp pour les broches et le calage).
// Le bras est commande en RADIANS, dans la convention du reseau de neurones (URDF) : q = 0 partout = bras vertical
// et droit, pointe vers l'avant. La pince (indice 4) est l'angle du pignon en radians (SERVO_OUVERT, SERVO_FERME).
// Chaque consigne est bornee pour que le servo reste entre ANGLE_MIN et ANGLE_MAX degres : jamais en butee
// (un servo bloque en butee chauffe et grille).
#pragma once
#include <Arduino.h>

enum { BASE, EPAULE, COUDE, POIGNET, PINCE, N_BRAS };
const float ANGLE_MIN = 10.0f, ANGLE_MAX = 170.0f;

extern const char *NOM_SERVO[N_BRAS];
extern bool ACTIF[N_BRAS];                // false = servo hors service : pas de signal (commande USB D pour l'activer)

void servos_demarrer();                   // a appeler une fois dans setup(), avant tout le reste
float servo_degres(int i, float q);       // consigne (rad) -> angle du servo (degres)
float servo_q(int i, float deg);          // angle du servo (degres) -> consigne (rad)
float servo_borne(int i, float q);        // consigne ramenee dans la plage autorisee
void bras_poser(int i, float q);          // envoie une consigne (attache le servo si besoin)
void bras_poser_tout(const float q[N_BRAS]);
float bras_consigne(int i);               // derniere consigne (rad)
void bras_relacher();                     // plus aucun signal : les servos ne forcent plus
void direction_ecrire(float deg);         // servo de direction, 90 = tout droit
void servos_diagnostic(Print &sortie);    // consigne, angle et impulsion mesuree sur chaque broche
