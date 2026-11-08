// Cerveau du bras 50 cm pour ESP32 : meme calcul que politique.py (observation, reseau, couche de securite).
// Les constantes (poids du reseau, geometrie du bras) sont dans reseau_poids.h, genere par exporter_esp32.py.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define N_OBS 27
#define N_ACT 5

typedef struct {
    float q[4];        // consignes des 4 servos du bras (rad, convention URDF)
    float servo;       // consigne du servo de pince (rad, angle du pignon)
} EtatBras;

// Position du point de prise (m, repere du robot : axe du bras, sol a z = 0) pour les angles q.
void cerveau_outil(const float q[4], float outil[3]);
// Observation du reseau (identique a politique.observation).
void cerveau_observation(const EtatBras *e, const float balle[3], const float cible[3], int grand, float obs[N_OBS]);
// Passe avant du reseau : 5 sorties dans [-1, 1] (4 servos du bras, pince : < 0 fermer, > 0 ouvrir).
void cerveau_reseau(const float obs[N_OBS], float action[N_ACT]);
// Un pas complet : observation -> reseau -> securite -> nouvelles consignes (e est mis a jour). Renvoie l'action brute.
void cerveau_pas(EtatBras *e, const float balle[3], const float cible[3], int grand, float action[N_ACT]);
// Posture de repos (bras replie, pince ouverte).
void cerveau_repos(EtatBras *e);

#ifdef __cplusplus
}
#endif
