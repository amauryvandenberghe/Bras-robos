// Cerveau du bras 50 cm (voir cerveau.h). Ecrit en C portable : se compile pour l'ESP32 (Arduino) et sur PC (test).
#include <math.h>
#include <string.h>
#include "cerveau.h"
#include "reseau_poids.h"

// ---------------------------------------------------------------- petite algebre 4x4
static void mul44(const float a[16], const float b[16], float r[16]) {
    float t[16];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
            t[i * 4 + j] = s;
        }
    memcpy(r, t, sizeof(t));
}

static void rotation_axe(const float axe[3], float angle, float r[16]) {
    // formule de Rodrigues (axe unitaire)
    float c = cosf(angle), s = sinf(angle), v = 1 - c, x = axe[0], y = axe[1], z = axe[2];
    float m[16] = {c + x * x * v, x * y * v - z * s, x * z * v + y * s, 0,
                   y * x * v + z * s, c + y * y * v, y * z * v - x * s, 0,
                   z * x * v - y * s, z * y * v + x * s, c + z * z * v, 0,
                   0, 0, 0, 1};
    memcpy(r, m, sizeof(m));
}

// Poses du coude ("bras trois"), du poignet ("bras quatre") : produit des transformations fixes de l'URDF
// (ORIGINE_JOINT) et des rotations des 4 servos.
static void chaine(const float q[4], float T_coude[16], float T_poignet[16]) {
    float T[16];
    memcpy(T, BASE_VERS_LACET, sizeof(T));                      // monde -> origine du joint de lacet
    for (int j = 0; j < 4; j++) {
        float R[16];
        if (j > 0) mul44(T, ORIGINE_JOINT[j], T);
        rotation_axe(AXE_JOINT[j], q[j], R);
        mul44(T, R, T);
        if (j == 2) memcpy(T_coude, T, sizeof(T));
    }
    memcpy(T_poignet, T, sizeof(T));
}

static void appliquer(const float T[16], const float p[3], float r[3]) {
    for (int i = 0; i < 3; i++) r[i] = T[i * 4] * p[0] + T[i * 4 + 1] * p[1] + T[i * 4 + 2] * p[2] + T[i * 4 + 3];
}

void cerveau_outil(const float q[4], float outil[3]) {
    float Tc[16], Tp[16];
    chaine(q, Tc, Tp);
    appliquer(Tp, OUTIL, outil);
}

static float borne(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

void cerveau_observation(const EtatBras *e, const float balle[3], const float cible[3], int grand, float obs[N_OBS]) {
    float o[3];
    cerveau_outil(e->q, o);
    int n = 0;
    for (int i = 0; i < 4; i++) obs[n++] = e->q[i] / 3.0f;
    obs[n++] = (e->servo - SERVO_FERME) / (SERVO_OUVERT - SERVO_FERME) * 2 - 1;
    for (int i = 0; i < 3; i++) obs[n++] = balle[i] * 4;
    for (int i = 0; i < 3; i++) obs[n++] = cible[i] * 4;
    for (int i = 0; i < 3; i++) obs[n++] = o[i] * 4;
    for (int i = 0; i < 3; i++) obs[n++] = (balle[i] - o[i]) * 10;
    for (int i = 0; i < 3; i++) obs[n++] = (cible[i] - o[i]) * 4;
    for (int i = 0; i < 3; i++) obs[n++] = borne((balle[i] - o[i]) * 50, -1, 1);
    for (int i = 0; i < 3; i++) obs[n++] = borne((cible[i] - o[i]) * 50, -1, 1);
    obs[n++] = grand ? 1.0f : -1.0f;
}

void cerveau_reseau(const float obs[N_OBS], float action[N_ACT]) {
    static float h1[CACHE], h2[CACHE];                 // 2 x 512 flottants en RAM ; les poids restent en flash
    for (int i = 0; i < CACHE; i++) {
        float s = B1[i];
        const float *w = &W1[i * N_OBS];
        for (int k = 0; k < N_OBS; k++) s += w[k] * obs[k];
        h1[i] = tanhf(s);
    }
    for (int i = 0; i < CACHE; i++) {
        float s = B2[i];
        const float *w = &W2[i * CACHE];
        for (int k = 0; k < CACHE; k++) s += w[k] * h1[k];
        h2[i] = tanhf(s);
    }
    for (int i = 0; i < N_ACT; i++) {
        float s = B3[i];
        const float *w = &W3[i * CACHE];
        for (int k = 0; k < CACHE; k++) s += w[k] * h2[k];
        action[i] = tanhf(s);
    }
}

static int dans_robot(const float p[3], float marge) {
    return fabsf(p[0]) < DEMI_ROBOT + marge && fabsf(p[1]) < DEMI_ROBOT + marge && p[2] < HAUT_ROBOT + marge;
}

void cerveau_pas(EtatBras *e, const float balle[3], const float cible[3], int grand, float action[N_ACT]) {
    float obs[N_OBS];
    cerveau_observation(e, balle, cible, grand, obs);
    cerveau_reseau(obs, action);
    float a[N_ACT];
    for (int i = 0; i < N_ACT; i++) a[i] = borne(action[i], -1, 1);
    // couche de securite (identique a politique.securite)
    int serrage = a[4] < 0 && e->servo > SERVO_FERME + 0.005f && e->servo <= SERVO_CONTACT + 0.03f;
    if (!serrage) {
        float qn[4], Tc[16], Tp[16], outil[3], poignet[3] = {0, 0, 0};
        for (int j = 0; j < 4; j++) qn[j] = borne(e->q[j] + a[j] * PAS_SERVO, BUTEE_BAS[j], BUTEE_HAUT[j]);
        chaine(qn, Tc, Tp);
        appliquer(Tp, OUTIL, outil);
        appliquer(Tp, poignet, poignet);
        if (!(outil[2] < Z_MIN_OUTIL || dans_robot(outil, 0.012f) || dans_robot(poignet, 0.0f)))
            memcpy(e->q, qn, sizeof(qn));
    }
    // servo de pince : serrage lent pres de la balle
    float cible_servo = a[4] < 0 ? SERVO_FERME : SERVO_OUVERT;
    float pas = (cible_servo == SERVO_FERME && e->servo <= SERVO_CONTACT + 0.03f) ? PAS_SERRAGE : PAS_PINCE;
    e->servo += borne(cible_servo - e->servo, -pas, pas);
}

void cerveau_repos(EtatBras *e) {
    memcpy(e->q, Q_REPOS, sizeof(e->q));
    e->servo = SERVO_OUVERT;
}
