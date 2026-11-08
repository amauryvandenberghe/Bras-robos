// Bras 50 cm pilote par le reseau de neurones (meme calcul que dans Gazebo), sur ESP32.
//
// Materiel : 5 servos (lacet MG996R, epaule MG996R, coude SG90/MG90S, poignet SG90, pince MG90S),
//            alimentes par une alimentation 5-6 V separee (2,5 A par MG996R au blocage), masses reliees a l'ESP32.
// Bibliotheque : ESP32Servo (gestionnaire de bibliotheques Arduino).
// Arduino IDE : carte "ESP32 Dev Module", schema de partition "Huge APP (3MB No OTA)" (les poids font ~1,1 Mo).
//
// Reseau : reseau_final.pt du 30/10 (reseau du parcours affine dans Gazebo), poids dans reseau_poids.h.
//
// Entree (port serie 115200 bauds, envoyee par le programme du robot ou a la main depuis le moniteur serie),
// positions en metres dans le repere du robot (origine : axe du bras au niveau du sol, x vers l'avant, z vers le haut).
// Le plateau est fixe : les positions de chaque station sont calculees d'avance a partir du plan
// (parcours/plateau.py, PDF chapitres 7 et 11) et envoyees une fois le robot arrete et recale sur son repere.
//   C x y z g       point de lacher et type de receptacle (g = 1 : grand receptacle de 50 cm, 0 : petit)
//   B x y z         position de la balle visee (une seule fois suffit : elle est gardee jusqu'au depot)
//   R               position home : retour au repos (bras replie au-dessus du robot), voir aller_repos()
// Sortie : une ligne d'etat par pas ("E q1 q2 q3 q4 pince") ;
//          "D" quand la pince se rouvre en hauteur apres une prise (balle deposee) : le bras se replie
//          et le programme du robot (suivi de ligne) peut reprendre la main ;
//          "H" quand le bras est arrive en position home (apres "D" ou "R").
#include <ESP32Servo.h>
#include "cerveau.h"
#include "reseau_poids.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---- A REGLER sur le vrai bras ----------------------------------------------------------------------------
// Pas de broche 12 : c'est une broche de demarrage (si un servo la tire a 1, l'ESP32 ne demarre pas).
const int BROCHE[5] = {13, 25, 14, 27, 26};              // lacet, epaule, coude, poignet, pince
// Chaque servo couvre la fenetre [BUTEE_BAS, BUTEE_HAUT] de son axe (180 degres). SENS = -1 si le servo est
// monte a l'envers ; DECALAGE_DEG corrige le calage du palonnier (mesurer : consigne repos -> bras au repos ?).
const int SENS[5] = {1, 1, 1, 1, 1};
const float DECALAGE_DEG[5] = {0, 0, 0, 0, 0};
const float PINCE_CENTRE = 0.0f;                         // angle du pignon (rad) quand le servo de pince est a 90 deg
// ------------------------------------------------------------------------------------------------------------

Servo servos[5];
EtatBras etat;
float balle[3] = {0, 0, 0}, cible[3] = {0, 0, 0};
int grand = 0, balle_vue = 0, cible_connue = 0;
int serree = 0;                                          // la pince s'est refermee (balle prise ?)
const float Z_DEPOT_MINI = 0.12f;                        // rouvrir plus haut que ca = lacher dans un receptacle
unsigned long prochain = 0;
const unsigned long PERIODE_MS = 100;                    // 10 decisions par seconde, comme a l'entrainement

// Memoire des consignes envoyees par le reseau : le retour au repos refait ce chemin a l'envers (il n'a rien touche
// a l'aller). Un retour direct heurte le bord du grand receptacle une fois sur quatre (essai_retour_repos.py).
const int N_HISTO = 300;                                 // 30 s de consignes (4,8 Ko)
const float R_RENTRE = 0.10f, Z_RENTRE = 0.31f;          // passage interieur bas : pince a < 10 cm de l'axe, < 31 cm
float histo[N_HISTO][4];
int n_histo = 0, debut_histo = 0;

static float vers_degres(int i, float q) {
    float deg;
    if (i < 4) deg = (q - BUTEE_BAS[i]) / (BUTEE_HAUT[i] - BUTEE_BAS[i]) * 180.0f;   // fenetre de l'axe -> 0..180
    else deg = 90.0f + (q - PINCE_CENTRE) * 180.0f / (float)M_PI;
    if (SENS[i] < 0) deg = 180.0f - deg;
    deg += DECALAGE_DEG[i];
    return deg < 0 ? 0 : (deg > 180 ? 180 : deg);
}

// En microsecondes (500-2500 us = 0-180 deg) : ~0,1 deg de resolution, au lieu du degre entier de write()
// (1 deg d'epaule = 4 mm au bout du bras).
static int vers_us(float deg) { return (int)lroundf(500.0f + deg * 2000.0f / 180.0f); }

static void commander() {
    for (int i = 0; i < 4; i++) servos[i].writeMicroseconds(vers_us(vers_degres(i, etat.q[i])));
    servos[4].writeMicroseconds(vers_us(vers_degres(4, etat.servo)));
}

static void memoriser() {                                // garde les N_HISTO dernieres consignes (anneau)
    int i = (debut_histo + n_histo) % N_HISTO;
    memcpy(histo[i], etat.q, sizeof(etat.q));
    if (n_histo < N_HISTO) n_histo++;
    else debut_histo = (debut_histo + 1) % N_HISTO;
}

// POSITION HOME : ramene le bras dans la posture de repos (replie au-dessus du robot), sans a-coup ni choc.
//  1. refait a l'envers le chemin du reseau jusqu'au passage interieur bas (pres de l'axe, sous 31 cm) ;
//  2. epaule, coude et poignet vont vers le repos (0,06 rad par pas au plus, comme le reseau) ;
//  3. puis le lacet. La pince garde son etat (une balle tenue n'est pas lachee).
// Appelee par l'ordre serie "R" et apres chaque depose ("D"). Bloque ~2 a 15 s.
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

static void lire_serie() {
    // 1. Lecture USB d'origine
    static char ligne[96];
    static int n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && n < (int)sizeof(ligne) - 1) { ligne[n++] = c; continue; }
        ligne[n] = 0;
        n = 0;
        if (ligne[0] == 'B' && sscanf(ligne + 1, "%f %f %f", &balle[0], &balle[1], &balle[2]) == 3) balle_vue = 1;
        else if (ligne[0] == 'C' && sscanf(ligne + 1, "%f %f %f %d", &cible[0], &cible[1], &cible[2], &grand) == 4)
            cible_connue = 1;
        else if (ligne[0] == 'R') { balle_vue = cible_connue = serree = 0; aller_repos(); Serial.println("H"); }
    }

    // 2. Ajout de la lecture Bluetooth (Séquences A, B, C, H)
    if (SerialBT.available()) {
        char cmd = SerialBT.read();
        
        // --- POSITION MAISON ---
        if (cmd == 'H' || cmd == 'h') {
            balle_vue = cible_connue = serree = 0; 
            aller_repos(); 
            Serial.println("H"); // Signal USB de retour effectué
            SerialBT.println("Retour position maison effectue.");
        }
        // --- SÉQUENCE BALLE 1 ---
        else if (cmd == 'A' || cmd == 'a') {
            SerialBT.println("Lancement Balle 1");
            // Remplacer ces valeurs par les coordonnées de la Balle 1 (X, Y, Z) en mètres
            balle[0] = 0.15; balle[1] = 0.10; balle[2] = 0.02; 
            // Remplacer ces valeurs par les coordonnées du Panier 1 (X, Y, Z, Grand)
            cible[0] = -0.10; cible[1] = 0.20; cible[2] = 0.15; grand = 0;
            cible_connue = 1; balle_vue = 1; 
        }
        // --- SÉQUENCE BALLE 2 ---
        else if (cmd == 'B' || cmd == 'b') {
            SerialBT.println("Lancement Balle 2");
            balle[0] = 0.20; balle[1] = 0.05; balle[2] = 0.02; 
            cible[0] = -0.15; cible[1] = 0.15; cible[2] = 0.15; grand = 1;
            cible_connue = 1; balle_vue = 1; 
        }
        // --- SÉQUENCE BALLE 3 ---
        else if (cmd == 'C' || cmd == 'c') {
            SerialBT.println("Lancement Balle 3");
            balle[0] = 0.10; balle[1] = -0.15; balle[2] = 0.02; 
            cible[0] = -0.05; cible[1] = 0.25; cible[2] = 0.15; grand = 0;
            cible_connue = 1; balle_vue = 1; 
        }
    }
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("Bras_Robot_ICAM"); // Démarrage du Bluetooth
    
    for (int i = 0; i < 5; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(BROCHE[i], 500, 2500);
    }
    cerveau_repos(&etat);
    commander();
    Serial.println("bras 50 cm pret (reseau final 30/10) : envoyer C (receptacle) puis B (balle) en USB, ou A, B, C, H via Bluetooth");
}

void loop() {
    lire_serie();
    if (millis() < prochain) return;
    prochain = millis() + PERIODE_MS;
    
    if (balle_vue && cible_connue) {
        float action[N_ACT];
        unsigned long t0 = micros();
        
        // Le réseau de neurones calcule le prochain mouvement
        cerveau_pas(&etat, balle, cible, grand, action);
        commander();
        memoriser();
        
        Serial.printf("E %.3f %.3f %.3f %.3f %.3f (%lu us)\n", etat.q[0], etat.q[1], etat.q[2], etat.q[3], etat.servo, micros() - t0);
        
        // fin de la depose : pince rouverte en hauteur apres une prise (rouverte pres du sol = nouvel essai de prise)
        float outil[3];
        cerveau_outil(etat.q, outil);
        if (etat.servo <= SERVO_CONTACT) serree = 1;
        else if (serree && outil[2] > Z_DEPOT_MINI) {
            serree = balle_vue = cible_connue = 0;
            Serial.println("D");
            delay(500);                                  // la balle tombe dans le receptacle
            aller_repos();
            Serial.println("H");                         // bras replie : le robot peut rouler
        }
    }
}