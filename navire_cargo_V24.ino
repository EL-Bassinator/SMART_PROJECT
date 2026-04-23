// =========================================================================
// PROGRAMME : NAVIRE CARGO - VERSION COMMENTÉE COMPLÈTE
// CONFIGURATION : Chenal 48cm | Charges : 5kg, 6kg, 9kg
//
// ARCHITECTURE :
//   - Machine à états
//   - Utilisation de millis() (pas de delay bloquant)
//   - Régulateur PID pour maintien de trajectoire
//   - Vitesse nominale 170
//   - Filtre EMA sur les ultrasons
//   - Ratios RGB pour détection couleur
//   - 3 profils PID selon la charge transportée
// =========================================================================

// --- BIBLIOTHÈQUES ---
#include <Wire.h>               // Permet la communication I2C (boussole + capteur couleur)
#include <NewPing.h>            // Bibliothèque optimisée pour les capteurs à ultrasons
#include <Adafruit_TCS34725.h>  // Bibliothèque pour le capteur de couleur (lecture des feux)
#include <avr/wdt.h>            // Watchdog Timer (système anti-plantage de l'Arduino)
#include <PID_v1.h>             // Bibliothèque standard pour le régulateur PID
#include <math.h>               // Fonctions mathématiques (sqrt, atan2, sin, cos...)

// =========================================================================
// ZONE DE CONFIGURATION GÉNÉRALE
// =========================================================================

#define DEBUG_MODE true          // true = logs série actifs (tests), false = silencieux (compétition)

// Macros LOG qui disparaissent à la compilation si DEBUG_MODE = false
#if DEBUG_MODE
  #define LOG(x)   Serial.print(x)    // LOG écrit sur le moniteur série
  #define LOGLN(x) Serial.println(x)  // LOGLN écrit + saut de ligne
#else
  #define LOG(x)                       // En mode compétition, LOG ne fait rien (économie Flash)
  #define LOGLN(x)                     // Idem pour LOGLN
#endif

const unsigned long TIMEOUT_NAV = 30000; // Sécurité : freinage d'urgence après 30s (en millisecondes)
const float CAP_CIBLE = 68.0;            // Direction cible du bateau (en degrés par rapport au nord)
const uint8_t pwmMax  = 255;             // Puissance PWM maximale (valeur de 0 à 255)

// =========================================================================
// PINS DE L'ARDUINO
// =========================================================================

// Moteur Gauche (A) sur le pont en H TB6612FNG
const int AIN1 = 2;  // Broche direction 1 du moteur gauche
const int AIN2 = 3;  // Broche direction 2 du moteur gauche
const int PWMA = 5;  // Broche vitesse PWM du moteur gauche (pin ~ compatible PWM)

// Moteur Droit (B) sur le pont en H TB6612FNG
const int BIN1 = 4;  // Broche direction 1 du moteur droit
const int BIN2 = 7;  // Broche direction 2 du moteur droit
const int PWMB = 6;  // Broche vitesse PWM du moteur droit (pin ~ compatible PWM)

const int STBY = 8;  // Broche "Standby" du pont en H (doit être à HIGH pour activer les moteurs)

// =========================================================================
// OBJETS CAPTEURS
// =========================================================================

// Initialisation des ultrasons : (broche Trig, broche Echo, distance max en cm)
NewPing sonarG(A3, A2, 45); // Ultrason GAUCHE : TRIG=A3, ECHO=A2, portée 45cm max
NewPing sonarD(A0, A1, 45); // Ultrason DROIT  : TRIG=A0, ECHO=A1, portée 45cm max

// Capteur de couleur : temps d'intégration court (2.4ms) pour être réactif, gain x16 pour bien capter la lumière
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

// Adresses I2C des capteurs (chaque composant I2C a un identifiant unique)
const uint8_t ICM_ADDR = 0x68; // Adresse de la boussole ICM-20948 (accéléromètre + gyroscope)
const uint8_t MAG_ADDR = 0x0C; // Adresse du magnétomètre AK09916 intégré dans l'ICM-20948

// =========================================================================
// VARIABLES DU RÉGULATEUR PID
// =========================================================================

double capInput;          // Variable d'entrée du PID : l'erreur de cap actuelle
double capOutput;         // Variable de sortie du PID : la correction à appliquer aux moteurs
double capSetpoint = 0;   // Objectif du PID : atteindre une erreur de 0

// Paramètres initiaux du PID (seront ajustés en temps réel selon la charge)
double Kp = 2.4;          // Kp = gain Proportionnel (force de braquage immédiate)
double Ki = 0.01;         // Ki = gain Intégral (corrige la dérive persistante)
double Kd = 1.2;          // Kd = gain Dérivé (amortit les oscillations / anti-zigzag)

// Création de l'objet PID avec ses paramètres
PID boussolePID(&capInput, &capOutput, &capSetpoint, Kp, Ki, Kd, DIRECT);

int tempsFreinage = 1000; // Temps de marche arrière pour freiner (en ms, sera ajusté selon la charge)

// =========================================================================
// VARIABLES DU FILTRE EMA (MOYENNE MOBILE EXPONENTIELLE)
// =========================================================================

float dG_filtre = 45.0;    // Distance gauche lissée initiale (45cm = aucun obstacle)
float dD_filtre = 45.0;    // Distance droite lissée initiale
const float alphaEMA = 0.3; // Poids du filtre : 30% nouvelle valeur + 70% ancienne valeur

// =========================================================================
// VARIABLES DE LA FUSION MAHONY (calcul du cap 9 axes)
// =========================================================================

// Quaternion d'orientation : représente la rotation 3D du bateau en valeurs mathématiques
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;  // Commence à l'identité (aucune rotation)

const float Kp_mahony = 2.0;   // Gain proportionnel Mahony : confiance envers la boussole
const float Ki_mahony = 0.005; // Gain intégral Mahony : correction de la dérive lente

float eix = 0, eiy = 0, eiz = 0; // Intégrateurs d'erreur Mahony (3 axes)

unsigned long derniereTempsMahony     = 0; // Timestamp du dernier calcul Mahony (pour calculer dt)
unsigned long derniereLectureBoussole = 0; // Timestamp de la dernière lecture boussole réussie

// =========================================================================
// VARIABLES DE LA MACHINE À ÉTATS
// =========================================================================

// Liste de tous les états possibles de la mission (comme un diagramme d'états)
enum Etat {
  ATTENTE_FEU_A,   // Départ, attend le feu vert en zone A
  VERS_B,          // Navigation de A vers B
  FREINAGE_B,      // Arrêt d'urgence en zone B
  ATTENTE_FEU_B,   // Attend le feu vert en zone B
  VERS_C,          // Navigation de B vers C
  FREINAGE_C,      // Arrêt d'urgence en zone C
  DECHARGEMENT_C,  // Pause de 5s pour simuler le déchargement
  FIN_MISSION      // Mission terminée (3 voyages effectués)
};

Etat etatActuel = ATTENTE_FEU_A;  // On démarre par l'état d'attente du feu vert A
int voyagesTermines  = 0;         // Compteur de voyages effectués (0, 1, 2 → fin à 3)
unsigned long chrono    = 0;      // Chronomètre pour le freinage et le déchargement
unsigned long chronoNav = 0;      // Chronomètre pour la sécurité des 30 secondes
float capActuel = 0;              // Cap réel du bateau (mis à jour par Mahony)

// =========================================================================
// FONCTION D'INITIALISATION DE LA BOUSSOLE ICM-20948
// =========================================================================
void initICM() {
  // Étape 1 : Reset de la puce (PWR_MGMT_1 register 0x06 avec valeur 0x01)
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x06); Wire.write(0x01);
  Wire.endTransmission();
  delay(100); // Attente du reset

  // Étape 2 : Sortie du mode sleep (valeur 0x00 = mode normal)
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x06); Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  // Étape 3 : Sélection de la banque de registres 0 (USER_BANK)
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x7F); Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // Étape 4 : Active le bypass I2C pour pouvoir parler directement au magnétomètre
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x0F); Wire.write(0x02); // INT_PIN_CFG : BYPASS_EN = 1
  Wire.endTransmission();
  delay(10);

  // Étape 5 : Configure le magnétomètre AK09916 en mode mesure continue 100Hz
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x31); Wire.write(0x08); // CNTL2 register, MODE = 0x08
  Wire.endTransmission();
  delay(10);

  // Étape 6 : Activation du gyroscope en mode normal
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x01); Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

// =========================================================================
// SETUP : S'EXÉCUTE UNE SEULE FOIS AU DÉMARRAGE
// =========================================================================
void setup() {
  Serial.begin(115200);   // Démarre le moniteur série à 115200 bauds
  Wire.begin();           // Démarre le bus I2C (pour boussole + capteur couleur)
  Wire.setClock(400000);  // Accélère l'I2C en mode Fast (400 kHz) pour plus de réactivité

  // --- 1. SÉCURITÉ MOTEURS ---
  // On configure les pins en sortie AVANT d'activer le pont en H pour éviter tout mouvement involontaire
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);    // Pins direction moteur gauche
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);    // Pins direction moteur droit
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);    // Pins vitesse PWM
  analogWrite(PWMA, 0);  analogWrite(PWMB, 0);     // Force le PWM à 0 (moteurs à l'arrêt)
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); // Direction neutre gauche
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); // Direction neutre droit
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);  // Active le pont en H (seulement maintenant)
  LOGLN(F("Moteurs : securises"));

  // --- 2. CAPTEUR COULEUR ---
  if (!tcs.begin()) LOGLN(F("ERR: TCS non detecte"));  // Si le capteur ne répond pas → erreur
  else LOGLN(F("TCS : OK"));                            // Sinon tout va bien

  // --- 3. BOUSSOLE + MAGNÉTOMÈTRE ---
  initICM();                              // Appelle la fonction d'initialisation
  LOGLN(F("ICM + AK09916 : OK"));

  // --- 4. RÉGULATEUR PID ---
  boussolePID.SetMode(AUTOMATIC);          // Active les calculs automatiques du PID
  boussolePID.SetOutputLimits(-80, 80);    // Limite la correction PID entre -80 et +80
  boussolePID.SetSampleTime(20);           // Le PID recalcule toutes les 20ms
  LOGLN(F("PID : OK"));

  // --- 5. ATTENTE PREMIÈRE LECTURE BOUSSOLE ---
  // Important : on attend que la boussole donne une vraie valeur avant de démarrer
  LOGLN(F("Attente boussole..."));
  derniereTempsMahony = micros();          // Initialise le timer Mahony
  unsigned long tBoot = millis();          // Démarre un timer de 5 secondes max
  while (capActuel == 0 && millis() - tBoot < 5000) {
    lireCapMahony();                       // Tente de lire le cap
    wdt_reset();                           // Nourrit le watchdog pendant l'attente
  }
  if (capActuel != 0) {
    LOG(F("Cap initial : ")); LOGLN(capActuel);
  } else {
    LOGLN(F("WARN: Boussole pas prete"));  // Si pas de lecture en 5s, on continue quand même
  }

  // --- 6. WATCHDOG TIMER ---
  // Active le système anti-plantage : si le code bloque plus de 2s, l'Arduino redémarre
  wdt_enable(WDTO_2S);

  LOGLN(F("============================================"));
  LOGLN(F("  PRET - ATTENTE FEU VERT A                "));
  LOGLN(F("============================================"));
}

// =========================================================================
// LOOP : S'EXÉCUTE EN BOUCLE À L'INFINI
// =========================================================================
void loop() {
  wdt_reset();  // On "nourrit" le watchdog : on lui signale que le code tourne bien

  if (etatActuel == FIN_MISSION) return;  // Si mission terminée, on bloque la boucle ici

  // --- 1. LECTURE DU CAP ---
  lireCapMahony();  // Met à jour 'capActuel' via la fusion Mahony 9 axes

  // --- 2. LECTURE ET FILTRAGE DES ULTRASONS ---
  int brutG = sonarG.ping_cm(); if (brutG == 0) brutG = 45; // 0 = hors portée, on met 45cm
  int brutD = sonarD.ping_cm(); if (brutD == 0) brutD = 45; // Idem pour le capteur droit

  // Application du filtre EMA (moyenne mobile exponentielle)
  // 30% de la nouvelle mesure + 70% de l'ancienne valeur lissée
  dG_filtre = (alphaEMA * brutG) + ((1.0 - alphaEMA) * dG_filtre);
  dD_filtre = (alphaEMA * brutD) + ((1.0 - alphaEMA) * dD_filtre);

  // --- 3. ADAPTATION DU PROFIL PID SELON LA CHARGE ---
  // Plus le bateau est lourd, plus il a d'inertie → il faut un braquage plus agressif
  if (voyagesTermines == 0) {             // MISSION 1 : GRAVIER (5kg total)
    Kp = 2.4; tempsFreinage = 900;        // Braquage standard, freinage court
  } else if (voyagesTermines == 1) {      // MISSION 2 : EAU (6kg avec ballotement)
    Kp = 2.8; tempsFreinage = 1100;       // Braquage plus nerveux, freinage moyen
  } else {                                // MISSION 3 : ACIER (9kg, forte inertie)
    Kp = 3.8; tempsFreinage = 2000;       // Braquage très agressif, freinage long
  }
  boussolePID.SetTunings(Kp, Ki, Kd);     // Envoie les nouveaux paramètres au PID

  // --- 4. LOG DE DEBUG ---
  LOG(F("ETAT:")); LOG(etatActuel);       // Affiche l'état actuel
  LOG(F(" CAP:"));  LOG(capActuel);        // Affiche le cap actuel
  LOG(F(" CG:"));   LOG(dG_filtre);        // Affiche la distance gauche filtrée
  LOG(F(" CD:"));   LOGLN(dD_filtre);      // Affiche la distance droite filtrée

  // --- 5. MACHINE À ÉTATS ---
  switch (etatActuel) {

    // ========== ÉTAT 1 : ATTENTE DU FEU VERT EN ZONE A ==========
    case ATTENTE_FEU_A:
      moteur(0, 0);                        // Moteurs à l'arrêt
      if (detecterVertIntense()) {         // Si on voit le feu vert...
        LOGLN(F("Feu vert A -> VERS_B"));
        chronoNav = millis();              // Démarre le chrono de sécurité (30s max)
        etatActuel = VERS_B;               // Transition vers l'état suivant
      }
      break;

    // ========== ÉTAT 2 : NAVIGATION DE A VERS B ==========
    case VERS_B:
      piloter_PID(180);                    // Navigation à vitesse limitée à 180/255
      if (detecterRougeIntense()) {        // Si on voit le feu rouge → arrivée en B
        LOGLN(F("Feu rouge B -> FREINAGE_B"));
        chrono = millis();                 // Démarre le chrono de freinage
        etatActuel = FREINAGE_B;
      }
      // Sécurité : si après 30s on n'a pas détecté le rouge, on freine quand même
      if (millis() - chronoNav > TIMEOUT_NAV) {
        LOGLN(F("WARN: Timeout VERS_B"));
        chrono = millis();
        etatActuel = FREINAGE_B;
      }
      break;

    // ========== ÉTAT 3 : FREINAGE D'URGENCE EN ZONE B ==========
    case FREINAGE_B:
      moteur(-180, -180);                  // Marche arrière pour contrer l'élan
      if (millis() - chrono > tempsFreinage) { // Après le temps défini...
        LOGLN(F("Freinage B OK"));
        etatActuel = ATTENTE_FEU_B;        // ...passage à l'attente du prochain feu
      }
      break;

    // ========== ÉTAT 4 : ATTENTE DU FEU VERT EN ZONE B ==========
    case ATTENTE_FEU_B:
      moteur(0, 0);                        // Arrêt complet sous le portique B
      if (detecterVertIntense()) {         // Si feu vert en B...
        LOGLN(F("Feu vert B -> VERS_C"));
        chronoNav = millis();              // Nouveau chrono de sécurité
        etatActuel = VERS_C;               // Départ vers C
      }
      break;

    // ========== ÉTAT 5 : NAVIGATION DE B VERS C (LIGNE DROITE FINALE) ==========
    case VERS_C:
      piloter_PID(pwmMax);                 // Vitesse maximale autorisée (255)
      if (detecterRougeIntense()) {        // Arrivée en C
        LOGLN(F("Feu rouge C -> FREINAGE_C"));
        chrono = millis();
        etatActuel = FREINAGE_C;
      }
      if (millis() - chronoNav > TIMEOUT_NAV) { // Sécurité timeout
        LOGLN(F("WARN: Timeout VERS_C"));
        chrono = millis();
        etatActuel = FREINAGE_C;
      }
      break;

    // ========== ÉTAT 6 : FREINAGE D'URGENCE EN ZONE C ==========
    case FREINAGE_C:
      moteur(-pwmMax, -pwmMax);            // Freinage maximal (plus d'inertie à contrer)
      if (millis() - chrono > tempsFreinage) {
        LOGLN(F("Freinage C OK"));
        chrono = millis();                 // Redémarre le chrono pour le déchargement
        etatActuel = DECHARGEMENT_C;
      }
      break;

    // ========== ÉTAT 7 : DÉCHARGEMENT EN ZONE C ==========
    case DECHARGEMENT_C:
      moteur(0, 0);                        // Arrêt complet pendant le déchargement
      if (millis() - chrono > 5000) {      // Après 5 secondes de pause...
        voyagesTermines++;                 // Incrémente le compteur de voyages
        LOG(F("Voyage : ")); LOGLN(voyagesTermines);
        if (voyagesTermines >= 3) {        // Si 3 voyages effectués...
          etatActuel = FIN_MISSION;        // ...mission terminée
          digitalWrite(STBY, LOW);         // Coupe le pont en H par sécurité
          LOGLN(F("MISSION COMPLETE !"));
        } else {
          etatActuel = ATTENTE_FEU_B;      // Sinon, retour à l'attente en B
        }
      }
      break;

    // ========== ÉTAT 8 : FIN DE MISSION ==========
    case FIN_MISSION:
      // Ne fait rien, le 'return' en haut de loop() bloque toute action
      break;
  }
}

// =========================================================================
// FONCTION DE PILOTAGE AVEC PID + ÉVITEMENT ULTRASONS
// =========================================================================
void piloter_PID(int limite) {
  // --- 1. CALCUL DE L'ERREUR DE CAP ---
  float erreur = capActuel - CAP_CIBLE;   // Ex: si je suis à 78° et cible 68°, erreur = +10°

  // Correction mathématique pour toujours tourner du côté le plus court (360° cyclique)
  if (erreur > 180)  erreur -= 360;
  if (erreur < -180) erreur += 360;

  // --- 2. CALCUL DE LA CORRECTION PAR LE PID ---
  capInput = erreur;                       // On donne l'erreur au PID
  boussolePID.Compute();                   // Le PID calcule et remplit capOutput

  // --- 3. VITESSE DE BASE ---
  int vitesseBase = 170;                   // Vitesse nominale : 170/255 (laisse de la marge)
  int evit = 0;                            // Force d'évitement latérale (init 0)

  // --- 4. ÉVITEMENT DES PAROIS ---
  // Si mur GAUCHE trop proche (<16cm), on ralentit et on pousse vers la DROITE
  if (dG_filtre < 16) {
    evit += map((int)dG_filtre, 7, 16, 85, 0);            // Force répulsive
    vitesseBase = map((int)dG_filtre, 7, 16, 100, 170);   // Ralentit pour mieux braquer
  }
  // Si mur DROIT trop proche, on pousse vers la GAUCHE (evit négatif)
  if (dD_filtre < 16) {
    evit -= map((int)dD_filtre, 7, 16, 85, 0);
    vitesseBase = min(vitesseBase, map((int)dD_filtre, 7, 16, 100, 170));
  }

  // --- 5. CALCUL FINAL DES PWM ---
  const int vitesseMin = 80;               // Aucun moteur ne descend sous 80 (pas de blocage)

  // Combinaison : vitesse base + correction PID + évitement
  int cmdG = constrain(vitesseBase + (int)capOutput + evit, vitesseMin, limite);
  int cmdD = constrain(vitesseBase - (int)capOutput - evit, vitesseMin, limite);

  moteur(cmdG, cmdD);                      // Envoie les commandes aux moteurs
}

// =========================================================================
// DÉTECTION DU FEU VERT (ratios RGB)
// =========================================================================
bool detecterVertIntense() {
  uint16_t r, g, b, c;                    // Variables Red, Green, Blue, Clear (luminosité totale)
  tcs.getRawData(&r, &g, &b, &c);         // Lit les 4 valeurs du capteur
  if (c < 1000) return false;             // Si trop sombre, on ignore (évite faux positifs)
  return ((float)g / c > 0.45);           // Si plus de 45% de vert → c'est le feu vert !
}

// =========================================================================
// DÉTECTION DU FEU ROUGE (ratios RGB)
// =========================================================================
bool detecterRougeIntense() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 1000) return false;
  return ((float)r / c > 0.50);           // Si plus de 50% de rouge → c'est le feu rouge
}

// =========================================================================
// CONTRÔLE DES MOTEURS
// speedA = vitesse moteur gauche, speedB = vitesse moteur droit
// Valeurs positives = marche avant, négatives = marche arrière
// =========================================================================
void moteur(int speedA, int speedB) {
  // --- MOTEUR GAUCHE ---
  if (speedA >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);   // Configuration marche AVANT
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);  // Configuration marche ARRIÈRE
    speedA = -speedA;                                    // PWM toujours positif, on inverse
  }

  // --- MOTEUR DROIT ---
  if (speedB >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);   // Marche AVANT
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);  // Marche ARRIÈRE
    speedB = -speedB;
  }

  // --- ENVOI DES COMMANDES PWM ---
  analogWrite(PWMA, speedA);              // Vitesse moteur gauche
  analogWrite(PWMB, speedB);              // Vitesse moteur droit
}

// =========================================================================
// LECTURE DE L'IMU AVEC FUSION MAHONY 9 AXES
// Fusionne accéléromètre + gyroscope + magnétomètre pour obtenir un cap précis
// =========================================================================
void lireCapMahony() {

  // --- 1. LECTURE ACCÉLÉROMÈTRE (ICM-20948, big-endian) ---
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x2D);                       // Registre de départ accéléromètre
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)6); // On demande 6 octets (2 par axe)
  if (Wire.available() < 6) return;       // Si données incomplètes, on abandonne

  int16_t axRaw = (Wire.read() << 8) | Wire.read(); // X (16 bits sur 2 octets)
  int16_t ayRaw = (Wire.read() << 8) | Wire.read(); // Y
  int16_t azRaw = (Wire.read() << 8) | Wire.read(); // Z

  // --- 2. LECTURE GYROSCOPE (ICM-20948, big-endian) ---
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x33);                       // Registre de départ gyroscope
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return;

  int16_t gxRaw = (Wire.read() << 8) | Wire.read();
  int16_t gyRaw = (Wire.read() << 8) | Wire.read();
  int16_t gzRaw = (Wire.read() << 8) | Wire.read();

  // --- 3. LECTURE MAGNÉTOMÈTRE (AK09916, little-endian, adresse séparée) ---
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x11);                       // Registre de départ magnétomètre
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)8); // 6 octets de données + 2 de statut
  if (Wire.available() < 8) return;

  int16_t mxRaw = Wire.read() | (Wire.read() << 8); // Little-endian pour AK09916 !
  int16_t myRaw = Wire.read() | (Wire.read() << 8);
  int16_t mzRaw = Wire.read() | (Wire.read() << 8);
  Wire.read(); Wire.read();               // Octets de statut ST2 (à lire pour finir)

  // --- 4. CONVERSION EN UNITÉS PHYSIQUES ---
  float ax = axRaw / 16384.0;             // Accéléro ±2g : 16384 LSB par g
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;

  float gx = gxRaw / 131.0 * (PI / 180.0); // Gyro ±250°/s : 131 LSB par °/s, conversion en rad/s
  float gy = gyRaw / 131.0 * (PI / 180.0);
  float gz = gzRaw / 131.0 * (PI / 180.0);

  float mx = (float)mxRaw;                 // Magnéto : valeurs brutes (on les normalisera)
  float my = (float)myRaw;
  float mz = (float)mzRaw;

  // --- 5. CALCUL DU PAS DE TEMPS dt ---
  unsigned long maintenant = micros();
  float dt = (maintenant - derniereTempsMahony) / 1000000.0; // dt en secondes
  derniereTempsMahony = maintenant;
  if (dt <= 0 || dt > 0.5) return;        // Ignore si dt aberrant (premier appel ou pause)

  // --- 6. NORMALISATION DES VECTEURS ---
  // Tout ramener à une longueur de 1 pour les calculs trigonométriques
  float normA = sqrt(ax*ax + ay*ay + az*az);
  if (normA == 0) return;
  ax /= normA; ay /= normA; az /= normA;

  float normM = sqrt(mx*mx + my*my + mz*mz);
  if (normM == 0) return;
  mx /= normM; my /= normM; mz /= normM;

  // --- 7. CALCUL DE LA DIRECTION MAGNÉTIQUE DANS LE RÉFÉRENTIEL TERRESTRE ---
  // Projection du vecteur magnétique dans le plan horizontal (compensation d'inclinaison)
  float hx = 2.0*(mx*(0.5-q2*q2-q3*q3) + my*(q1*q2-q0*q3) + mz*(q1*q3+q0*q2));
  float hy = 2.0*(mx*(q1*q2+q0*q3) + my*(0.5-q1*q1-q3*q3) + mz*(q2*q3-q0*q1));
  float bx = sqrt(hx*hx + hy*hy);         // Composante horizontale du champ magnétique
  float bz = 2.0*(mx*(q1*q3-q0*q2) + my*(q2*q3+q0*q1) + mz*(0.5-q1*q1-q2*q2));

  // --- 8. CALCUL DE L'ERREUR MAHONY (produit vectoriel) ---
  // Vecteur gravité estimé
  float vx = 2.0*(q1*q3-q0*q2);
  float vy = 2.0*(q0*q1+q2*q3);
  float vz = q0*q0-q1*q1-q2*q2+q3*q3;

  // Vecteur nord magnétique estimé
  float wx = 2.0*(bx*(0.5-q2*q2-q3*q3) + bz*(q1*q3-q0*q2));
  float wy = 2.0*(bx*(q1*q2-q0*q3) + bz*(q0*q1+q2*q3));
  float wz = 2.0*(bx*(q0*q2+q1*q3) + bz*(0.5-q1*q1-q2*q2));

  // Somme des erreurs (accéléro + magnéto) via produit vectoriel
  float ex = (ay*vz-az*vy) + (my*wz-mz*wy);
  float ey = (az*vx-ax*vz) + (mz*wx-mx*wz);
  float ez = (ax*vy-ay*vx) + (mx*wy-my*wx);

  // --- 9. CORRECTION DU GYROSCOPE AVEC L'ERREUR ---
  // Intégration de l'erreur (Ki) pour compenser la dérive lente
  eix += ex * Ki_mahony;
  eiy += ey * Ki_mahony;
  eiz += ez * Ki_mahony;

  // Application de la correction proportionnelle immédiate (Kp) + intégrale
  gx += Kp_mahony*ex + eix;
  gy += Kp_mahony*ey + eiy;
  gz += Kp_mahony*ez + eiz;

  // --- 10. INTÉGRATION : MISE À JOUR DU QUATERNION ---
  float dq0 = 0.5*(-q1*gx-q2*gy-q3*gz)*dt;
  float dq1 = 0.5*( q0*gx+q2*gz-q3*gy)*dt;
  float dq2 = 0.5*( q0*gy-q1*gz+q3*gx)*dt;
  float dq3 = 0.5*( q0*gz+q1*gy-q2*gx)*dt;

  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;  // Mise à jour

  // Renormalisation : le quaternion doit toujours avoir une longueur de 1
  float normQ = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0 /= normQ; q1 /= normQ; q2 /= normQ; q3 /= normQ;

  // --- 11. EXTRACTION DU CAP (YAW) DEPUIS LE QUATERNION ---
  float yaw = atan2(2.0*(q1*q2+q0*q3), q0*q0+q1*q1-q2*q2-q3*q3);
  float cap = yaw * 180.0 / PI;           // Conversion radians → degrés
  if (cap < 0) cap += 360.0;              // Normalise sur 0-360°

  capActuel = cap;                        // Met à jour la variable globale
  derniereLectureBoussole = millis();     // Note l'heure de cette lecture réussie

  // --- 12. PROTECTION CONTRE LE FREEZE DE LA BOUSSOLE ---
  // Si plus de 2 secondes sans lecture valide, on garde l'ancien cap
  if (derniereLectureBoussole != 0 && millis() - derniereLectureBoussole > 2000) {
    LOGLN(F("WARN: Boussole figee"));
    derniereLectureBoussole = millis();   // Reset pour pas spammer le log
  }
}
