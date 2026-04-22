// =========================================================================
// PROGRAMME : NAVIRE CARGO V24.0 - ÉDITION FINALE COMPÉTITION
// CONFIGURATION : Chenal 48cm | Charges : 5kg, 6kg, 9kg
// MODIFICATIONS V24.0 (par rapport à V23.4) :
//   - Bouton physique SUPPRIMÉ (démarrage direct sur feu vert A)
//   - Profil PID unique fixe (valeurs acier) pour les 3 charges
// =========================================================================

#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_TCS34725.h>
#include <avr/wdt.h>
#include <PID_v1.h>
#include <math.h>
#include <EEPROM.h>

// =========================================================================
// ZONE DE CONFIGURATION
// =========================================================================
#define DEBUG_MODE true

#if DEBUG_MODE
  #define LOG(x)   Serial.print(x)
  #define LOGLN(x) Serial.println(x)
#else
  #define LOG(x)
  #define LOGLN(x)
#endif

const unsigned long TIMEOUT_NAV = 30000;
const float CAP_CIBLE = 68.0;
const uint8_t pwmMax  = 255;

// =========================================================================
// PINS
// =========================================================================
const int AIN1 = 2, AIN2 = 3, PWMA = 5;
const int BIN1 = 4, BIN2 = 7, PWMB = 6;
const int STBY = 8;

// =========================================================================
// OBJETS CAPTEURS
// =========================================================================
NewPing sonarG(A3, A2, 45);
NewPing sonarD(A0, A1, 45);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

const uint8_t ICM_ADDR = 0x68;
const uint8_t MAG_ADDR = 0x0C;

// =========================================================================
// CORRECTIONS HARD IRON
// =========================================================================
const int EEPROM_ADDR_X = 0;
const int EEPROM_ADDR_Y = 4;
const int EEPROM_ADDR_Z = 8;
const int EEPROM_VALID  = 12;

float hardIronX = 0, hardIronY = 0, hardIronZ = 0;

// =========================================================================
// VARIABLES PID - PROFIL UNIQUE FIXE (paramètres agressifs acier)
// =========================================================================
double capInput, capOutput, capSetpoint = 0;
// PID réglé sur les valeurs les plus agressives (acier 9kg)
// Ça marchera aussi pour les charges plus légères, un peu nerveux mais stable
double Kp = 3.8, Ki = 0.01, Kd = 1.2;
PID boussolePID(&capInput, &capOutput, &capSetpoint, Kp, Ki, Kd, DIRECT);

// Temps de freinage fixe (valeur acier = le plus long pour être sûr de s'arrêter)
const int tempsFreinage = 2000;

// =========================================================================
// VARIABLES FILTRES ULTRASONS
// =========================================================================
float dG_filtre = 45.0, dD_filtre = 45.0;
const float alphaEMA = 0.3;

// =========================================================================
// VARIABLES FUSION MAHONY 9 AXES
// =========================================================================
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
const float Kp_mahony = 2.0;
const float Ki_mahony = 0.005;
float eix = 0, eiy = 0, eiz = 0;
unsigned long derniereTempsMahony     = 0;
unsigned long derniereLectureBoussole = 0;

// =========================================================================
// VARIABLES DE MISSION
// =========================================================================
enum Etat { ATTENTE_FEU_A, VERS_B, FREINAGE_B, ATTENTE_FEU_B, VERS_C, FREINAGE_C, DECHARGEMENT_C, FIN_MISSION };
Etat etatActuel = ATTENTE_FEU_A; // Démarrage direct sur feu vert (pas de bouton)
int voyagesTermines  = 0;
unsigned long chrono    = 0;
unsigned long chronoNav = 0;
float capActuel = 0;

// =========================================================================
// INITIALISATION ICM + MAGNÉTOMÈTRE
// =========================================================================
void initICM() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x06); Wire.write(0x01);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x06); Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x7F); Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x0F); Wire.write(0x02);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x31); Wire.write(0x08);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x01); Wire.write(0x00);
  Wire.endTransmission();
  delay(50);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Sécurité moteurs
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  analogWrite(PWMA, 0);  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  LOGLN(F("Moteurs : securises"));

  // Capteur couleur
  if (!tcs.begin()) LOGLN(F("ERR: TCS non detecte"));
  else LOGLN(F("TCS : OK"));

  // ICM + magnétomètre
  initICM();
  LOGLN(F("ICM + AK09916 : OK"));

  // Calibration Hard Iron
  if (EEPROM.read(EEPROM_VALID) == 0xAB) {
    EEPROM.get(EEPROM_ADDR_X, hardIronX);
    EEPROM.get(EEPROM_ADDR_Y, hardIronY);
    EEPROM.get(EEPROM_ADDR_Z, hardIronZ);
    LOGLN(F("Calibration chargee"));
  } else {
    LOGLN(F("WARN: Pas de calibration"));
    hardIronX = 0; hardIronY = 0; hardIronZ = 0;
  }

  // PID (profil fixe)
  boussolePID.SetMode(AUTOMATIC);
  boussolePID.SetOutputLimits(-80, 80);
  boussolePID.SetSampleTime(20);
  boussolePID.SetTunings(Kp, Ki, Kd);
  LOGLN(F("PID : OK (profil fixe)"));

  // Attente première lecture boussole
  LOGLN(F("Attente boussole..."));
  derniereTempsMahony = micros();
  unsigned long tBoot = millis();
  while (capActuel == 0 && millis() - tBoot < 5000) {
    lireCapMahony();
    wdt_reset();
  }
  if (capActuel != 0) {
    LOG(F("Cap initial : ")); LOGLN(capActuel);
  } else {
    LOGLN(F("WARN: Boussole pas prete"));
  }

  wdt_enable(WDTO_2S);
  LOGLN(F("============================================"));
  LOGLN(F("  V24.0 PRET - ATTENTE FEU VERT A          "));
  LOGLN(F("============================================"));
}

// =========================================================================
// BOUCLE PRINCIPALE
// =========================================================================
void loop() {
  wdt_reset();
  if (etatActuel == FIN_MISSION) return;

  lireCapMahony();

  int brutG = sonarG.ping_cm(); if (brutG == 0) brutG = 45;
  int brutD = sonarD.ping_cm(); if (brutD == 0) brutD = 45;
  dG_filtre = (alphaEMA * brutG) + ((1.0 - alphaEMA) * dG_filtre);
  dD_filtre = (alphaEMA * brutD) + ((1.0 - alphaEMA) * dD_filtre);

  LOG(F("ETAT:")); LOG(etatActuel);
  LOG(F(" CAP:"));  LOG(capActuel);
  LOG(F(" CG:"));   LOG(dG_filtre);
  LOG(F(" CD:"));   LOGLN(dD_filtre);

  switch (etatActuel) {

    case ATTENTE_FEU_A:
      moteur(0, 0);
      if (detecterVertIntense()) {
        LOGLN(F("Feu vert A -> VERS_B"));
        chronoNav = millis();
        etatActuel = VERS_B;
      }
      break;

    case VERS_B:
      piloter_PID(180);
      if (detecterRougeIntense()) {
        LOGLN(F("Feu rouge B -> FREINAGE"));
        chrono = millis();
        etatActuel = FREINAGE_B;
      }
      if (millis() - chronoNav > TIMEOUT_NAV) {
        LOGLN(F("WARN: Timeout VERS_B"));
        chrono = millis();
        etatActuel = FREINAGE_B;
      }
      break;

    case FREINAGE_B:
      moteur(-180, -180);
      if (millis() - chrono > tempsFreinage) {
        LOGLN(F("Freinage B OK"));
        etatActuel = ATTENTE_FEU_B;
      }
      break;

    case ATTENTE_FEU_B:
      moteur(0, 0);
      if (detecterVertIntense()) {
        LOGLN(F("Feu vert B -> VERS_C"));
        chronoNav = millis();
        etatActuel = VERS_C;
      }
      break;

    case VERS_C:
      piloter_PID(pwmMax);
      if (detecterRougeIntense()) {
        LOGLN(F("Feu rouge C -> FREINAGE"));
        chrono = millis();
        etatActuel = FREINAGE_C;
      }
      if (millis() - chronoNav > TIMEOUT_NAV) {
        LOGLN(F("WARN: Timeout VERS_C"));
        chrono = millis();
        etatActuel = FREINAGE_C;
      }
      break;

    case FREINAGE_C:
      moteur(-pwmMax, -pwmMax);
      if (millis() - chrono > tempsFreinage) {
        LOGLN(F("Freinage C OK"));
        chrono = millis();
        etatActuel = DECHARGEMENT_C;
      }
      break;

    case DECHARGEMENT_C:
      moteur(0, 0);
      if (millis() - chrono > 5000) {
        voyagesTermines++;
        LOG(F("Voyage : ")); LOGLN(voyagesTermines);
        if (voyagesTermines >= 3) {
          etatActuel = FIN_MISSION;
          digitalWrite(STBY, LOW);
          LOGLN(F("MISSION COMPLETE !"));
        } else {
          etatActuel = ATTENTE_FEU_B;
        }
      }
      break;

    case FIN_MISSION:
      break;
  }
}

// =========================================================================
// FONCTIONS SECONDAIRES
// =========================================================================

void piloter_PID(int limite) {
  float erreur = capActuel - CAP_CIBLE;
  if (erreur > 180)  erreur -= 360;
  if (erreur < -180) erreur += 360;

  capInput = erreur;
  boussolePID.Compute();

  int vitesseBase = 170;
  int evit = 0;

  if (dG_filtre < 16) {
    evit += map((int)dG_filtre, 7, 16, 85, 0);
    vitesseBase = map((int)dG_filtre, 7, 16, 100, 170);
  }
  if (dD_filtre < 16) {
    evit -= map((int)dD_filtre, 7, 16, 85, 0);
    vitesseBase = min(vitesseBase, map((int)dD_filtre, 7, 16, 100, 170));
  }

  const int vitesseMin = 80;
  int cmdG = constrain(vitesseBase + (int)capOutput + evit, vitesseMin, limite);
  int cmdD = constrain(vitesseBase - (int)capOutput - evit, vitesseMin, limite);
  moteur(cmdG, cmdD);
}

bool detecterVertIntense() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 500) return false;
  return ((float)g / c > 0.45);
}

bool detecterRougeIntense() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 500) return false;
  return ((float)r / c > 0.50);
}

void moteur(int speedA, int speedB) {
  if (speedA >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else             { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speedA = -speedA; }
  if (speedB >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else             { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); speedB = -speedB; }
  analogWrite(PWMA, speedA);
  analogWrite(PWMB, speedB);
}

void lireCapMahony() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x2D);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return;
  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  int16_t ayRaw = (Wire.read() << 8) | Wire.read();
  int16_t azRaw = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x33);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return;
  int16_t gxRaw = (Wire.read() << 8) | Wire.read();
  int16_t gyRaw = (Wire.read() << 8) | Wire.read();
  int16_t gzRaw = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x11);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)8);
  if (Wire.available() < 8) return;
  int16_t mxRaw = Wire.read() | (Wire.read() << 8);
  int16_t myRaw = Wire.read() | (Wire.read() << 8);
  int16_t mzRaw = Wire.read() | (Wire.read() << 8);
  Wire.read(); Wire.read();

  float ax = axRaw / 16384.0;
  float ay = ayRaw / 16384.0;
  float az = azRaw / 16384.0;
  float gx = gxRaw / 131.0 * (PI / 180.0);
  float gy = gyRaw / 131.0 * (PI / 180.0);
  float gz = gzRaw / 131.0 * (PI / 180.0);

  float mx = (float)mxRaw - hardIronX;
  float my = (float)myRaw - hardIronY;
  float mz = (float)mzRaw - hardIronZ;

  unsigned long maintenant = micros();
  float dt = (maintenant - derniereTempsMahony) / 1000000.0;
  derniereTempsMahony = maintenant;
  if (dt <= 0 || dt > 0.5) return;

  float normA = sqrt(ax*ax + ay*ay + az*az);
  if (normA == 0) return;
  ax /= normA; ay /= normA; az /= normA;

  float normM = sqrt(mx*mx + my*my + mz*mz);
  if (normM == 0) return;
  mx /= normM; my /= normM; mz /= normM;

  float hx = 2.0*(mx*(0.5-q2*q2-q3*q3) + my*(q1*q2-q0*q3) + mz*(q1*q3+q0*q2));
  float hy = 2.0*(mx*(q1*q2+q0*q3) + my*(0.5-q1*q1-q3*q3) + mz*(q2*q3-q0*q1));
  float bx = sqrt(hx*hx + hy*hy);
  float bz = 2.0*(mx*(q1*q3-q0*q2) + my*(q2*q3+q0*q1) + mz*(0.5-q1*q1-q2*q2));

  float vx = 2.0*(q1*q3-q0*q2);
  float vy = 2.0*(q0*q1+q2*q3);
  float vz = q0*q0-q1*q1-q2*q2+q3*q3;

  float wx = 2.0*(bx*(0.5-q2*q2-q3*q3) + bz*(q1*q3-q0*q2));
  float wy = 2.0*(bx*(q1*q2-q0*q3) + bz*(q0*q1+q2*q3));
  float wz = 2.0*(bx*(q0*q2+q1*q3) + bz*(0.5-q1*q1-q2*q2));

  float ex = (ay*vz-az*vy) + (my*wz-mz*wy);
  float ey = (az*vx-ax*vz) + (mz*wx-mx*wz);
  float ez = (ax*vy-ay*vx) + (mx*wy-my*wx);

  eix += ex * Ki_mahony;
  eiy += ey * Ki_mahony;
  eiz += ez * Ki_mahony;

  gx += Kp_mahony*ex + eix;
  gy += Kp_mahony*ey + eiy;
  gz += Kp_mahony*ez + eiz;

  float dq0 = 0.5*(-q1*gx-q2*gy-q3*gz)*dt;
  float dq1 = 0.5*( q0*gx+q2*gz-q3*gy)*dt;
  float dq2 = 0.5*( q0*gy-q1*gz+q3*gx)*dt;
  float dq3 = 0.5*( q0*gz+q1*gy-q2*gx)*dt;

  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

  float normQ = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0 /= normQ; q1 /= normQ; q2 /= normQ; q3 /= normQ;

  float yaw = atan2(2.0*(q1*q2+q0*q3), q0*q0+q1*q1-q2*q2-q3*q3);
  float cap = yaw * 180.0 / PI;
  if (cap < 0) cap += 360.0;

  capActuel = cap;
  derniereLectureBoussole = millis();

  if (derniereLectureBoussole != 0 && millis() - derniereLectureBoussole > 2000) {
    LOGLN(F("WARN: Boussole figee"));
    derniereLectureBoussole = millis();
  }
}
