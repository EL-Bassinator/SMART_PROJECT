// =========================================================================
// TEST 4 - MOTEURS EN BOUCLE CONTINUE (FONCTIONNEL)
// Les 2 moteurs tournent en avant en continu
//
// BRANCHEMENTS Pont en H TB6612FNG :
//   VCC  → 5V
//   VM   → Batterie + (9V)
//   GND  → GND (commun Arduino + batterie)
//   STBY → Pin 8
//   AIN1 → Pin 2, AIN2 → Pin 3, PWMA → Pin 5  (moteur gauche)
//   BIN1 → Pin 4, BIN2 → Pin 7, PWMB → Pin 6  (moteur droit)
// =========================================================================

const int AIN1 = 2, AIN2 = 3, PWMA = 5;
const int BIN1 = 4, BIN2 = 7, PWMB = 6;
const int STBY = 8;

const int VITESSE = 180; // 0-255

void setup() {
  Serial.begin(115200);

  // Configuration pins en sortie
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Active le pont en H
  digitalWrite(STBY, HIGH);

  // Direction : les 2 moteurs en AVANT
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  // Vitesse PWM
  analogWrite(PWMA, VITESSE);
  analogWrite(PWMB, VITESSE);

  Serial.println(F("============================================"));
  Serial.println(F("   TEST MOTEURS EN BOUCLE CONTINUE         "));
  Serial.println(F("============================================"));
  Serial.print(F("Vitesse : ")); Serial.print(VITESSE); Serial.println(F("/255"));
  Serial.println(F("Les 2 moteurs tournent EN AVANT en continu"));
  Serial.println(F("Appuyer sur RESET pour arreter"));
  Serial.println(F("============================================"));
}

void loop() {
  delay(2000);
  Serial.println(F("... moteurs en rotation ..."));
}
