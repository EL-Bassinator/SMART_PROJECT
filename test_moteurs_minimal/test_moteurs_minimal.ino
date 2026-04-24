// =========================================================================
// TEST MOTEURS - UNIQUEMENT DRIVER + ARDUINO
// =========================================================================

const int AIN1 = 8;
const int AIN2 = 9;
const int PWMA = 3;

const int BIN1 = 11;
const int BIN2 = 12;
const int PWMB = 5;

const int STBY = 10;

const int VITESSE = 255;

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  analogWrite(PWMA, VITESSE);
  analogWrite(PWMB, VITESSE);

  Serial.println(F("Moteurs en rotation..."));
}

void loop() {
  delay(2000);
  Serial.println(F("... OK ..."));
}
