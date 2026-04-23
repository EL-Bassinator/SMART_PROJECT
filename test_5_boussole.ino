// =========================================================================
// TEST 5 - BOUSSOLE ICM-20948 + MAGNÉTOMÈTRE AK09916 (FONCTIONNEL)
// Active correctement le magnétomètre via le bypass I2C
//
// BRANCHEMENTS ICM-20948 (Pimoroni PIM448) :
//   VCC → 3.3V  (IMPORTANT : pas 5V !)
//   GND → GND
//   SDA → A4
//   SCL → A5
//
// UTILISATION :
//   1. Téléverser
//   2. Ouvrir Moniteur Série à 115200 bauds
//   3. Tourner le module et observer le cap qui change
// =========================================================================

#include <Wire.h>

const uint8_t ICM_ADDR = 0x68; // Boussole
const uint8_t MAG_ADDR = 0x0C; // Magnétomètre intégré

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("============================================"));
  Serial.println(F("    TEST BOUSSOLE ICM-20948                "));
  Serial.println(F("============================================"));

  // Étape 1 : Réveil ICM
  Wire.beginTransmission(ICM_ADDR); Wire.write(0x06); Wire.write(0x01); Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(ICM_ADDR); Wire.write(0x06); Wire.write(0x00); Wire.endTransmission();
  delay(50);

  // Étape 2 : Sélection banque 0
  Wire.beginTransmission(ICM_ADDR); Wire.write(0x7F); Wire.write(0x00); Wire.endTransmission();
  delay(10);

  // Étape 3 : Activation bypass I2C (pour accéder au magnétomètre)
  Wire.beginTransmission(ICM_ADDR); Wire.write(0x0F); Wire.write(0x02); Wire.endTransmission();
  delay(10);

  // Étape 4 : Magnétomètre en mode continu 100Hz
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x31); Wire.write(0x08); Wire.endTransmission();
  delay(50);

  // Vérification
  Wire.beginTransmission(ICM_ADDR);
  if (Wire.endTransmission() == 0) Serial.println(F("ICM-20948 (0x68) : OK"));
  else { Serial.println(F("ICM NON DETECTE !")); while(true); }

  Wire.beginTransmission(MAG_ADDR);
  if (Wire.endTransmission() == 0) Serial.println(F("Magneto AK09916 (0x0C) : OK"));
  else Serial.println(F("Magneto NON DETECTE !"));

  Serial.println(F(""));
  Serial.println(F("CAP     | AX    AY    AZ    | MX      MY      MZ"));
  Serial.println(F("------------------------------------------------------"));
}

void loop() {
  // Lecture accéléromètre
  Wire.beginTransmission(ICM_ADDR); Wire.write(0x2D); Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, (uint8_t)6);
  if (Wire.available() < 6) { delay(200); return; }
  int16_t axR = (Wire.read() << 8) | Wire.read();
  int16_t ayR = (Wire.read() << 8) | Wire.read();
  int16_t azR = (Wire.read() << 8) | Wire.read();
  float ax = axR / 16384.0;
  float ay = ayR / 16384.0;
  float az = azR / 16384.0;

  // Lecture magnétomètre (little-endian spécifique AK09916)
  Wire.beginTransmission(MAG_ADDR); Wire.write(0x11); Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (uint8_t)8);
  if (Wire.available() < 8) { delay(200); return; }
  int16_t mx = Wire.read() | (Wire.read() << 8);
  int16_t my = Wire.read() | (Wire.read() << 8);
  int16_t mz = Wire.read() | (Wire.read() << 8);
  Wire.read(); Wire.read(); // Octets de statut

  // Calcul du cap avec compensation d'inclinaison
  float normA = sqrt(ax*ax + ay*ay + az*az);
  if (normA == 0) return;
  float axn = ax / normA;
  float ayn = ay / normA;
  float azn = az / normA;

  float roll  = atan2(ayn, azn);
  float pitch = atan2(-axn, sqrt(ayn*ayn + azn*azn));

  float Xh = mx * cos(pitch) + mz * sin(pitch);
  float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
  float cap = atan2(Yh, Xh) * 180.0 / PI;
  if (cap < 0) cap += 360.0;

  // Affichage
  if (cap < 100) Serial.print(F(" "));
  if (cap < 10)  Serial.print(F(" "));
  Serial.print(cap, 1); Serial.print(F("° | "));
  Serial.print(ax, 2); Serial.print(F("  "));
  Serial.print(ay, 2); Serial.print(F("  "));
  Serial.print(az, 2); Serial.print(F("  | "));
  Serial.print(mx); Serial.print(F("\t"));
  Serial.print(my); Serial.print(F("\t"));
  Serial.println(mz);

  delay(200);
}
