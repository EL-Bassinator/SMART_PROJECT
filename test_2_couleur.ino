// =========================================================================
// TEST 2 - CAPTEUR COULEUR TCS34725 (FONCTIONNEL)
// Affiche les valeurs R/G/B/C et les ratios en temps réel
//
// BRANCHEMENTS :
//   VIN → 3.3V ou 5V
//   GND → GND
//   SDA → A4
//   SCL → A5
//
// PROTOCOLE DE TEST :
//   1. Téléverser et ouvrir Moniteur Série à 115200 bauds
//   2. Présenter différentes couleurs devant le capteur
//   3. Observer les valeurs brutes et les ratios
// =========================================================================

#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println(F("============================================"));
  Serial.println(F("       TEST CAPTEUR COULEUR TCS34725       "));
  Serial.println(F("============================================"));

  if (!tcs.begin()) {
    Serial.println(F("ERREUR : Capteur non detecte !"));
    while(true);
  }
  Serial.println(F("Capteur OK - Lecture en cours..."));
  Serial.println(F(""));
  Serial.println(F("R       G       B       C       | R/C   G/C   B/C"));
  Serial.println(F("-----------------------------------------------------"));
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print(r); Serial.print(F("\t"));
  Serial.print(g); Serial.print(F("\t"));
  Serial.print(b); Serial.print(F("\t"));
  Serial.print(c); Serial.print(F("\t| "));

  if (c > 100) {
    Serial.print((float)r/c, 2); Serial.print(F("  "));
    Serial.print((float)g/c, 2); Serial.print(F("  "));
    Serial.print((float)b/c, 2);
  }

  Serial.println();
  delay(300);
}
