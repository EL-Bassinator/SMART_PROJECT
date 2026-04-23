// =========================================================================
// TEST 1 - SCANNER I2C (FONCTIONNEL)
// Détecte tous les composants connectés sur le bus I2C
//
// RÉSULTAT ATTENDU pour notre projet :
//   0x0C → Magnétomètre AK09916
//   0x29 → Capteur couleur TCS34725
//   0x68 → Boussole ICM-20948
// =========================================================================

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println(F("============================================"));
  Serial.println(F("              SCANNER I2C                  "));
  Serial.println(F("============================================"));
  Serial.println(F("Scan en cours..."));
  Serial.println(F(""));

  int nbTrouves = 0;

  for (byte adresse = 1; adresse < 127; adresse++) {
    Wire.beginTransmission(adresse);
    byte erreur = Wire.endTransmission();

    if (erreur == 0) {
      Serial.print(F("  ✓ 0x"));
      if (adresse < 16) Serial.print(F("0"));
      Serial.print(adresse, HEX);
      Serial.print(F("  →  "));

      switch (adresse) {
        case 0x0C: Serial.println(F("AK09916 (magnetometre)")); break;
        case 0x29: Serial.println(F("TCS34725 (capteur couleur)")); break;
        case 0x68: Serial.println(F("ICM-20948 (boussole)")); break;
        default:   Serial.println(F("Composant inconnu")); break;
      }
      nbTrouves++;
    }
    delay(5);
  }

  Serial.println(F(""));
  Serial.print(nbTrouves);
  Serial.println(F(" composant(s) detecte(s)"));
}

void loop() {}
