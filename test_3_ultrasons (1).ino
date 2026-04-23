// =========================================================================
// TEST 3 - ULTRASONS HC-SR05 (FONCTIONNEL)
// Teste les DEUX capteurs avec les pins correctes du projet
//
// BRANCHEMENTS :
//   Ultrason GAUCHE : VCC→5V, GND→GND, TRIG→A3, ECHO→A2
//   Ultrason DROIT  : VCC→5V, GND→GND, TRIG→A0, ECHO→A1
//
// UTILISATION :
//   1. Téléverser
//   2. Ouvrir Moniteur Série à 115200 bauds
//   3. Approcher la main ou un obstacle pour tester
// =========================================================================

#include <NewPing.h>

NewPing sonarG(A3, A2, 100);  // Gauche
NewPing sonarD(A0, A1, 100);  // Droit

void setup() {
  Serial.begin(115200);
  Serial.println(F("============================================"));
  Serial.println(F("         TEST ULTRASONS HC-SR05            "));
  Serial.println(F("============================================"));
  Serial.println(F("Gauche  | Droit   | Alerte"));
  Serial.println(F("-----------------------------------"));
}

void loop() {
  int dG = sonarG.ping_cm();
  int dD = sonarD.ping_cm();

  // Affichage Gauche
  if (dG == 0) Serial.print(F("---   "));
  else {
    Serial.print(dG);
    Serial.print(F(" cm "));
    if (dG < 10) Serial.print(F("  "));
    else if (dG < 100) Serial.print(F(" "));
  }
  Serial.print(F("| "));

  // Affichage Droit
  if (dD == 0) Serial.print(F("---   "));
  else {
    Serial.print(dD);
    Serial.print(F(" cm "));
    if (dD < 10) Serial.print(F("  "));
    else if (dD < 100) Serial.print(F(" "));
  }
  Serial.print(F("| "));

  // Alertes obstacles
  bool alerte = false;
  if (dG > 0 && dG < 15) { Serial.print(F("OBSTACLE GAUCHE ")); alerte = true; }
  if (dD > 0 && dD < 15) { Serial.print(F("OBSTACLE DROIT "));  alerte = true; }
  if (!alerte) Serial.print(F("OK"));

  Serial.println();
  delay(200);
}
