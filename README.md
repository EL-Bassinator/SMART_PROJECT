# Codes de test - Navire Cargo

Ce dossier contient les tests individuels pour chaque composant. Ils ont été utilisés pendant le développement pour valider le bon fonctionnement de chaque partie du système.

## Ordre de test recommandé

| Ordre | Fichier | Ce qu'il teste | Résultat attendu |
|---|---|---|---|
| 1 | `test_1_scanner_I2C.ino` | Détecte les composants I2C | 3 adresses : 0x0C, 0x29, 0x68 |
| 2 | `test_2_couleur.ino` | Capteur couleur TCS34725 | Valeurs RGBC qui changent selon la couleur |
| 3 | `test_3_ultrasons.ino` | Les 2 ultrasons | Distance en cm qui change avec la main |
| 4 | `test_4_moteurs.ino` | Pont en H + moteurs | Les 2 moteurs tournent en avant |
| 5 | `test_5_boussole.ino` | Boussole + magnétomètre | Cap qui change quand on tourne le module |

## Branchements de référence

**Alimentation partagée :**
- 3.3V → ICM-20948 uniquement (important !)
- 5V → TCS34725, ultrasons, TB6612FNG
- 9V (batterie) → VM du TB6612FNG uniquement

**Bus I2C (partagé par boussole + capteur couleur) :**
- SDA → A4
- SCL → A5

**Ultrasons :**
- Gauche : TRIG=A3, ECHO=A2
- Droit : TRIG=A0, ECHO=A1

**Moteurs (via TB6612FNG) :**
- Moteur gauche : AIN1=2, AIN2=3, PWMA=5
- Moteur droit : BIN1=4, BIN2=7, PWMB=6
- STBY=8
