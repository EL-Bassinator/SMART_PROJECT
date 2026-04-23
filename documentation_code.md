# Documentation du code - Navire Cargo

Ce document explique le fonctionnement du programme Arduino qui pilote le bateau autonome. Il détaille chaque partie du code avec les extraits correspondants.

---

## 1. Vue d'ensemble

Le programme est structuré en **trois grandes parties** :

1. **Le setup** — exécuté une seule fois au démarrage, il initialise tous les capteurs et les moteurs
2. **La boucle principale (loop)** — exécutée en permanence, elle lit les capteurs et prend les décisions
3. **Les fonctions secondaires** — appelées depuis la boucle principale pour effectuer des tâches précises (piloter, freiner, détecter une couleur)

Le comportement global est géré par une **machine à états** qui définit ce que le bateau doit faire à chaque moment.

---

## 2. Les capteurs utilisés

Le bateau utilise **trois types de capteurs** pour percevoir son environnement :

### 2.1 Les ultrasons (HC-SR05)

Deux capteurs à ultrasons sont positionnés à gauche et à droite du bateau. Ils mesurent en permanence la distance avec les parois du chenal.

```cpp
NewPing sonarG(A3, A2, 45); // Ultrason GAUCHE : TRIG=A3, ECHO=A2
NewPing sonarD(A0, A1, 45); // Ultrason DROIT  : TRIG=A0, ECHO=A1
```

La valeur 45 correspond à la distance maximale en centimètres : au-delà, le capteur ignore la mesure.

### 2.2 La boussole (ICM-20948)

Ce module combine 3 capteurs : un accéléromètre, un gyroscope et un magnétomètre. Ensemble, ils permettent de connaître précisément l'orientation du bateau en temps réel, même quand il bouge ou penche.

### 2.3 Le capteur de couleur (TCS34725)

Placé vers le haut, il détecte les faisceaux lumineux des portiques. Un faisceau vert signale le départ, un faisceau rouge signale l'arrêt.

```cpp
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
```

---

## 3. La machine à états

C'est le cœur de la logique du programme. Chaque "état" correspond à une phase précise de la mission.

```cpp
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
```

Le programme passe d'un état à l'autre selon des conditions précises. Par exemple, depuis l'état ATTENTE_FEU_A, on passe à VERS_B uniquement quand le feu vert est détecté :

```cpp
case ATTENTE_FEU_A:
  moteur(0, 0);                        // Moteurs à l'arrêt
  if (detecterVertIntense()) {         // Si on voit le feu vert...
    chronoNav = millis();              // Démarre le chrono de sécurité
    etatActuel = VERS_B;               // Transition vers l'état suivant
  }
  break;
```

Cette organisation rend le code clair et facilement modifiable : pour ajouter un nouveau comportement, il suffit d'ajouter un état.

---

## 4. Le régulateur PID pour le maintien de trajectoire

Le bateau doit naviguer en ligne droite sur un cap précis (68°). Pour cela, il utilise un **régulateur PID** qui calcule en permanence l'écart entre le cap souhaité et le cap réel.

```cpp
double Kp = 2.4;  // Gain Proportionnel (force de braquage immédiate)
double Ki = 0.01; // Gain Intégral (corrige la dérive persistante)
double Kd = 1.2;  // Gain Dérivé (amortit les oscillations)
PID boussolePID(&capInput, &capOutput, &capSetpoint, Kp, Ki, Kd, DIRECT);
```

À chaque boucle, on calcule l'erreur de cap :

```cpp
float erreur = capActuel - CAP_CIBLE;   // Ex: si je suis à 78° et cible 68°, erreur = +10°
if (erreur > 180)  erreur -= 360;       // Correction cyclique (plus court chemin)
if (erreur < -180) erreur += 360;

capInput = erreur;                      // On donne l'erreur au PID
boussolePID.Compute();                  // Le PID calcule la correction
```

Le PID renvoie une valeur de correction qui est ensuite appliquée aux deux moteurs de façon asymétrique pour faire tourner le bateau dans la bonne direction.

---

## 5. La gestion de la vitesse

La vitesse des moteurs est contrôlée par PWM (Modulation de Largeur d'Impulsion). La valeur maximale est de 255, mais la vitesse nominale choisie est **170** pour garder une marge de correction.

```cpp
int vitesseBase = 170;  // Vitesse nominale (laisse ±85 de marge au PID)
```

La vitesse maximale (255) n'est utilisée que sur la dernière ligne droite entre les zones B et C pour gagner du temps :

```cpp
case VERS_C:
  piloter_PID(pwmMax);   // pwmMax = 255, pleine vitesse
```

---

## 6. Le filtre EMA (Moyenne Mobile Exponentielle)

Les mesures des ultrasons peuvent être instables à cause des vagues ou de mesures parasites. Pour lisser ces valeurs, on utilise un **filtre EMA**.

```cpp
const float alphaEMA = 0.3; // 30% nouveau + 70% ancien

dG_filtre = (alphaEMA * brutG) + ((1.0 - alphaEMA) * dG_filtre);
dD_filtre = (alphaEMA * brutD) + ((1.0 - alphaEMA) * dD_filtre);
```

**Le principe :** à chaque nouvelle mesure, on conserve seulement 30% de la valeur brute et on garde 70% de l'ancienne valeur lissée. Une mesure parasite ponctuelle (par exemple une vague qui renvoie mal l'onde) est ainsi atténuée et n'affecte pas le comportement du bateau.

---

## 7. La détection des couleurs par ratios RGB

Le capteur de couleur renvoie 4 valeurs : Rouge (R), Vert (G), Bleu (B) et Clarté totale (C). Au lieu d'utiliser les valeurs brutes qui dépendent de la luminosité, on utilise des **ratios**.

```cpp
bool detecterVertIntense() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 1000) return false;             // Si trop sombre, on ignore
  return ((float)g / c > 0.45);           // Si plus de 45% de vert → feu vert !
}

bool detecterRougeIntense() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 1000) return false;
  return ((float)r / c > 0.50);           // Si plus de 50% de rouge → feu rouge
}
```

**Pourquoi les ratios ?** Parce qu'une lumière verte forte et une lumière verte faible auront des valeurs absolues très différentes, mais le même ratio. Cela rend la détection robuste aux changements de luminosité ambiante.

---

## 8. L'adaptation à la charge transportée

Le bateau doit transporter trois charges différentes (graviers, eau, acier). Plus la charge est lourde, plus l'inertie est grande. Le programme **adapte automatiquement** le PID et le temps de freinage selon le voyage en cours.

```cpp
if (voyagesTermines == 0) {             // MISSION 1 : GRAVIER (5kg total)
  Kp = 2.4; tempsFreinage = 900;        // Braquage standard, freinage court
} else if (voyagesTermines == 1) {      // MISSION 2 : EAU (6kg avec ballotement)
  Kp = 2.8; tempsFreinage = 1100;       // Braquage plus nerveux
} else {                                // MISSION 3 : ACIER (9kg, forte inertie)
  Kp = 3.8; tempsFreinage = 2000;       // Braquage très agressif, freinage long
}
boussolePID.SetTunings(Kp, Ki, Kd);
```

Cette adaptation est essentielle car un bateau chargé de 9kg d'acier nécessite **deux fois plus de temps** pour s'arrêter qu'un bateau avec 5kg de graviers.

---

## 9. L'évitement des parois

En plus du maintien de cap par le PID, le bateau réagit aux parois détectées par les ultrasons :

```cpp
if (dG_filtre < 16) {                                      // Mur GAUCHE trop proche
  evit += map((int)dG_filtre, 7, 16, 85, 0);              // Force de répulsion vers la droite
  vitesseBase = map((int)dG_filtre, 7, 16, 100, 170);     // Ralentit pour mieux braquer
}
if (dD_filtre < 16) {                                      // Mur DROIT trop proche
  evit -= map((int)dD_filtre, 7, 16, 85, 0);              // Force vers la gauche
}
```

La fonction `map()` traduit la proximité en force de braquage : plus le mur est proche, plus la correction est forte. En parallèle, la vitesse est réduite pour que le bateau puisse braquer sans glisser.

---

## 10. Les protections logicielles

Plusieurs sécurités sont intégrées au programme :

### 10.1 Le Watchdog Timer

Si le programme se bloque plus de 2 secondes, l'Arduino redémarre automatiquement :

```cpp
wdt_enable(WDTO_2S);  // Active le watchdog au démarrage

// Dans la boucle principale :
wdt_reset();  // On "nourrit" le watchdog régulièrement pour dire que tout va bien
```

### 10.2 Le timeout de navigation

Si le feu rouge n'est jamais détecté (panne de capteur par exemple), le bateau freine automatiquement après 30 secondes :

```cpp
if (millis() - chronoNav > TIMEOUT_NAV) {  // TIMEOUT_NAV = 30000 ms
  chrono = millis();
  etatActuel = FREINAGE_B;
}
```

### 10.3 La sécurité au démarrage

Les moteurs sont forcés à l'arrêt AVANT d'activer le pont en H pour éviter tout mouvement involontaire :

```cpp
analogWrite(PWMA, 0);  analogWrite(PWMB, 0);      // PWM à 0 en premier
digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); // Direction neutre
pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);  // Pont en H activé seulement après
```

### 10.4 L'utilisation de millis() au lieu de delay()

La fonction `delay()` bloque complètement le programme. On utilise donc `millis()` pour mesurer le temps sans interrompre la boucle principale :

```cpp
case FREINAGE_B:
  moteur(-180, -180);                      // Lance le freinage
  if (millis() - chrono > tempsFreinage) { // Vérifie si le temps est écoulé
    etatActuel = ATTENTE_FEU_B;            // Passe à l'état suivant
  }
  break;
```

Pendant le freinage, le programme continue à lire les capteurs et à réagir.

---

## 11. La fusion Mahony pour le cap 9 axes

La boussole utilise un algorithme appelé **fusion Mahony** qui combine les données des 3 capteurs (accéléromètre, gyroscope, magnétomètre) pour obtenir un cap précis et stable.

**Principe :**
- Le **gyroscope** donne la rotation instantanée (précis à court terme mais dérive dans le temps)
- L'**accéléromètre** détecte la gravité et corrige la dérive en roulis/tangage
- Le **magnétomètre** donne le nord réel et corrige la dérive en cap

Les trois sources sont fusionnées dans un **quaternion d'orientation** qui représente mathématiquement la rotation 3D du bateau :

```cpp
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
```

À partir de ce quaternion, on extrait le cap (yaw) :

```cpp
float yaw = atan2(2.0*(q1*q2+q0*q3), q0*q0+q1*q1-q2*q2-q3*q3);
float cap = yaw * 180.0 / PI;
if (cap < 0) cap += 360.0;
capActuel = cap;
```

---

## 12. Chronologie complète d'une mission

Voici ce qui se passe étape par étape lors d'un voyage :

| Étape | État | Action |
|---|---|---|
| 1 | ATTENTE_FEU_A | Le bateau est immobile en zone A, attend le feu vert |
| 2 | VERS_B | Navigation vers B avec correction de cap + évitement |
| 3 | FREINAGE_B | Marche arrière pour s'arrêter précisément en B |
| 4 | ATTENTE_FEU_B | Arrêt sous le portique B, attend le nouveau feu vert |
| 5 | VERS_C | Navigation à pleine vitesse vers C |
| 6 | FREINAGE_C | Freinage maximal en C |
| 7 | DECHARGEMENT_C | Pause de 5 secondes pour simuler le déchargement |
| 8 | Retour à l'étape 4 | Nouveau voyage (tant que voyagesTermines < 3) |

À la fin des 3 voyages, l'état devient FIN_MISSION et les moteurs sont définitivement coupés.
