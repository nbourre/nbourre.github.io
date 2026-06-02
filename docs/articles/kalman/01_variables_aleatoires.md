# Variables aléatoires

## Le monde réel n'est pas déterministe

Dans un monde parfait, le sonar de ton robot mesurerait exactement 42,0 cm à chaque lecture. Tes encodeurs compteraient exactement le bon nombre d'impulsions. Le gyroscope donnerait un angle précis à la milliseconde près.

Mais dans la réalité, ça ne se passe jamais ainsi.

Voici ce qu'on observe en lisant le sonar 10 fois de suite sur un objet immobile à environ 40 cm :

```
42.1  39.8  41.5  40.2  43.0  38.9  41.8  40.6  42.3  39.5
```

Les valeurs tournent autour de 40 cm, mais aucune n'est identique. Cette variabilité s'appelle le **bruit de mesure**, et elle est inévitable dans tout système physique.

## Pourquoi les capteurs sont bruités

Plusieurs phénomènes causent ce bruit :

- **Sonar** : les ultrasons rebondissent de façon légèrement différente selon la surface, la température de l'air, les vibrations du robot
- **Encodeurs** : les roues glissent légèrement, les engrenages ont un jeu mécanique
- **MPU-6050** : le gyroscope dérive lentement, l'accéléromètre capte les vibrations du moteur

Pour décrire ces phénomènes mathématiquement, on utilise les **variables aléatoires**.

## Qu'est-ce qu'une variable aléatoire ?

Une variable aléatoire est une quantité dont la valeur n'est pas fixe, mais suit une distribution de probabilité. En d'autres mots, elle peut prendre différentes valeurs avec différentes probabilités.

Pour le filtre de Kalman, on modélise chaque mesure comme :

```
mesure = vraie_valeur + bruit
```

où le bruit est une variable aléatoire. Les deux paramètres les plus importants de cette variable sont :

- **La moyenne (µ)** : vers quelle valeur les mesures convergent en moyenne
- **La variance (σ²)** : à quel point les mesures s'éparpillent autour de la moyenne

!!! example "Exemple concret : sonar du Ranger"
    Si le robot est à 40 cm d'un mur et que le sonar a une variance de 4 cm² (σ = 2 cm),
    cela signifie qu'environ 68% des lectures tomberont entre 38 et 42 cm.

## Vérifier le bruit de tes capteurs

Avant d'utiliser le filtre de Kalman, il est utile de mesurer expérimentalement le bruit de tes capteurs. Voici un sketch Arduino pour le sonar :

```cpp
#include <MeAuriga.h>

MeUltrasonicSensor sonar(PORT_10);

const int NB_MESURES = 100;
float mesures[NB_MESURES];

void setup() {
  Serial.begin(115200);
  Serial.println("Mesure du bruit du sonar...");
  Serial.println("Garde le robot immobile !");
  delay(2000);

  // Collecte des mesures
  float somme = 0;
  for (int i = 0; i < NB_MESURES; i++) {
    mesures[i] = sonar.distanceCm();
    somme += mesures[i];
    delay(50);
  }

  // Calcul de la moyenne
  float moyenne = somme / NB_MESURES;

  // Calcul de la variance
  float variance = 0;
  for (int i = 0; i < NB_MESURES; i++) {
    float ecart = mesures[i] - moyenne;
    variance += ecart * ecart;
  }
  variance /= NB_MESURES;

  Serial.print("Moyenne     : ");
  Serial.print(moyenne, 2);
  Serial.println(" cm");

  Serial.print("Variance    : ");
  Serial.print(variance, 4);
  Serial.println(" cm²");

  Serial.print("Écart-type  : ");
  Serial.print(sqrt(variance), 4);
  Serial.println(" cm");
}

void loop() {}
```

!!! tip "À retenir"
    La variance que tu mesures ici servira directement comme paramètre **R** (bruit de mesure) dans ton filtre de Kalman. Plus cette valeur est grande, moins le filtre fera confiance au capteur.

## Même chose pour les encodeurs

```cpp
#include <MeAuriga.h>

MeEncoderOnBoard encodeurGauche(SLOT1);

void isr_process_encoder1(void) {
  if (digitalRead(encodeurGauche.getPortB()) == 0) {
    encodeurGauche.pulsePosMinus();
  } else {
    encodeurGauche.pulsePosPlus();
  }
}

void setup() {
  Serial.begin(115200);
  attachInterrupt(encodeurGauche.getIntNum(), isr_process_encoder1, RISING);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void loop() {
  encodeurGauche.loop();

  // Affiche la vitesse en RPM toutes les 100ms
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage > 100) {
    dernierAffichage = millis();
    Serial.println(encodeurGauche.getCurrentSpeed());
  }
}
```

Fais tourner les roues à vitesse constante (par exemple à main) et observe la variabilité des lectures — c'est le bruit de l'encodeur.

## Résumé

| Concept | Définition simple |
|---------|------------------|
| Variable aléatoire | Une valeur qui varie de façon imprévisible |
| Moyenne (µ) | La valeur "centrale" vers laquelle on tend |
| Variance (σ²) | L'ampleur de la dispersion autour de la moyenne |
| Bruit de capteur | La variabilité inévitable des mesures physiques |

Dans la prochaine section, on verra comment modéliser ce bruit avec la distribution gaussienne — la forme en cloche que tu as sûrement déjà vue.
