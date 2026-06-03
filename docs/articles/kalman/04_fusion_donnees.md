# Fusion de Données : Combiner des Capteurs Imparfaits

## Le problème

Suppose que deux capteurs mesurent la même chose au même moment :

$$y_1 = x + e_1, \quad e_1 \sim \mathcal{N}(0, \sigma_1^2)$$

$$y_2 = x + e_2, \quad e_2 \sim \mathcal{N}(0, \sigma_2^2)$$

Quelle est la meilleure estimation de *x* ?

La **moyenne simple** `(y1 + y2) / 2` serait une erreur : elle donnerait autant de poids à un capteur médiocre qu'à un capteur précis. Il faut une **moyenne pondérée**, avec plus de poids pour le capteur le plus précis.

## La formule de fusion

La meilleure estimation est :

$$\hat{x} = \frac{y_1 \sigma_2^2 + y_2 \sigma_1^2}{\sigma_1^2 + \sigma_2^2}$$

On peut réécrire ça d'une façon qui reviendra au cœur du filtre de Kalman. On définit un **gain K** :

$$K = \frac{\sigma_1^2}{\sigma_1^2 + \sigma_2^2}$$

Et l'estimation fusionnée devient :

$$\hat{x} = y_1 + K \cdot (y_2 - y_1)$$

En d'autres mots : on part de y₁ et on se **déplace de K × (la différence)** vers y₂.

### La variance de l'estimation fusionnée

Après fusion, la variance est **plus petite que les deux capteurs** :

$$\text{Var}(\hat{x}) = \sigma_1^2 (1 - K)$$

La fusion ne peut qu'augmenter la confiance — jamais la réduire. Chaque capteur supplémentaire ajoute de l'information.

## Exemple chiffré : deux lectures de distance

Le sonar lit **42 cm** avec σ₁² = 4 cm² (σ₁ = 2 cm).
Les encodeurs estiment **45 cm** avec σ₂² = 9 cm² (σ₂ = 3 cm).

**Calcul du gain :**

$$K = \frac{4}{4 + 9} = \frac{4}{13} \approx 0.31$$

**Estimation fusionnée :**

$$\hat{x} = 42 + 0.31 \times (45 - 42) = 42 + 0.93 \approx 42.9 \text{ cm}$$

**Variance fusionnée :**

$$\text{Var}(\hat{x}) = 4 \times (1 - 0.31) = 2.76 \text{ cm}^2$$

L'estimation 42.9 cm est plus proche du sonar (plus précis), mais tient légèrement compte des encodeurs. Et la variance de 2.76 cm² est **plus petite que les deux entrées** (4 et 9), même si aucune des deux mesures n'était fiable à 100%.

## Implémentation sur le Ranger

Voici une fusion simple entre le sonar et les encodeurs pour estimer la distance à un mur :

```cpp
#include <MeAuriga.h>

MeUltrasonicSensor sonar(PORT_10);
MeEncoderOnBoard encodeurGauche(SLOT1);
MeEncoderOnBoard encodeurDroit(SLOT2);

// Interruptions des encodeurs
void isr_encodeur1() {
  if (digitalRead(encodeurGauche.getPortB()) == 0)
    encodeurGauche.pulsePosMinus();
  else
    encodeurGauche.pulsePosPlus();
}

void isr_encodeur2() {
  if (digitalRead(encodeurDroit.getPortB()) == 0)
    encodeurDroit.pulsePosMinus();
  else
    encodeurDroit.pulsePosPlus();
}

// Paramètres du robot (à calibrer)
const float RAYON_ROUE_CM = 3.25;       // cm
const float IMPULSIONS_PAR_TOUR = 360.0; // impulsions/tour (en degrés)

// Variances mesurées expérimentalement
const float VARIANCE_SONAR      = 4.0;  // cm²
const float VARIANCE_ENCODEURS  = 1.0;  // cm² par cm parcouru (dérive)

// État de la fusion
float estimationFusionnee = 0;
float varianceFusionnee   = 100.0;  // Grande incertitude initiale
float distanceEncodeurs   = 0;
float varianceEncodeurs   = 0;

// Convertit les degrés d'encodeur en cm parcourus
float degresEnCm(float degres) {
  return (degres / 360.0) * (2 * PI * RAYON_ROUE_CM);
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(encodeurGauche.getIntNum(), isr_encodeur1, RISING);
  attachInterrupt(encodeurDroit.getIntNum(), isr_encodeur2, RISING);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Serial.println("Sonar_cm\tEncodeurs_cm\tFusion_cm\tVariance");
}

void loop() {
  encodeurGauche.loop();
  encodeurDroit.loop();

  // Distance estimée par encodeurs (moyenne des deux roues)
  float degG = abs(encodeurGauche.getCurPos());
  float degD = abs(encodeurDroit.getCurPos());
  distanceEncodeurs = degresEnCm((degG + degD) / 2.0);

  // La variance des encodeurs croît avec la distance (dérive)
  varianceEncodeurs = distanceEncodeurs * VARIANCE_ENCODEURS;

  // Lecture du sonar
  float distanceSonar = sonar.distanceCm();

  // --- FUSION DE DONNÉES ---

  // Étape 1 : prédiction avec les encodeurs
  float variancePred = varianceFusionnee + varianceEncodeurs;
  // (Pour cette démo simple, estimationFusionnee reste inchangée ici)

  // Étape 2 : gain et correction avec le sonar
  float K = variancePred / (variancePred + VARIANCE_SONAR);
  estimationFusionnee = estimationFusionnee + K * (distanceSonar - estimationFusionnee);
  varianceFusionnee   = variancePred * (1 - K);

  // Affichage
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage > 200) {
    dernierAffichage = millis();
    Serial.print(distanceSonar, 1);
    Serial.print("\t");
    Serial.print(distanceEncodeurs, 1);
    Serial.print("\t");
    Serial.print(estimationFusionnee, 2);
    Serial.print("\t");
    Serial.println(varianceFusionnee, 4);
  }
}
```

## Interpréter le gain K

Le gain K est la clé de la fusion :

| Situation | Valeur de K | Effet |
|-----------|------------|-------|
| σ₁² = σ₂² (capteurs identiques) | K = 0.5 | L'estimation est à mi-chemin |
| σ₁² ≪ σ₂² (capteur 1 très précis) | K → 0 | L'estimation reste près de y₁ |
| σ₁² ≫ σ₂² (capteur 2 très précis) | K → 1 | L'estimation saute vers y₂ |

Ce gain automatique — qui décide combien se déplacer vers la nouvelle mesure — est **l'essence même du filtre de Kalman**.

## Résumé

La fusion de données, c'est une moyenne pondérée par la confiance :

- Variance petite = forte confiance = fort poids
- Le résultat fusionné est **toujours plus précis** que les deux entrées seules
- Le gain K contrôle automatiquement l'équilibre entre les capteurs

Dans la prochaine section, on ajoute la **dimension temporelle** : le robot se déplace, l'état change à chaque instant. C'est le filtre de Kalman 1D.
