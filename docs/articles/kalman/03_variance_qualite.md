# Variance et Qualité des Capteurs

## La variance comme mesure de confiance

Toute mesure peut se modéliser comme :

$$y = x + e, \quad e \sim \mathcal{N}(0, \sigma^2)$$

où *x* est la vraie valeur et *e* est le bruit gaussien de variance σ². Cette variance est exactement ce qui dit **à quel point on peut faire confiance** au capteur.

- **Petite variance** → mesures serrées autour de la vraie valeur → capteur fiable
- **Grande variance** → mesures éparpillées → capteur peu fiable

Les deux capteurs sont en moyenne centrés sur la bonne valeur. Mais le capteur précis (petite σ) donne des lectures individuelles bien plus utiles.

## Comparer les capteurs du Ranger

### Sonar vs encodeurs pour estimer la distance à un mur

Supposons que le robot avance vers un mur. On peut estimer la distance de deux façons :

1. **Sonar** : mesure directement la distance, mais chaque lecture est bruyante
2. **Encodeurs** : on sait la distance parcourue depuis le départ, donc `distance_au_mur = distance_départ - distance_parcourue`

Ces deux méthodes ont des profils de variance très différents :

| Méthode | Variance au départ | Variance après 1m | Variance après 5m |
|---------|-------------------|-------------------|-------------------|
| Sonar | ~4 cm² | ~4 cm² | ~4 cm² |
| Encodeurs | ~0 cm² | ~2 cm² | ~10 cm² |

Le sonar a une variance **constante** (indépendante du temps). Les encodeurs ont une variance qui **croît** au fil du temps (dérive par accumulation d'erreurs). C'est pourquoi la fusion des deux est si puissante.

## Mesurer les variances expérimentalement

```cpp
#include <MeAuriga.h>

MeUltrasonicSensor sonar(PORT_10);
MeGyro gyro(0, 0x69);

// Calcule moyenne et variance sur N mesures
struct Stats {
  float moyenne;
  float variance;
};

Stats calculerStats(float* donnees, int n) {
  Stats s;
  float somme = 0;
  for (int i = 0; i < n; i++) somme += donnees[i];
  s.moyenne = somme / n;

  float sommeCarre = 0;
  for (int i = 0; i < n; i++) {
    float ecart = donnees[i] - s.moyenne;
    sommeCarre += ecart * ecart;
  }
  s.variance = sommeCarre / n;
  return s;
}

const int N = 200;
float mesuresSonar[N];
float mesuresAccX[N];
float mesuresAccY[N];
float mesuresAngZ[N];

void setup() {
  Serial.begin(115200);
  gyro.begin();
  delay(500);

  Serial.println("Collecte des mesures (robot immobile)...");

  for (int i = 0; i < N; i++) {
    gyro.update();
    mesuresSonar[i] = sonar.distanceCm();
    mesuresAccX[i] = gyro.getAccX();
    mesuresAccY[i] = gyro.getAccY();
    mesuresAngZ[i] = gyro.getAngleZ();
    delay(20);
  }

  Stats sSonar = calculerStats(mesuresSonar, N);
  Stats sAccX  = calculerStats(mesuresAccX, N);
  Stats sAccY  = calculerStats(mesuresAccY, N);
  Stats sAngZ  = calculerStats(mesuresAngZ, N);

  Serial.println("\n=== Résultats ===");

  Serial.print("Sonar      — µ="); Serial.print(sSonar.moyenne, 2);
  Serial.print(" cm, σ²="); Serial.print(sSonar.variance, 4);
  Serial.println(" cm²");

  Serial.print("Acc X      — µ="); Serial.print(sAccX.moyenne, 4);
  Serial.print(" g, σ²="); Serial.println(sAccX.variance, 6);

  Serial.print("Acc Y      — µ="); Serial.print(sAccY.moyenne, 4);
  Serial.print(" g, σ²="); Serial.println(sAccY.variance, 6);

  Serial.print("Angle Z    — µ="); Serial.print(sAngZ.moyenne, 4);
  Serial.print(" °, σ²="); Serial.println(sAngZ.variance, 6);

  Serial.println("\nUtilise ces variances comme paramètres R dans ton filtre de Kalman.");
}

void loop() {}
```

## Interpréter les résultats

Voici des valeurs typiques qu'on peut obtenir avec le Ranger :

| Capteur | Variance typique | Unité |
|---------|-----------------|-------|
| Sonar | 1.0 – 9.0 | cm² |
| Accéléromètre X/Y | 0.001 – 0.01 | g² |
| Gyroscope (angle) | 0.001 – 0.1 | deg² |

!!! warning "La variance change selon les conditions"
    - Le sonar est plus bruité sur des surfaces molles (mousse, tissu) que sur des surfaces dures
    - L'accéléromètre est **beaucoup** plus bruité quand les moteurs tournent (vibrations)
    - Mesure toujours la variance dans les conditions réelles d'utilisation

## L'intuition clé : plus la variance est petite, plus on fait confiance

Imagine deux règles pour mesurer une même longueur :
- Une règle artisanale en bois (précision ± 5 mm, σ² = 25 mm²)
- Un pied à coulisse numérique (précision ± 0.1 mm, σ² = 0.01 mm²)

Si tu dois choisir laquelle croire, tu prends le pied à coulisse — même si les deux donnent à peu près la même lecture. Le filtre de Kalman fait exactement ce choix automatiquement, basé sur les variances.

## Résumé

La variance σ² d'un capteur mesure à quel point ses lectures sont dispersées. C'est le paramètre de confiance fondamental du filtre de Kalman :

- σ² petit → fort poids dans la fusion
- σ² grand → faible poids dans la fusion

Dans la prochaine section, on voit comment **fusionner deux capteurs** de qualités différentes pour obtenir une estimation meilleure que les deux ensemble.
