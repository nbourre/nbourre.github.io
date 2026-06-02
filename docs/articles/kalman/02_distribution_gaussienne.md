# La Distribution Gaussienne et le Bruit de Capteur

## La forme en cloche

Si tu prends 1000 lectures du sonar et que tu traces un histogramme, tu obtiens quelque chose qui ressemble à une cloche symétrique : la plupart des valeurs sont proches de la vraie distance, et de moins en moins de valeurs s'en éloignent. C'est la **distribution gaussienne** (ou distribution normale).

Elle est définie par seulement deux paramètres :

- **µ (mu)** : la moyenne — le centre de la cloche
- **σ² (sigma carré)** : la variance — la largeur de la cloche

Sa formule est :

$$p(x) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\!\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

**Bonne nouvelle** : on n'a presque jamais besoin de calculer cette formule directement. Ce qui compte, c'est l'intuition : µ et σ² suffisent à décrire complètement le bruit d'un capteur.

## La règle des 68-95-99.7

Voici la règle pratique la plus utile des gaussiennes :

| Plage | Probabilité |
|-------|------------|
| µ ± 1σ | ~68% des mesures |
| µ ± 2σ | ~95% des mesures |
| µ ± 3σ | ~99.7% des mesures |

!!! example "Exemple : sonar à 40 cm, σ = 2 cm"
    - 68% des lectures tomberont entre 38 et 42 cm
    - 95% des lectures tomberont entre 36 et 44 cm
    - Presque toutes les lectures tomberont entre 34 et 46 cm

## Pourquoi les gaussiennes sont pratiques

Les gaussiennes ont une propriété extraordinaire : elles se combinent de façon très simple.

**Addition de variables indépendantes** — si on additionne deux variables aléatoires, leurs moyennes et variances s'additionnent :

$$\mu_{x_1 + x_2} = \mu_{x_1} + \mu_{x_2}$$
$$\sigma^2_{x_1 + x_2} = \sigma^2_{x_1} + \sigma^2_{x_2}$$

**Mise à l'échelle** — si on multiplie par une constante *a*, la variance est multipliée par *a²* :

$$\sigma^2_{ax} = a^2 \cdot \sigma^2_x$$

Ces deux règles sont toute l'algèbre dont le filtre de Kalman a besoin. Chaque étape du filtre — prédiction et correction — n'est rien d'autre que ces manipulations simples.

## Application au Ranger : modéliser le bruit de chaque capteur

### Sonar

Le sonar mesure la distance. Son bruit est typiquement entre 1 et 3 cm pour des surfaces plates.

```
distance_mesurée = distance_réelle + bruit_sonar
bruit_sonar ~ N(0, σ²_sonar)
```

### Encodeurs

Les encodeurs mesurent la rotation des roues. Le bruit provient surtout du glissement des roues. On peut estimer la variance en degrés² ou en cm² selon l'application.

```
position_mesurée = position_réelle + bruit_encodeur
bruit_encodeur ~ N(0, σ²_encodeur)
```

### MPU-6050 (gyroscope)

Le gyroscope mesure la vitesse angulaire. Il a deux types de bruit : un bruit blanc (variance fixe) et une **dérive** (l'erreur qui s'accumule avec le temps). L'accéléromètre mesure l'accélération, mais il est très sensible aux vibrations des moteurs.

```
angle_mesuré = angle_réel + dérive + bruit_gyro
```

## Code : visualiser la distribution en temps réel

Ce sketch Arduino envoie les lectures du sonar et du gyroscope au Serial Plotter pour visualiser leur distribution :

```cpp
#include <MeAuriga.h>

MeUltrasonicSensor sonar(PORT_10);
MeGyro gyro(0, 0x69);

void setup() {
  Serial.begin(115200);
  gyro.begin();

  // En-têtes pour le Serial Plotter
  Serial.println("Sonar_cm\tAngle_X_deg");
}

void loop() {
  gyro.update();

  float distanceSonar = sonar.distanceCm();
  float angleX = gyro.getAngleX();

  Serial.print(distanceSonar, 2);
  Serial.print("\t");
  Serial.println(angleX, 2);

  delay(50);
}
```

!!! tip "Comment utiliser ce sketch"
    1. Garde le robot parfaitement immobile
    2. Ouvre le **Serial Plotter** (Ctrl+Shift+L dans Arduino IDE)
    3. Observe la dispersion autour de la valeur moyenne
    4. Plus la trace est "épaisse", plus la variance est grande

## La dérive du gyroscope : une gaussienne qui change de centre

Un phénomène important à comprendre : la dérive du gyroscope. Même au repos, `getAngleX()` augmente lentement avec le temps. Ce n'est pas du bruit gaussien pur — c'est une erreur systématique qui s'accumule.

```cpp
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);
unsigned long tempsDebut;

void setup() {
  Serial.begin(115200);
  gyro.begin();
  gyro.resetData();  // Remet les angles à zéro
  tempsDebut = millis();
  Serial.println("Temps_s\tAngle_Z_deg");
}

void loop() {
  gyro.update();

  float temps = (millis() - tempsDebut) / 1000.0;
  float angleZ = gyro.getAngleZ();

  Serial.print(temps, 2);
  Serial.print("\t");
  Serial.println(angleZ, 3);

  delay(100);
}
```

Laisse tourner ce sketch pendant 2-3 minutes avec le robot immobile. Tu verras l'angle dériver lentement — c'est exactement le problème que le filtre de Kalman va corriger en fusionnant avec d'autres capteurs.

## Résumé

| Concept | Ce que ça veut dire |
|---------|---------------------|
| Distribution gaussienne | La "cloche" qui décrit comment les erreurs se distribuent |
| µ (moyenne) | Vers où les mesures convergent |
| σ² (variance) | L'amplitude de la dispersion |
| Règle 68-95-99.7 | Combien de mesures tombent dans ±1σ, ±2σ, ±3σ |
| Dérive | Erreur qui s'accumule — la vraie ennemie des gyroscopes |

Dans la prochaine section, on verra comment la variance permet de quantifier la **qualité** d'un capteur et décider auquel faire confiance.
