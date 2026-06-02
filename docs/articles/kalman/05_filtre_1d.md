# Le Filtre de Kalman 1D

## Le problème : le robot se déplace dans le temps

Jusqu'ici, on a fusionné deux capteurs mesurant la **même chose au même moment**. Mais en robotique, le robot **bouge** — son état change à chaque instant. On doit suivre cette évolution dans le temps.

Considère le Ranger qui avance vers un mur. À chaque instant, on veut connaître sa **position** aussi précisément que possible, en utilisant :

- Les **encodeurs** : qui mesurent la vitesse (et donc permettent de calculer la distance parcourue)
- Le **sonar** : qui mesure directement la distance au mur

Chacun a son problème :

**Encodeurs seuls — la dérive**

On fait de la *navigation à l'estime* : on part d'une position connue et on additionne les déplacements mesurés. L'erreur de chaque mesure s'accumule — la variance grandit indéfiniment et le robot finit par ne plus savoir où il est.

$$x_\text{pred}[n] = \hat{x}[n-1] + v[n] \cdot \Delta t$$
$$\sigma^2_\text{pred}[n] = \sigma^2[n-1] + \sigma^2_\text{enc}$$

**Sonar seul — le bruit**

Le sonar ne dérive pas (il voit directement le mur), mais chaque lecture individuelle est bruyante. Sans filtrage, la position estimée saute partout.

**Filtre de Kalman — le meilleur des deux**

Le filtre alterne deux phases à chaque pas de temps :

1. **Prédiction** : on avance l'estimation avec les encodeurs (dérive incluse)
2. **Correction** : on fusionne avec la mesure du sonar (exactement comme au chapitre 4)

## Les équations du filtre 1D

### Phase 1 : Prédiction

On avance l'état avec le modèle de mouvement :

$$x_\text{pred} = \hat{x}[n-1] + v[n] \cdot \Delta t$$
$$\sigma^2_\text{pred} = \sigma^2[n-1] + \sigma^2_\text{enc}$$

La position estimée avance. La variance augmente (on est moins certain — on a bougé sans regarder).

### Phase 2 : Correction

Une mesure arrive. On calcule le gain, puis on corrige :

$$K = \frac{\sigma^2_\text{pred}}{\sigma^2_\text{pred} + \sigma^2_\text{mes}}$$

$$\hat{x}[n] = x_\text{pred} + K \cdot (z - x_\text{pred})$$

$$\sigma^2[n] = \sigma^2_\text{pred} \cdot (1 - K)$$

où *z* est la mesure du sonar. La correction réduit la variance — on a regardé, on est plus certain.

## Exemple chiffré : un pas complet

**État initial** : position estimée 30 cm, σ² = 4 cm²

**Prédiction** : les encodeurs rapportent une vitesse de 5 cm/pas, bruit σ²_enc = 1 cm²

$$x_\text{pred} = 30 + 5 = 35 \text{ cm}$$
$$\sigma^2_\text{pred} = 4 + 1 = 5 \text{ cm}^2$$

**Correction** : le sonar lit 38 cm, avec σ²_sonar = 9 cm²

$$K = \frac{5}{5 + 9} \approx 0.36$$

$$\hat{x} = 35 + 0.36 \times (38 - 35) \approx 36.1 \text{ cm}$$

$$\sigma^2 = 5 \times (1 - 0.36) = 3.2 \text{ cm}^2$$

L'estimation a bougé vers le sonar (seulement 36% du chemin, car le sonar est moins fiable que la prédiction ici). La variance est descendue de 5 à 3.2.

## Implémentation complète sur le Ranger

Ce sketch implémente le filtre de Kalman 1D pour suivre la position du robot face à un mur :

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

// --- Paramètres du filtre de Kalman ---
// À calibrer avec le sketch de mesure de variance (chapitre 3)
const float SIGMA2_PROCESS   = 0.5;  // σ² encodeurs (bruit de procédé)
const float SIGMA2_MESURE    = 4.0;  // σ² sonar    (bruit de mesure)

// Constantes physiques
const float RAYON_ROUE_CM = 3.25;
const float DEG_PAR_TOUR  = 360.0;

// --- État du filtre ---
float x_estime   = 40.0;   // Position initiale supposée (cm)
float sigma2     = 100.0;  // Incertitude initiale (grande — on ne sait pas)

// Position précédente des encodeurs (en degrés)
float prevDegG = 0, prevDegD = 0;
unsigned long prevTemps = 0;

float degresEnCm(float degres) {
  return (degres / DEG_PAR_TOUR) * (2.0 * PI * RAYON_ROUE_CM);
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(encodeurGauche.getIntNum(), isr_encodeur1, RISING);
  attachInterrupt(encodeurDroit.getIntNum(),  isr_encodeur2, RISING);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  prevTemps = millis();

  // En-têtes pour Serial Plotter
  Serial.println("Sonar\tEstime\t2sigma_plus\t2sigma_moins");
}

void loop() {
  encodeurGauche.loop();
  encodeurDroit.loop();

  unsigned long maintenant = millis();
  float dt = (maintenant - prevTemps) / 1000.0;
  if (dt < 0.05) return;  // Mise à jour à ~20 Hz
  prevTemps = maintenant;

  // -----------------------------------------------
  // PHASE 1 : PRÉDICTION (avec les encodeurs)
  // -----------------------------------------------

  // Déplacement mesuré par les encodeurs depuis le dernier pas
  float degG = encodeurGauche.getCurPos();
  float degD = encodeurDroit.getCurPos();

  float deltaCmG = degresEnCm(degG - prevDegG);
  float deltaCmD = degresEnCm(degD - prevDegD);
  float deplacement = (deltaCmG + deltaCmD) / 2.0;

  prevDegG = degG;
  prevDegD = degD;

  // Le robot avance → la distance au mur diminue
  float x_pred    = x_estime - deplacement;
  float sigma2_pred = sigma2 + SIGMA2_PROCESS;

  // -----------------------------------------------
  // PHASE 2 : CORRECTION (avec le sonar)
  // -----------------------------------------------

  float z = sonar.distanceCm();  // Mesure du sonar

  float K       = sigma2_pred / (sigma2_pred + SIGMA2_MESURE);
  x_estime = x_pred + K * (z - x_pred);
  sigma2   = sigma2_pred * (1.0 - K);

  // -----------------------------------------------
  // AFFICHAGE (compatible Serial Plotter)
  // -----------------------------------------------
  float sigma = sqrt(sigma2);
  Serial.print(z, 2);
  Serial.print("\t");
  Serial.print(x_estime, 2);
  Serial.print("\t");
  Serial.print(x_estime + 2.0 * sigma, 2);
  Serial.print("\t");
  Serial.println(x_estime - 2.0 * sigma, 2);
}
```

## Comment interpréter les courbes

Avec le Serial Plotter, tu verras :

- **Sonar** (ligne verte) : bruyante, qui saute à chaque lecture
- **Estimé** (ligne bleue) : lisse, qui suit la vraie distance sans sauter
- **±2σ** (lignes orange) : la bande d'incertitude — elle rétrécit quand le sonar confirme la prédiction

!!! tip "Jouer avec les paramètres"
    - **Augmente SIGMA2_PROCESS** (plus grande confiance aux encodeurs) : l'estimé suit moins le sonar, mais peut dériver
    - **Augmente SIGMA2_MESURE** (moins confiance au sonar) : l'estimé est plus lisse mais réagit moins vite aux vrais changements
    - Le bon équilibre se trouve expérimentalement avec les valeurs mesurées au chapitre 3

## Le gain K en pratique

```
K proche de 0  →  on fait surtout confiance à la prédiction (encodeurs)
K proche de 1  →  on fait surtout confiance au capteur (sonar)
```

Le filtre calcule automatiquement K à chaque pas, basé sur les variances courantes. C'est là sa beauté : il s'adapte continuellement.

## Résumé

Le filtre de Kalman 1D, c'est deux étapes répétées en boucle :

| Phase | Action | Effet sur σ² |
|-------|--------|-------------|
| **Prédiction** | Avancer avec le modèle (encodeurs) | σ² augmente |
| **Correction** | Fusionner avec la mesure (sonar) | σ² diminue |

La variance oscille autour d'une valeur d'équilibre — elle ne dérive plus à l'infini comme avec les encodeurs seuls. Dans la prochaine section, on étend ça à plusieurs variables simultanées avec la **matrice de covariance**.
