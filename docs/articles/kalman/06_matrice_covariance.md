# La Matrice de Covariance

## Limites du filtre 1D

Dans le filtre 1D, l'incertitude était un seul nombre : σ². Mais en robotique, on suit souvent plusieurs quantités à la fois :

- Position **et** vitesse
- Coordonnées **x et y**
- Angle **et** vitesse angulaire

Quand on a plusieurs variables, une subtilité apparaît : elles peuvent être **corrélées**. Et ignorer cette corrélation, c'est perdre de l'information précieuse.

## L'intuition : la corrélation entre variables

Imagine le Ranger qui tourne sur lui-même. Sa vitesse angulaire (gyro) et la différence de vitesse entre les deux roues (encodeurs) sont liées — si l'une augmente, l'autre aussi. Ce lien, c'est la **corrélation**.

Exemples de variables corrélées :

- **Corrélation positive** : quand la vitesse augmente, la distance parcourue augmente aussi
- **Corrélation négative** : quand la distance à un mur diminue, le sonar lit une valeur plus petite
- **Non corrélées** : la température ambiante et la vitesse du robot

Dans un nuage de points, la corrélation se voit à l'inclinaison :
- Corrélation positive → nuage incliné vers le haut-droite
- Corrélation négative → incliné vers le bas-droite
- Nulle → nuage rond ou horizontal/vertical

## La covariance

La covariance entre deux variables x₁ et x₂ mesure leur tendance à varier ensemble :

$$\sigma_{x_1 x_2} = \frac{1}{N} \sum_{k=1}^{N} (x_1[k] - \mu_{x_1})(x_2[k] - \mu_{x_2})$$

- σ > 0 → corrélation positive (elles augmentent ensemble)
- σ < 0 → corrélation négative (l'une monte quand l'autre descend)
- σ ≈ 0 → pas de lien apparent

## La matrice de covariance

Pour un vecteur d'état à deux variables [x₁, x₂]ᵀ, on regroupe toutes les variances et covariances dans une matrice :

$$\Sigma = \begin{bmatrix} \sigma_{x_1}^2 & \sigma_{x_1 x_2} \\ \sigma_{x_2 x_1} & \sigma_{x_2}^2 \end{bmatrix}$$

- **Diagonale** : les variances individuelles (comme les σ² du filtre 1D)
- **Hors-diagonale** : les covariances (les corrélations entre paires de variables)
- La matrice est toujours **symétrique** : σ_{x₁x₂} = σ_{x₂x₁}

Pour un état de dimension n, la matrice de covariance est de taille n×n.

## Application au Ranger : position et vitesse angulaire

Considère l'état [θ, ω]ᵀ = [angle, vitesse angulaire]. Ces deux variables sont corrélées : si ω est positif depuis longtemps, θ a probablement augmenté. La matrice de covariance capture ce lien.

```cpp
// Exemple : matrice de covariance 2×2 pour [angle, vitesse_angulaire]
//
//  Σ = | σ²_θ        σ_θω    |
//      | σ_ωθ        σ²_ω    |
//
//  Σ = | 0.1    0.02 |   (σ²_θ = 0.1 deg²,  σ²_ω = 0.5 (deg/s)²)
//      | 0.02   0.5  |   (covariance = 0.02 — légèrement corrélés)
```

## Comment la matrice de covariance se transforme

Dans le filtre 1D, on avait la règle : `Var(ax) = a² × Var(x)`.

La généralisation matricielle est : si on applique une transformation linéaire **y = Ax + b**, alors :

$$\Sigma_y = A \Sigma_x A^\top$$

C'est la règle fondamentale que le filtre de Kalman utilise à chaque étape de prédiction. La constante **b** déplace la moyenne mais ne change pas la covariance.

**Vérification simple** : si on étire x₁ d'un facteur 2 (A = diag(2, 1)), la variance de x₁ quadruple — exactement comme la règle scalaire.

## Exemple : l'ellipse d'incertitude

Au lieu d'une bande d'incertitude ±2σ (comme dans le filtre 1D), la covariance 2D se visualise comme une **ellipse** :

- La **taille** de l'ellipse représente les variances
- L'**inclinaison** représente la corrélation

```
Sans corrélation (ρ = 0) :  ellipse alignée avec les axes
Corrélation positive       :  ellipse inclinée vers le haut-droite
Corrélation négative       :  ellipse inclinée vers le bas-droite
```

Le filtre de Kalman maintient cette ellipse à chaque pas : elle grandit à la prédiction, elle rétrécit à la correction.

## Mesure de covariance sur le Ranger

```cpp
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);

const int N = 500;
float anglesZ[N];
float vitessesZ[N]; // en deg/s (différence d'angles / dt)

void setup() {
  Serial.begin(115200);
  gyro.begin();
  delay(500);

  Serial.println("Collecte en cours (robot immobile)...");

  float prevAngle = 0;
  unsigned long prevTemps = millis();

  for (int i = 0; i < N; i++) {
    gyro.update();
    unsigned long t = millis();
    float dt = (t - prevTemps) / 1000.0;
    prevTemps = t;

    anglesZ[i] = gyro.getAngleZ();

    // Vitesse angulaire estimée par différence finie
    if (i > 0 && dt > 0) {
      vitessesZ[i] = (anglesZ[i] - anglesZ[i-1]) / dt;
    } else {
      vitessesZ[i] = 0;
    }
    delay(20);
  }

  // Calcul des moyennes
  float muAngle = 0, muVitesse = 0;
  for (int i = 0; i < N; i++) {
    muAngle   += anglesZ[i];
    muVitesse += vitessesZ[i];
  }
  muAngle   /= N;
  muVitesse /= N;

  // Calcul des variances et covariance
  float varAngle = 0, varVitesse = 0, covar = 0;
  for (int i = 0; i < N; i++) {
    float dA = anglesZ[i]   - muAngle;
    float dV = vitessesZ[i] - muVitesse;
    varAngle   += dA * dA;
    varVitesse += dV * dV;
    covar      += dA * dV;
  }
  varAngle   /= N;
  varVitesse /= N;
  covar      /= N;

  Serial.println("\n=== Matrice de covariance [angle, vitesse] ===");
  Serial.print("| ");  Serial.print(varAngle, 5);
  Serial.print("  ");  Serial.print(covar, 5);
  Serial.println(" |");
  Serial.print("| ");  Serial.print(covar, 5);
  Serial.print("  ");  Serial.print(varVitesse, 5);
  Serial.println(" |");
}

void loop() {}
```

## Pourquoi c'est important pour Kalman

Dans le filtre multivarié (chapitre 8), la matrice de covariance Σ **remplace** le scalaire σ². Elle joue exactement le même rôle :

| Filtre 1D | Filtre multivarié |
|-----------|------------------|
| σ² (scalaire) | Σ (matrice n×n) |
| σ² augmente à la prédiction | Σ = FΣFᵀ + Q |
| σ² diminue à la correction | Σ = (I - KH)Σ |

La diagonale de Σ donne les incertitudes sur chaque variable. Les termes hors-diagonale donnent les corrélations — informations que le filtre 1D ignorait.

## Résumé

| Concept | Rôle |
|---------|------|
| σ² (scalaire) | Incertitude sur une seule variable |
| Matrice de covariance Σ | Incertitude sur toutes les variables + leurs corrélations |
| Diagonale de Σ | Variances individuelles |
| Hors-diagonale de Σ | Covariances (liens entre variables) |
| Ellipse d'incertitude | Visualisation 2D de Σ |

Dans la prochaine section, on voit comment modéliser le **mouvement du robot** avec la représentation espace-état — la dernière pièce avant le filtre complet.
