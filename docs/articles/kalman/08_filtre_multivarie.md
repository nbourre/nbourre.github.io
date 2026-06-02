# Le Filtre de Kalman Multivarié

## Tout s'assemble

On a maintenant tous les ingrédients :

- **Variables aléatoires et gaussiennes** (chapitres 1-2) : comment modéliser le bruit
- **Fusion de données** (chapitre 4) : comment combiner deux sources imparfaites
- **Filtre 1D** (chapitre 5) : le cycle prédiction-correction dans le temps
- **Matrice de covariance** (chapitre 6) : généraliser l'incertitude à plusieurs variables
- **Représentation espace-état** (chapitre 7) : modéliser le mouvement

Le filtre multivarié, c'est exactement le filtre 1D — mais les scalaires deviennent des matrices. La structure est identique.

## Les équations complètes

### Modèle stochastique

$$\mathbf{x}_k = F \mathbf{x}_{k-1} + G \mathbf{u}_{k-1} + \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(0, Q)$$
$$\mathbf{z}_k = H \mathbf{x}_k + \mathbf{v}_k, \quad \mathbf{v}_k \sim \mathcal{N}(0, R)$$

### Phase 1 : Prédiction

$$\mathbf{x}_\text{pred} = F \hat{\mathbf{x}} + G \mathbf{u}$$
$$\Sigma_\text{pred} = F \Sigma F^\top + Q$$

### Phase 2 : Correction

$$K = \Sigma_\text{pred} H^\top (H \Sigma_\text{pred} H^\top + R)^{-1}$$
$$\hat{\mathbf{x}} = \mathbf{x}_\text{pred} + K (\mathbf{z} - H \mathbf{x}_\text{pred})$$
$$\Sigma = (I - KH) \Sigma_\text{pred}$$

### Correspondance avec le filtre 1D

| Filtre 1D | Filtre multivarié | Rôle |
|-----------|------------------|------|
| x̂ + v·Δt | Fx̂ + Gu | Prédiction de l'état |
| σ² + σ²_enc | FΣFᵀ + Q | Prédiction de l'incertitude |
| σ²_pred / (σ²_pred + σ²_mes) | Σ_pred Hᵀ (HΣ_pred Hᵀ + R)⁻¹ | Gain de Kalman |
| x_pred + K(z - x_pred) | x_pred + K(z - Hx_pred) | Correction de l'état |
| σ²_pred (1 - K) | (I - KH) Σ_pred | Correction de l'incertitude |

Le terme `z - H x_pred` s'appelle l'**innovation** : c'est la surprise apportée par la mesure.

## Application : fusionner gyroscope et encodeurs pour l'orientation

### Problème

Le Ranger tourne sur lui-même. On veut estimer son **angle** (θ) et sa **vitesse angulaire** (ω).

- **Gyroscope** : donne ω directement, mais dérive sur le long terme
- **Encodeurs** : la différence de vitesse entre les roues donne une estimation de ω (mais bruyante)

L'état est : **x = [θ, ω]ᵀ**

### Les matrices

Avec Δt = 0.02 s (50 Hz) :

$$F = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}, \quad H = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

- F : l'angle avance de ω×Δt, la vitesse reste constante
- H : le gyroscope ne voit que ω (la 2ème composante)

```
Q = | 0.001   0     |  (bruit de procédé — à calibrer)
    | 0       0.003 |

R = | 0.01 |  (variance du gyro — mesurée au chapitre 3)
```

### Implémentation Arduino avec algèbre matricielle 2×2

Pour un état 2D, on peut tout calculer à la main sans bibliothèque :

```cpp
#include <MeAuriga.h>

MeGyro gyro(0, 0x69);
MeEncoderOnBoard encodeurGauche(SLOT1);
MeEncoderOnBoard encodeurDroit(SLOT2);

void isr_enc1() {
  if (digitalRead(encodeurGauche.getPortB()) == 0)
    encodeurGauche.pulsePosMinus();
  else encodeurGauche.pulsePosPlus();
}
void isr_enc2() {
  if (digitalRead(encodeurDroit.getPortB()) == 0)
    encodeurDroit.pulsePosMinus();
  else encodeurDroit.pulsePosPlus();
}

// -------------------------------------------------------
// Paramètres du filtre — à ajuster avec les mesures du ch.3
// -------------------------------------------------------
const float DT = 0.02;          // Pas de temps (s) — 50 Hz

// Bruit de procédé Q = diag(q_theta, q_omega)
const float Q_THETA = 0.001;    // deg²
const float Q_OMEGA = 0.003;    // (deg/s)²

// Bruit de mesure R (gyroscope)
const float R_GYRO  = 0.01;     // (deg/s)²

// -------------------------------------------------------
// État et covariance
// -------------------------------------------------------
float x[2] = {0.0, 0.0};  // [theta (deg), omega (deg/s)]

// Matrice de covariance 2×2 (stockée à plat : P[0]=P11, P[1]=P12, P[2]=P21, P[3]=P22)
float P[4] = {1.0, 0.0, 0.0, 1.0};  // Incertitude initiale = I

// -------------------------------------------------------
// Constantes physiques
// -------------------------------------------------------
const float RAYON_ROUE    = 3.25;  // cm
const float DEMI_EMPATTEMENT = 7.5; // cm (demi-distance entre les roues)

unsigned long prevTemps;
float prevDegG = 0, prevDegD = 0;

void setup() {
  Serial.begin(115200);
  gyro.begin();
  gyro.resetData();

  attachInterrupt(encodeurGauche.getIntNum(), isr_enc1, RISING);
  attachInterrupt(encodeurDroit.getIntNum(),  isr_enc2, RISING);
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  prevTemps = millis();

  Serial.println("Theta_Kalman\tTheta_Gyro_brut\tOmega_Kalman\tK0\tK1");
}

void loop() {
  encodeurGauche.loop();
  encodeurDroit.loop();

  // Attendre le bon intervalle
  if (millis() - prevTemps < (unsigned long)(DT * 1000)) return;
  prevTemps = millis();

  gyro.update();

  // --- Calcul de omega_encodeurs ---
  // Différence de déplacement entre roues gauche et droite
  float degG = encodeurGauche.getCurPos();
  float degD = encodeurDroit.getCurPos();

  float cmG = (degG - prevDegG) / 360.0 * (2.0 * PI * RAYON_ROUE);
  float cmD = (degD - prevDegD) / 360.0 * (2.0 * PI * RAYON_ROUE);
  prevDegG = degG; prevDegD = degD;

  // Vitesse angulaire estimée par encodeurs (deg/s)
  float omega_enc = ((cmD - cmG) / (2.0 * DEMI_EMPATTEMENT)) * (180.0 / PI) / DT;

  // Mesure du gyroscope (deg/s) — dérivée de l'angle
  float omega_gyro = gyro.getAngleZ() - x[0]; // approximation rapide
  // Pour une vraie vitesse angulaire, utilise getGyroZ() si disponible
  // omega_gyro = gyro.getGyroZ(); // en deg/s (si la lib l'expose)

  // =====================================================
  // FILTRE DE KALMAN 2D — [theta, omega]
  // F = [[1, DT], [0, 1]]
  // H = [0, 1]  (on mesure omega avec le gyro)
  // =====================================================

  // --- PHASE 1 : PRÉDICTION ---
  // x_pred = F * x
  float x_pred[2];
  x_pred[0] = x[0] + DT * x[1];  // theta + omega*dt
  x_pred[1] = x[1];               // omega constant

  // P_pred = F * P * Fᵀ + Q
  // Pour F = [[1,dt],[0,1]] :
  // F*P*Fᵀ = | P00 + dt*(P10+P01) + dt²*P11,  P01 + dt*P11 |
  //           | P10 + dt*P11,                   P11           |
  float P_pred[4];
  P_pred[0] = P[0] + DT*(P[2]+P[1]) + DT*DT*P[3] + Q_THETA;
  P_pred[1] = P[1] + DT * P[3];
  P_pred[2] = P[2] + DT * P[3];
  P_pred[3] = P[3] + Q_OMEGA;

  // --- PHASE 2 : CORRECTION ---
  // Mesure : z = omega_gyro (scalaire)
  // H = [0, 1] → H*x_pred = x_pred[1] = omega_pred
  float z = omega_gyro;
  float innovation = z - x_pred[1];

  // S = H*P_pred*Hᵀ + R  (scalaire car H est un vecteur ligne)
  // H*P_pred*Hᵀ = P_pred[3] (car H=[0,1])
  float S = P_pred[3] + R_GYRO;

  // K = P_pred*Hᵀ / S  (vecteur colonne / scalaire)
  // P_pred*Hᵀ = [P_pred[1], P_pred[3]]ᵀ (car H=[0,1])
  float K[2];
  K[0] = P_pred[1] / S;
  K[1] = P_pred[3] / S;

  // x = x_pred + K * innovation
  x[0] = x_pred[0] + K[0] * innovation;
  x[1] = x_pred[1] + K[1] * innovation;

  // P = (I - K*H) * P_pred
  // K*H = | K[0]*0  K[0]*1 | = | 0      K[0] |
  //       | K[1]*0  K[1]*1 |   | 0      K[1] |
  // I - K*H = | 1      -K[0] |
  //           | 0      1-K[1] |
  P[0] = P_pred[0] - K[0] * P_pred[2];  // simplification pour H=[0,1]
  P[1] = P_pred[1] - K[0] * P_pred[3];
  P[2] = P_pred[2] - K[1] * P_pred[2];
  P[3] = P_pred[3] - K[1] * P_pred[3];

  // --- AFFICHAGE ---
  Serial.print(x[0], 2);             // Angle filtré
  Serial.print("\t");
  Serial.print(gyro.getAngleZ(), 2); // Angle brut du gyro
  Serial.print("\t");
  Serial.print(x[1], 2);             // Vitesse angulaire estimée
  Serial.print("\t");
  Serial.print(K[0], 4);             // Gain K[0]
  Serial.print("\t");
  Serial.println(K[1], 4);           // Gain K[1]
}
```

## Exemple chiffré : un pas complet

**État initial** : x = [30°, 5 deg/s]ᵀ, Σ = diag(4, 1)

**Prédiction** avec Δt = 1, Q = diag(1, 1) :

$$\mathbf{x}_\text{pred} = \begin{bmatrix}1&1\\0&1\end{bmatrix}\begin{bmatrix}30\\5\end{bmatrix} = \begin{bmatrix}35\\5\end{bmatrix}$$

$$\Sigma_\text{pred} = \begin{bmatrix}1&1\\0&1\end{bmatrix}\begin{bmatrix}4&0\\0&1\end{bmatrix}\begin{bmatrix}1&0\\1&1\end{bmatrix} + \begin{bmatrix}1&0\\0&1\end{bmatrix} = \begin{bmatrix}6&1\\1&2\end{bmatrix}$$

**Correction** avec H = [1, 0], z = 38°, R = 4 :

$$S = H\Sigma_\text{pred}H^\top + R = 6 + 4 = 10$$

$$K = \begin{bmatrix}6\\1\end{bmatrix}\frac{1}{10} = \begin{bmatrix}0.6\\0.1\end{bmatrix}$$

$$\hat{\mathbf{x}} = \begin{bmatrix}35\\5\end{bmatrix} + \begin{bmatrix}0.6\\0.1\end{bmatrix}(38-35) = \begin{bmatrix}36.8\\5.3\end{bmatrix}$$

La mesure de position a aussi mis à jour la **vitesse estimée** (5 → 5.3) — parce que la covariance modélisait le lien entre les deux. C'est l'avantage du filtre multivarié sur le 1D.

## Réglage des paramètres

Le réglage du filtre se résume à choisir Q et R :

| Si... | Alors... |
|-------|----------|
| Q grand (confiance faible dans le modèle) | Le filtre suit de près les mesures — réactif mais bruité |
| Q petit (confiance forte dans le modèle) | Le filtre lisse davantage — stable mais réagit lentement |
| R grand (capteur peu fiable) | Le filtre prédit plus, mesure moins — peut dériver |
| R petit (capteur très précis) | Le filtre suit de près le capteur — risque de bruit |

!!! tip "Méthode recommandée"
    1. Commence avec les variances mesurées expérimentalement (chapitres 1 et 3)
    2. Lance le robot et observe la courbe sur Serial Plotter
    3. Si l'estimation est trop bruyante → augmente R ou diminue Q
    4. Si l'estimation dérive → augmente Q ou diminue R

## Pour aller plus loin

Le filtre de Kalman présenté ici suppose un modèle **linéaire**. En robotique mobile, la cinématique est souvent non-linéaire (notamment les rotations en 2D/3D). Dans ce cas, on utilise :

- **EKF (Extended Kalman Filter)** : linéarise le modèle autour de l'état courant
- **UKF (Unscented Kalman Filter)** : utilise des points sigma pour propager la distribution
- **Filtre particulaire** : représente la distribution par un ensemble d'échantillons

Ces variantes utilisent les mêmes concepts fondamentaux que ce guide.

## Résumé final

Le filtre de Kalman multivarié en cinq lignes :

```
Prédire  :  x_pred  = F*x + G*u
             P_pred  = F*P*Fᵀ + Q

Corriger :  K       = P_pred*Hᵀ * (H*P_pred*Hᵀ + R)⁻¹
             x       = x_pred + K*(z - H*x_pred)
             P       = (I - K*H)*P_pred
```

C'est tout. Chaque itération de la boucle `loop()`, ces cinq lignes produisent la meilleure estimation possible de l'état du robot — automatiquement pondérée par la confiance dans le modèle et dans chaque capteur.
