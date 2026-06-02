# Le Filtre de Kalman

## Qu'est-ce que le filtre de Kalman ?

Imagine que tu conduis dans le brouillard. Ton GPS te donne une position, mais avec plusieurs mètres d'erreur. Tes encodeurs de roue te disent quelle distance tu as parcourue, mais l'erreur s'accumule au fil du temps. Comment combiner ces deux sources imparfaites pour obtenir la meilleure estimation possible de ta position ?

C'est exactement ce que fait le **filtre de Kalman** : il fusionne des mesures bruitées provenant de plusieurs capteurs, pondérées par leur niveau de confiance respectif, pour produire une estimation meilleure que n'importe quel capteur seul.

Développé en 1960 par Rudolf Kálmán, cet algorithme a guidé les vaisseaux Apollo vers la Lune. Aujourd'hui, il est partout : robots, drones, voitures autonomes, téléphones, satellites.

## Pourquoi c'est utile sur le MakeBlock Ranger ?

Le Ranger est équipé de trois types de capteurs qui peuvent bénéficier du filtre de Kalman :

| Capteur | Ce qu'il mesure | Son problème |
|---------|-----------------|--------------|
| **Encodeurs** | Vitesse et distance parcourue | L'erreur s'accumule (dérive) |
| **MPU-6050** (gyroscope + accéléromètre) | Orientation et accélération | Dérive du gyroscope, bruit de l'accéléromètre |
| **Sonar** | Distance à un obstacle | Lectures bruyantes, faux échos |

En fusionnant intelligemment ces capteurs, on obtient une estimation bien plus précise de la position et de l'orientation du robot.

## Structure de ce guide

Ce guide construit les idées étape par étape, du plus simple au plus complexe :

1. [Variables aléatoires](01_variables_aleatoires.md) — pourquoi les capteurs ne sont jamais parfaits
2. [Distribution gaussienne](02_distribution_gaussienne.md) — le modèle mathématique du bruit
3. [Variance et qualité des capteurs](03_variance_qualite.md) — comment quantifier la confiance
4. [Fusion de données](04_fusion_donnees.md) — combiner deux capteurs imparfaits
5. [Filtre de Kalman 1D](05_filtre_1d.md) — le filtre complet, en une dimension
6. [Matrice de covariance](06_matrice_covariance.md) — étendre l'incertitude à plusieurs variables
7. [Représentation espace-état](07_espace_etat.md) — modéliser le mouvement du robot
8. [Filtre de Kalman multivarié](08_filtre_multivarie.md) — la version complète, avec des matrices

Chaque section inclut des exemples concrets avec le MakeBlock Ranger et du code Arduino.

## Ce dont tu auras besoin

```cpp
#include <MeAuriga.h>

// Encodeurs des moteurs (slots 1 et 2 sur l'Auriga)
MeEncoderOnBoard encodeurGauche(SLOT1);
MeEncoderOnBoard encodeurDroit(SLOT2);

// Gyroscope / accéléromètre MPU-6050
MeGyro gyro(0, 0x69);

// Sonar ultrasonique
MeUltrasonicSensor sonar(PORT_10);
```

!!! note "Version de la librairie"
    Ces exemples utilisent la librairie **Makeblock Drive Updated** (v3.29+) disponible sur [github.com/nbourre/Makeblock-Libraries](https://github.com/nbourre/Makeblock-Libraries).
