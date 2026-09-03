# Robot Balance V70 Cleanup

Version de référence issue de la **V69 validée**, avec nettoyage du code et quelques améliorations de robustesse sans modifier le comportement de contrôle du robot.

## 1. Matériel

- Carte FPGA : **Terasic DE10-Lite / Intel MAX10 10M50**
- IMU : **MPU9250** en SPI
- 2 moteurs pas à pas en entraînement direct
- Drivers moteurs : **A4988**
- Roues : **Ø 96 mm**
- Bluetooth : **HM-10**
- Application iPhone : **Dabble – GamePad / Joystick**
- Batterie LiPo surveillée par l'ADC interne du MAX10

## 2. Architecture de contrôle

Chaîne principale :

```text
MPU9250
   ↓
Estimation du pitch
   ↓
PID d'équilibre
   ↓
Commande mécanique commune
   ↓
Gestion microstep commune aux 2 moteurs
   ↓
Ajout du différentiel de direction
   ↓
A4988 gauche / droite
```

Boucle vitesse externe :

```text
Joystick Dabble
   ↓
Consigne de vitesse
   ↓
P vitesse + I vitesse
   ↓
Pitch target
   ↓
PID d'équilibre
```

L'intégrale de vitesse est utilisée pour apprendre automatiquement le biais mécanique nécessaire à l'arrêt.

- **Au neutre** : le I vitesse est actif et corrige lentement la dérive.
- **En mouvement avant/arrière** : le I vitesse est gelé (`HOLD`) à sa dernière valeur.
- Il n'y a **plus de PITCH_ZERO / offset fixe** dans la mesure MPU.

## 3. Réglages par défaut au reset

### PID d'équilibre

| Paramètre | Valeur |
|---|---:|
| KP | 150 |
| KI | 20 |
| KD | 0 |

L'intégrale du PID d'équilibre est mise à jour toutes les **10 µs** avec une mise à l'échelle interne `/1000`.

### Boucle vitesse

| Paramètre | Valeur |
|---|---:|
| Gain P vitesse | 1 / 20 |
| Force I vitesse `S` | 100 |
| Limite mémoire I vitesse | ±3,5° |
| Vitesse joystick max | 150 rpm |
| Rampe vitesse | 20 dixièmes de rpm / 10 ms |

`S=100` correspond à la force historique de l'intégrale vitesse.

Exemples :

```text
S 000 = intégrale vitesse réellement désactivée
S 050 = moitié de la force historique
S 100 = force historique
S 200 = deux fois la force historique
```

En V70, **S=0 efface également la mémoire intégrale apprise**, afin que l'intégrale soit réellement désactivée.

## 4. Commande iPhone / Dabble

Communication via HM-10 :

- UART : **115200 bauds, 8N1**
- FPGA RX HM-10 : **PIN_W5**
- FPGA TX HM-10 : **PIN_AA15**

Le mode utilisé dans Dabble est **GamePad / Joystick**.

### Joystick

- Haut / bas : avant / arrière
- Gauche / droite : direction
- Les deux moteurs utilisent toujours le **même mode microstep**, y compris pendant les virages.

La dernière commande joystick est mémorisée jusqu'à réception d'une nouvelle commande.

> Attention : une perte Bluetooth brutale ne peut pas encore être détectée de manière totalement fiable sans utiliser la broche `STATE` du HM-10.

## 5. Réglage des paramètres depuis Dabble

Les anciens réglages par `SW7 / SW8 / SW9` ont été supprimés.

| Bouton Dabble | Action |
|---|---|
| Triangle | KP +1 |
| Carré | KP -1 |
| Rond | KI +1 |
| Croix | KI -1 |
| Start | Force I vitesse `S` +1 |
| Select | Force I vitesse `S` -1 |

KP, KI et S sont codés sur 8 bits et bouclent sur `0…255`.

Exemples :

```text
255 + 1 → 0
0 - 1   → 255
```

Chaque modification déclenche automatiquement l'affichage de la valeur pendant **5 secondes** :

```text
P 150   → KP
I 020   → KI
S 100   → force intégrale vitesse
```

## 6. Microstep adaptatif

Les deux moteurs partagent obligatoirement un **mode microstep commun**.

Modes utilisés :

```text
1/16 → 1/8 → 1/4
```

Seuils en vitesse mécanique équivalente 1/4 :

| Transition | Seuil |
|---|---:|
| 1/16 → 1/8 | 150 |
| 1/8 → 1/16 | 80 |
| 1/8 → 1/4 | 500 |
| 1/4 → 1/8 | 300 |

Le choix du microstep est basé sur la **commande d'équilibre commune avant ajout du virage**. Un ordre de virage seul ne provoque donc plus de changement de résolution indépendant entre les deux moteurs.

### Correction V66 conservée

La correction anti-deadlock des transitions microstep est conservée :

1. chaque moteur rejoint une position compatible avec le futur mode ;
2. il s'arrête et maintient son état `READY` ;
3. les deux moteurs attendent d'être simultanément prêts ;
4. les deux A4988 changent de microstep au même instant ;
5. un délai de garde est appliqué avant reprise des STEP.

Cela évite le blocage historique où le robot pouvait rester en 1/16 alors qu'une résolution plus grossière était demandée.

## 7. Limites moteur

- Commande mécanique maximale : **4000 STEP/s équivalent 1/4**
- Fréquence STEP électrique maximale : **5000 STEP/s**
- STEP HIGH : **2 µs**
- Setup DIR : **5 µs**
- Setup après changement microstep : **5 µs**

Avec des roues Ø 96 mm, 4000 STEP/s en 1/4 correspond théoriquement à environ **5,4 km/h**. La consigne iPhone reste volontairement limitée à **150 rpm**, soit environ **2,7 km/h**, afin de conserver de la réserve de couple pour l'équilibre.

## 8. LEDs

| LED | Fonction |
|---|---|
| LED5 | microstep commun 1/16 |
| LED6 | microstep commun 1/8 |
| LED7 | microstep commun 1/4 |
| LED8 | batterie OK |
| LED9 | défaut batterie |

Si LED5/6/7 sont toutes éteintes alors que les moteurs sont actifs, vérifier la synchronisation du mode microstep commun.

## 9. Afficheurs 7 segments

### SW2

Affichage du **pitch target**.

### SW3

Affichage de la tension LiPo avec 2 décimales :

```text
4.08
3.72
```

### SW4

Diagnostic Dabble / joystick, par exemple :

```text
A090R7
```

avec :

- `Axxx` : angle joystick en degrés
- `R0…R7` : rayon / amplitude joystick

Les affichages temporaires de réglage `P / I / S` ont priorité pendant 5 secondes après un appui Dabble.

## 10. Protection LiPo

La tension batterie est mesurée avec l'ADC interne du MAX10, canal ADC historique du projet d'origine.

Seuils :

| Fonction | Valeur |
|---|---:|
| Coupure batterie | 3,40 V |
| Durée avant coupure | 300 ms |
| Réarmement | 3,55 V |
| Durée avant réarmement | 500 ms |
| Timeout ADC | 250 ms |

En cas de défaut batterie :

```text
PID / commande moteur interdits
A4988 désactivés
LED9 = ON
```

## 11. Sécurité de chute

La coupure moteur intervient à environ :

```text
pitch réel > +45°
ou
pitch réel < -45°
```

Cette sécurité est indépendante du pitch target et de la boucle vitesse.

## 12. Reset et démarrage

La V70 conserve le reset automatique après programmation du FPGA afin que les registres soient initialisés sans devoir actionner manuellement le switch RESET.

Valeurs après programmation :

```text
KP = 150
KI = 20
KD = 0
S  = 100
```

`ROT_EN` est désormais synchronisé par deux flip-flops avant utilisation dans la logique synchrone FPGA.

## 13. Timing / SDC

Horloge principale :

```text
50 MHz
T = 20 ns
```

Le fichier `top.sdc` contient les contraintes principales du design et les chemins asynchrones liés aux entrées/sorties de la carte.

La V70 nettoie les commentaires SDC sans changer le modèle de timing validé de la V69.

## 14. Changements V70 par rapport à V69

La V70 est une version de **cleanup / robustesse**, pas une nouvelle version de contrôle.

Changements :

- synchronisation 2 FF de `ROT_EN` ;
- `S=0` désactive réellement l'intégrale vitesse et efface sa mémoire ;
- suppression de plusieurs signaux et paramètres devenus inutiles ;
- nettoyage des commentaires obsolètes ;
- correction des commentaires batterie vers 3,40 / 3,55 V ;
- mise à jour des commentaires de gains vers KP=150 / KI=20 ;
- nettoyage du `top.sdc` ;
- mise à jour des commentaires du projet Quartus.

Comportement de contrôle volontairement conservé par rapport à la V69 validée.

## 15. Version de référence

- **V69** : version fonctionnelle validée avant cleanup
- **V70** : version nettoyée destinée à devenir la nouvelle base si les essais matériels confirment un comportement identique

