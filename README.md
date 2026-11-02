# MFX Track Station

*[English version](README.en.md)*

Centrale numérique DIY compatible mfx® sur ESP32, pour piloter des locomotives Märklin au format mfx, sans Gleisbox ni Central Station. Le signal de voie est généré par le périphérique RMT de l'ESP32.

*Anciennement directMFX_ESP32 : les anciens liens redirigent ici.*

Ce projet fait l'objet d'un fil sur le forum 3R : https://forum.3rails.fr/t/controler-vos-trains-mfx-depuis-un-arduino-due-sans-passer-par-la-gleisbox/27563

> /!\ Projet en cours de développement, mais fonctionnel : des locomotives mfx se pilotent depuis Rocrail.

### Ce qui fonctionne aujourd'hui

- **Signal mfx** généré par le RMT de l'ESP32 : vitesse (127 crans), direction et fonctions F0 à F15.
- **Pilotage depuis Rocrail** en WiFi, avec le protocole de la CS2 (port TCP 15731). Rocrail peut se déconnecter et se reconnecter sans redémarrer la centrale.
- **La centrale se comporte comme une Gleisbox Märklin**, et Rocrail (option **Master** du pilote MBUS) comme la MS2 : posez une locomotive mfx sur la voie, la centrale la trouve et la signale à Rocrail, qui lui donne une adresse, lit son nom et **crée la locomotive dans sa liste**. Rien à déclarer, ni dans `src/main.cpp`, ni à la main dans Rocrail (pensez seulement à enregistrer le plan). Essayé avec quatre locomotives (deux décodeurs ESU, deux Märklin).
- **Plusieurs locomotives** (16 au plus pilotées en même temps).
- **Présence des locomotives** : chacune est interrogée à tour de rôle, comme le fait la MS2, ce qui garde son inscription ; le moniteur série signale une locomotive qui quitte la voie ou qui y revient.
- **Suppression d'une locomotive**, comme sur la MS2 (ordre du maître, ou commande `D` du moniteur série).
- **Protection contre les courts-circuits** : la voie est coupée au-delà du seuil (1 A par défaut) et ne repart que sur un Go de Rocrail.
- **FreeRTOS** : chaque fonction (signal, réception, surveillance du courant) tourne dans sa propre tâche.

### Ce qui reste à faire

- Fonctions des locomotives dans Rocrail : Rocrail lit les CV 25 à 70, qui ne correspondent pas à l'organisation mfx des décodeurs ; voir ce que fait la MS2.
- Bruit des moteurs avec le comparateur à 1 MΩ : à vérifier pendant une lecture.
- La 040 DE (décodeur ESU récent) ne répond qu'une fois sur deux environ aux questions de la centrale, alors qu'elle répond à la Gleisbox : chantier en cours, au renifleur.
- Écriture des CV.
- Fonctions F16 et au-delà, arrêt d'urgence, réponses à Rocrail (état, surcharge).
- Passage au nouveau pilote RMT d'ESP-IDF 5.
- Circuit imprimé pour le capteur de retour.

### Matériel requis

- **ESP32** : microcontrôleur principal assurant la gestion des communications et du contrôle.
- **Pont en H** (par exemple L293D ou équivalent) : pour la commande de la puissance des rails.
- **Alimentation** : adaptée à la tension requise par les rails Märklin (environ 18 V).

La Box de Locoduino convient parfaitement et apporte une base matérielle complète et éprouvée : https://forum.locoduino.org/index.php?topic=1618.msg20062#msg20062

<img src="include/_DSC1639.jpg" alt="La Box de Locoduino" width="1000">

Voir sur YouTube : https://youtu.be/kIXx-TSOVpQ?si=jwideeNshVRPlX4l

### Retour d'information

Un capteur sur un fil de voie permet aux décodeurs de répondre à la centrale. Il sert à l'**inscription des décodeurs**, reproduite d'après des relevés d'une MS2 et d'une Gleisbox Märklin au renifleur (voie et bus CAN). Comme dans le couple MS2 + Gleisbox, les rôles sont partagés : **la centrale fait le travail de la Gleisbox, et Rocrail celui de la MS2** (le maître, qui choisit les adresses et tient la liste des locomotives).

- **Recherche**, par la centrale : toutes les 2 s, elle cherche les décodeurs mfx qui ne sont pas encore inscrits et lit leur UID bit par bit, comme la Gleisbox (chaque bit est d'abord demandé à 1, puis à 0 sans réponse ; 150 ms de pause après une réponse). L'UID trouvé est **signalé au maître** (commande CAN 0x01, réponse).
- **Attribution de l'adresse** (« bind »), sur ordre du maître (0x02) : envoyée 4 fois à 43 ms d'intervalle, chacune suivie de deux synchronisations et d'un maintien de la voie de 1,6 ms, comme la Gleisbox. Rocrail choisit l'adresse (les 12 bits de poids faible de l'UID : 404 pour l'UID FFE9B194) ; les adresses vont de 1 à 16383.
- **Interrogation** (0x03) et **lecture de CV** (0x07) sur ordre du maître : Rocrail lit ainsi le nom du décodeur, puis crée la locomotive. Une interrogation avec l'UID 0 fait oublier son adresse au décodeur (suppression, comme la MS2).
- **Présence** : la centrale interroge aussi chaque locomotive connue à tour de rôle, entre deux recherches : un décodeur qui n'est plus interrogé pendant 5 à 10 minutes se réinscrit (Krauss, § 4.3). Une locomotive qui ne répond pas 3 fois de suite est considérée hors de la voie et n'est plus interrogée qu'une fois par minute.
- **Ordres de conduite** : comme une Gleisbox, la centrale transmet sur la voie les ordres du maître pour toute adresse mfx, même celle d'une locomotive inscrite lors d'une autre séance.

Face à Rocrail, la centrale se présente comme une Gleisbox : réponse au ping (version 1.47, type 0011, comme la Gleisbox relevée), description de l'appareil (commande 0x1D), identifiant (0x0C), accusé des commandes système (Go, Stop, protocoles de voie, compteur de réinscription).

Le capteur (schéma ci-dessous) :

- **T1** : tore en poudre de fer Micrometals T50-26B. Le fil de voie OUT1 le traverse une fois (un seul des deux fils). Secondaire : 52 spires de fil émaillé soudable 0,20 mm (L ≈ 118 µH).
- **Circuit accordé** sur environ 53 kHz, la porteuse de la réponse des décodeurs : 68 nF + 6,8 nF (polypropylène) et 150 Ω en parallèle sur le secondaire. Son point bas est à VREF = 1,65 V (pont 10 kΩ / 10 kΩ découplé par 1 µF).
- **Comparateur** MCP6541 (DIP-8, 3,3 V) avec une hystérésis d'environ ±16 mV (10 kΩ et **1 MΩ**). Sa sortie va sur **GPIO 26** (`FEEDBACK_PIN` dans `src/main.cpp`).
  *Attention : le schéma ci-dessous montre encore 330 kΩ (±50 mV), la première valeur. Avec 330 kΩ, une réponse faible est perdue dès qu'une deuxième loco est sur la voie (probablement parce que ses condensateurs d'entrée fournissent une partie du courant de la réponse, qui ne passe plus par le tore) : la 241 A ne se lisait plus en présence de la 150 Y. Avec 1 MΩ (essayé avec 3 × 330 kΩ en série), elle se lit sans erreur, et le bruit reste nul quand aucun décodeur ne répond.*

Le +3,3 V et la masse sont ceux de l'ESP32 ; le tore isole le capteur de la voie.

[![Capteur de retour](https://github.com/BOBILLEChristophe/MFX-Track-Station/raw/main/include/capteur_retour.png)](/BOBILLEChristophe/MFX-Track-Station/blob/main/include/capteur_retour.png)

**Retour 1 bit** (`Feedback::ask()`) : après une commande qui attend une réponse (recherche, vérification), la centrale insère sa propre trame (UID de la centrale, compteur), envoie deux synchronisations et les bits 0011, puis maintient la voie à niveau constant pendant deux fenêtres de 6,0 ms, comme la MS2. Un décodeur répond « oui » dans la première en consommant un courant à 52,6 kHz ; la seconde sert de référence pour le bruit. L'ESP32 compte, dans chaque fenêtre, les intervalles d'une période de porteuse (17 à 21 µs) entre deux fronts montants du comparateur (`src/Feedback.cpp`) : une réponse en donne environ 300, le bruit d'un moteur presque aucun, car ses fronts arrivent à intervalles aléatoires. La centrale conclut à une réponse au-delà de 150, avec au moins 120 de plus que dans la fenêtre de référence.

**Lecture des CV** (`Feedback::readCV()`) : la centrale envoie la commande de lecture (`111 000`, numéro de CV, index, nombre d'octets : 1, 2, 4 ou 8), puis une longue pause rythmée par de courtes impulsions (23 toutes les 912 µs, puis toutes les 456 µs). Le décodeur répond pendant toute la pause en inversant la phase de sa porteuse : une inversion à chaque période de 912 µs, et une de plus au milieu pour un bit à 0. L'ESP32 horodate chaque front du comparateur ; `lib/MfxDemod` retrouve les inversions de phase, les bits, les octets, et vérifie la somme de contrôle. Le format a été relevé sur une Gleisbox Märklin et les tests unitaires décodent ces réponses réelles. Au premier allumage de la voie, la centrale lit la configuration de chaque locomotive déclarée (`lib/MfxConfig`, comme la MS2 : table des blocs, nom, fabricant, symboles des fonctions F0 à F15, accélération, freinage, Vmin, Vmax, vitesse du tachymètre) et l'affiche sur le moniteur série ; elle le fait aussi pour chaque nouveau décodeur trouvé. Selon le décodeur, 40 à 70 lectures de 2 octets (comme la MS2), 15 à 50 s par locomotive : la centrale attend 250 ms entre deux lectures, comme la MS2 avec un décodeur ESU. La configuration n'est lue que locomotive arrêtée (le bruit du moteur gênerait la lecture) ; une locomotive qui roule est lue plus tard.

Réponse de la Serie 150X à une lecture de CV, vue à l'oscilloscope (Rigol DS1000Z). En jaune, la sortie du comparateur (GPIO 26) : la porteuse du décodeur, mise en forme. En bleu, GPIO 25 (`Feedback::setScope()`) : haut pendant que l'ESP32 enregistre les fronts.

<img src="include/oscillo_porteuse.jpg" alt="Porteuse de la réponse mesurée aux curseurs : 19,00 µs, 52,63 kHz" width="800">

À 5 µs/div, mesurée aux curseurs : la porteuse a une période de 19,00 µs, soit 52,63 kHz, et une amplitude de 0 à 3,3 V en sortie du comparateur.

<img src="include/oscillo_sauts_de_phase.jpg" alt="Saut de phase au milieu de l'écran, 50 µs/div" width="800">

À 50 µs/div : la fréquence ne change pas, l'information est dans les sauts de phase. Au milieu de l'écran, un créneau plus long décale toute la suite d'une demi-période : c'est un saut de phase. Le décodeur en fait un à chaque limite de bit (toutes les 912 µs), et un de plus au milieu d'un bit à 0.

**Décodeurs essayés** :

| Locomotive | Décodeur | Inscription automatique | Lecture de la configuration |
|---|---|---|---|
| Serie 150X | ESU, première génération (produit `0200000E`) | oui | oui : 60 lectures sans erreur, y compris avec une autre loco en marche sur la voie |
| S. 150 X 1518 | ESU (produit `02000022`) | oui | oui |
| 150 Y 17 | Märklin (produit `0004068B`) | oui | oui : 63 lectures sans erreur |
| 241 A 004 EST | Märklin (produit `00054D47`) | oui, du premier coup | oui : 66 lectures sans erreur |
| 040 DE | ESU récent (produit `020000DA`), avec réserve d'énergie | non : elle manque des questions | partielle : elle se pilote, mais ne répond qu'une fois sur deux environ |

**Commandes du moniteur série** (une lettre, puis Entrée) :

- `L` : liste des locomotives connues de la centrale (adresse, UID, nom).
- `D` suivi de l'adresse (par exemple `D404`) : la centrale envoie à cette adresse une interrogation avec l'UID 0, qui fait oublier son adresse au décodeur (même si la centrale ne la connaît pas), puis l'oublie à son tour. **Voie sous tension** : sinon le décodeur garde son adresse. Si la locomotive est sur la voie, elle se réinscrit aussitôt et Rocrail l'inscrit de nouveau.
- `C` : relecture de la configuration de toutes les locomotives.
- `E` : effacement de toutes les locomotives en mémoire ; le compteur de réinscription est augmenté et la centrale redémarre.
- `R` : réinscription de tous les décodeurs. Le compteur de réinscription est augmenté (et gardé en mémoire), puis la centrale redémarre ; faites ensuite le Go dans Rocrail. Seules les locomotives **posées sur la voie** à ce moment se réinscrivent, une par une (environ 45 s chacune avec la lecture de Rocrail). Un compteur envoyé par le maître (commande système 0x09, comme la MS2) est aussi pris en compte.
- `V` : vérification de chaque locomotive (« décodeur de l'adresse 5, ton UID est-il FFFEBF74 ? »), avec le compte d'intervalles réguliers de chaque fenêtre.
- `?` : aide.

**Traces et oscilloscope** (`src/main.cpp`) : `Feedback::setTrace(cv, search)` affiche une ligne par lecture de CV et/ou par requête 1 bit de la découverte. `Feedback::setScope(GPIO_NUM_25, mode, adresse)` fait passer GPIO 25 au niveau haut pendant les fenêtres du retour 1 bit (`ONE_BIT`), pendant la réponse à une lecture de CV (`DATA`), ou les deux (`BOTH`), pour déclencher un oscilloscope ; `SCOPE_OFF` laisse la broche libre.

**Liste des locos pour Rocrail** (`lib/Cs2Config`) : comme une CS2, la centrale répond à la demande « loks » de Rocrail (commande 0x20) par le fichier `lokomotive.cs2` (commande 0x21) : nom, adresse, UID mfx et, quand la configuration a été lue dans le décodeur, accélération, freinage, Vmin, Vmax, tachymètre et type de chaque fonction. Le format du flux (taille, données zlib, CRC CCITT) est celui de `can2lan` du projet railroad de Gerhard Bertelsmann. La centrale répond aussi au ping de Rocrail comme une CS2.

### Compilation

Le projet se compile avec PlatformIO. La plateforme est figée (`espressif32@6.9.0`, core Arduino 2.0.17) dans `platformio.ini`.

### Configuration du WiFi

Copiez `src/secrets.h.example` en `src/secrets.h`, puis indiquez-y le nom (SSID) et le mot de passe de votre réseau. Ce fichier est ignoré par Git : votre mot de passe ne part jamais sur GitHub.

L'adresse IP fixe de la centrale (`192.168.1.210` par défaut) se règle dans `src/main.cpp`.

### Pour ajouter vos propres locomotives

Avec le capteur de retour, il suffit de poser la locomotive sur la voie (voir « Retour d'information ») : elle est inscrite et gardée en mémoire automatiquement.

Sans capteur, chaque locomotive est déclarée dans `src/main.cpp` avec une adresse (unique) et l'UID mfx de son décodeur :

```cpp
loco[2] = Loco::createLoco(5, "150 X 5", 0xFF, 0xE9, 0xB1, 0x94);
```

L'UID se lit par exemple avec une MS2 et l'exemple `MfxObserver` de [Railuino](https://github.com/BOBILLEChristophe/Railuino).

<img src="include/ajout_locomotives.png" alt="Ajout de locomotives" width="1000">

### Utilisation avec Rocrail

1. Déclarez une centrale **MBUS** (Märklin CAN, sous-bibliothèque TCP), avec l'adresse IP de la centrale et le port **15731**.
2. Dans l'onglet **Options**, cochez **Master** : Rocrail joue alors le rôle de la MS2, donne les adresses et crée les locomotives. Redémarrez Rocrail.
3. Cliquez sur **Go** pour mettre la voie sous tension, puis posez les locomotives : chacune apparaît dans la liste de Rocrail, avec son nom lu dans le décodeur. **Enregistrez le plan** pour garder les locomotives créées.

### Bus CAN (MS2)

La centrale a aussi le bus CAN Märklin (250 kbit/s) de la Box : RX sur GPIO 4, TX sur GPIO 5, avec un transceiver CAN 3,3 V. Une MS2 peut y être branchée, comme sur une Gleisbox. La centrale fait alors passerelle entre le bus et Rocrail, comme une CS2 :

- chaque trame de Rocrail est recopiée sur le bus, chaque trame du bus est transmise à Rocrail ;
- la centrale exécute les ordres venus de l'un ou de l'autre (vitesse, sens, fonctions, arrêt et marche), et y répond comme une Gleisbox : un ordre est confirmé, une question reçoit l'état de la locomotive ;
- elle ne répond pas aux demandes de fichiers de configuration venues du bus : c'est le maître (Rocrail) qui en a la charge. Exception, à l'essai : la demande « ms2-conf » d'une MS2 en esclave, à laquelle la centrale répond elle-même dans le format de can2udp (la réponse de Rocrail est refusée par la MS2) ; `Message::setAnswerMs2Conf(false)` rend la main à Rocrail.

Rocrail reste le maître : la MS2 doit être en esclave. Le journal série affiche chaque trame reçue du bus (lignes `CAN:`, sans les pings) ; `Message::setBusTrace(false)` les supprime. La bibliothèque est ACAN_ESP32 de Pierre Molinaro, version 1.1.2 (les versions 1.2 et suivantes demandent le cœur Arduino 3).

### Contribution

Les contributions sont les bienvenues ! Si vous souhaitez participer, veuillez ouvrir une **issue** ou soumettre une **pull request**.

### Licence et remerciements

Ce projet est distribué sous licence AGPL-3.0 (voir `LICENSE`).

Un grand merci à Stefan Krauss, qui a réalisé un travail colossal sur le protocole mfx : http://www.skrauss.de/modellbahn/Schienenformat.pdf

Ce projet est aussi basé sur un travail de Gérald Litzistorf, qui peut être consulté ici : https://gelit.ch/Train/DirectMFX.ino et https://gelit.ch/Train/MFX.pdf

### Marques

mfx® est une marque déposée de Gebr. Märklin & Cie GmbH. Ce projet est indépendant : il n'est ni affilié à Märklin, ni approuvé ou soutenu par Märklin.

*mfx® is a registered trademark of Gebr. Märklin & Cie GmbH. This project is independent and is not affiliated with, endorsed or sponsored by Märklin.*
