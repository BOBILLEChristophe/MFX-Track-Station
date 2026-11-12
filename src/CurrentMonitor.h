
/**********************************************************************

CurrentMonitor.h

**********************************************************************/

#ifndef CURRENTMONITOR_H
#define CURRENTMONITOR_H

#include "Arduino.h"
#include "Centrale.h"

#define CURRENT_SAMPLE_SMOOTHING 0.01f

/*
 La mesure se lit en millivolts (analogReadMilliVolts), puis se convertit en mA.
 La tension max acceptable sur la broche de l'ESP32 est de 3,3 V.

 CURRENT_MV_PER_A : tension fournie par le capteur pour 1 A.
 Étalonné sur le prototype le 12 novembre 2026 : charge de 4,7 Ω à la place
 de la voie, alimentation de labo, ampèremètre dans le + de l'alimentation.
   ampèremètre 1,52 A -> 1746 mV ; 1,83 A -> 2104 mV : 1150 mV par A, zéro à 0.
   1,83 A, 2,07 A, 2,31 A -> 2104, 1903, 2102 mV : la mesure SATURE vers
   2,1 V, soit environ 1,8 A. Au-delà, la station ne voit plus le courant
   monter : le seuil de coupure doit rester en dessous.
 Une autre carte (nouveau circuit, autre shunt) est à étalonner de la même façon.
*/
#define CURRENT_MV_PER_A 1150

// Seuil de coupure, en mA réels : 1,6 A, sous la saturation de la mesure
// (environ 1,8 A sur le prototype). La moyenne sur environ 100 ms laisse passer
// les pointes brèves (démarrage d'une loco, décodeur sonore).
#define MAX_CURRENT_MA 1600

#define CURRENT_SAMPLE_TIME 1 // ms

// Affichage du courant sur le port série : seulement après un changement
// d'au moins cette valeur, en mA (0 : chaque seconde)
#define CURRENT_DISPLAY_STEP_MA 50

class CurrentMonitor
{
public:
    gpio_num_t m_pin;
    float m_current; // mA, lissé
    // Constructeur
    CurrentMonitor(gpio_num_t pin)
        : m_pin(pin), m_current(0)
    {
    }
};

#endif
