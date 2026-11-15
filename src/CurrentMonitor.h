
/**********************************************************************

CurrentMonitor.h

**********************************************************************/

#ifndef CURRENTMONITOR_H
#define CURRENTMONITOR_H

#include "Arduino.h"
#include "Centrale.h"
#include "Board.h"

#define CURRENT_SAMPLE_SMOOTHING 0.01f

/*
 La mesure se lit en millivolts (analogReadMilliVolts), puis se convertit en mA.
 La tension max acceptable sur la broche de l'ESP32 est de 3,3 V.

 CURRENT_MV_PER_A : tension fournie par le capteur pour 1 A.
 1000 mV par A correspond au commentaire d'origine (1000 mV -> 1 A).
 A VERIFIER sur le schéma de votre carte (La Box) avant de se fier au seuil.
*/
#if defined(BOARD_CURRENT_MV_PER_A)
#define CURRENT_MV_PER_A BOARD_CURRENT_MV_PER_A
#else
#define CURRENT_MV_PER_A 1000
#endif

// Seuil de coupure, en mA. La limite est ici fixée à 1 A.
#if defined(BOARD_MAX_CURRENT_MA)
#define MAX_CURRENT_MA BOARD_MAX_CURRENT_MA
#else
#define MAX_CURRENT_MA 1000
#endif

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
