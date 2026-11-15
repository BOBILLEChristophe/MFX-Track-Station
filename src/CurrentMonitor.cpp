/**********************************************************************

CurrentMonitor.cpp

**********************************************************************/

#include <Arduino.h>
#include "CurrentMonitor.h"

///////////////////////////////////////////////////////////////////////////////

/*
 Surveillance du courant.
 En cas de surcharge, la voie est coupée et le reste jusqu'au prochain Go de
 Rocrail : plus de remise sous tension automatique, qui faisait un cycle sans fin
 sur un court-circuit persistant et réalimentait la voie après un Stop.
 Pas de printf de flottant ici : la tâche a une petite pile.
*/
void currentMonitorTask(void *p)
{
  // Récupérer le pointeur vers l'instance de CurrentMonitor
  CurrentMonitor *monitor = static_cast<CurrentMonitor *>(p);

  TickType_t xLastWakeTime = xTaskGetTickCount(); // Obtenir l'heure actuelle pour le délai
  bool wasPowered = false;

  for (;;)
  {
    const bool powered = Centrale::power();

    // Remise sous tension : on repart d'une mesure nulle, sinon la moyenne
    // glissante de la surcharge précédente redéclencherait aussitôt.
    if (powered && !wasPowered)
      monitor->m_current = 0;
    wasPowered = powered;

    // Lecture en mV, conversion en mA, lissage
    const float mA = analogReadMilliVolts(monitor->m_pin) * 1000.0f / CURRENT_MV_PER_A;
    monitor->m_current = mA * CURRENT_SAMPLE_SMOOTHING +
                         monitor->m_current * (1.0f - CURRENT_SAMPLE_SMOOTHING);

    // Protection matérielle (carte proto 2) : elle a déjà coupé ENABLE ;
    // la voie passe en surcharge, comme pour la mesure ci-dessous
    if (powered && Centrale::protectionTripped())
    {
      Centrale::setOverload();
      Serial.printf("Surintensite : protection materielle (FAULT, > 4,1 A), voie coupee jusqu'au prochain Go\n");
      monitor->m_current = 0;
    }
    // Vérification si le courant dépasse la limite
    else if (powered && monitor->m_current > MAX_CURRENT_MA)
    {
      Centrale::setOverload();
      Serial.printf("Surcharge : %ld mA, voie coupee jusqu'au prochain Go\n",
                    (long)monitor->m_current);
      monitor->m_current = 0;
    }

    // Délai pour la prochaine itération
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CURRENT_SAMPLE_TIME));
  }
}

/*
 Affichage du courant : seulement quand il a changé d'au moins
 CURRENT_DISPLAY_STEP_MA depuis le dernier affichage (une loco démarre, la voie
 est mise sous tension...), ou quand l'état de surcharge change. Vérifié chaque
 seconde. CURRENT_DISPLAY_STEP_MA à 0 : affichage chaque seconde, comme avant.
*/
void currentDisplayTask(void *p)
{
  // Récupérer le pointeur vers l'instance de CurrentMonitor
  CurrentMonitor *monitor = static_cast<CurrentMonitor *>(p);
  long shown = -1000; // rien d'affiché encore
  bool shownOverload = false;

  for (;;)
  {
    const long mA = (long)monitor->m_current;
    const bool overload = Centrale::overload();
    const long change = mA > shown ? mA - shown : shown - mA;
    if (change >= CURRENT_DISPLAY_STEP_MA || overload != shownOverload)
    {
      Serial.printf("Courant mesure : %ld mA%s\n", mA, overload ? " (surcharge)" : "");
      shown = mA;
      shownOverload = overload;
    }

    // Attendre une seconde avant la prochaine vérification
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
