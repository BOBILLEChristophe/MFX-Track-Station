#pragma once

#include <Arduino.h>
#include <unordered_map>

// Configuration read in the mfx decoder (Discovery::readConfig())
struct MfxInfo
{
  bool valid;
  bool complete;      // every read succeeded: no need to read it again
  uint8_t av, bv;     // acceleration, braking (x 0.25 s)
  uint8_t vmin, vmax;
  uint16_t tachoMax;  // km/h
  uint8_t functions;  // functions known (F0 to F15)
  uint8_t fnType[16]; // CS2 function type (Cs2Config::cs2FunctionType()), 0: none
};

class Loco
{
private:
  uint16_t m_addr;
  uint8_t m_uid[4];
  uint8_t m_dir;
  uint8_t m_speed;
  uint8_t m_funct[32];
  uint16_t m_high;     // F16 to F31 switched at least once (bit n = F16 + n)
  uint8_t m_highNext;  // next of them to repeat in the refresh
  String m_name;
  MfxInfo m_mfx;

public:
  Loco(); // Constructeur par défaut : ne modifie aucun état global
  Loco(uint16_t, String, uint8_t, uint8_t, uint8_t, uint8_t);

  uint8_t UID(uint8_t);
  // Rail address (SID): 1 to 16383, chosen by the master
  void addr(uint16_t);
  uint16_t addr() const;
  void dir(uint8_t);
  uint8_t dir() const;
  void toggleDir();
  void speed(uint8_t);
  uint8_t speed() const;
  void funct(uint32_t, bool);
  uint8_t funct(uint32_t) const;
  void toggleFunct(uint32_t);
  // Number of functions handled (F0 to F31, as a CS2)
  static const uint8_t FUNCTIONS = 32;
  // F16 and beyond are sent one by one (MfxFrame::function()); the refresh
  // repeats those switched at least once, in turn. Returns the next one,
  // or -1 if none was switched.
  int8_t nextHighFunction();
  String name() const;
  void name(const String &name);
  MfxInfo mfxInfo() const;
  void mfxInfo(const MfxInfo &info);

  // Copies of the locomotives (at most max), sorted by address. Returns
  // how many were copied.
  static uint8_t list(Loco *out, uint8_t max);

  static void addLoco(const Loco &loco);

  static std::unordered_map<uint16_t, Loco> locoMap;

  // Renvoie nullptr si aucune loco n'a cette adresse.
  static Loco *findLoco(uint16_t);

  // UID du décodeur sur 32 bits (UID(0) = octet de poids fort)
  uint32_t uid32() const;

  // Renvoie nullptr si aucune loco n'a ce décodeur.
  static Loco *findByUid(uint32_t uid);

  // Forgets the locomotive at this address (Message::removeLoco() first:
  // the pointers to it become invalid). Returns false if there is none.
  static bool removeLoco(uint16_t address);

  // Crée la loco, ou remplace celle qui a déjà cette adresse.
  static Loco *createLoco(uint16_t address, String name, uint8_t uid0, uint8_t uid1, uint8_t uid2, uint8_t uid3);
};
