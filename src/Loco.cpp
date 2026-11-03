/*

  Loco.cpp


*/

#include "Loco.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// The map is read by the Rocrail task and filled by the decoder discovery:
// every access goes through this mutex. Pointers to the locomotives stay
// valid when the map grows (std::unordered_map keeps its nodes).
static SemaphoreHandle_t mapMutex()
{
  static SemaphoreHandle_t m = xSemaphoreCreateMutex();
  return m;
}

struct MapLock
{
  MapLock() { xSemaphoreTake(mapMutex(), portMAX_DELAY); }
  ~MapLock() { xSemaphoreGive(mapMutex()); }
};

// Constructeur par défaut.
// Il n'incrémente plus le compteur de réinscription MFX : unordered_map::operator[]
// appelle ce constructeur, et chaque appel changeait le compteur diffusé aux
// décodeurs, ce qui forçait la réinscription de toutes les locos.
Loco::Loco() : m_addr(0), m_dir(0), m_speed(0), m_name("")
{
  memset(m_uid, 0, sizeof(m_uid));
  memset(m_funct, 0, sizeof(m_funct));
  m_high = 0;
  m_highNext = 0;
  memset(&m_mfx, 0, sizeof(m_mfx));
}

Loco::Loco(uint16_t address, String name, uint8_t uid0, uint8_t uid1, uint8_t uid2, uint8_t uid3)
    : m_addr(address),
      m_uid{uid0, uid1, uid2, uid3},
      m_dir(0),
      m_speed(0),
      m_name(name)
{
  memset(m_funct, 0, sizeof(m_funct));
  m_high = 0;
  m_highNext = 0;
  memset(&m_mfx, 0, sizeof(m_mfx));
}

byte Loco::UID(byte i) { return i < 4 ? m_uid[i] : 0; }
void Loco::addr(uint16_t address) { m_addr = address; }
uint16_t Loco::addr() const { return m_addr; }
void Loco::dir(uint8_t dir) { m_dir = dir ? 1 : 0; }
uint8_t Loco::dir() const { return m_dir; }
void Loco::toggleDir() { m_dir ^= 1; }
void Loco::speed(uint8_t speed) { m_speed = speed; }
uint8_t Loco::speed() const { return m_speed; }
void Loco::funct(uint32_t x, bool y)
{
  if (x < FUNCTIONS)
  {
    m_funct[x] = y;
    if (x >= 16)
      m_high |= (uint16_t)(1u << (x - 16));
  }
}
int8_t Loco::nextHighFunction()
{
  if (m_high == 0)
    return -1;
  for (uint8_t i = 0; i < 16; i++)
  {
    const uint8_t n = (uint8_t)((m_highNext + i) & 15);
    if (m_high & (1u << n))
    {
      m_highNext = (uint8_t)((n + 1) & 15);
      return (int8_t)(16 + n);
    }
  }
  return -1;
}
uint8_t Loco::funct(uint32_t x) const { return x < 32 ? m_funct[x] : 0; }
void Loco::toggleFunct(uint32_t x)
{
  if (x < FUNCTIONS)
    funct(x, !m_funct[x]);
}
String Loco::name() const { return m_name; }

// The name and the configuration are written by the discovery task while
// the Rocrail task may copy the list: under the mutex of the map.
void Loco::name(const String &name)
{
  MapLock lock;
  m_name = name;
}

MfxInfo Loco::mfxInfo() const
{
  MapLock lock;
  return m_mfx;
}

void Loco::mfxInfo(const MfxInfo &info)
{
  MapLock lock;
  m_mfx = info;
}

uint8_t Loco::list(Loco *out, uint8_t max)
{
  MapLock lock;
  uint8_t n = 0;
  for (auto &entry : locoMap)
  {
    if (n >= max)
      break;
    // Insertion, sorted by address
    uint8_t i = n++;
    while (i > 0 && out[i - 1].addr() > entry.second.addr())
    {
      out[i] = out[i - 1];
      i--;
    }
    out[i] = entry.second;
  }
  return n;
}

void Loco::addLoco(const Loco &loco)
{
  MapLock lock;
  locoMap[loco.addr()] = loco;
}

uint32_t Loco::uid32() const
{
  return (uint32_t)m_uid[0] << 24 | (uint32_t)m_uid[1] << 16 | (uint32_t)m_uid[2] << 8 | m_uid[3];
}

Loco *Loco::findByUid(uint32_t uid)
{
  MapLock lock;
  for (auto &entry : locoMap)
    if (entry.second.uid32() == uid)
      return &entry.second;
  return nullptr;
}

// Utilisation d'un unordered_map pour stocker les locomotives en fonction de leur adresse
std::unordered_map<uint16_t, Loco> Loco::locoMap;

// Recherche d'une locomotive par son adresse.
// Une loco inconnue n'est plus créée : elle n'aurait jamais été rafraîchie
// (absente du tableau parcouru par Message::periodic) et sa création modifiait
// le compteur de réinscription.
Loco *Loco::findLoco(uint16_t address)
{
  MapLock lock;
  auto it = locoMap.find(address);
  return it == locoMap.end() ? nullptr : &it->second;
}

bool Loco::removeLoco(uint16_t address)
{
  MapLock lock;
  return locoMap.erase(address) > 0;
}

Loco *Loco::createLoco(uint16_t address, String name, uint8_t uid0, uint8_t uid1, uint8_t uid2, uint8_t uid3)
{
  Loco loco(address, name, uid0, uid1, uid2, uid3);
  MapLock lock;
  auto it = locoMap.find(address);
  if (it != locoMap.end())
  {
    // Two locos with the same address: the second one replaces the first.
    Serial.printf("ERREUR : adresse %u deja utilisee par %s, remplacee par %s\n",
                  address, it->second.name().c_str(), name.c_str());
    it->second = loco;
  }
  else
    it = locoMap.emplace(address, loco).first;
  return &it->second;
}
