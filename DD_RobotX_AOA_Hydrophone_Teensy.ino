/* ──────────────────────────────────────────────────────────────
   FIRST-THRESHOLD  AoA  MEASUREMENT  – Teensy 4.1
   Dean Dcruz / April 2025
   -------------------------------------------------------------
   Pin-out
       HYDRO-1  →  A2  (ADC0)   ← reference channel
       HYDRO-2  →  A1  (ADC1)
   Signals must already be ≤ 3 .3 V (use your divider).
   ----------------------------------------------------------- */

#include <ADC.h>
#include <algorithm>      // std::sort
#include <cstring>        // memcpy

/* ───────── user-tunable constants ──────────────────────────── */
constexpr uint16_t ADC_THRESH       = 2800;          // ≈2.4 V threshold
constexpr uint32_t MAX_WINDOW_US    = 300;           // wait this long for H-2
constexpr uint32_t DEAD_TIME_MS     = 1500;          // ignore anything for 1.5 s
constexpr float    SOUND_SPEED      = 1500.0f;       // m s⁻¹
constexpr float    HYDRO_SPACING    = 0.095f;        // 8.5 cm
constexpr uint8_t  MED_WIN          = 5;             // 2–9  (rolling-median length)

/* ───────── pins ────────────────────────────────────────────── */
const uint8_t PIN_H1 = A2;     // ADC0
const uint8_t PIN_H2 = A1;     // ADC1

ADC          adc;              // PJRC ADC object
elapsedMillis deadTimer;       // blanking timer

/* ───────── helpers ─────────────────────────────────────────── */
inline float adc2V(uint16_t adc)           { return adc * 3.3f / 4095.0f; }

/* simple rolling median */
float  medBuf[MED_WIN] = {0};
uint8_t medPos         = 0;
bool    medFilled      = false;

float pushMedian(float v)
{
  medBuf[medPos] = v;
  medPos         = (medPos + 1) % MED_WIN;
  uint8_t n      = medFilled ? MED_WIN : medPos;
  float   temp[MED_WIN];
  memcpy(temp, medBuf, n * sizeof(float));
  std::sort(temp, temp + n);
  return (n & 1) ? temp[n / 2]
                 : 0.5f * (temp[n / 2 - 1] + temp[n / 2]);
}

/* ───────── Arduino setup ───────────────────────────────────── */
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) ;

  Serial.println("\n---  AoA by first-threshold  |  median window = "
                 + String(MED_WIN) + "  ---");

  analogReadResolution(12);

  adc.adc0->setAveraging(1);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed  (ADC_SAMPLING_SPEED::HIGH_SPEED);

  adc.adc1->setAveraging(1);
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc1->setSamplingSpeed  (ADC_SAMPLING_SPEED::HIGH_SPEED);
}

/* ───────── main loop ───────────────────────────────────────── */
void loop()
{
  if (deadTimer < DEAD_TIME_MS) return;          // still blanking

  /* --- wait for Hydro-1 to cross the threshold ---------------- */
  uint16_t s1;
  do { s1 = analogRead(PIN_H1); } while (s1 < ADC_THRESH);

  const uint32_t t1 = micros();                  // reference timestamp
  const uint16_t p1 = s1;

  /* --- open a short window for Hydro-2 ------------------------ */
  uint32_t t2   = 0;
  uint16_t p2   = 0;
  bool     hit2 = false;

  while (micros() - t1 < MAX_WINDOW_US)
  {
    uint16_t s2 = analogRead(PIN_H2);
    if (s2 >= ADC_THRESH) {
      hit2 = true;
      t2   = micros();
      p2   = s2;
      break;
    }
  }

  if (!hit2) {                     // missed → discard & restart quickly
    deadTimer = MAX_WINDOW_US / 1000 + 50;
    return;
  }

  /* --- compute AoA ------------------------------------------- */
  const int32_t dT_us = int32_t(t2) - int32_t(t1);      // signed Δt
  float sinx = (SOUND_SPEED * dT_us * 1e-6f) / HYDRO_SPACING;
  sinx       = constrain(sinx, -1.0f, 1.0f);
  float aoa  = asin(sinx) * 180.0f / PI;

  float med  = pushMedian(aoa);
  medFilled |= (medPos == 0);

  /* --- print -------------------------------------------------- */
  Serial.printf("AoA median %7.2f° | raw %+7.2f° | Δt %+6ld µs | "
                "H1 %.3f V  H2 %.3f V\n",
                medFilled ? med : aoa, aoa, dT_us,
                adc2V(p1), adc2V(p2));

  deadTimer = 0;                                   // restart blanking
}
