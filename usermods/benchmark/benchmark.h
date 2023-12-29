#pragma once

#include "wled.h"

#define NAMED_FUNCTION(F) F, #F

class Benchmark : public Usermod
{
  private:
    bool enabled = false;
    bool initDone = false;
    unsigned long previousTimeStamp = 0;
    int index = 0;

    // string that are used multiple time (this will save some flash memory)
    static const char _name[];

  public:
    void setup()
    {
      initDone = true;
    }

    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top["enabled"] = enabled;
    }

    bool readFromConfig(JsonObject& root)
    {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top["enabled"], enabled);
      return configComplete;
    }

    uint16_t getId()
    {
      return 43;
    }

    void loop()
    {
      if (!enabled || strip.isUpdating()) return;

      const unsigned long now = millis();
      if ((index == 0) && (now - previousTimeStamp < 10000)) return;
      previousTimeStamp = now;

      switch (index)
      {
        case 0: measure(NAMED_FUNCTION(&sin8)); break;
        case 1: measure(NAMED_FUNCTION(&sin16)); break;
        case 2: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&sin66868438)); break;
        case 3: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&cos28050328<float>)); break;
        case 4: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::sin)); break;
        case 5: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&sinf)); break;
        case 6: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&sin_t)); break;
        default:
          index = 0;
          return;
      }
      ++index;
    }

    template<typename OutType, typename InType>
    static void measure(int (*benchmarkFunction)(OutType (*)(InType)), OutType (*function)(InType), const char * name)
    {
        const unsigned long t0 = millis();
        const int count = (*benchmarkFunction)(function);
        const unsigned long t1 = millis();
        Serial.print(name);
        Serial.print(": ");
#ifdef ARDUINO_ARCH_ESP32
        Serial.print(((t1 - t0) * 1000) / (float)count * 1000.0f, std::numeric_limits<float>::digits10);
        Serial.print(" ns (");
#else
        Serial.print(((t1 - t0) * 1000) / (float)count, std::numeric_limits<float>::digits10);
        Serial.print(" us (");
#endif
        Serial.print(t1 - t0);
        Serial.print(" ms / ");
        Serial.print(count);
        Serial.println(" iterations)");
    }

    static void measure(uint8_t (*function)(uint8_t), const char * name)
    {
        measure(&Benchmark::benchUint8, function, name);
    }

    static void measure(uint16_t (*function)(uint16_t), const char * name)
    {
        measure(&Benchmark::benchUint16, function, name);
    }

    static void measure(int16_t (*function)(uint16_t), const char * name)
    {
        measure(&Benchmark::benchUint16, function, name);
    }

    static int benchFloat(float (*fn)(float)) {
      static_assert(sizeof(float) == sizeof(uint32_t), "sizes don't match");
      union U
      {
        uint32_t i;
        float f;
      };
      constexpr uint32_t posMin = 0b00000000100000000000000000000000;
      constexpr uint32_t posMax = 0b01111111011111111111111111111111;
      int counter = 0;
      for (uint32_t i = posMin; i <= posMax; i += 10000u) {
        if (++counter % 1000000 == 0) {
          Serial.print(counter);
          Serial.print("/");
          Serial.println(posMax - posMin + 1);
        }
        float f;
        memcpy(&f, &i, sizeof(i));
        (*fn)(f);
      }
      return counter;
    }

    static int benchFloatTrig(float (*fn)(float)) {
      static_assert(sizeof(float) == sizeof(uint32_t), "sizes don't match");
      union U
      {
        uint32_t i;
        float f;
      };
      constexpr U uMin{.f = std::numeric_limits<float>::min()};
      constexpr U uMax{.f = float(TWO_PI)};
#ifdef ARDUINO_ARCH_ESP32
      constexpr uint32_t increment = 1000u;
#else
      constexpr uint32_t increment = 50000u;
#endif
      for (U i = uMin; i.i <= uMax.i; i.i += increment) {
        (*fn)(i.f);
      }
      return (uMax.i - uMin.i + 1) / increment + 1;
    }

    static int benchUint8(uint8_t (*fn)(uint8_t)) {
      constexpr int repetitionCount = 35 * 256;
      for (int repetitions = 0; repetitions < repetitionCount; ++repetitions) {
        uint8_t i = 0;
        do {
          (*fn)(i);
        } while (++i != 0);
      }
      return (((int)std::numeric_limits<uint8_t>::max()) + 1) * repetitionCount;
    }

    static int benchUint16(uint16_t (*fn)(uint16_t)) {
      constexpr int repetitionCount = 35;
      for (int repetitions = 0; repetitions < repetitionCount; ++repetitions) {
        uint16_t i = 0;
        do {
          (*fn)(i);
        } while (++i != 0);
      }
      return (((int)std::numeric_limits<uint16_t>::max()) + 1) * repetitionCount;
    }

    static int benchUint16(int16_t (*fn)(uint16_t)) {
      constexpr int repetitionCount = 35;
      for (int repetitions = 0; repetitions < repetitionCount; ++repetitions) {
        uint16_t i = 0;
        do {
          (*fn)(i);
        } while (++i != 0);
      }
      return (((int)std::numeric_limits<uint16_t>::max()) + 1) * repetitionCount;
    }

    // https://stackoverflow.com/a/66868438
    static float sin66868438(float x)
    {
        const float B = 4.0f/float(M_PI);
        const float C = -4.0f/float(M_PI*M_PI);

        float y = B * x + C * x * std::abs(x);

        #ifdef EXTRA_PRECISION
        //  const float Q = 0.775;
            const float P = 0.225;

            y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)
        #endif
        return y;
    }

    // https://stackoverflow.com/a/28050328
    template<typename T>
    static inline T cos28050328(T x) noexcept
    {
        constexpr T tp = 1./(2.*M_PI);
        x *= tp;
        x -= T(.25) + std::floor(x + T(.25));
        x *= T(16.) * (std::abs(x) - T(.5));
        #if EXTRA_PRECISION
        x += T(.225) * x * (std::abs(x) - T(1.));
        #endif
        return x;
    }
};


// add more strings here to reduce flash memory usage
const char Benchmark::_name[] PROGMEM = "Benchmark";

#undef NAMED_FUNCTION