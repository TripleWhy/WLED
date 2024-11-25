#pragma once

#include "wled.h"

#define NAMED_FUNCTION(F) F, #F

class Benchmark : public Usermod {
  private:
    bool enabled = false;
    bool initDone = false;
    unsigned long previousTimeStamp = 0;
    int index = 0;

    // string that are used multiple time (this will save some flash memory)
    static const char _name[];

  public:
    void setup() {
      initDone = true;
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top["enabled"] = enabled;
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top["enabled"], enabled);
      return configComplete;
    }

    uint16_t getId() {
      return 43;
    }

    void loop() {
      if (!enabled || strip.isUpdating()) return;

      const unsigned long now = millis();
      if ((index == 0) && (now - previousTimeStamp < 20000)) return;
      previousTimeStamp = now;

      switch (index) {
        case  0: measure(NAMED_FUNCTION(&Benchmark::benchEmpty)); break;
        case  1: measure(NAMED_FUNCTION(&Benchmark::benchEmptyLoop)); break;
        case  2: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::identityF)); break;
        case  3: measure(NAMED_FUNCTION(&Benchmark::identityUint16)); break;
        case  4: measure(NAMED_FUNCTION(&::sin8)); break;
        case  5: measure(NAMED_FUNCTION(&::sin8_t)); break;
        case  6: measure(NAMED_FUNCTION(&::sin16)); break;
        case  7: measure(NAMED_FUNCTION(&::sin16_t)); break;
        case  8: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::sin)); break;
        case  9: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::sinf)); break;
        case 10: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::sin_t)); break;
        case 11: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::sin_approx)); break;

        case 12: measure(NAMED_FUNCTION(&::cos8)); break;
        case 13: measure(NAMED_FUNCTION(&::cos8_t)); break;
        case 14: measure(NAMED_FUNCTION(&::cos16)); break;
        case 15: measure(NAMED_FUNCTION(&::cos16_t)); break;
        case 16: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::cos)); break;
        case 17: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::cosf)); break;
        case 18: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::cos_t)); break;
        case 19: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::cos_approx)); break;

        case 20: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&Benchmark::sin66868438)); break;
        case 21: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&Benchmark::cos28050328<float>)); break;

        case 22: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::tan)); break;
        case 23: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::tanf)); break;
        case 24: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::tan_approx)); break;

        case 25: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::acos)); break;
        case 26: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::acosf)); break;
        case 27: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::acos_t)); break;

        case 28: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::asin)); break;
        case 29: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::asinf)); break;
        case 30: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::asin_t)); break;

        case 31: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&std::atan)); break;
        case 32: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::atanf)); break;
        case 33: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::atan_t<float>)); break;
      //  case 20: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::atan2_t)); break;
      //  case 20: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::floor_t)); break;
      //  case 20: measure(&Benchmark::benchFloatTrig, NAMED_FUNCTION(&::fmod_t)); break;

        // case 11: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::addF)); break;
        // case 12: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::subtractF)); break;
        // case 13: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::multiplyF)); break;
        // case 14: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::divideF)); break;
        // case 15: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::fmaManual)); break;
        // case 16: measure(&Benchmark::benchFloat, NAMED_FUNCTION(&Benchmark::fmaLib)); break;
        // case 17: measure(NAMED_FUNCTION(&Benchmark::addUint8)); break;
        // case 18: measure(NAMED_FUNCTION(&Benchmark::subtractUint8)); break;
        // case 19: measure(NAMED_FUNCTION(&Benchmark::multiplyUint8)); break;
        // case 20: measure(NAMED_FUNCTION(&Benchmark::divideUint8)); break;
        // case 21: measure(NAMED_FUNCTION(&Benchmark::addUint16)); break;
        // case 22: measure(NAMED_FUNCTION(&Benchmark::subtractUint16)); break;
        // case 23: measure(NAMED_FUNCTION(&Benchmark::multiplyUint16)); break;
        // case 24: measure(NAMED_FUNCTION(&Benchmark::divideUint16)); break;
        // case 25: measure(NAMED_FUNCTION(&Benchmark::addUint32)); break;
        // case 26: measure(NAMED_FUNCTION(&Benchmark::subtractUint32)); break;
        // case 27: measure(NAMED_FUNCTION(&Benchmark::multiplyUint32)); break;
        // case 28: measure(NAMED_FUNCTION(&Benchmark::divideUint32)); break;
        // case 29: measure(NAMED_FUNCTION(&Benchmark::addInt)); break;
        // case 30: measure(NAMED_FUNCTION(&Benchmark::subtractInt)); break;
        // case 31: measure(NAMED_FUNCTION(&Benchmark::multiplyInt)); break;
        // case 32: measure(NAMED_FUNCTION(&Benchmark::divideInt)); break;
        // case 33: measure(NAMED_FUNCTION(&Benchmark::addInt64)); break;
        // case 34: measure(NAMED_FUNCTION(&Benchmark::subtractInt64)); break;
        // case 35: measure(NAMED_FUNCTION(&Benchmark::multiplyInt64)); break;
        // case 36: measure(NAMED_FUNCTION(&Benchmark::divideInt64)); break;
        // case 37: measure(NAMED_FUNCTION(&Benchmark::color_wheel_original)); break;
        // case 38: measure(NAMED_FUNCTION(&Benchmark::color_wheel_hsv)); break;
        default:
          index = 0;
          return;
      }
      ++index;
    }

    static void printMeasurementResults(const unsigned long t0, const unsigned long t1, int count, const char * name) {
        Serial.print(name);
        Serial.print(": ");
// #ifdef ARDUINO_ARCH_ESP32
        // Serial.print(((t1 - t0) * 1000) / (float)count * 1000.0f, std::numeric_limits<float>::digits10);
        Serial.print(((t1 - t0) * 1000) / (float)count * 1000.0f);
        Serial.print(" ns (");
// #else
//         Serial.print(((t1 - t0) * 1000) / (float)count, std::numeric_limits<float>::digits10);
//         Serial.print(" us (");
// #endif
        Serial.print(t1 - t0);
        Serial.print(" ms / ");
        Serial.print(count);
        Serial.println(" iterations)");
    }

    template<typename OutType, typename InType>
    static void measure(int (*benchmarkFunction)(OutType (*)(InType)), OutType (*function)(InType), const char * name) {
        const unsigned long t0 = millis();
        const int count = (*benchmarkFunction)(function);
        const unsigned long t1 = millis();
        printMeasurementResults(t0, t1, count, name);
    }

    static void measure(int (*function)(), const char * name) {
        const unsigned long t0 = millis();
        const int count = (*function)();
        const unsigned long t1 = millis();
        printMeasurementResults(t0, t1, count, name);
    }

    template<typename OutType>
    static void measure(OutType (*function)(uint8_t), const char * name) {
        measure(&Benchmark::benchUint8, function, name);
    }

    template<typename OutType>
    static void measure(OutType (*function)(uint16_t), const char * name) {
        measure(&Benchmark::benchUint16, function, name);
    }

    static void measure(uint32_t (*function)(uint32_t), const char * name) {
        measure(&Benchmark::benchUint32, function, name);
    }

    static void measure(int (*function)(int), const char * name) {
        measure(&Benchmark::benchInt, function, name);
    }

    static void measure(int64_t (*function)(int64_t), const char * name) {
        measure(&Benchmark::benchInt64, function, name);
    }

    static int benchFloat(float (*fn)(float)) {
      static_assert(sizeof(float) == sizeof(uint32_t), "sizes don't match");
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

    static int benchEmpty() {
      return 1;
    }

    static int benchEmptyLoop() {
      constexpr int repetitionCount = 35;
      for (int repetitions = 0; repetitions < repetitionCount; ++repetitions) {
        uint16_t i = 0;
        do {
        } while (++i != 0);
      }
      return (((int)std::numeric_limits<uint16_t>::max()) + 1) * repetitionCount;
    }

    template<typename OutType>
    static int benchUint8(OutType (*fn)(uint8_t)) {
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

    static int benchUint32(uint32_t (*fn)(uint32_t)) {
      constexpr uint32_t increment = (uint32_t)(std::numeric_limits<uint16_t>::max()) / uint32_t(35);
      constexpr uint32_t limit = std::numeric_limits<uint32_t>::max() - increment;
      for (uint32_t i = 0; ; i += increment) {
        (*fn)(i);
        if (limit < i) {
          break;
        }
      }
      return std::numeric_limits<uint32_t>::max() / increment + 1;
    }

    static int benchInt(int (*fn)(int)) {
      constexpr int increment = (int)(std::numeric_limits<uint16_t>::max()) / 50;
      constexpr int limit = std::numeric_limits<int>::max() - increment;
      for (int i = 0; ; i += increment) {
        (*fn)(i);
        if (limit < i) {
          break;
        }
      }
      return std::numeric_limits<int>::max() / increment + 1;
    }

    static int benchInt64(int64_t (*fn)(int64_t)) {
      constexpr int64_t increment = (int64_t)(std::numeric_limits<int32_t>::max()) * int64_t(4000L);
      constexpr int64_t limit = std::numeric_limits<int64_t>::max() - increment;
      for (int64_t i = 0; ; i += increment) {
        (*fn)(i);
        if (limit < i) {
          break;
        }
      }
      constexpr int64_t count = std::numeric_limits<int64_t>::max() / increment + int64_t(1);
      static_assert(count < (int64_t)std::numeric_limits<int>::max(), "?");
      return count;
    }

    static uint16_t identityUint16(uint16_t x) {
      return x;
    }
    static float identityF(float x) {
      return x;
    }

    // https://stackoverflow.com/a/66868438
    static float sin66868438(float x) {
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
    static inline T cos28050328(T x) noexcept {
        constexpr T tp = 1./(2.*M_PI);
        x *= tp;
        x -= T(.25) + std::floor(x + T(.25));
        x *= T(16.) * (std::abs(x) - T(.5));
        #if EXTRA_PRECISION
        x += T(.225) * x * (std::abs(x) - T(1.));
        #endif
        return x;
    }

    static float addF(float f) {
      return f + 2.0f;
    }
    static float subtractF(float f) {
      return f - 2.0f;
    }
    static float multiplyF(float f) {
      return f * 1.001f;
    }
    static float divideF(float f) {
      return 1.0f / f;
    }
    static float fmaManual(float f) {
      return f * f + 42.0f;
    }
    static float fmaLib(float f) {
      return fmaf(f, f, 42.0f);
    }

    static uint8_t addUint8(uint8_t i) {
      return i + uint8_t(42u);
    }
    static uint8_t subtractUint8(uint8_t i) {
      return i - uint8_t(42u);
    }
    static uint8_t multiplyUint8(uint8_t i) {
      return i * uint8_t(42u);
    }
    static uint8_t divideUint8(uint8_t i) {
      return i / uint8_t(42u);
    }

    static uint16_t addUint16(uint16_t i) {
      return i + uint16_t(2u);
    }
    static uint16_t subtractUint16(uint16_t i) {
      return i - uint16_t(2u);
    }
    static uint16_t multiplyUint16(uint16_t i) {
      return i * uint16_t(2u);
    }
    static uint16_t divideUint16(uint16_t i) {
      return i / uint16_t(2u);
    }

    static uint32_t addUint32(uint32_t i) {
      return i + uint32_t(2u);
    }
    static uint32_t subtractUint32(uint32_t i) {
      return i - uint32_t(2u);
    }
    static uint32_t multiplyUint32(uint32_t i) {
      return i * uint32_t(2u);
    }
    static uint32_t divideUint32(uint32_t i) {
      return i / uint32_t(2u);
    }

    static int addInt(int i) {
      return i + int(2u);
    }
    static int subtractInt(int i) {
      return i - int(2u);
    }
    static int multiplyInt(int i) {
      return i * int(2u);
    }
    static int divideInt(int i) {
      return i / int(2u);
    }

    static int64_t addInt64(int64_t i) {
      return i + int64_t(2u);
    }
    static int64_t subtractInt64(int64_t i) {
      return i - int64_t(2u);
    }
    static int64_t multiplyInt64(int64_t i) {
      return i * int64_t(2u);
    }
    static int64_t divideInt64(int64_t i) {
      return i / int64_t(2u);
    }

    static uint32_t color_wheel_original(uint8_t pos) {
      uint8_t w = 0;
      pos = 255 - pos;
      if (pos < 85) {
        return RGBW32((255 - pos * 3), 0, (pos * 3), w);
      } else if(pos < 170) {
        pos -= 85;
        return RGBW32(0, (pos * 3), (255 - pos * 3), w);
      } else {
        pos -= 170;
        return RGBW32((pos * 3), (255 - pos * 3), 0, w);
      }
    }

    static uint32_t color_wheel_hsv(uint8_t pos) {
      uint8_t w = 0;
      // These h and f values are the same h and f you have in the regular HSV to RGB conversion.
      // The whole funciton really is just a HSV conversion, but assuming H=pos, S=1 and V=1.
      const uint32_t h = (pos * 3) / 128;
      const uint32_t f = (pos * 6) % 256;
      switch (h) {
        case 0: return RGBW32(255    , f      , 0      , w);
        case 1: return RGBW32(255 - f, 255    , 0      , w);
        case 2: return RGBW32(0      , 255    , f      , w);
        case 3: return RGBW32(0      , 255 - f, 255    , w);
        case 4: return RGBW32(f      , 0      , 255    , w);
        case 5: return RGBW32(255    , 0      , 255 - f, w);
        default: return 0;
      }
    }
};


// add more strings here to reduce flash memory usage
const char Benchmark::_name[] PROGMEM = "Benchmark";

#undef NAMED_FUNCTION