// #ifndef ROTARY_NUMBER_SELECTOR_HPP
// #define ROTARY_NUMBER_SELECTOR_HPP

// #include "RotaryEncoder.hpp"
// #include "esp_log.h"
// #include <cmath>
// #include <functional>
// #include <memory>

// namespace COMPONENT {
// #define TAG_SELECTOR "NumberSelector"

// struct RangeConfig {
//   float minValue = 0;
//   float maxValue = 100;
//   float step = 2;
//   bool cycleValues = false;
//   unsigned int decimals = 0;
// };

// class EncoderNumberSelector {
// private:
//   RangeConfig rangeConfig;
//   float coefficient = 1;
//   std::unique_ptr<Encoder> encoder;
//   std::function<int(long)> customAccelerationMapping;
//   constexpr static int ACCELERATION_NONE = 0;
//   constexpr static int ACCELERATION_LOW = 20;
//   constexpr static int ACCELERATION_MEDIUM = 100;
//   constexpr static int ACCELERATION_HIGH = 300;
//   constexpr static int ACCELERATION_MAX = 500;

// public:
//   EncoderNumberSelector(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
//                         gpio_num_t buttonPin = GPIO_NUM_NC,
//                         bool encoderPinPulledDown = true,
//                         bool buttonPulledDown = false,
//                         std::function<int(long)> customMapping = nullptr)
//       : encoder(std::make_unique<Encoder>(steps, aPin, bPin, buttonPin,
//                                           encoderPinPulledDown,
//                                           buttonPulledDown)),
//         customAccelerationMapping(customMapping) {}

//   // Struct-based setRange
//   void setRange(const RangeConfig &config) {
//     if (config.step <= 0) {
//       ESP_LOGE(TAG_SELECTOR, "Step value must be greater than zero.");
//       return;
//     }
//     if (config.maxValue < config.minValue) {
//       ESP_LOGE(TAG_SELECTOR, "Max value cannot be less than min value.");
//       return;
//     }

//     this->rangeConfig = config;
//     this->coefficient = std::pow(10.0, config.decimals);
//     float effectiveStep = config.step * this->coefficient;

//     long minEncoderValue = static_cast<long>(
//         (this->coefficient * config.minValue) / effectiveStep);
//     long maxEncoderValue = static_cast<long>(
//         (this->coefficient * config.maxValue) / effectiveStep);
//     long range = maxEncoderValue - minEncoderValue;

//     encoder->setBoundaries(minEncoderValue, maxEncoderValue,
//                            config.cycleValues);

//     // Apply acceleration logic
//     applyAcceleration(range);
//   }

//   // Parameter-based overload of setRange
//   void setRange(float minValue, float maxValue, float step, bool cycleValues,
//                 unsigned int decimals = 0) {
//     RangeConfig config = {minValue, maxValue, step, cycleValues, decimals};
//     setRange(config); // Delegate to the struct-based version
//   }

//   void setValue(float value) {
//     if (rangeConfig.step <= 0) {
//       ESP_LOGE(TAG_SELECTOR, "Step value must be greater than zero.");
//     } else {
//       long encoderValue = static_cast<long>((coefficient * value) /
//                                             (rangeConfig.step *
//                                             coefficient));
//       encoder->setEncoderValue(encoderValue);
//     }
//   }

//   float getValue() {
//     long encoderValue = encoder->readEncoder();
//     return static_cast<float>(encoderValue) * rangeConfig.step / coefficient;
//   }

// private:
//   void applyAcceleration(long range) {
//     if (customAccelerationMapping) {
//       encoder->setAcceleration(customAccelerationMapping(range));
//     } else {
//       if (range < 20)
//         encoder->setAcceleration(ACCELERATION_NONE);
//       else if (range < 60)
//         encoder->setAcceleration(ACCELERATION_LOW);
//       else if (range < 200)
//         encoder->setAcceleration(ACCELERATION_MEDIUM);
//       else if (range < 1000)
//         encoder->setAcceleration(ACCELERATION_HIGH);
//       else
//         encoder->setAcceleration(ACCELERATION_MAX);
//     }
//   }
// };

// } // namespace COMPONENT

// #endif

#ifndef ROTARY_SELECTOR_HPP
#define ROTARY_SELECTOR_HPP

#include "RotaryEncoder.hpp"
#include "esp_log.h"
#include <cmath>
#include <functional>
#include <memory>
#include <type_traits>

namespace Rotary {
#define TAG_SELECTOR "RotarySelector"

template <typename T> class EncoderSelector {
  static_assert(
      std::is_arithmetic<T>::value || std::is_enum<T>::value,
      "EncoderSelector can only be used with arithmetic types or enums.");

private:
  struct RangeConfig {
    T minValue;
    T maxValue;
    T step;
    bool cycleValues = false;
    unsigned int decimals = 0; // Only applicable for arithmetic types
  };

  RangeConfig rangeConfig;
  float coefficient = 1; // For decimals, only used for arithmetic types
  std::unique_ptr<Encoder> encoder;
  std::function<int(long)> customAccelerationMapping;

  constexpr static int ACCELERATION_NONE = 0;
  constexpr static int ACCELERATION_LOW = 20;
  constexpr static int ACCELERATION_MEDIUM = 100;
  constexpr static int ACCELERATION_HIGH = 300;
  constexpr static int ACCELERATION_MAX = 500;

public:
  EncoderSelector(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
                  gpio_num_t buttonPin = GPIO_NUM_NC,
                  PullType encoderPinPull = PullType::EXTERNAL_PULLUP,
                  PullType buttonPinPull = PullType::EXTERNAL_PULLDOWN,
                  std::function<int(long)> customMapping = nullptr)
      : encoder(std::make_unique<Encoder>(steps, aPin, bPin, buttonPin,
                                          encoderPinPull, buttonPinPull)),
        customAccelerationMapping(customMapping) {}

  // Struct-based setRange for arithmetic types
  template <typename U = T>
  typename std::enable_if<std::is_arithmetic<U>::value, void>::type
  setRange(const T &minValue, const T &maxValue, const T &step,
           bool cycleValues, unsigned int decimals = 0) {
    if (step <= 0) {
      ESP_LOGE(TAG_SELECTOR, "Step value must be greater than zero.");
      return;
    }
    if (maxValue < minValue) {
      ESP_LOGE(TAG_SELECTOR, "Max value cannot be less than min value.");
      return;
    }

    rangeConfig = {minValue, maxValue, step, cycleValues, decimals};
    coefficient = std::pow(10.0, decimals);
    float effectiveStep = step * coefficient;

    long minEncoderValue =
        static_cast<long>((coefficient * minValue) / effectiveStep);
    long maxEncoderValue =
        static_cast<long>((coefficient * maxValue) / effectiveStep);
    long range = maxEncoderValue - minEncoderValue;

    encoder->setBoundaries(minEncoderValue, maxEncoderValue, cycleValues);
    applyAcceleration(range);
  }

  // Struct-based setRange for enums
  template <typename U = T>
  typename std::enable_if<std::is_enum<U>::value, void>::type
  setRange(const T &minValue, const T &maxValue, const T &step,
           bool cycleValues) {
    rangeConfig = {minValue, maxValue, step, cycleValues};

    long minEncoderValue = static_cast<long>(minValue);
    long maxEncoderValue = static_cast<long>(maxValue);
    long range = maxEncoderValue - minEncoderValue;

    encoder->setBoundaries(minEncoderValue, maxEncoderValue, cycleValues);
    applyAcceleration(range);
  }

  void setValue(T value) {
    long encoderValue;
    if constexpr (std::is_arithmetic<T>::value) {
      encoderValue = static_cast<long>((coefficient * value) /
                                       (rangeConfig.step * coefficient));
    } else {
      encoderValue = static_cast<long>(value);
    }
    encoder->setEncoderValue(encoderValue);
  }

  T getValue() {
    long encoderValue = encoder->readEncoder();
    if constexpr (std::is_arithmetic<T>::value) {
      return static_cast<T>(encoderValue) * rangeConfig.step / coefficient;
    } else {
      return static_cast<T>(encoderValue);
    }
  }

private:
  void applyAcceleration(long range) {
    if (customAccelerationMapping) {
      encoder->setAcceleration(customAccelerationMapping(range));
    } else {
      if (range < 20)
        encoder->setAcceleration(ACCELERATION_NONE);
      else if (range < 60)
        encoder->setAcceleration(ACCELERATION_LOW);
      else if (range < 200)
        encoder->setAcceleration(ACCELERATION_MEDIUM);
      else if (range < 1000)
        encoder->setAcceleration(ACCELERATION_HIGH);
      else
        encoder->setAcceleration(ACCELERATION_MAX);
    }
  }
};

} // namespace Rotary

#endif
