#ifndef ROTARY_ENCODER_HPP
#define ROTARY_ENCODER_HPP
#include "Arduino.h"
#include "RotaryDefines.hpp"

#include "freertos/queue.h"
#include "freertos/task.h"
#include "stdint.h"
#include <atomic>
#include <climits>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

namespace Rotary {

constexpr uint16_t QUEUE_SIZE = 10;

static constexpr unsigned long DEBOUNCE_DELAY = 50;
static constexpr unsigned long ENCODER_DEBOUNCE_DELAY = 10;
static constexpr unsigned long ACCELERATION_LONG_CUTOFF = 200;
static constexpr unsigned long ACCELERATION_SHORT_CUTOFF = 4;

class Encoder {

private:
  gpio_num_t aPin_;
  gpio_num_t bPin_;
  gpio_num_t buttonPin_;

  uint8_t encoderSteps_;
  int correctionOffset_ = 2;
  PullType encoderPinPull_ = PullType::NONE;
  PullType buttonPinPull_ = PullType::NONE;
  bool isEnabled_;

  TaskHandle_t encoderTaskHandle = nullptr;
  TaskHandle_t buttonTaskHandle = nullptr;

  long minEncoderValue_ = LONG_MIN;
  long maxEncoderValue_ = LONG_MAX;
  std::atomic<int8_t> oldAB_{0};
  std::atomic<long> encoderPosition_{0};
  std::atomic<int8_t> oldDirection_{0};
  std::atomic<unsigned long> lastMovementTime_{0};

  long lastReadEncoderPosition_ = 0;
  unsigned long rotaryAccelerationCoef_ = 150;
  bool circleValues_ = false;

  const int8_t encoderStates_[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                     -1, 0,  0, 1, 0, 1, -1, 0};
  std::atomic<ButtonState> buttonState_{ButtonState::UP};

  std::atomic<void *> encoderCallback_{nullptr};
  std::atomic<void *> buttonCallback_{nullptr};

  bool isEncoderButtonClicked(unsigned long maximumWaitMilliseconds = 300);
  bool isEncoderButtonDown();
  // Helper Functions
  void configurePin(gpio_num_t pin, gpio_int_type_t intrType,
                    PullType pullType);
  bool debounce(bool currentState, unsigned long &lastTime,
                unsigned long delay);
  void initGPIOS();
  int8_t updateOldABState();
  static void EncoderMonitorTask(void *param);
  static void ButtonMonitorTask(void *param);

public:
  Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin,
          gpio_num_t buttonPin = GPIO_NUM_NC,
          PullType encoderPinPull = PullType::EXTERNAL_PULLUP,
          PullType buttonPinPull = PullType::EXTERNAL_PULLDOWN);

  void setBoundaries(long minValue = -100, long maxValue = 100,
                     bool circleValues = false);

  void setup(void (*ISR_callback)(void));
  void setup(void (*ISR_callback)(void), void (*ISR_button)(void));
  static void setupTimer();
  void begin();
  void reset(long newValue = 0);
  void enable() { this->isEnabled_ = true; }
  void disable() { this->isEnabled_ = false; }
  long readEncoder() const;
  void setEncoderValue(long newValue) { reset(newValue); };
  long encoderChanged();

  ButtonState readButtonState() { return buttonState_; };
  unsigned long getAcceleration() { return this->rotaryAccelerationCoef_; }
  void setAcceleration(unsigned long acceleration) {
    rotaryAccelerationCoef_ = acceleration;
  }
  void disableAcceleration() { setAcceleration(0); }

  void IRAM_ATTR encoderISR();
  void IRAM_ATTR buttonISR();
};

} // namespace Rotary

#endif