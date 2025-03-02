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

namespace Rotary
{
struct EncoderData
{
	long position = 0;
	int8_t oldDirection = 0;
	unsigned long lastMovementTime = 0;
	long minValue = LONG_MIN;
	long maxValue = LONG_MAX;
	uint8_t steps = 4;
};
class Encoder
{
  public:
	Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin, gpio_num_t buttonPin = GPIO_NUM_NC,
			PullType encoderPinPull = PullType::EXTERNAL_PULLUP,
			PullType buttonPinPull = PullType::EXTERNAL_PULLDOWN);

	void setBoundaries(long minValue = -100, long maxValue = 100, bool circleValues = false);

	void setup(void (*ISR_callback)(void));
	void setup(void (*ISR_callback)(void), void (*ISR_button)(void));

	void begin();
	void reset(long newValue = 0);
	void enable();
	void disable();
	long readEncoder() const;
	void setEncoderValue(long newValue);
	long encoderChanged();

	ButtonState readButtonState() const;
	unsigned long getAcceleration() const;
	void setAcceleration(unsigned long acceleration);
	void disableAcceleration();

	void IRAM_ATTR encoderISR();
	void IRAM_ATTR buttonISR();

  private:
	gpio_num_t aPin_;
	gpio_num_t bPin_;
	gpio_num_t buttonPin_;

	uint8_t encoderSteps_;
	int correctionOffset_ = 2;
	PullType encoderPinPull_ = PullType::NONE;
	PullType buttonPinPull_ = PullType::NONE;
	bool isEnabled_;
	EncoderData data_;
	bool circleValues_ = false;
	long lastReadEncoderPosition_ = 0;
	unsigned long rotaryAccelerationCoef_ = 150;

	long minEncoderValue_ = LONG_MIN;
	long maxEncoderValue_ = LONG_MAX;
	std::atomic<int8_t> oldAB_{0};
	std::atomic<long> encoderPosition_{0};
	std::atomic<int8_t> oldDirection_{0};
	std::atomic<unsigned long> lastMovementTime_{0};

	const int8_t encoderStates_[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	std::atomic<ButtonState> buttonState_{ButtonState::UP};

	bool isEncoderButtonClicked(unsigned long maximumWaitMilliseconds = 300);
	bool isEncoderButtonDown() const;
	// Helper Functions
	void configurePin(gpio_num_t pin, gpio_int_type_t intrType, PullType pullType);
	bool debounce(bool currentState, unsigned long& lastTime, unsigned long delay);
	void initGPIOS();
	int8_t updateOldABState();

	std::atomic<void*> encoderCallback_{nullptr};
	std::atomic<void*> buttonCallback_{nullptr};
	static void EncoderMonitorTask(void* param);
	static void ButtonMonitorTask(void* param);

	TaskHandle_t encoderTaskHandle = nullptr;
	TaskHandle_t buttonTaskHandle = nullptr;

	long updatePosition(EncoderData& data, int8_t direction, unsigned long currentTime,
						unsigned long acceCoef);
	long wrapPosition(EncoderData& data);
};

} // namespace Rotary

#endif