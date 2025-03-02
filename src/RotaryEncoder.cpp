#include "RotaryEncoder.hpp"
#include "esp_log.h"
#define LOG_TAG "RotaryEncoder"
using namespace Rotary;
static volatile unsigned long lastInterruptTime = 0;

Encoder::Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin, gpio_num_t buttonPin,
				 PullType encoderPinPull, PullType buttonPinPull) :
	aPin_(aPin), bPin_(bPin), buttonPin_(buttonPin), encoderSteps_(steps),
	encoderPinPull_(encoderPinPull), buttonPinPull_(buttonPinPull), isEnabled_(true)
{
}

void Encoder::setEncoderValue(long newValue) { reset(newValue); }
void Encoder::enable() { this->isEnabled_ = true; }
void Encoder::disable() { this->isEnabled_ = false; }
ButtonState Encoder::readButtonState() { return buttonState_; }
unsigned long Encoder::getAcceleration() { return this->rotaryAccelerationCoef_; }
void Encoder::setAcceleration(unsigned long acceleration)
{
	rotaryAccelerationCoef_ = acceleration;
}
void Encoder::disableAcceleration() { setAcceleration(0); }

void Encoder::configurePin(gpio_num_t pin, gpio_int_type_t intrType, PullType pullType)
{
	gpio_config_t ioConfig = {};
	ioConfig.intr_type = intrType;
	ioConfig.mode = GPIO_MODE_INPUT;
	ioConfig.pin_bit_mask = (1ULL << pin);

	switch(pullType)
	{
		case PullType::NONE:
			ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
			ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
			break;
		case PullType::INTERNAL_PULLUP:
			ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
			ioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
			break;
		case PullType::INTERNAL_PULLDOWN:
			ioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
			ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
			break;
		case PullType::EXTERNAL_PULLUP:
		case PullType::EXTERNAL_PULLDOWN:

			ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
			ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
			break;
		default:

			ESP_LOGE(LOG_TAG, "Unsupported PullType specified.");
			return;
	}

	gpio_config(&ioConfig);
}
void Encoder::initGPIOS()
{
	configurePin(aPin_, GPIO_INTR_ANYEDGE, encoderPinPull_);
	configurePin(bPin_, GPIO_INTR_ANYEDGE, encoderPinPull_);
	if(buttonPin_ != GPIO_NUM_NC)
	{
		configurePin(buttonPin_, GPIO_INTR_POSEDGE, buttonPinPull_);
	}
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(aPin_, [](void* arg) { static_cast<Encoder*>(arg)->encoderISR(); }, this);
	gpio_isr_handler_add(bPin_, [](void* arg) { static_cast<Encoder*>(arg)->encoderISR(); }, this);
	if(buttonPin_ != GPIO_NUM_NC)
	{
		gpio_isr_handler_add(
			buttonPin_, [](void* arg) { static_cast<Encoder*>(arg)->buttonISR(); }, this);
	}
}
void Encoder::begin()
{
	initGPIOS();

	xTaskCreatePinnedToCore(EncoderMonitorTask, "EncoderMonitor", EncoderTaskStack, this,
							EncoderTask_Priority, &encoderTaskHandle, EncoderTask_CORE);

	if(buttonPin_ != GPIO_NUM_NC)
	{
		xTaskCreatePinnedToCore(ButtonMonitorTask, "ButtonMonitor", BtnTaskStack, this,
								BtnTask_Priority, &buttonTaskHandle, BtnTask_CORE);
	}
	if(encoderTaskHandle == nullptr)
	{
		ESP_LOGE(LOG_TAG, "Task handle is null; notification cannot be sent.");
		return;
	}
}
void Encoder::setup(void (*ISR_callback)(void))
{
	encoderCallback_.store(reinterpret_cast<void*>(ISR_callback), std::memory_order_release);
}

void Encoder::setup(void (*ISR_callback)(void), void (*ISR_button)(void))
{
	encoderCallback_.store(reinterpret_cast<void*>(ISR_callback), std::memory_order_release);
	buttonCallback_.store(reinterpret_cast<void*>(ISR_button), std::memory_order_release);
}

int8_t Encoder::updateOldABState()
{
	int8_t oldAB = oldAB_.load(std::memory_order_acquire);
	oldAB <<= 2;
	uint8_t state = (gpio_get_level(bPin_) << 1) | gpio_get_level(aPin_);
	oldAB |= (state & 0x03);
	oldAB_.store(oldAB, std::memory_order_release);
	oldAB = oldAB_.load(std::memory_order_acquire);
	return oldAB;
}
void IRAM_ATTR Encoder::encoderISR()
{
	if(!isEnabled_)
		return;

	unsigned long now = esp_timer_get_time() / 1000;

	if((now - lastInterruptTime) < ENCODER_DEBOUNCE_DELAY)
		return;
	lastInterruptTime = now;
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(encoderTaskHandle, &higherPriorityTaskWoken);
	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void IRAM_ATTR Encoder::buttonISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Encoder::EncoderMonitorTask(void* param)
{
	Encoder* encoder = static_cast<Encoder*>(param);
	int8_t currentDirection;

	while(true)
	{

		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{

			unsigned long now = esp_timer_get_time() / 1000;

			if(!encoder->isEnabled_)
			{
				continue;
			}

			int8_t oldAB = encoder->updateOldABState();
			currentDirection = encoder->encoderStates_[oldAB & 0x0F];

			long encoderPosition = encoder->encoderPosition_.load();
			uint8_t encoderSteps = encoder->encoderSteps_;

			if(currentDirection != 0)
			{
				long prevCount = encoderPosition / encoderSteps;
				encoderPosition += currentDirection;
				long newCount = encoderPosition / encoderSteps;

				if(newCount != prevCount && encoder->rotaryAccelerationCoef_ > 1)
				{

					int8_t lastDirection = encoder->oldDirection_.load();
					if(currentDirection == lastDirection && currentDirection != 0 &&
					   lastDirection != 0)
					{
						unsigned long timeSinceLastMotion = now - encoder->lastMovementTime_.load();
						if(timeSinceLastMotion < ACCELERATION_LONG_CUTOFF)
						{
							unsigned long limitedTime =
								std::max(ACCELERATION_SHORT_CUTOFF, timeSinceLastMotion);
							int adjustment = encoder->rotaryAccelerationCoef_ / limitedTime;
							encoderPosition += currentDirection > 0 ? adjustment : -adjustment;
						}
					}
				}

				long minVal = encoder->minEncoderValue_ / encoderSteps;
				long maxVal = encoder->maxEncoderValue_ / encoderSteps;
				long range = maxVal - minVal + 1;
				long currentValue = encoderPosition / encoderSteps;

				if(currentValue > maxVal || currentValue < minVal)
				{
					// Compute wrapped value using modulo arithmetic
					long wrappedValue = ((currentValue - minVal) % range + range) % range + minVal;
					encoderPosition = wrappedValue * encoderSteps;
				}

				encoder->encoderPosition_.store(encoderPosition);
				encoder->lastMovementTime_.store(now);
				encoder->oldDirection_.store(currentDirection);
				// long maxEncoderValue = encoder->maxEncoderValue_;
				// long minEncoderValue = encoder->minEncoderValue_;
				// long adjustedValue = encoderPosition / encoderSteps;

				// if(adjustedValue > maxEncoderValue / encoderSteps)
				// {
				// 	encoderPosition =
				// 		encoder->circleValues_ ?
				// 			minEncoderValue + ((adjustedValue - (maxEncoderValue / encoderSteps)) *
				// 							   encoderSteps) :
				// 			maxEncoderValue;
				// }
				// else if(adjustedValue < minEncoderValue / encoderSteps)
				// {
				// 	encoderPosition =
				// 		encoder->circleValues_ ?
				// 			maxEncoderValue + ((adjustedValue - (minEncoderValue / encoderSteps)) *
				// 							   encoderSteps) :
				// 			minEncoderValue;
				// }
				// encoder->encoderPosition_.store(encoderPosition);
				// encoder->lastMovementTime_.store(now);
				// encoder->oldDirection_.store(currentDirection);
			}
			if(auto callback = reinterpret_cast<void (*)()>(
				   encoder->encoderCallback_.load(std::memory_order_acquire)))
			{
				callback();
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

bool Encoder::debounce(bool currentState, unsigned long& lastTime, unsigned long delay)
{
	unsigned long now = esp_timer_get_time() / 1000;
	if(now - lastTime > delay)
	{
		lastTime = now;
		return currentState;
	}
	return false;
}

void Encoder::ButtonMonitorTask(void* param)
{
	Encoder* encoder = static_cast<Encoder*>(param);

	unsigned long lastDebounceTime = 0;

	while(true)
	{

		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{

			if(!encoder->isEnabled_)
			{
				encoder->buttonState_.store(ButtonState::BTN_DISABLED);
				continue;
			}
			bool buttonPressed = !gpio_get_level(encoder->buttonPin_);
			if(encoder->debounce(buttonPressed, lastDebounceTime, DEBOUNCE_DELAY))
			{
				ButtonState currentState = encoder->buttonState_.load();

				if(buttonPressed && currentState == ButtonState::UP)
				{
					encoder->buttonState_.store(ButtonState::PUSHED);
				}
				else if(!buttonPressed && currentState == ButtonState::DOWN)
				{
					encoder->buttonState_.store(ButtonState::RELEASED);
				}
				else
				{
					encoder->buttonState_.store(buttonPressed ? ButtonState::DOWN :
																ButtonState::UP);
				}

				if(auto callback = reinterpret_cast<void (*)()>(
					   encoder->buttonCallback_.load(std::memory_order_acquire)))
				{
					callback();
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}

	vTaskDelete(NULL);
}

void Encoder::setBoundaries(long minValue, long maxValue, bool circleValues)
{
	minEncoderValue_ = minValue * encoderSteps_;
	maxEncoderValue_ = maxValue * encoderSteps_;
	circleValues_ = circleValues;
}
long Encoder::readEncoder() const
{

	long position = encoderPosition_.load() / encoderSteps_;
	if(position > maxEncoderValue_ / encoderSteps_)
		return maxEncoderValue_ / encoderSteps_;
	if(position < minEncoderValue_ / encoderSteps_)
		return minEncoderValue_ / encoderSteps_;
	return position;
}

// void Encoder::reset(long newValue)
// {

// 	long adjustedValue = newValue * encoderSteps_ + correctionOffset_;
// 	encoderPosition_ = adjustedValue;

// 	if(encoderPosition_ > maxEncoderValue_)
// 	{
// 		encoderPosition_ = circleValues_ ? minEncoderValue_ : maxEncoderValue_;
// 	}
// 	if(encoderPosition_ < minEncoderValue_)
// 	{
// 		encoderPosition_ = circleValues_ ? maxEncoderValue_ : minEncoderValue_;
// 	}

// 	lastReadEncoderPosition_ = encoderPosition_ / encoderSteps_;
// }
void Encoder::reset(long newValue)
{
	long steps = encoderSteps_;
	long minVal = minEncoderValue_ / steps;
	long maxVal = maxEncoderValue_ / steps;
	long range = maxVal - minVal + 1;

	// Adjust newValue to be within the range using modulo arithmetic
	long wrappedValue = ((newValue - minVal) % range + range) % range + minVal;
	encoderPosition_ = wrappedValue * steps + correctionOffset_;

	// Ensure it's within the physical bounds after applying correction
	minVal = minEncoderValue_;
	maxVal = maxEncoderValue_;
	if(encoderPosition_ > maxVal || encoderPosition_ < minVal)
	{
		wrappedValue =
			((encoderPosition_ / steps - minVal / steps) % range + range) % range + minVal / steps;
		encoderPosition_ = wrappedValue * steps;
	}

	lastReadEncoderPosition_ = encoderPosition_ / steps;
}
bool Encoder::isEncoderButtonDown() { return gpio_get_level(buttonPin_) == 0 ? false : true; }

bool Encoder::isEncoderButtonClicked(unsigned long maximumWaitMilliseconds)
{
	static enum class ButtonClickState {
		IDLE,
		WAIT_FOR_RELEASE,
		WAIT_FOR_TIMEOUT
	} state = ButtonClickState::IDLE;

	static unsigned long waitStartTime = 0;
	static unsigned long lastDebounceTime = 0; // Debounce timer
	static bool wasTimeouted = false;

	bool buttonPressed = !gpio_get_level(buttonPin_);

	// Use debounce helper to validate button press/release
	if(!debounce(buttonPressed, lastDebounceTime, DEBOUNCE_DELAY))
	{
		return false; // Ignore changes within debounce period
	}

	switch(state)
	{
		case ButtonClickState::IDLE:
			if(buttonPressed)
			{
				// Start timing when button is first pressed
				waitStartTime = esp_timer_get_time() / 1000;
				state = ButtonClickState::WAIT_FOR_RELEASE;
			}
			break;

		case ButtonClickState::WAIT_FOR_RELEASE:
			if(!buttonPressed)
			{
				// Button released after debounce period
				if(esp_timer_get_time() / 1000 - waitStartTime > DEBOUNCE_DELAY)
				{
					wasTimeouted = false;
					state = ButtonClickState::IDLE;
					return true;
				}
				else
				{
					// Release was too quick; ignore
					state = ButtonClickState::IDLE;
				}
			}
			else if(esp_timer_get_time() / 1000 - waitStartTime > maximumWaitMilliseconds)
			{
				// Timeout occurred before button was released
				wasTimeouted = true;
				state = ButtonClickState::IDLE;
			}
			break;

		case ButtonClickState::WAIT_FOR_TIMEOUT:
			if(!buttonPressed)
			{
				// Reset state when button is released after timeout
				state = ButtonClickState::IDLE;
			}
			else if(esp_timer_get_time() / 1000 - waitStartTime > maximumWaitMilliseconds)
			{
				// Timeout occurred while waiting for release
				wasTimeouted = true;
				state = ButtonClickState::IDLE;
			}
			break;
	}

	return false;
}
