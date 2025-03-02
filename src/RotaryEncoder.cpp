
#include "RotaryEncoder.hpp"
#include "esp_log.h"

#define LOG_TAG "RotaryEncoder"

using namespace Rotary;

// Global variable for debouncing encoder ISR events
static volatile unsigned long lastInterruptTime = 0;

//////////////////////
/// Constructor  ///
//////////////////////
Encoder::Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin, gpio_num_t buttonPin,
				 PullType encoderPinPull, PullType buttonPinPull) :
	aPin_(aPin), bPin_(bPin), buttonPin_(buttonPin), encoderSteps_(steps),
	encoderPinPull_(encoderPinPull), buttonPinPull_(buttonPinPull), isEnabled_(true)
{
}

//////////////////////////
/// Basic Operations  ///
//////////////////////////

void Encoder::setEncoderValue(long newValue) { reset(newValue); }

void Encoder::enable() { isEnabled_ = true; }

void Encoder::disable() { isEnabled_ = false; }

ButtonState Encoder::readButtonState() const { return buttonState_; }

unsigned long Encoder::getAcceleration() const { return rotaryAccelerationCoef_; }

void Encoder::setAcceleration(unsigned long acceleration)
{
	rotaryAccelerationCoef_ = acceleration;
}

void Encoder::disableAcceleration() { setAcceleration(0); }

//////////////////////////
/// GPIO Configuration ///
//////////////////////////

void Encoder::configurePin(gpio_num_t pin, gpio_int_type_t intrType, PullType pullType)
{
	gpio_config_t ioConfig = {};
	ioConfig.intr_type = intrType;
	ioConfig.mode = GPIO_MODE_INPUT;
	ioConfig.pin_bit_mask = (1ULL << pin);

	// Configure pull-up/pull-down settings
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
	// Configure encoder pins
	configurePin(aPin_, GPIO_INTR_ANYEDGE, encoderPinPull_);
	configurePin(bPin_, GPIO_INTR_ANYEDGE, encoderPinPull_);
	if(buttonPin_ != GPIO_NUM_NC)
	{
		configurePin(buttonPin_, GPIO_INTR_POSEDGE, buttonPinPull_);
	}
	// Install ISR service and attach handlers
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
	}
}

//////////////////////////
/// Callback Setup     ///
//////////////////////////

void Encoder::setup(void (*ISR_callback)(void))
{
	encoderCallback_.store(reinterpret_cast<void*>(ISR_callback), std::memory_order_release);
}

void Encoder::setup(void (*ISR_callback)(void), void (*ISR_button)(void))
{
	setup(ISR_callback);
	buttonCallback_.store(reinterpret_cast<void*>(ISR_button), std::memory_order_release);
}

//////////////////////////
/// Encoder State      ///
//////////////////////////

int8_t Encoder::updateOldABState()
{
	int8_t oldAB = oldAB_.load(std::memory_order_acquire);
	// Shift left 2 bits and append current A/B levels (each level is 1 bit)
	oldAB = (oldAB << 2) | (((gpio_get_level(bPin_) << 1) | gpio_get_level(aPin_)) & 0x03);
	oldAB_.store(oldAB, std::memory_order_release);
	return oldAB;
}

//////////////////////////
/// Interrupt Handlers ///
//////////////////////////

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

//////////////////////////
/// Task Functions     ///
//////////////////////////
void Encoder::EncoderMonitorTask(void* param)
{
	Encoder* encoder = static_cast<Encoder*>(param);

	// Create an EncoderData struct instance with the current state.

	while(true)
	{
		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			unsigned long now = esp_timer_get_time() / 1000;
			if(!encoder->isEnabled_)
				continue;
			EncoderData data;
			data.position = encoder->encoderPosition_.load();
			data.steps = encoder->encoderSteps_;
			data.oldDirection = encoder->oldDirection_.load();
			data.maxValue = encoder->maxEncoderValue_;
			data.minValue = encoder->minEncoderValue_;
			data.lastMovementTime = encoder->lastMovementTime_.load();

			int8_t oldAB = encoder->updateOldABState();
			int8_t direction = encoder->encoderStates_[oldAB & 0x0F];

			if(direction != 0)
			{
				// Update the position using the helper function
				encoder->updatePosition(data, direction, now, encoder->rotaryAccelerationCoef_);
				// Wrap the value if necessary
				encoder->wrapPosition(data);

				// Save updated state back to the encoder object
				encoder->encoderPosition_.store(data.position);
				encoder->lastMovementTime_.store(now);
				encoder->oldDirection_.store(direction);
			}
			// Call user callback if registered
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

// // Task to monitor encoder rotation
// void Encoder::EncoderMonitorTask(void* param)
// {
// 	auto* encoder = static_cast<Encoder*>(param);
// 	int8_t currentDirection = 0;

// 	while(true)
// 	{
// 		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
// 		{
// 			unsigned long now = esp_timer_get_time() / 1000;
// 			if(!encoder->isEnabled_)
// 				continue;

// 			int8_t oldAB = encoder->updateOldABState();
// 			currentDirection = encoder->encoderStates_[oldAB & 0x0F];

// 			long encoderPosition = encoder->encoderPosition_.load();
// 			const uint8_t steps = encoder->encoderSteps_;

// 			if(currentDirection != 0)
// 			{
// 				long prevCount = encoderPosition / steps;
// 				encoderPosition += currentDirection;
// 				long newCount = encoderPosition / steps;

// 				// Apply acceleration adjustment if enabled and direction is maintained
// 				if(newCount != prevCount && encoder->rotaryAccelerationCoef_ > 1)
// 				{
// 					int8_t lastDirection = encoder->oldDirection_.load();
// 					if(currentDirection == lastDirection && currentDirection != 0)
// 					{
// 						unsigned long timeSinceLastMotion = now - encoder->lastMovementTime_.load();
// 						if(timeSinceLastMotion < ACCELERATION_LONG_CUTOFF)
// 						{
// 							unsigned long limitedTime =
// 								std::max(ACCELERATION_SHORT_CUTOFF, timeSinceLastMotion);
// 							int adjustment = encoder->rotaryAccelerationCoef_ / limitedTime;
// 							encoderPosition += (currentDirection > 0 ? adjustment : -adjustment);
// 						}
// 					}
// 				}

// 				// Wrap around if out of boundaries
// 				long minVal = encoder->minEncoderValue_ / steps;
// 				long maxVal = encoder->maxEncoderValue_ / steps;
// 				long range = maxVal - minVal + 1;
// 				long currVal = encoderPosition / steps;
// 				if(currVal > maxVal || currVal < minVal)
// 				{
// 					long wrappedValue = ((currVal - minVal) % range + range) % range + minVal;
// 					encoderPosition = wrappedValue * steps;
// 				}

// 				encoder->encoderPosition_.store(encoderPosition);
// 				encoder->lastMovementTime_.store(now);
// 				encoder->oldDirection_.store(currentDirection);
// 			}
// 			// Call user callback if registered
// 			if(auto callback = reinterpret_cast<void (*)()>(
// 				   encoder->encoderCallback_.load(std::memory_order_acquire)))
// 			{
// 				callback();
// 			}
// 		}
// 		vTaskDelay(pdMS_TO_TICKS(10));
// 	}
// 	vTaskDelete(NULL);
// }

// Helper function for debouncing; returns true if the debounce period has elapsed
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

// Task to monitor the encoder button
void Encoder::ButtonMonitorTask(void* param)
{
	auto* encoder = static_cast<Encoder*>(param);
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
			// Button is assumed active low
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
				// Call user callback if registered
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

//////////////////////////
/// Value and Reset    ///
//////////////////////////

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

void Encoder::reset(long newValue)
{
	const long steps = encoderSteps_;
	long minVal = minEncoderValue_ / steps;
	long maxVal = maxEncoderValue_ / steps;
	long range = maxVal - minVal + 1;

	// Wrap newValue into the valid range using modulo arithmetic
	long wrappedValue = ((newValue - minVal) % range + range) % range + minVal;
	encoderPosition_ = wrappedValue * steps + correctionOffset_;

	// Double-check boundaries after applying correction
	if(encoderPosition_ > maxEncoderValue_ || encoderPosition_ < minEncoderValue_)
	{
		wrappedValue =
			((encoderPosition_ / steps - minEncoderValue_ / steps) % range + range) % range +
			minVal;
		encoderPosition_ = wrappedValue * steps;
	}
	lastReadEncoderPosition_ = encoderPosition_ / steps;
}

//////////////////////////
/// Button Queries     ///
//////////////////////////

bool Encoder::isEncoderButtonDown() const
{
	// Assuming active-low button logic: low (0) means pressed
	return gpio_get_level(buttonPin_) != 0;
}

bool Encoder::isEncoderButtonClicked(unsigned long maximumWaitMilliseconds)
{
	enum class ButtonClickState
	{
		IDLE,
		WAIT_FOR_RELEASE,
		WAIT_FOR_TIMEOUT
	};
	static ButtonClickState state = ButtonClickState::IDLE;
	static unsigned long waitStartTime = 0;
	static unsigned long lastDebounceTime = 0;
	static bool wasTimeouted = false;

	bool buttonPressed = !gpio_get_level(buttonPin_);
	if(!debounce(buttonPressed, lastDebounceTime, DEBOUNCE_DELAY))
	{
		return false;
	}

	switch(state)
	{
		case ButtonClickState::IDLE:
			if(buttonPressed)
			{
				waitStartTime = esp_timer_get_time() / 1000;
				state = ButtonClickState::WAIT_FOR_RELEASE;
			}
			break;
		case ButtonClickState::WAIT_FOR_RELEASE:
			if(!buttonPressed)
			{
				if((esp_timer_get_time() / 1000) - waitStartTime > DEBOUNCE_DELAY)
				{
					wasTimeouted = false;
					state = ButtonClickState::IDLE;
					return true;
				}
				else
				{
					state = ButtonClickState::IDLE;
				}
			}
			else if((esp_timer_get_time() / 1000) - waitStartTime > maximumWaitMilliseconds)
			{
				wasTimeouted = true;
				state = ButtonClickState::IDLE;
			}
			break;
		case ButtonClickState::WAIT_FOR_TIMEOUT:
			if(!buttonPressed)
			{
				state = ButtonClickState::IDLE;
			}
			else if((esp_timer_get_time() / 1000) - waitStartTime > maximumWaitMilliseconds)
			{
				wasTimeouted = true;
				state = ButtonClickState::IDLE;
			}
			break;
	}
	return false;
}

// Helper: Update position with optional acceleration
long Encoder::updatePosition(EncoderData& data, int8_t direction, unsigned long currentTime,
							 unsigned long acceCoef)
{
	// Update the raw position
	long prevCount = data.position / data.steps;
	data.position += direction;
	long newCount = data.position / data.steps;

	// If there is a count change and acceleration is enabled...
	if(newCount != prevCount && acceCoef > 1)
	{
		if(direction == data.oldDirection && direction != 0)
		{
			unsigned long timeSinceLast = currentTime - data.lastMovementTime;
			if(timeSinceLast < ACCELERATION_LONG_CUTOFF)
			{
				// Limit the time interval to avoid excessive acceleration
				unsigned long limitedTime = std::max(ACCELERATION_SHORT_CUTOFF, timeSinceLast);
				int adjustment = acceCoef / limitedTime;
				data.position += (direction > 0 ? adjustment : -adjustment);
			}
		}
	}
	return data.position;
}

long Encoder::wrapPosition(EncoderData& data)
{
	long minVal = data.minValue / data.steps;
	long maxVal = data.maxValue / data.steps;
	long range = maxVal - minVal + 1;
	long currentValue = data.position / data.steps;

	if(currentValue > maxVal || currentValue < minVal)
	{
		// Apply modulo arithmetic to wrap the value
		long wrappedValue = ((currentValue - minVal) % range + range) % range + minVal;
		data.position = wrappedValue * data.steps;
	}
	return data.position;
}