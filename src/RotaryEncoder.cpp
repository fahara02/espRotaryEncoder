
#include "RotaryEncoder.hpp"
#include "esp_log.h"

#define LOG_TAG "RotaryEncoder"

using namespace Rotary;

// Global variable for debouncing encoder ISR events
static volatile unsigned long lastInterruptTime = 0;

//////////////////////
/// Constructor  ///
//////////////////////

Encoder::Encoder(uint8_t steps, gpio_num_t clk, gpio_num_t dt, gpio_num_t btn,
				 PullType encoder_pull, PullType btn_pull) :
	clk_(clk), dt_(dt), btn_(btn), encoder_pull_(encoder_pull), btn_pull_(btn_pull),
	config_(std::make_unique<EncoderConfig>())
{
}

//////////////////////
///  Configuration ///
//////////////////////
void Encoder::configure(const EncoderConfig& cfg)
{
	config_ = std::make_unique<EncoderConfig>(cfg);
}

unsigned long Encoder::get_acceleration() const { return config_->acceleration; }

void Encoder::set_acceleration(unsigned long acc)
{
	EncoderConfig config(config_->steps, config_->max_value, config_->min_value, config_->circular,
						 acc);
	configure(config);
}
void Encoder::set_range(long minValue, long maxValue, bool circleValues)
{
	EncoderConfig config(config_->steps, maxValue, minValue, circleValues);
	configure(config);
}

//////////////////////////
/// GPIO Configuration ///
//////////////////////////

void Encoder::configure_pin(gpio_num_t pin, gpio_int_type_t intr_type, PullType pull_type)
{
	gpio_config_t ioConfig = {};
	ioConfig.intr_type = intr_type;
	ioConfig.mode = GPIO_MODE_INPUT;
	ioConfig.pin_bit_mask = (1ULL << pin);

	// Configure pull-up/pull-down settings
	switch(pull_type)
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

void Encoder::init_gpio()
{
	// Configure encoder pins
	configure_pin(clk_, GPIO_INTR_ANYEDGE, encoder_pull_);
	configure_pin(dt_, GPIO_INTR_ANYEDGE, encoder_pull_);
	if(btn_ != GPIO_NUM_NC)
	{
		configure_pin(btn_, GPIO_INTR_POSEDGE, btn_pull_);
	}
	// Install ISR service and attach handlers
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(clk_, [](void* arg) { static_cast<Encoder*>(arg)->isr_handler(); }, this);
	gpio_isr_handler_add(dt_, [](void* arg) { static_cast<Encoder*>(arg)->isr_handler(); }, this);
	if(btn_ != GPIO_NUM_NC)
	{
		gpio_isr_handler_add(
			btn_, [](void* arg) { static_cast<Encoder*>(arg)->btn_isr_handler(); }, this);
	}
}

void Encoder::init()
{
	init_gpio();
	xTaskCreatePinnedToCore([](void* arg) { static_cast<Encoder*>(arg)->monitor_encoder(); },
							"encoder", 4096, this, EncoderTask_Priority, &encoderTaskHandle,
							EncoderTask_CORE);

	if(btn_ != GPIO_NUM_NC)
	{
		xTaskCreatePinnedToCore([](void* arg) { static_cast<Encoder*>(arg)->monitor_button(); },
								"button", BtnTaskStack, this, BtnTask_Priority, &buttonTaskHandle,
								BtnTask_CORE);
	}
}

//////////////////////////
/// Callback Setup     ///
//////////////////////////

void Encoder::set_callbacks(void (*encoder_cb)(), void (*btn_cb)())
{
	encoder_cb_.store(encoder_cb);
	btn_cb_.store(btn_cb);
}

//////////////////////////
/// Interrupt Handlers ///
//////////////////////////

void IRAM_ATTR Encoder::isr_handler()
{
	static uint32_t last_isr = 0;
	const auto now = xTaskGetTickCountFromISR();
	if(now - last_isr > DEBOUNCE_MS)
	{
		last_isr = now;
		BaseType_t wake = pdFALSE;
		vTaskNotifyGiveFromISR(encoderTaskHandle, &wake);
		portYIELD_FROM_ISR(wake);
	}
}

void IRAM_ATTR Encoder::btn_isr_handler()
{
	static uint32_t last_isr = 0;
	const auto now = xTaskGetTickCountFromISR();
	if(now - last_isr > DEBOUNCE_MS)
	{
		last_isr = now;
		BaseType_t wake = pdFALSE;
		vTaskNotifyGiveFromISR(buttonTaskHandle, &wake);
		portYIELD_FROM_ISR(wake);
	}
}

//////////////////////////
/// Task Functions     ///
//////////////////////////
void Encoder::monitor_encoder()
{
	uint8_t state = 0;
	while(true)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		state = (state << 2) | (((gpio_get_level(dt_) << 1) | gpio_get_level(clk_)) & 0x03);

		if(auto dir = STATE_TABLE[state & 0x0F])
		{

			update_position(dir);
			if(auto cb = reinterpret_cast<void (*)()>(encoder_cb_.load(std::memory_order_acquire)))
				cb();
		}
		vTaskDelay(1);
	}
}
bool Encoder::button_pressed() const
{
	if(btn_pull_ == PullType::EXTERNAL_PULLUP || btn_pull_ == PullType::INTERNAL_PULLUP)
	{
		return !gpio_get_level(btn_);
	}
	else
	{
		return gpio_get_level(btn_);
	}
}

void Encoder::monitor_button()
{
	ButtonState state = ButtonState::UP;
	uint32_t last_transition = xTaskGetTickCount();
	static const TickType_t debounce_ticks = pdMS_TO_TICKS(DEBOUNCE_MS);

	while(true)
	{
		// Wait for notification or timeout (10ms)
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
		const uint32_t now = xTaskGetTickCount();
		const bool pressed = button_pressed(); // Single read per iteration

		switch(state)
		{
			case ButtonState::UP:
				if(pressed)
				{
					state = ButtonState::DEBOUNCE_PRESS;
					last_transition = now;
					Serial.println("Transition: UP -> DEBOUNCE_PRESS");
				}
				break;

			case ButtonState::DEBOUNCE_PRESS:
				if((now - last_transition) > debounce_ticks)
				{
					if(pressed)
					{
						state = ButtonState::PRESSED;
						Serial.println("Transition: DEBOUNCE_PRESS -> PRESSED");
					}
					else
					{
						state = ButtonState::UP;
						Serial.println("Transition: DEBOUNCE_PRESS -> UP (false alarm)");
					}
				}
				break;

			case ButtonState::PRESSED:
				if(!pressed)
				{
					state = ButtonState::DEBOUNCE_RELEASE;
					last_transition = now;
					Serial.println("Transition: PRESSED -> DEBOUNCE_RELEASE");
				}
				break;

			case ButtonState::DEBOUNCE_RELEASE:
				if((now - last_transition) > debounce_ticks)
				{
					if(!pressed)
					{
						state = ButtonState::RELEASED;
						Serial.println("Transition: DEBOUNCE_RELEASE -> RELEASED");
					}
					else
					{
						state = ButtonState::PRESSED;
						Serial.println("Transition: DEBOUNCE_RELEASE -> PRESSED (bounce detected)");
					}
				}
				break;

			case ButtonState::RELEASED:
				if(auto cb = reinterpret_cast<void (*)()>(btn_cb_.load(std::memory_order_acquire)))
				{
					ESP_LOGI(LOG_TAG, "Invoking button callback on release");
					cb();
				}
				else
				{
					ESP_LOGW(LOG_TAG, "No button callback set");
				}
				state = ButtonState::UP;
				Serial.println("Transition: RELEASED -> UP");
				break;
		}
	}
}

//////////////////////////
/// Value and Reset ///
//////////////////////////
long Encoder::read() const
{

	const EncoderConfig& conf = *config_;

	long count = state_.position.load() / conf.steps;
	if(count > conf.max_count)
		return conf.max_count;
	if(count < conf.min_count)
		return conf.min_count;
	return count;
}

void Encoder::reset(long newValue)
{
	const EncoderConfig& conf = *config_;
	const long steps = conf.steps;
	long bounded_count;

	if(conf.circular)
	{
		const long range = conf.range;
		if(range > 0)
		{
			bounded_count = ((newValue - conf.min_count) % range + range) % range + conf.min_count;
		}
		else
		{
			bounded_count = conf.min_count;
		}
	}
	else
	{
		bounded_count = etl::clamp(newValue, conf.min_count, conf.max_count);
	}

	long position = bounded_count * steps;
	state_.position.store(position);
}

//////////////////////////
/// Button Queries     ///
//////////////////////////

bool Encoder::is_btn_clicked(unsigned long max_wait)
{
	enum class State
	{
		IDLE,
		DEBOUNCING_PRESS,
		WAIT_RELEASE,
		DEBOUNCING_RELEASE
	};
	static State state = State::IDLE;
	static unsigned long debounce_start = 0;
	static unsigned long press_time = 0;

	const bool pressed = button_pressed();
	unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	ESP_LOGD(LOG_TAG, "State: %d, Pressed: %d, Now: %lu", (int)state, pressed, now);
	switch(state)
	{
		case State::IDLE:
			if(pressed)
			{
				debounce_start = now;
				state = State::DEBOUNCING_PRESS;
			}
			break;

		case State::DEBOUNCING_PRESS:
			if(now - debounce_start >= DEBOUNCE_MS)
			{
				if(pressed)
				{ // Valid press
					press_time = now;
					state = State::WAIT_RELEASE;
				}
				else
				{ // Noise, reset
					state = State::IDLE;
				}
			}
			break;

		case State::WAIT_RELEASE:
			if(!pressed)
			{
				debounce_start = now;
				state = State::DEBOUNCING_RELEASE;
			}
			else if(now - press_time > max_wait)
			{
				state = State::IDLE; // Timeout
			}
			break;

		case State::DEBOUNCING_RELEASE:
			if(now - debounce_start >= DEBOUNCE_MS)
			{
				state = State::IDLE;
				return (!pressed && (now - press_time <= max_wait));
			}
			break;
	}
	return false;
}
//////////////////////////
/// Core Logic    ///
//////////////////////////
/*This function updates the encoder’s position based on a direction input (dir), which is typically
 * +1 or -1, representing a clockwise or counterclockwise turn. It optionally applies acceleration
 * when crossing step boundaries and ensures the new position respects configured bounds.*/
void Encoder::update_position(int8_t dir)
{
	const EncoderConfig& conf = *config_;
	const auto now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	const long current_position = state_.position.load();
	const long current_count = current_position / conf.steps;
	long new_pos = current_position + dir;
	const long new_count = new_pos / conf.steps;

	// Apply acceleration only when crossing step boundaries
	if(conf.acceleration && dir == state_.direction && new_count != current_count)
	{
		const unsigned long time_since_last = now - state_.last_update;
		if(time_since_last < ACCELERATION_LONG_CUTOFF)
		{
			const unsigned long limited_time = std::max(ACCELERATION_SHORT_CUTOFF, time_since_last);
			new_pos += dir * (conf.acceleration / limited_time);
		}
	}

	state_.position.store(apply_bounds(new_pos));
	state_.direction.store(dir);
	state_.last_update.store(now);
}

/*apply_bounds ensures the encoder position stays within configured limits, either by clamping
 * (non-circular) or wrapping (circular). It operates on the count (steps) but returns a position.*/
long Encoder::apply_bounds(long value) const
{
	const EncoderConfig& conf = *config_;
	const long current_count = value / conf.steps;
	long bounded_count = current_count;

	if(conf.circular)
	{
		const long range = conf.range;
		if(range > 0) // Prevent modulo by zero or negative
		{
			bounded_count =
				((current_count - conf.min_count) % range + range) % range + conf.min_count;
		}
	}
	else
	{
		bounded_count = etl::clamp(current_count, conf.min_count, conf.max_count);
	}

	// Only modify position if count actually changed
	return (bounded_count != current_count) ? bounded_count * conf.steps : value;
}

// void Encoder::monitor_button()
// {
// 	ButtonState state = ButtonState::UP;
// 	uint32_t last_transition = xTaskGetTickCount();

// 	while(true)
// 	{
// 		// Use a short timeout so the task wakes up even if no notification occurs.
// 		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
// 		uint32_t now = xTaskGetTickCount();
// 		bool pressed = button_pressed();

// 		switch(state)
// 		{
// 			case ButtonState::UP:
// 				if(pressed)
// 				{
// 					state = ButtonState::DEBOUNCE_PRESS;
// 					last_transition = now;
// 					Serial.println("Transition: UP -> DEBOUNCE_PRESS");
// 				}
// 				break;

// 			case ButtonState::DEBOUNCE_PRESS:
// 				if((now - last_transition) > pdMS_TO_TICKS(DEBOUNCE_MS))
// 				{
// 					if(pressed)
// 					{
// 						state = ButtonState::PRESSED;
// 						Serial.println("Transition: DEBOUNCE_PRESS -> PRESSED");
// 					}
// 					else
// 					{
// 						state = ButtonState::UP;
// 						Serial.println("Transition: DEBOUNCE_PRESS -> UP (false alarm)");
// 					}
// 				}
// 				break;

// 			case ButtonState::PRESSED:
// 				if(!pressed)
// 				{
// 					state = ButtonState::DEBOUNCE_RELEASE;
// 					last_transition = now;
// 					Serial.println("Transition: PRESSED -> DEBOUNCE_RELEASE");
// 				}
// 				break;

// 			case ButtonState::DEBOUNCE_RELEASE:
// 				if((now - last_transition) > pdMS_TO_TICKS(DEBOUNCE_MS))
// 				{
// 					if(!pressed)
// 					{
// 						state = ButtonState::RELEASED;
// 						Serial.println("Transition: DEBOUNCE_RELEASE -> RELEASED");
// 					}
// 					else
// 					{
// 						state = ButtonState::PRESSED;
// 						Serial.println("Transition: DEBOUNCE_RELEASE -> PRESSED (bounce detected)");
// 					}
// 				}
// 				break;

// 			case ButtonState::RELEASED:
// 				if(auto cb = reinterpret_cast<void (*)()>(btn_cb_.load(std::memory_order_acquire)))
// 				{
// 					ESP_LOGI(LOG_TAG, "Invoking button callback on release");
// 					cb();
// 				}
// 				else
// 				{
// 					ESP_LOGW(LOG_TAG, "No button callback set");
// 				}
// 				state = ButtonState::UP;
// 				Serial.println("Transition: RELEASED -> UP");
// 				break;
// 		}

// 		vTaskDelay(1); // small delay to avoid hogging the CPU
// 	}
// }

// void Encoder::monitor_button()
// {
// 	ButtonState state = ButtonState::UP;
// 	uint32_t last_transition = xTaskGetTickCount();

// 	while(true)
// 	{
// 		// Use a short timeout so the task wakes up even if no notification occurs.
// 		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
// 		uint32_t now = xTaskGetTickCount();
// 		bool pressed = button_pressed();

// 		switch(state)
// 		{
// 			case ButtonState::UP:
// 				if(pressed)
// 				{
// 					state = ButtonState::DEBOUNCE_PRESS;
// 					last_transition = now;
// 					Serial.println("Transition: UP -> DEBOUNCE_PRESS");
// 				}
// 				break;

// 			case ButtonState::DEBOUNCE_PRESS:
// 				if((now - last_transition) > pdMS_TO_TICKS(DEBOUNCE_MS))
// 				{
// 					if(pressed)
// 					{
// 						state = ButtonState::PRESSED;
// 						Serial.println("Transition: DEBOUNCE_PRESS -> PRESSED");
// 					}
// 					else
// 					{
// 						state = ButtonState::UP;
// 						Serial.println("Transition: DEBOUNCE_PRESS -> UP (false alarm)");
// 					}
// 				}
// 				break;

// 			case ButtonState::PRESSED:
// 				if(!pressed)
// 				{
// 					state = ButtonState::DEBOUNCE_RELEASE;
// 					last_transition = now;
// 					Serial.println("Transition: PRESSED -> DEBOUNCE_RELEASE");
// 				}
// 				break;

// 			case ButtonState::DEBOUNCE_RELEASE:
// 				if((now - last_transition) > pdMS_TO_TICKS(DEBOUNCE_MS))
// 				{
// 					if(!pressed)
// 					{
// 						state = ButtonState::WAIT_RELEASE;
// 						Serial.println("Transition: DEBOUNCE_RELEASE -> WAIT_RELEASE");
// 					}
// 					else
// 					{
// 						state = ButtonState::PRESSED;
// 						Serial.println("Transition: DEBOUNCE_RELEASE -> PRESSED (bounce detected)");
// 					}
// 				}
// 				break;

// 			case ButtonState::WAIT_RELEASE:
// 				// Additional logic can be added here if needed before releasing
// 				state = ButtonState::RELEASED;
// 				Serial.println("Transition: WAIT_RELEASE -> RELEASED");
// 				break;

// 			case ButtonState::RELEASED:
// 				if(auto cb = reinterpret_cast<void (*)()>(btn_cb_.load(std::memory_order_acquire)))
// 				{
// 					ESP_LOGI(LOG_TAG, "Invoking button callback on release");
// 					cb();
// 				}
// 				else
// 				{
// 					ESP_LOGW(LOG_TAG, "No button callback set");
// 				}
// 				state = ButtonState::UP;
// 				Serial.println("Transition: RELEASED -> UP");
// 				break;
// 		}

// 		vTaskDelay(1); // small delay to avoid hogging the CPU
// 	}
// }