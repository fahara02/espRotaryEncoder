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
#include "etl/algorithm.h"

namespace Rotary
{

enum class ButtonState
{
	UP = 0,
	DEBOUNCE_PRESS,
	PRESSED,
	DEBOUNCE_RELEASE,
	RELEASED,
	BTN_DISABLED = 99,
};
struct EncoderConfig
{
	const uint8_t steps;
	const long min_value;
	const long max_value;
	const bool circular;
	const unsigned long acceleration;
	const int correction_offset;
	const long min_count;
	const long max_count;
	const long range;
	EncoderConfig(uint8_t step = 4) :
		steps(step), max_value(LONG_MAX), min_value(LONG_MIN), circular(false), acceleration(150),
		correction_offset(2), min_count(min_value / steps), max_count(max_value / steps),
		range(max_count - min_count + 1)
	{
	}
	EncoderConfig(uint8_t step, long maxV, long minV, bool iscircular, unsigned long acc = 150,
				  int off = 2) :
		steps(step), max_value(maxV * step), min_value(minV * step), circular(iscircular),
		acceleration(acc), correction_offset(off), min_count(min_value / steps),
		max_count(max_value / steps), range(max_count - min_count + 1)
	{
	}
};
enum class PullType
{
	INTERNAL_PULLUP,
	INTERNAL_PULLDOWN,
	EXTERNAL_PULLUP,
	EXTERNAL_PULLDOWN,
	NONE
};

class Encoder
{
  public:
	Encoder(uint8_t steps, gpio_num_t aPin, gpio_num_t bPin, gpio_num_t buttonPin = GPIO_NUM_NC,
			PullType encoderPinPull = PullType::EXTERNAL_PULLUP,
			PullType buttonPinPull = PullType::EXTERNAL_PULLDOWN);
	void configure(const EncoderConfig& config);
	void set_range(long minValue = -100, long maxValue = 100, bool circleValues = false);

	void init();
	void reset(long newValue = 0);
	long read() const;
	ButtonState button() const { return btn_state_.load(); }
	void button_state(ButtonState new_state) { return btn_state_.store(new_state); }

	bool is_btn_clicked(unsigned long max_wait);
	void set_callbacks(void (*encoder_cb)(), void (*btn_cb)() = nullptr);

  protected:
	unsigned long get_acceleration() const;
	void set_acceleration(unsigned long acc);

  private:
	struct State
	{
		std::atomic<long> position{0};
		std::atomic<int8_t> direction{0};
		std::atomic<unsigned long> last_update{0};
	};
	void init_gpio();
	void isr_handler();
	void btn_isr_handler();
	void monitor_encoder();
	void monitor_button();
	void update_position(int8_t dir);
	bool button_pressed() const;
	long apply_bounds(long value) const;
	void configure_pin(gpio_num_t pin, gpio_int_type_t intr_type, PullType pull_type);

	gpio_num_t clk_, dt_, btn_;
	PullType encoder_pull_, btn_pull_;
	// EncoderConfig config_;
	std::unique_ptr<EncoderConfig> config_;
	State state_;
	std::atomic<void (*)()> encoder_cb_{nullptr};
	std::atomic<void (*)()> btn_cb_{nullptr};
	std::atomic<ButtonState> btn_state_{ButtonState::UP};
	int correction_offset_ = 2;
	static constexpr uint32_t DEBOUNCE_MS = 10;
	static constexpr int8_t STATE_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

	// // Helper Functions

	TaskHandle_t encoderTaskHandle = nullptr;
	TaskHandle_t buttonTaskHandle = nullptr;
};

} // namespace Rotary

#endif