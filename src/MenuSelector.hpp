#ifndef MENU_SELECTOR_HPP
#define MENU_SELECTOR_HPP

#include "RotaryEncoder.hpp"

struct MenuItem
{
	const char* label;
	void (*action)();
};

class MenuSelector : public Rotary::Encoder
{
  public:
	using selection_changed_cb = void (*)(size_t index);
	using item_selected_cb = void (*)(size_t index);

	MenuSelector(const MenuItem* items, size_t itemCount, uint8_t pulsesPerClick, gpio_num_t aPin,
				 gpio_num_t bPin, gpio_num_t buttonPin = GPIO_NUM_NC,
				 Rotary::PullType encoderPinPull = Rotary::PullType::EXTERNAL_PULLUP,
				 Rotary::PullType buttonPinPull = Rotary::PullType::EXTERNAL_PULLDOWN);

	void init();
	void set_selection_changed_Cb(selection_changed_cb cb);
	void set_item_selected_cb(item_selected_cb cb);
	size_t get_selected_index() const;
	const MenuItem* get_selected_item() const;

  private:
	const MenuItem* menuItems_;
	size_t itemCount_;
	uint8_t pulsesPerClick_;
	size_t previousIndex_;
	selection_changed_cb selectionChangedCb_;
	item_selected_cb itemSelectedCb_;
	static MenuSelector* instance_;

	static void encoder_callback();
	static void button_callback();
	void handle_encoder();
	void handle_button();
};

#endif