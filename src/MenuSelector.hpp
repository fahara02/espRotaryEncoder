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
				 Rotary::PullType buttonPinPull = Rotary::PullType::EXTERNAL_PULLDOWN) :
		Rotary::Encoder(1, aPin, bPin, buttonPin, encoderPinPull, buttonPinPull), menuItems_(items),
		itemCount_(itemCount), pulsesPerClick_(pulsesPerClick), previousIndex_(0)
	{

		if(!items || itemCount == 0 || pulsesPerClick == 0)
		{
			itemCount_ = 0;
			return;
		}

		instance_ = this;
	}

	void init()
	{
		Rotary::Encoder::init();
		set_callbacks(encoder_callback, button_callback);

		long maxValue = (itemCount_ - 1) * pulsesPerClick_;
		configure(Rotary::EncoderConfig(1, maxValue, 0, true /* circular */));
	}

	void set_selection_changed_Cb(selection_changed_cb cb) { selectionChangedCb_ = cb; }

	void set_item_selected_cb(item_selected_cb cb) { itemSelectedCb_ = cb; }

	size_t get_selected_index() const { return read() / pulsesPerClick_; }

	const MenuItem* get_selected_item() const
	{
		size_t index = get_selected_index();
		return (index < itemCount_) ? &menuItems_[index] : nullptr;
	}

  private:
	const MenuItem* menuItems_;
	size_t itemCount_;
	uint8_t pulsesPerClick_;
	size_t previousIndex_;
	selection_changed_cb selectionChangedCb_ = nullptr;
	item_selected_cb itemSelectedCb_ = nullptr;
	static MenuSelector* instance_;

	static void encoder_callback()
	{
		if(instance_)
		{
			instance_->handle_encoder();
		}
	}

	static void button_callback()
	{
		if(instance_)
		{
			instance_->handle_button();
		}
	}

	void handle_encoder()
	{
		long value = read();
		size_t currentIndex = value / pulsesPerClick_;
		if(currentIndex != previousIndex_) // Removed redundant check
		{
			previousIndex_ = currentIndex;
			if(selectionChangedCb_)
			{
				selectionChangedCb_(currentIndex);
			}
		}
	}

	void handle_button()
	{

		size_t index = get_selected_index();
		if(index < itemCount_)
		{
			if(menuItems_[index].action)
			{
				menuItems_[index].action();
			}
			if(itemSelectedCb_)
			{
				itemSelectedCb_(index);
			}
		}
	}
};

// Initialize static member
MenuSelector* MenuSelector::instance_ = nullptr;
