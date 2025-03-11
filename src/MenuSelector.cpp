#include "MenuSelector.hpp"

MenuSelector::MenuSelector(const MenuItem* items, size_t itemCount, uint8_t pulsesPerClick,
						   gpio_num_t aPin, gpio_num_t bPin, gpio_num_t buttonPin,
						   Rotary::PullType encoderPinPull, Rotary::PullType buttonPinPull) :
	Rotary::Encoder(1, aPin, bPin, buttonPin, encoderPinPull, buttonPinPull), menuItems_(items),
	itemCount_(itemCount), pulsesPerClick_(pulsesPerClick), previousIndex_(0),
	selectionChangedCb_(nullptr), itemSelectedCb_(nullptr)
{

	if(!items || itemCount == 0 || pulsesPerClick == 0)
	{
		itemCount_ = 0;
		return;
	}

	instance_ = this;
}

void MenuSelector::init()
{
	Rotary::Encoder::init();
	set_callbacks(encoder_callback, button_callback);

	long maxValue = (itemCount_ - 1) * pulsesPerClick_;
	configure(Rotary::EncoderConfig(1, maxValue, 0, true /* circular */));
}

void MenuSelector::set_selection_changed_cb(selection_changed_cb cb, void* user_data)
{
	selectionChangedCb_ = cb;
	selection_changed_user_data_ = user_data;
}

void MenuSelector::set_item_selected_cb(item_selected_cb cb, void* user_data)
{
	itemSelectedCb_ = cb;
	item_selected_user_data_ = user_data;
}

size_t MenuSelector::get_selected_index() const { return read() / pulsesPerClick_; }

const MenuItem* MenuSelector::get_selected_item() const
{
	size_t index = get_selected_index();
	return (index < itemCount_) ? &menuItems_[index] : nullptr;
}

void MenuSelector::encoder_callback()
{
	if(instance_)
	{
		instance_->handle_encoder();
	}
}

void MenuSelector::button_callback()
{
	if(instance_)
	{
		instance_->handle_button();
	}
}

void MenuSelector::handle_encoder()
{
	long value = read();
	size_t currentIndex = value / pulsesPerClick_;
	if(currentIndex != previousIndex_)
	{
		previousIndex_ = currentIndex;
		if(selectionChangedCb_)
		{
			selectionChangedCb_(currentIndex, selection_changed_user_data_);
		}
	}
}

void MenuSelector::handle_button()
{
	size_t currentIndex = get_selected_index();
	if(currentIndex < itemCount_)
	{
		if(menuItems_[currentIndex].action)
		{
			menuItems_[currentIndex].action(item_selected_user_data_);
		}
		if(itemSelectedCb_)
		{
			itemSelectedCb_(currentIndex, item_selected_user_data_);
		}
	}
}

// Initialize static member
MenuSelector* MenuSelector::instance_ = nullptr;