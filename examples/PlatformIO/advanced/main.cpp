#include <Arduino.h>
#include "RotaryEncoder.hpp"

gpio_num_t pinA = GPIO_NUM_37; // GPIO pin for Channel A
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t btn = GPIO_NUM_32;
void action1() { Serial.println("Action 1 selected"); }
void action2() { Serial.println("Action 2 selected"); }
void action3() { Serial.println("Action 3 selected"); }

const MenuItem menuItems[] = {{"Item 1", action1},
							  {"Item 2", action2},
							  {"Item 3", action3},
							  {"Item 4", action3},
							  {"Item 5", action3}};
constexpr size_t MENU_ITEM_COUNT = sizeof(menuItems) / sizeof(menuItems[0]);
MenuSelector menu(menuItems, MENU_ITEM_COUNT, 4, pinA, pinB, btn);
void onSelectionChanged(size_t index) { Serial.println(menu.get_selected_item()->label); }
void onItemSelected(size_t index) { Serial.println("Item confirmed"); }
void setup()
{

	Serial.begin(115200);

	delay(1000);
	menu.set_selection_changed_Cb(onSelectionChanged);
	menu.set_item_selected_cb(onItemSelected);
	menu.init();
}
void loop() { vTaskDelete(NULL); }
