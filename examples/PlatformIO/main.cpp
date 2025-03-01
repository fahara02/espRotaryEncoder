#include <Arduino.h>
#include "RotaryEncoder.hpp"

gpio_num_t pinA = GPIO_NUM_37; // GPIO pin for Channel A
gpio_num_t pinB = GPIO_NUM_38;

Rotary::Encoder encoder = Rotary::Encoder(4, pinA, pinB);
void readEncoderISRcb() { Serial.println(" encoder button isr"); }
void setup()
{

	Serial.begin(115200);

	delay(1000);
	encoder.begin();
	encoder.setup(readEncoderISRcb);
	bool circleValues = true;
	encoder.setBoundaries(0, 10, circleValues);
}
void loop()
{

	Serial.println("....");
	Serial.printf("count is %ld \n", encoder.readEncoder());
	delay(500);
}
