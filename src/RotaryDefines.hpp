#ifndef ROTARY_DEFINES_HPP
#define ROTARY_DEFINES_HPP
#include "stdint.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

constexpr uint16_t QUEUE_SIZE = 10;
static constexpr unsigned long DEBOUNCE_DELAY = 50;
static constexpr unsigned long ENCODER_DEBOUNCE_DELAY = 10;
static constexpr unsigned long ACCELERATION_LONG_CUTOFF = 200;
static constexpr unsigned long ACCELERATION_SHORT_CUTOFF = 4;
static const uint32_t EncoderTaskStack = 4096;
static const UBaseType_t EncoderTask_Priority = 3;
static const BaseType_t EncoderTask_CORE = tskNO_AFFINITY;

static const uint32_t BtnTaskStack = 4096;
static const UBaseType_t BtnTask_Priority = 3;
static const BaseType_t BtnTask_CORE = tskNO_AFFINITY;

enum class ButtonState
{
	DOWN = 0,
	PUSHED = 1,
	UP = 2,
	RELEASED = 3,
	BTN_DISABLED = 99,
};
enum class PullType
{
	INTERNAL_PULLUP,
	INTERNAL_PULLDOWN,
	EXTERNAL_PULLUP,
	EXTERNAL_PULLDOWN,
	NONE
};

#endif