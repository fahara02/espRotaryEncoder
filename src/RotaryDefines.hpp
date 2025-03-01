#ifndef ROTARY_DEFINES_HPP
#define ROTARY_DEFINES_HPP
#include "stdint.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
enum class ButtonState {
  DOWN = 0,
  PUSHED = 1,
  UP = 2,
  RELEASED = 3,
  BTN_DISABLED = 99,
};
enum class PullType {
  INTERNAL_PULLUP,
  INTERNAL_PULLDOWN,
  EXTERNAL_PULLUP,
  EXTERNAL_PULLDOWN,
  NONE
};

static const uint32_t EncoderTaskStack = 4096;
static const UBaseType_t EncoderTask_Priority = 3;
static const BaseType_t EncoderTask_CORE = tskNO_AFFINITY;

static const uint32_t BtnTaskStack = 4096;
static const UBaseType_t BtnTask_Priority = 3;
static const BaseType_t BtnTask_CORE = tskNO_AFFINITY;
#endif