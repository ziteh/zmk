/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/event_manager.h>

struct zmk_slider_event {
  uint8_t slider_id;
  int16_t delta_position;
  int delta_time; // in us
  int64_t update_time;
};

ZMK_EVENT_DECLARE(zmk_slider_event);
