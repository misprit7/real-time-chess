/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    event_responder_support.cpp
 * @brief   FreeRTOS support implementations for Teensy EventResponder
 * @author  Timo Sandmann
 * @date    20.05.2020
 */

#include "event_responder_support.h"
#include "teensy.h"

#include "arduino_freertos.h"
#include "timers.h"
#include "EventResponder.h"


namespace freertos {
TaskHandle_t g_yield_task {};
TaskHandle_t g_event_responder_task {};

void setup_event_responder() {
    ::xTaskCreate(
        [](void*) {
            while (true) {
                ::xTaskNotifyWait(0, 0, nullptr, pdMS_TO_TICKS(YIELD_TASK_PERIOD_MS));
                freertos::yield();
            }
        },
        PSTR("YIELD"), YIELD_TASK_STACK_SIZE, nullptr, YIELD_TASK_PRIORITY, &g_yield_task);

    auto p_event_timer_ { ::xTimerCreate(PSTR("event_t"), pdMS_TO_TICKS(1), true, nullptr, [](TimerHandle_t) { ::MillisTimer::runFromTimer(); }) };
    xTimerStart(p_event_timer_, 0);

    ::xTaskCreate(
        [](void*) {
            while (true) {
                ::xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
                ::EventResponder::runFromInterrupt();
            }
        },
        PSTR("EVENT"), EVENT_TASK_STACK_SIZE, nullptr, configMAX_PRIORITIES - 2, &g_event_responder_task);
}
} // namespace freertos
