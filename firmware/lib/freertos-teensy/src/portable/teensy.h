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
 * @file    teensy.h
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 4
 * @author  Timo Sandmann
 * @date    04.06.2023
 */

#pragma once

#include "arduino_freertos.h"

#include <cstdint>
#include <sys/time.h>


namespace std {
template <class... Types>
class tuple;
}

#ifdef PRINT_DEBUG_STUFF
#define EXC_PRINTF(...) exc_printf(putchar_debug, __VA_ARGS__)
#define EXC_FLUSH()
#else
#define EXC_PRINTF(...) exc_printf(serialport_put, __VA_ARGS__)
#define EXC_FLUSH() ::serialport_flush()
#endif


extern "C" {
uint8_t get_debug_led_pin();

void exc_printf(void (*print)(const char), const char* format, ...) __attribute__((format(printf, 2, 3)));

/**
 * @brief Write character c to Serial
 * @param[in] c: Character to be written
 */
void serialport_put(const char c);

/**
 * @brief Write every character from the null-terminated C-string str and one additional newline character '\n' to Serial
 * @param[in] str: Character C-string to be written
 */
void serialport_puts(const char* str);

void serialport_flush();

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
void assert_blink(const char* file, int line, const char* func, const char* expr) __attribute__((noreturn));

void mcu_shutdown() __attribute__((noreturn, used));
} // extern C

namespace freertos {
void yield();

/**
 * @brief Delay between led error flashes
 * @param[in] ms: Milliseconds to delay
 * @note Doesn't use a timer to work with interrupts disabled
 */
void delay_ms(const uint32_t ms);

/**
 * @brief Indicate an error with the onboard LED
 * @param[in] n: Number of short LED pulses to encode the error
 */
void error_blink(const uint8_t n) __attribute__((noreturn));

/**
 * @brief Get amount of used and free RAM1
 * @return Tuple of: free RAM in byte, used data in byte, used bss in byte, used heap in byte, system free in byte, size of itcm in byte, ram size in byte
 */
std::tuple<size_t, size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage();

/**
 * @brief Get amount of used and free RAM2
 * @return Tuple of: free RAM in byte, ram size in byte
 */
std::tuple<size_t, size_t> ram2_usage();

/**
 * @brief Get amount of used and free external RAM on teensy 4.1
 * @return Tuple of: free RAM in byte, ram size of external RAM in byte
 */
std::tuple<size_t, size_t> ram3_usage();

/**
 * @brief Print amount of used and free RAM to Serial
 */
void print_ram_usage();

/**
 * @brief Get the current time in microseconds
 * @return Current time in us
 */
uint64_t get_us();

uint64_t get_us_from_isr();

/**
 * @brief Get the current time in milliseconds
 * @return Current time in ms
 */
static inline uint32_t get_ms() __attribute__((always_inline, unused));
static inline uint32_t get_ms() {
    return ::millis();
}

void print_stack_trace(TaskHandle_t task);

class clock {
    static timeval offset_;

public:
    static void sync_rtc();

    static inline const timeval* get_offset() {
        return &offset_;
    }
};
} // namespace freertos
