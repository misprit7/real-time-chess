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
 * @file    teensy_3.cpp
 * @brief   FreeRTOS support implementations for Teensy 3.5 and 3.6 boards with newlib 4
 * @author  Timo Sandmann
 * @date    04.06.2023
 */

#if defined __MK64FX512__ || defined __MK66FX1M0__
#include <cstring>
#include <malloc.h>
#include <tuple>

#include "teensy.h"
#include "event_responder_support.h"
#include "util/atomic.h"
#include "EventResponder.h"

#define __ASM __asm
#define __STATIC_INLINE static inline
#define __CORTEX_M 4
#include "core_cmInstr.h"


static constexpr bool DEBUG { false };

extern "C" {
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _sdata;
extern unsigned long _edata;
extern uint8_t* _g_current_heap_end;

uint32_t set_arm_clock(uint32_t) { // dummy
    return F_CPU;
}

void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
} // extern C


#ifdef USB_TRIPLE_SERIAL
extern uint8_t yield_active_check_flags;
extern const uint8_t _serialEventUSB2_default;
extern const uint8_t _serialEventUSB1_default;
#elif defined(USB_DUAL_SERIAL)
extern uint8_t yield_active_check_flags;
extern const uint8_t _serialEventUSB1_default;
#else
extern uint8_t yield_active_check_flags;
#endif
extern const uint8_t _serialEvent_default;


namespace freertos {
void yield() {
    static uint8_t running = 0;
    if (!yield_active_check_flags) {
        // nothing to do
        return;
    }
    if (running) {
        return;
    }
    running = 1;

    // USB Serial - Add hack to minimize impact...
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIAL) {
        if (Serial.available()) {
            serialEvent();
        }
        if (_serialEvent_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIAL;
        }
    }
    // Current workaround until integrate with EventResponder.

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIALUSB1) {
        if (SerialUSB1.available()) {
            serialEventUSB1();
        }
        if (_serialEventUSB1_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIALUSB1;
        }
    }
#endif // USB_DUAL_SERIAL || USB_TRIPLE_SERIAL
#ifdef USB_TRIPLE_SERIAL
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIALUSB2) {
        if (SerialUSB2.available()) {
            serialEventUSB2();
        }
        if (_serialEventUSB2_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIALUSB2;
        }
    }
#endif // USB_TRIPLE_SERIAL

#if !defined DISABLE_ARDUINO_HWSERIAL
    if (yield_active_check_flags & YIELD_CHECK_HARDWARE_SERIAL) {
        HardwareSerial::processSerialEventsList();
    }
#endif // !DISABLE_ARDUINO_HWSERIAL

    running = 0;
    if (yield_active_check_flags & YIELD_CHECK_EVENT_RESPONDER) {
        EventResponder::runFromYield();
    }
}

/**
 * @brief Check for USB events pending and call the USB ISR
 */
static void poll_usb() {
    if (SIM_SCGC4 & SIM_SCGC4_USBOTG) {
        ::usb_isr();
    }
}

void delay_ms(const uint32_t ms) {
    constexpr uint32_t CYCLES_MS { F_CPU / 7'000UL };
    const uint32_t n { ms / 10 };
    for (uint32_t i {}; i < n; ++i) {
        poll_usb();

        for (uint32_t i {}; i < 10UL * CYCLES_MS; ++i) {
            __asm volatile("nop");
        }
    }

    const uint32_t iterations { (ms % 10) * CYCLES_MS };
    for (uint32_t i {}; i < iterations; ++i) {
        __asm volatile("nop");
    }
}

std::tuple<size_t, size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() {
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - reinterpret_cast<uint8_t*>(0x1fff'0000)) };
    const size_t system_free { (reinterpret_cast<uint8_t*>(&_estack) - _g_current_heap_end) - 1024UL };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t, size_t, size_t, size_t, size_t> ret { system_free + info.fordblks, 0, 0, info.uordblks, system_free, 0, ram_size };
    return ret;
}

std::tuple<size_t, size_t> ram2_usage() {
    const std::tuple<size_t, size_t> ret { 0, 0 };
    return ret;
}

std::tuple<size_t, size_t> ram3_usage() {
    const std::tuple<size_t, size_t> ret { 0, 0 };
    return ret;
}

uint64_t get_us() {
    uint32_t current, load, count, istatus;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = SYST_CVR;
        count = get_ms();
        istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
        load = SYST_RVR;
    }

    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) {
        ++count;
    }

    current = load - current;
#if (configUSE_TICKLESS_IDLE == 1)
#warning "tickless idle mode is untested"
    return static_cast<uint64_t>(count) * 1'000U + current * 1'000U / (load + 1U);
#else
    return static_cast<uint64_t>(count) * 1'000U + current / (configCPU_CLOCK_HZ / configTICK_RATE_HZ / 1'000U);
#endif
}

uint64_t get_us_from_isr() {
    uint32_t current, load, count, istatus;
    current = SYST_CVR;
    count = get_ms();
    istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
    load = SYST_RVR;

    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) {
        ++count;
    }

    current = load - current;
#if (configUSE_TICKLESS_IDLE == 1)
#warning "tickless idle mode is untested"
    return static_cast<uint64_t>(count) * 1'000U + current * 1'000U / (load + 1U);
#else
    return static_cast<uint64_t>(count) * 1'000U + current / (configCPU_CLOCK_HZ / configTICK_RATE_HZ / 1'000U);
#endif
}
} // namespace freertos

extern "C" {
void xPortPendSVHandler();
void xPortSysTickHandler();
void vPortSVCHandler();
void vPortSetupTimerInterrupt() FLASHMEM;

void vPortSetupTimerInterrupt() {
    if (DEBUG) {
        EXC_PRINTF(PSTR("vPortSetupTimerInterrupt()\r\n"));
    }

    /* stop and clear the SysTick */
    SYST_CSR = 0;
    SYST_CVR = 0;

    /* override arduino vector table entries */
    _VectorsRam[11] = vPortSVCHandler;
    _VectorsRam[14] = xPortPendSVHandler;
    _VectorsRam[15] = xPortSysTickHandler;
    __NVIC_SetPriorityGrouping(0);

    /* configure SysTick to interrupt at the requested rate */
    static_assert(
        (static_cast<int32_t>(configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1) > 0, "unsupported configTICK_RATE_HZ for the used clock source detected!");
    SYST_RVR = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
    SYST_CSR = SYST_CSR_TICKINT | SYST_CSR_ENABLE;

/* calculate the constants required to configure the tick interrupt */
#if configUSE_TICKLESS_IDLE == 1
    { // FIXME: doesn't work
        ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
        xMaximumPossibleSuppressedTicks = 0xffffffUL / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = 94UL / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
    }
#endif // configUSE_TICKLESS_IDLE

    freertos::clock::sync_rtc();

    freertos::setup_event_responder();

    if (DEBUG) {
        EXC_PRINTF(PSTR("vPortSetupTimerInterrupt() done.\r\n"));
    }
}

#if configUSE_TICK_HOOK > 0
void vApplicationTickHook();
void vApplicationTickHook() {
    static_assert(configTICK_RATE_HZ % 1'000 == 0, "unsupported configTICK_RATE_HZ detected, please adjust vApplicationTickHook()!");

    static uint32_t n {};
    if (configTICK_RATE_HZ == 1'000UL || ++n == configTICK_RATE_HZ / 1'000UL) {
        systick_millis_count = systick_millis_count + 1;
        n = 0;
    }
}
#else
#error "configUSE_TICK_HOOK == 0 isn't supported!"
#endif // configUSE_TICK_HOOK
} // extern C

namespace freertos {
void print_stack_trace(TaskHandle_t) {
    // not implemented
}
} // namespace freertos
#endif // __MK64FX512__ || __MK66FX1M0__
