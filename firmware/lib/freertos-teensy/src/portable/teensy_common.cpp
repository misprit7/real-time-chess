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
 * @file    teensy_common.cpp
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 4
 * @author  Timo Sandmann
 * @date    04.06.2023
 */

#define _DEFAULT_SOURCE
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <unwind.h>
#include <tuple>
#include <cstdarg>

#include "avr/pgmspace.h"
#include "teensy.h"
#include "event_responder_support.h"


#if !(defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41 || defined __MK64FX512__ || defined __MK66FX1M0__)
#error "Unsupported board"
#endif

static constexpr bool DEBUG { false };

using namespace arduino;

extern "C" {
asm(".global _printf_float"); /**< printf supporting floating point values */

extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _ebss;

extern volatile uint32_t systick_millis_count;
extern volatile uint32_t systick_cycle_count;
extern uint32_t set_arm_clock(uint32_t frequency);
uint8_t* _g_current_heap_end { reinterpret_cast<uint8_t*>(&_ebss) + 32 };


FLASHMEM __attribute__((weak)) uint8_t get_debug_led_pin() {
    return LED_BUILTIN;
}

static FLASHMEM void exc_puint(void (*print)(const char), unsigned int num) {
    // based on puint_debug() of teensy cores library (https://github.com/PaulStoffregen/cores)
    char buf[12];
    unsigned int i = sizeof(buf) - 2;

    buf[sizeof(buf) - 1] = 0;
    while (1) {
        buf[i] = (num % 10) + '0';
        num /= 10;
        if (num == 0) {
            break;
        }
        i--;
    }
    exc_printf(print, buf + i);
}

FLASHMEM void exc_printf(void (*print)(const char), const char* format, ...) {
    // based on printf_debug() of teensy cores library (https://github.com/PaulStoffregen/cores)
    std::va_list args;
    unsigned int val;
    int n;

    va_start(args, format);
    for (; *format != 0; format++) { // no-frills stand-alone printf
        if (*format == '%') {
            ++format;
            if (*format == '%') {
                goto out;
            }
            if (*format == '-') {
                format++; // ignore size
            }
            while (*format >= '0' && *format <= '9') {
                format++; // ignore size
            }
            if (*format == 'l') {
                format++; // ignore long
            }
            if (*format == '\0') {
                break;
            }
            if (*format == 's') {
                exc_printf(print, (char*) va_arg(args, int));
            } else if (*format == 'd') {
                n = va_arg(args, int);
                if (n < 0) {
                    n = -n;
                    print('-');
                }
                exc_puint(print, n);
            } else if (*format == 'u') {
                exc_puint(print, va_arg(args, unsigned int));
            } else if (*format == 'x' || *format == 'X') {
                val = va_arg(args, unsigned int);
                for (n = 0; n < 8; n++) {
                    unsigned int d = (val >> 28) & 15;
                    print((d < 10) ? d + '0' : d - 10 + 'A');
                    val <<= 4;
                }
            } else if (*format == 'c') {
                print((char) va_arg(args, int));
            }
        } else {
        out:
            print(*format);
        }
    }
    va_end(args);
}

FLASHMEM __attribute__((weak)) void serialport_put(const char c) {
    ::Serial.print(c);
}

FLASHMEM __attribute__((weak)) void serialport_puts(const char* str) {
    ::Serial.println(str);
    ::Serial.flush();
}

FLASHMEM __attribute__((weak)) void serialport_flush() {
    ::Serial.flush();
    freertos::delay_ms(100);
}

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos 16U /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_PRIGROUP_Pos 8U /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos) /*!< SCB AIRCR: PRIGROUP Mask */

/**
  \brief        Set Priority Grouping
  \details      Sets the priority grouping field using the required unlock sequence.
                The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
                Only values from 0..7 are used.
                In case of a conflict between priority grouping and available
                priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]   PriorityGroup  Priority grouping field.
 */
FLASHMEM void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup) {
    const uint32_t PriorityGroupTmp { (PriorityGroup & static_cast<uint32_t>(0x7)) }; // only values 0..7 are used

    uint32_t reg_value { SCB_AIRCR }; // read old register configuration
    reg_value &= ~(static_cast<uint32_t>(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); // clear bits to change
    /* Insert write key and priority group */
    reg_value = (reg_value | (static_cast<uint32_t>(0x5FAUL) << SCB_AIRCR_VECTKEY_Pos) | (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos));
    SCB_AIRCR = reg_value;
}

uint32_t g_trace_lr;
void prvTaskExitError();

FLASHMEM _Unwind_Reason_Code trace_fcn(_Unwind_Context* ctx, void* depth) {
    int* p_depth { static_cast<int*>(depth) };

    const auto ip { _Unwind_GetIP(ctx) };
    const auto start { _Unwind_GetRegionStart(ctx) };
    EXC_PRINTF(PSTR("\t#%d"), *p_depth);

    if (ip == (reinterpret_cast<uintptr_t>(&prvTaskExitError) & ~1) || ip == 0) {
        EXC_PRINTF(PSTR(":\t[Task entry point]\r\n"));
        return _URC_END_OF_STACK;
    } else {
        EXC_PRINTF(PSTR(":\t0x%04x"), *p_depth ? (ip - 1) & ~1 : ip);
        EXC_PRINTF(PSTR(" [0x%04x]\r\n"), start);
    }

    if (g_trace_lr) {
        _Unwind_SetGR(ctx, 14, g_trace_lr);
        g_trace_lr = 0;
    }

    ++(*p_depth);
    if (*p_depth == 32) {
        return _URC_END_OF_STACK;
    }

    return _URC_NO_REASON;
}

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
FLASHMEM void assert_blink(const char* file, int line, const char* func, const char* expr) {
    portDISABLE_INTERRUPTS();
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
    NVIC_SET_PRIORITY(IRQ_USB1, (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << (8 - configPRIO_BITS));
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41

    EXC_PRINTF(PSTR("\r\nASSERT in [%s:%u]\t"), file, line);
    EXC_PRINTF(PSTR("%s(): "), func);
    EXC_PRINTF(PSTR("%s\r\n"), expr);

    EXC_PRINTF(PSTR("\r\nStack trace:\r\n"));
    EXC_FLUSH();

    int depth {};
    _Unwind_Backtrace(&trace_fcn, &depth);
    EXC_PRINTF(PSTR("\r\n"));

    freertos::error_blink(1);
}

FLASHMEM void mcu_shutdown() {
    freertos::error_blink(0);
}
} // extern C


namespace freertos {
timeval clock::offset_ { 0, 0 };

FLASHMEM void error_blink(const uint8_t n) {
    ::vTaskSuspendAll();
    const uint8_t debug_led_pin { get_debug_led_pin() };
    ::pinMode(debug_led_pin, OUTPUT);
    ::set_arm_clock(16'000'000UL);

    while (true) {
        for (uint8_t i {}; i < n; ++i) {
            ::digitalWriteFast(debug_led_pin, true);
            delay_ms(300UL);
            ::digitalWriteFast(debug_led_pin, false);
            delay_ms(300UL);
        }
        delay_ms(2'000UL);
    }
}

FLASHMEM void print_ram_usage() {
    const auto info1 { ram1_usage() };
    const auto info2 { ram2_usage() };

    EXC_PRINTF(PSTR("RAM1 size: %u KB, free RAM1: %u KB, data used: %u KB, bss used: %u KB, used heap: %u KB, system free: %u KB\r\n"),
        std::get<5>(info1) / 1'024UL, std::get<0>(info1) / 1'024UL, std::get<1>(info1) / 1'024UL, std::get<2>(info1) / 1'024UL, std::get<3>(info1) / 1'024UL,
        std::get<4>(info1) / 1'024UL);
    EXC_PRINTF(PSTR("RAM2 size: %u KB, free RAM2: %u KB, used RAM2: %u KB\r\n"), std::get<1>(info2) / 1'024UL, std::get<0>(info2) / 1'024UL,
        (std::get<1>(info2) - std::get<0>(info2)) / 1'024UL);
    EXC_PRINTF(PSTR("\r\n"));
    EXC_FLUSH();
}

void clock::sync_rtc() {
    taskENTER_CRITICAL();

    const auto now_us { freertos::get_us() };
    const timeval now { static_cast<time_t>(now_us / 1'000'000UL), static_cast<suseconds_t>(now_us % 1'000'000UL) };
    const timeval rtc { static_cast<time_t>(::rtc_get()), 0 };

    if (timercmp(&now, &rtc, <)) {
        timersub(&rtc, &now, &offset_);
    } else {
        timersub(&now, &rtc, &offset_);
    }

    taskEXIT_CRITICAL();
}
} // namespace freertos

extern "C" {
void setup_systick_with_timer_events() {}

void event_responder_set_pend_sv() {
    if (freertos::g_event_responder_task) {
        ::xTaskNotify(freertos::g_event_responder_task, 0, eNoAction);
    }
}

FLASHMEM void yield() {
    if (::xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED && freertos::g_yield_task) {
        if (xPortIsInsideInterrupt() == pdTRUE) {
            BaseType_t higher_woken { pdFALSE };
            ::xTaskNotifyFromISR(freertos::g_yield_task, 0, eNoAction, &higher_woken);
            portYIELD_FROM_ISR(higher_woken);
            portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
        } else {
            ::xTaskNotify(freertos::g_yield_task, 0, eNoAction);
        }
    } else {
        freertos::yield();
    }
}

#if configUSE_IDLE_HOOK == 1
void vApplicationIdleHook() {}
#endif // configUSE_IDLE_HOOK

void vApplicationStackOverflowHook(TaskHandle_t, char*) FLASHMEM;

void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    static char taskname[configMAX_TASK_NAME_LEN + 1];

    std::memcpy(taskname, task_name, configMAX_TASK_NAME_LEN);
    EXC_PRINTF(PSTR("STACK OVERFLOW: %s\r\n"), taskname);
    EXC_FLUSH();

    freertos::error_blink(3);
}

#if defined PLATFORMIO || TEENSYDUINO >= 158
#if configUSE_MALLOC_FAILED_HOOK == 1
FLASHMEM void vApplicationMallocFailedHook() {
    freertos::error_blink(2);
}
#endif // configUSE_MALLOC_FAILED_HOOK

void* _sbrk_r(struct _reent* p_reent, ptrdiff_t incr) {
    static_assert(portSTACK_GROWTH == -1, "Stack growth down assumed");

    if (DEBUG) {
        EXC_PRINTF(PSTR("_sbrk_r(%d): "), incr);
        EXC_PRINTF(PSTR("current_heap_end=0x%x "), reinterpret_cast<uintptr_t>(_g_current_heap_end));
        EXC_PRINTF(PSTR("_ebss=0x%x "), reinterpret_cast<uintptr_t>(&_ebss));
        EXC_PRINTF(PSTR("_estack=0x%x\r\n"), reinterpret_cast<uintptr_t>(&_estack));
    }

    const auto primask = __get_PRIMASK();
    __disable_irq();
    void* previous_heap_end { _g_current_heap_end };

    if ((reinterpret_cast<uintptr_t>(_g_current_heap_end) + incr >= reinterpret_cast<uintptr_t>(&_estack) - 8'192U)
        || (reinterpret_cast<uintptr_t>(_g_current_heap_end) + incr < reinterpret_cast<uintptr_t>(&_ebss))) {
        __set_PRIMASK(primask);

        EXC_PRINTF(PSTR("_sbrk_r(%d): no mem available.\r\n"), incr);

#if configUSE_MALLOC_FAILED_HOOK == 1
        ::vApplicationMallocFailedHook();
        (void) p_reent;
#else
        p_reent->_errno = ENOMEM;
#endif

        return reinterpret_cast<void*>(-1); // the malloc-family routine that called sbrk will return 0
    }

    _g_current_heap_end += incr;
    __set_PRIMASK(primask);

    return previous_heap_end;
}

void* sbrk(ptrdiff_t incr) {
    return _sbrk_r(_impure_ptr, incr);
}
void* _sbrk(ptrdiff_t incr) {
    return sbrk(incr);
};
#endif // PLATFORMIO || TEENSYDUINO >= 158

FLASHMEM int _gettimeofday(timeval* tv, void*) {
    const auto p_offset { freertos::clock::get_offset() };

    const auto now_us { freertos::get_us() };
    const timeval now { static_cast<time_t>(now_us / 1'000'000UL), static_cast<suseconds_t>(now_us % 1'000'000UL) };

    timeradd(p_offset, &now, tv);
    return 0;
}

#if configGENERATE_RUN_TIME_STATS == 1
uint64_t freertos_get_us() {
    return freertos::get_us();
}
#endif // configGENERATE_RUN_TIME_STATS

void init_newlib_locks() FLASHMEM;

void startup_late_hook() __attribute__((noinline, section(".flashmem")));
void startup_late_hook() {
    init_newlib_locks();
}
} // extern C
