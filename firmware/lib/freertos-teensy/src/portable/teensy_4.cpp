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
 * @file    teensy_4.cpp
 * @brief   FreeRTOS support implementations for Teensy 4.0 and 4.1 boards with newlib 4
 * @author  Timo Sandmann
 * @date    04.06.2023
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include <cstring>
#include <malloc.h>
#include <tuple>
#include <unwind.h>

#include "teensy.h"
#include "event_responder_support.h"
#include "avr/pgmspace.h"
#include "EventResponder.h"
#include "imxrt.h"

#define __ASM __asm
#define __STATIC_INLINE static inline
#define __CORTEX_M 7
#include "core_cmInstr.h"

#ifdef PRINT_DEBUG_STUFF
#define EXC_PRINTF_EARLY(...) exc_printf(putchar_debug, __VA_ARGS__)
#else
#define EXC_PRINTF_EARLY(...)
#endif


static constexpr bool DEBUG { false }; // compile with -DPRINT_DEBUG_STUFF for debug output on Serial4

extern "C" {
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _extram_start;
extern unsigned long _extram_end;
extern unsigned long _itcm_block_count;
extern uint8_t* _g_current_heap_end;

extern volatile uint32_t systick_millis_count;
extern volatile uint32_t systick_cycle_count;
extern volatile uint32_t scale_cpu_cycles_to_microseconds;
extern uint32_t systick_safe_read;
extern uint8_t external_psram_size;

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

#if TEENSYDUINO <= 158
extern const uint8_t _serialEvent_default;
#endif

namespace freertos {
#if TEENSYDUINO <= 158

FLASHMEM void yield() {
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
    // Current workaround until integrate with EventResponder.
    if (yield_active_check_flags & YIELD_CHECK_HARDWARE_SERIAL) {
        HardwareSerial::processSerialEventsList();
    }
#endif // !DISABLE_ARDUINO_HWSERIAL

    running = 0;
    if (yield_active_check_flags & YIELD_CHECK_EVENT_RESPONDER) {
        EventResponder::runFromYield();
    }
}
#else // TEENSYDUINO > 158
FLASHMEM void yield() {
    static uint8_t running = 0;

    const uint8_t check_flags = yield_active_check_flags;
    if (!check_flags) {
        return; // nothing to do
    }

    if (running) {
        return;
    }
    running = 1;

    // USB Serial - Add hack to minimize impact...
    if (check_flags & YIELD_CHECK_USB_SERIAL) {
        if (Serial.available()) {
            serialEvent();
        }
    }

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    if (check_flags & YIELD_CHECK_USB_SERIALUSB1) {
        if (SerialUSB1.available()) {
            serialEventUSB1();
        }
    }
#endif // USB_DUAL_SERIAL || USB_TRIPLE_SERIAL
#ifdef USB_TRIPLE_SERIAL
    if (check_flags & YIELD_CHECK_USB_SERIALUSB2) {
        if (SerialUSB2.available()) {
            serialEventUSB2();
        }
    }
#endif // USB_TRIPLE_SERIAL

#if !defined DISABLE_ARDUINO_HWSERIAL
    // Current workaround until integrate with EventResponder.
    if (check_flags & YIELD_CHECK_HARDWARE_SERIAL) {
        HardwareSerialIMXRT::processSerialEventsList();
    }
#endif // !DISABLE_ARDUINO_HWSERIAL

    running = 0;
    if (check_flags & YIELD_CHECK_EVENT_RESPONDER) {
        EventResponder::runFromYield();
    }
}
#endif // TEENSYDUINO

FLASHMEM void delay_ms(const uint32_t ms) {
    const uint32_t cycles_ms { static_cast<uint32_t>((1ULL << 32) * 1'000'000ULL / 2'000ULL / static_cast<uint64_t>(scale_cpu_cycles_to_microseconds)) };
    const uint32_t n { ms / 10 };
    const uint32_t iterations { (ms % 10) * cycles_ms };

    for (uint32_t i {}; i < n; ++i) {
        for (uint32_t i {}; i < 10UL * cycles_ms; ++i) {
            __asm volatile("nop");
        }
    }
    for (uint32_t i {}; i < iterations; ++i) {
        __asm volatile("nop");
    }
    portINSTR_SYNC_BARRIER();
}

FLASHMEM std::tuple<size_t, size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() {
    const size_t blk_cnt { reinterpret_cast<uintptr_t>(&_itcm_block_count) };
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - reinterpret_cast<uint8_t*>(0x20'000'000)) + blk_cnt * 32'768U };
    const size_t bss { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_ebss) - reinterpret_cast<uint8_t*>(&_sbss)) };
    const size_t data { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_edata) - reinterpret_cast<uint8_t*>(&_sdata)) };
    const size_t system_free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - _g_current_heap_end) - 8'192U };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t, size_t, size_t, size_t, size_t> ret { system_free + info.fordblks, data, bss, info.uordblks, system_free,
        blk_cnt * 32'768U, ram_size };
    return ret;
}

FLASHMEM std::tuple<size_t, size_t> ram2_usage() {
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(0x20'280'000) - reinterpret_cast<uint8_t*>(0x20'200'000)) };
    const size_t free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_heap_end) - reinterpret_cast<uint8_t*>(&_heap_start)) };

    const std::tuple<size_t, size_t> ret { free, ram_size };
    return ret;
}

FLASHMEM std::tuple<size_t, size_t> ram3_usage() {
    const size_t ram_size { static_cast<size_t>(external_psram_size * (1 << 20)) };
    const size_t free { ram_size - static_cast<size_t>(reinterpret_cast<uint8_t*>(&_extram_end) - reinterpret_cast<uint8_t*>(&_extram_start)) };

    const std::tuple<size_t, size_t> ret { free, ram_size };
    return ret;
}

FASTRUN uint64_t get_us() {
    uint32_t smc, scc, cyccnt;
    do {
        __LDREXW(&systick_safe_read);
        smc = systick_millis_count;
        scc = systick_cycle_count;
        cyccnt = ARM_DWT_CYCCNT;
    } while (__STREXW(1, &systick_safe_read));

    const uint32_t ccdelta { cyccnt - scc };
    const uint32_t frac { static_cast<uint32_t>((static_cast<uint64_t>(ccdelta) * scale_cpu_cycles_to_microseconds) >> 32) };

    return static_cast<uint64_t>(smc) * 1'000ULL + frac;
}

FASTRUN uint64_t get_us_from_isr() {
    const uint32_t smc { systick_millis_count };
    const uint32_t scc { systick_cycle_count };
    const uint32_t cyccnt { ARM_DWT_CYCCNT };
    const uint32_t ccdelta { cyccnt - scc };
    const uint32_t frac { static_cast<uint32_t>((static_cast<uint64_t>(ccdelta) * scale_cpu_cycles_to_microseconds) >> 32) };

    return static_cast<uint64_t>(smc) * 1'000ULL + frac;
}
} // namespace freertos

extern "C" {
void xPortPendSVHandler();
void xPortSysTickHandler();
void vPortSVCHandler();
void vPortSetupTimerInterrupt() FLASHMEM;
void unused_interrupt_vector();

/**
 * Stack frame (code from: https://community.nxp.com/thread/389002):
 *  xPSR
 *  ReturnAddress
 *  LR (R14) - typically FFFFFFF9 for IRQ or Exception
 *  R12
 *  R3
 *  R2
 *  R1
 *  R0
 */
void unused_isr_freertos() FLASHMEM __attribute__((naked, used));
void unused_isr_freertos() {
    __asm(".syntax unified      \n"
          "MOV R1, LR           \n"
          "TST R1, #4           \n"
          "BEQ 1f               \n"
          "MRS R0, PSP          \n"
          "B HardFault_HandlerC \n"
          "1:                   \n"
          "MRS R0, MSP          \n"
          "B HardFault_HandlerC \n"
          ".syntax divided      \n");
}

#if (configUSE_TICKLESS_IDLE == 1)
extern uint32_t ulTimerCountsForOneTick;
extern uint32_t xMaximumPossibleSuppressedTicks;
extern uint32_t ulStoppedTimerCompensation;
#endif /* configUSE_TICKLESS_IDLE */

void vPortSetupTimerInterrupt() {
    if (DEBUG) {
        EXC_PRINTF_EARLY(PSTR("vPortSetupTimerInterrupt()\r\n"));
    }

    __disable_irq();

    /* stop and clear the SysTick */
    SYST_CSR = 0;
    SYST_CVR = 0;

    /* override unused / fault irq handler */
    for (auto i { 1 }; i < NVIC_NUM_INTERRUPTS + 16; ++i) {
        if (_VectorsRam[i] == &unused_interrupt_vector) {
            _VectorsRam[i] = &unused_isr_freertos;

            if (DEBUG) {
                // EXC_PRINTF_EARLY(PSTR("vPortSetupTimerInterrupt(): IRQ %d updated.\r\n"), i);
            }
        } else if (DEBUG) {
            EXC_PRINTF_EARLY(PSTR("vPortSetupTimerInterrupt(): IRQ %d NOT updated.\r\n"), i);
        }
    }

    /* override arduino vector table entries */
    _VectorsRam[0] = reinterpret_cast<void (*)()>(&_estack);
    _VectorsRam[11] = vPortSVCHandler;
    _VectorsRam[14] = xPortPendSVHandler;
    _VectorsRam[15] = xPortSysTickHandler;
    __NVIC_SetPriorityGrouping(0);

    portDATA_SYNC_BARRIER();
    portINSTR_SYNC_BARRIER();

    /* configure SysTick to interrupt at the requested rate */
    static_assert(
        (static_cast<int32_t>(configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1) > 0, "unsupported configTICK_RATE_HZ for the used clock source detected!");
    SYST_RVR = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
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
        EXC_PRINTF_EARLY(PSTR("SCB_SHPR3=0x%x\r\n"), SCB_SHPR3);
        EXC_PRINTF_EARLY(PSTR("vPortSetupTimerInterrupt() done.\r\n"));
    }
}

#if configUSE_TICK_HOOK > 0
void vApplicationTickHook();
void vApplicationTickHook() {
    static_assert(configTICK_RATE_HZ % 1'000 == 0, "unsupported configTICK_RATE_HZ detected, please adjust vApplicationTickHook()!");
    static uint32_t n {};

    const uint32_t cyccnt { ARM_DWT_CYCCNT };
    if (configTICK_RATE_HZ == 1'000UL || ++n == configTICK_RATE_HZ / 1'000UL) {
        systick_cycle_count = cyccnt;
        systick_millis_count = systick_millis_count + 1;
        n = 0;
    }
}
#else
#error "configUSE_TICK_HOOK == 0 isn't supported!"
#endif // configUSE_TICK_HOOK

/// This struct definition mimics the internal structures of libgcc in
/// arm-none-eabi binary. It's not portable and might break in the future.
struct core_regs {
    unsigned r[16];
};

/// This struct definition mimics the internal structures of libgcc in
/// arm-none-eabi binary. It's not portable and might break in the future.
typedef struct {
    unsigned demand_save_flags;
    struct core_regs core;
} phase2_vrs;

TaskHandle_t pxGetTaskFromStack(StackType_t*);
_Unwind_Reason_Code trace_fcn(_Unwind_Context*, void*);
_Unwind_Reason_Code __gnu_Unwind_Backtrace(_Unwind_Trace_Fn, void*, phase2_vrs*);

static void mcu_hardfault() FLASHMEM __attribute__((noreturn, used));
static void mcu_hardfault() {
    freertos::error_blink(4);
}

extern uint32_t g_trace_lr;

void HardFault_HandlerC(unsigned int* hardfault_args) FLASHMEM __attribute__((used));
void HardFault_HandlerC(unsigned int* hardfault_args) {
    unsigned int sp;
    __asm__ volatile("mov %0, r0" : "=r"(sp)::);

    unsigned int lr;
    __asm__ volatile("mov %0, r1" : "=r"(lr)::);

    unsigned int addr;
    __asm__ volatile("mrs %0, ipsr" : "=r"(addr)::);

    /* based on CrashReportClass of teensy cores library (https://github.com/PaulStoffregen/cores) */
    struct arm_fault_info_struct* info = (struct arm_fault_info_struct*) 0x2027FF80;
    info->ipsr = addr;
    info->cfsr = SCB_CFSR;
    info->hfsr = SCB_HFSR;
    info->mmfar = SCB_MMFAR;
    info->bfar = SCB_BFAR;
    info->ret = hardfault_args[6];
    info->xpsr = hardfault_args[7];
    info->temp = tempmonGetTemp();
    info->time = rtc_get();
    info->len = sizeof(*info) / 4;
    // add CRC to crash report
    uint32_t crc { 0xFFFFFFFF };
    auto ptr { reinterpret_cast<const uint32_t*>(info) };
    auto p_end { ptr + (sizeof(*info) / 4 - 1) };
    while (ptr < p_end) {
        crc ^= *ptr++;
        for (size_t i {}; i < 32; ++i) {
            crc = (crc >> 1) ^ (crc & 1) * 0xEDB88320;
        }
    }
    info->crc = crc;
    arm_dcache_flush_delete(info, sizeof(*info));

    EXC_PRINTF(PSTR("Fault IRQ:    0x%x\r\n"), addr & 0x1ff);
    EXC_PRINTF(PSTR(" sp:          0x%x\r\n"), sp);
    EXC_PRINTF(PSTR(" lr:          0x%x\r\n"), lr);
    EXC_PRINTF(PSTR(" stacked_r0:  0x%x\r\n"), hardfault_args[0]);
    EXC_PRINTF(PSTR(" stacked_r1:  0x%x\r\n"), hardfault_args[1]);
    EXC_PRINTF(PSTR(" stacked_r2:  0x%x\r\n"), hardfault_args[2]);
    EXC_PRINTF(PSTR(" stacked_r3:  0x%x\r\n"), hardfault_args[3]);
    EXC_PRINTF(PSTR(" stacked_r12: 0x%x\r\n"), hardfault_args[4]);
    EXC_PRINTF(PSTR(" stacked_lr:  0x%x\r\n"), hardfault_args[5]);
    EXC_PRINTF(PSTR(" stacked_pc:  0x%x\r\n"), hardfault_args[6]);
    EXC_PRINTF(PSTR(" stacked_psr: 0x%x\r\n"), hardfault_args[7]);

    const auto _CFSR = *reinterpret_cast<volatile unsigned int*>(0xE000ED28);
    EXC_PRINTF(PSTR(" _CFSR:       0x%x\r\n"), _CFSR);

    if (_CFSR > 0) {
        /* Memory Management Faults */
        if ((_CFSR & 1) == 1) {
            EXC_PRINTF(PSTR("  (IACCVIOL)    Instruction Access Violation\r\n"));
        } else if (((_CFSR & (0x02)) >> 1) == 1) {
            EXC_PRINTF(PSTR("  (DACCVIOL)    Data Access Violation\r\n"));
        } else if (((_CFSR & (0x08)) >> 3) == 1) {
            EXC_PRINTF(PSTR("  (MUNSTKERR)   MemMange Fault on Unstacking\r\n"));
        } else if (((_CFSR & (0x10)) >> 4) == 1) {
            EXC_PRINTF(PSTR("  (MSTKERR)     MemMange Fault on stacking\r\n"));
        } else if (((_CFSR & (0x20)) >> 5) == 1) {
            EXC_PRINTF(PSTR("  (MLSPERR)     MemMange Fault on FP Lazy State\r\n"));
        }
        if (((_CFSR & (0x80)) >> 7) == 1) {
            EXC_PRINTF(PSTR("  (MMARVALID)   MemMange Fault Address Valid\r\n"));
        }
        /* Bus Fault Status Register */
        if (((_CFSR & 0x100) >> 8) == 1) {
            EXC_PRINTF(PSTR("  (IBUSERR)     Instruction Bus Error\r\n"));
        } else if (((_CFSR & (0x200)) >> 9) == 1) {
            EXC_PRINTF(PSTR("  (PRECISERR)   Data bus error (address in BFAR)\r\n"));
        } else if (((_CFSR & (0x400)) >> 10) == 1) {
            EXC_PRINTF(PSTR("  (IMPRECISERR) Data bus error but address not related to instruction\r\n"));
        } else if (((_CFSR & (0x800)) >> 11) == 1) {
            EXC_PRINTF(PSTR("  (UNSTKERR)    Bus Fault on unstacking for a return from exception\r\n"));
        } else if (((_CFSR & (0x1000)) >> 12) == 1) {
            EXC_PRINTF(PSTR("  (STKERR)      Bus Fault on stacking for exception entry\r\n"));
        } else if (((_CFSR & (0x2000)) >> 13) == 1) {
            EXC_PRINTF(PSTR("  (LSPERR)      Bus Fault on FP lazy state preservation\r\n"));
        }
        if (((_CFSR & (0x8000)) >> 15) == 1) {
            EXC_PRINTF(PSTR("  (BFARVALID)   Bus Fault Address Valid\r\n"));
        }
        /* Usuage Fault Status Register */
        if (((_CFSR & 0x10000) >> 16) == 1) {
            EXC_PRINTF(PSTR("  (UNDEFINSTR)  Undefined instruction\r\n"));
        } else if (((_CFSR & (0x20000)) >> 17) == 1) {
            EXC_PRINTF(PSTR("  (INVSTATE)    Instruction makes illegal use of EPSR\r\n"));
        } else if (((_CFSR & (0x40000)) >> 18) == 1) {
            EXC_PRINTF(PSTR("  (INVPC)       Usage fault: invalid EXC_RETURN\r\n"));
        } else if (((_CFSR & (0x80000)) >> 19) == 1) {
            EXC_PRINTF(PSTR("  (NOCP)        No Coprocessor\r\n"));
        } else if (((_CFSR & (0x1000000)) >> 24) == 1) {
            EXC_PRINTF(PSTR("  (UNALIGNED)   Unaligned access UsageFault\r\n"));
        } else if (((_CFSR & (0x2000000)) >> 25) == 1) {
            EXC_PRINTF(PSTR("  (DIVBYZERO)   Divide by zero\r\n"));
        }
    }

    const auto _HFSR = *reinterpret_cast<volatile unsigned int*>(0xE000ED2C);
    EXC_PRINTF(PSTR(" _HFSR:       0x%x\r\n"), _HFSR);
    if (_HFSR > 0) {
        /* Memory Management Faults */
        if (((_HFSR & (0x02)) >> 1) == 1) {
            EXC_PRINTF(PSTR("  (VECTTBL)     Bus Fault on Vec Table Read\r\n"));
        } else if (((_HFSR & (0x40000000)) >> 30) == 1) {
            EXC_PRINTF(PSTR("  (FORCED)      Forced Hard Fault\r\n"));
        } else if (((_HFSR & (0x80000000)) >> 31) == 31) {
            EXC_PRINTF(PSTR("  (DEBUGEVT)    Reserved for Debug\r\n"));
        }
    }

    EXC_PRINTF(PSTR(" _DFSR:       0x%x\r\n"), *reinterpret_cast<volatile unsigned int*>(0xE000ED30));
    EXC_PRINTF(PSTR(" _AFSR:       0x%x\r\n"), *reinterpret_cast<volatile unsigned int*>(0xE000ED3C));
    EXC_PRINTF(PSTR(" _BFAR:       0x%x\r\n"), *reinterpret_cast<volatile unsigned int*>(0xE000ED38));
    EXC_PRINTF(PSTR(" _MMAR:       0x%x\r\n"), *reinterpret_cast<volatile unsigned int*>(0xE000ED34));

    if (::xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto p_active_task { pxGetTaskFromStack(reinterpret_cast<StackType_t*>(sp)) };
        EXC_PRINTF(PSTR("\nActive task (TCB): %s\r\n"), pcTaskGetName(nullptr));
        EXC_PRINTF(PSTR("Active task (stack): %s\r\n"), p_active_task ? pcTaskGetName(p_active_task) : PSTR("unknown"));
    }

    EXC_PRINTF(PSTR("\r\nStack trace:\r\n"));
    phase2_vrs pre_signal_state = {};
    pre_signal_state.demand_save_flags = 0;
    pre_signal_state.core.r[0] = hardfault_args[0];
    pre_signal_state.core.r[1] = hardfault_args[1];
    pre_signal_state.core.r[2] = hardfault_args[2];
    pre_signal_state.core.r[3] = hardfault_args[3];
    pre_signal_state.core.r[12] = hardfault_args[4];
    pre_signal_state.core.r[11] = 0;
    pre_signal_state.core.r[14] = hardfault_args[6];
    pre_signal_state.core.r[15] = 0;
    pre_signal_state.core.r[13] = sp + 8 * sizeof(unsigned int);
    g_trace_lr = hardfault_args[5];

    int depth {};
    __gnu_Unwind_Backtrace(trace_fcn, &depth, &pre_signal_state);
    EXC_PRINTF(PSTR("\n"));
    EXC_FLUSH();

    hardfault_args[6] = reinterpret_cast<uintptr_t>(&mcu_hardfault);
    portDATA_SYNC_BARRIER();
    portINSTR_SYNC_BARRIER();
    portDISABLE_INTERRUPTS();
    NVIC_SET_PRIORITY(IRQ_USB1, (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << (8 - configPRIO_BITS));

    static_assert((configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) > 0, "invalid interrupt priority config");
}
} // extern C

namespace freertos {
FLASHMEM void print_stack_trace(TaskHandle_t task) {
    StaticTask_t* p_tcb { reinterpret_cast<StaticTask_t*>(task) };
    StackType_t** p_stack_args { reinterpret_cast<StackType_t**>(task) };
    configASSERT(p_stack_args);

    StackType_t* stack_args { *p_stack_args };
    configASSERT(stack_args);

    EXC_PRINTF(PSTR("\tstack_args=0x%x\r\n"), stack_args);
    EXC_PRINTF(PSTR("\tpxTopOfStack=0x%x\r\n"), p_tcb->pxDummy1);
    EXC_PRINTF(PSTR("\tpxStack=0x%x\r\n"), p_tcb->pxDummy6);
    EXC_PRINTF(PSTR("\tpxEndOfStack=0x%x\r\n"), p_tcb->pxDummy8);

    const size_t sp_idx { stack_args[8] & 0x10 ? 9U : 25U };

    EXC_PRINTF(PSTR("\r\nStack trace:\r\n"));
    phase2_vrs pre_signal_state = {};
    pre_signal_state.demand_save_flags = (stack_args[8] & 0x10) == 0 ? 0 : ~0;
    pre_signal_state.core.r[4] = stack_args[0];
    pre_signal_state.core.r[5] = stack_args[1];
    pre_signal_state.core.r[6] = stack_args[2];
    pre_signal_state.core.r[7] = stack_args[3];
    pre_signal_state.core.r[8] = stack_args[4];
    pre_signal_state.core.r[9] = stack_args[5];
    pre_signal_state.core.r[10] = stack_args[6];
    pre_signal_state.core.r[11] = stack_args[7];
    pre_signal_state.core.r[14] = stack_args[8];

    pre_signal_state.core.r[0] = stack_args[sp_idx];
    pre_signal_state.core.r[1] = stack_args[sp_idx + 1];
    pre_signal_state.core.r[2] = stack_args[sp_idx + 2];
    pre_signal_state.core.r[3] = stack_args[sp_idx + 3];
    pre_signal_state.core.r[12] = stack_args[sp_idx + 4];
    pre_signal_state.core.r[14] = stack_args[sp_idx + 6];
    pre_signal_state.core.r[15] = 0;
    pre_signal_state.core.r[13] = reinterpret_cast<uintptr_t>(&stack_args[sp_idx + 8]);
    if (stack_args[sp_idx + 7] & 0x200) {
        pre_signal_state.core.r[13] += sizeof(StackType_t); // adjust stack pointer alignment
    }
    if ((stack_args[8] & 0x10) == 0) {
        pre_signal_state.core.r[13] += 18U * sizeof(StackType_t); // extended stack frame
    }
    g_trace_lr = stack_args[sp_idx + 5];

    int depth {};
    __gnu_Unwind_Backtrace(trace_fcn, &depth, &pre_signal_state);
    EXC_PRINTF(PSTR("\n"));
    EXC_FLUSH();
}
} // namespace freertos
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
