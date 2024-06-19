/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020 Timo Sandmann
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
 * @file    main.cpp
 * @brief   FreeRTOS example for Teensy boards
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

#include "arduino_freertos.h"
#if _GCC_VERSION < 60100
#error "Compiler too old for std::thread support with FreeRTOS."
#endif

#include <thread>
#include <future>
#include <chrono>
#include <string>
#include <ctime>


using namespace std::chrono_literals;

static bool print_time() {
    std::string tmp (32, '\0');
    struct timeval tv;
    ::gettimeofday(&tv, nullptr);
    std::time_t t { tv.tv_sec };
    struct tm* info { std::localtime(&t) };

    if (std::strftime(tmp.data(), tmp.size(), PSTR("%c UTC"), info) > 0) {
        ::Serial.println(tmp.c_str());
        return true;
    }

    return false;
}

static void task1(void*) {
    while (true) {
        ::digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
        ::vTaskDelay(pdMS_TO_TICKS(250));

        ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
        ::vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void task2(void*) {
    std::thread t1 { []() {
        ::vTaskPrioritySet(nullptr, 3);

        while (true) {
            ::Serial.println(PSTR("TICK"));
            std::this_thread::sleep_for(500ms);

            ::Serial.print(PSTR("TOCK\tnow: "));
            print_time();
            std::this_thread::sleep_for(500ms);
        }
    } };

    ::vTaskSuspend(nullptr);
}

static void task3(void*) {
    ::Serial.println("task3:");
    ::Serial.flush();

    std::this_thread::sleep_for(5s);

    ::Serial.println(PSTR("task3: creating futures..."));
    ::Serial.flush();

    std::future<int32_t> result0 { std::async([]() -> int32_t { return 2; }) };
    std::future<int32_t> result1 { std::async(std::launch::async, []() -> int32_t { return 3; }) };
    std::future<int32_t> result2 { std::async(std::launch::deferred, []() -> int32_t { return 5; }) };

    int32_t r { result0.get() + result1.get() + result2.get() };
    ::Serial.printf(PSTR("r=%d\n\r"), r);
    configASSERT(2 + 3 + 5 == r);


    // future from a packaged_task
    std::packaged_task<int()> task([] { return 7; }); // wrap the function
    std::future<int> f1 = task.get_future(); // get a future
    std::thread t2(std::move(task)); // launch on a thread

    // future from an async()
    std::future<int> f2 = std::async(std::launch::async, [] { return 8; });

    // future from a promise
    std::promise<int> p;
    std::future<int> f3 = p.get_future();
    std::thread([&p] { p.set_value_at_thread_exit(9); }).detach();

    ::Serial.println(PSTR("Waiting..."));
    ::Serial.flush();
    f1.wait();
    f2.wait();
    f3.wait();
    const auto r1 { f1.get() };
    const auto r2 { f2.get() };
    const auto r3 { f3.get() };
    ::Serial.printf(PSTR("Done!\nResults are: %d %d %d\n\r"), r1, r2, r3);
    ::Serial.flush();
    configASSERT(7 + 8 + 9 == r1 + r2 + r3);
    t2.join();

    ::vTaskSuspend(nullptr);
}

void setup() {
    ::Serial.begin(115'200);
    ::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    ::digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

    while (::millis() < 2'000) {
    }

    ::Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    ::xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
    ::xTaskCreate(task2, "task2", 8192, nullptr, configMAX_PRIORITIES - 1, nullptr);
    ::xTaskCreate(task3, "task3", 8192, nullptr, 3, nullptr);


    {
        freertos::clock::sync_rtc();
        print_time();

        ::rtc_set(::rtc_get() + 3'600);
        freertos::clock::sync_rtc();

        print_time();
    }

    ::Serial.println(PSTR("setup(): starting scheduler..."));
    ::Serial.flush();

    ::vTaskStartScheduler();
}

void loop() {}
