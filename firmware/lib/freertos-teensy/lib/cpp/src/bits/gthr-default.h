/// @file
///
/// @author: Piotr Grygorczuk grygorek@gmail.com
///
/// @copyright Copyright 2019 Piotr Grygorczuk
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// o Redistributions of source code must retain the above copyright notice,
///   this list of conditions and the following disclaimer.
///
/// o Redistributions in binary form must reproduce the above copyright notice,
///   this list of conditions and the following disclaimer in the documentation
///   and/or other materials provided with the distribution.
///
/// o My name may not be used to endorse or promote products derived from this
///   software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// adapted for use with the teensy FreeRTOS port by Timo Sandmann
///

#pragma once

#include "arduino_freertos.h"
#include "semphr.h"

#if _GCC_VERSION >= 60100 && defined PLATFORMIO
#include "thread_gthread.h"
#include "condition_variable.h"
#include "gthr_key.h"

#include <ctime>
#include <sys/time.h>


typedef free_rtos_std::gthr_freertos __gthread_t;

extern "C" {

#define _GLIBCXX_HAS_GTHREADS 1
#define __GTHREADS 1
#define __GTHREADS_CXX0X 1
#define __GTHREAD_ONCE_INIT 0
#define _GLIBCXX_USE_SCHED_YIELD
#define __GTHREAD_COND_INIT {}

typedef free_rtos_std::Key* __gthread_key_t;
typedef int __gthread_once_t;
typedef SemaphoreHandle_t __gthread_mutex_t;
typedef SemaphoreHandle_t __gthread_recursive_mutex_t;
typedef free_rtos_std::cv_task_list __gthread_cond_t;


static inline void __GTHREAD_RECURSIVE_MUTEX_INIT_FUNCTION(__gthread_recursive_mutex_t* mutex) {
    *mutex = ::xSemaphoreCreateRecursiveMutex();
}
static inline void __GTHREAD_MUTEX_INIT_FUNCTION(__gthread_mutex_t* mutex) {
    *mutex = ::xSemaphoreCreateMutex();
}

int __gthread_active_p();
int __gthread_once(__gthread_once_t*, void (*)(void));

static inline int __gthread_key_create(__gthread_key_t* keyp, void (*dtor)(void*)) {
    return free_rtos_std::freertos_gthread_key_create(keyp, dtor);
}

static inline int __gthread_key_delete(__gthread_key_t key) {
    return free_rtos_std::freertos_gthread_key_delete(key);
}

static inline void* __gthread_getspecific(__gthread_key_t key) {
    return free_rtos_std::freertos_gthread_getspecific(key);
}

static inline int __gthread_setspecific(__gthread_key_t key, const void* ptr) {
    return free_rtos_std::freertos_gthread_setspecific(key, ptr);
}


static inline int __gthread_mutex_destroy(__gthread_mutex_t* mutex) {
    ::vSemaphoreDelete(*mutex);
    return 0;
}
static inline int __gthread_recursive_mutex_destroy(__gthread_recursive_mutex_t* mutex) {
    ::vSemaphoreDelete(*mutex);
    return 0;
}

static inline int __gthread_mutex_lock(__gthread_mutex_t* mutex) {
    return (::xSemaphoreTake(*mutex, portMAX_DELAY) == pdTRUE) ? 0 : 1;
}
static inline int __gthread_mutex_trylock(__gthread_mutex_t* mutex) {
    return (::xSemaphoreTake(*mutex, 0) == pdTRUE) ? 0 : 1;
}
static inline int __gthread_mutex_unlock(__gthread_mutex_t* mutex) {
    return (::xSemaphoreGive(*mutex) == pdTRUE) ? 0 : 1;
}

static inline int __gthread_recursive_mutex_lock(__gthread_recursive_mutex_t* mutex) {
    return (::xSemaphoreTakeRecursive(*mutex, portMAX_DELAY) == pdTRUE) ? 0 : 1;
}
static inline int __gthread_recursive_mutex_trylock(__gthread_recursive_mutex_t* mutex) {
    return (::xSemaphoreTakeRecursive(*mutex, 0) == pdTRUE) ? 0 : 1;
}
static inline int __gthread_recursive_mutex_unlock(__gthread_recursive_mutex_t* mutex) {
    return (::xSemaphoreGiveRecursive(*mutex) == pdTRUE) ? 0 : 1;
}


struct __gthread_time_t {
    std::time_t sec;
    long nsec;
    int64_t milliseconds() const {
        return static_cast<int64_t>(sec) * 1'000LL + nsec / 1'000'000LL;
    }

    int64_t microseconds() const {
        return static_cast<int64_t>(sec) * 1'000'000LL + nsec / 1'000LL;
    }
};

static inline __gthread_time_t operator-(const __gthread_time_t& lhs, const timeval& rhs) {
    auto s { lhs.sec - rhs.tv_sec };
    int64_t ns { lhs.nsec - rhs.tv_usec * 1'000LL };
    if (ns < 0) {
        --s;
        ns += 1'000'000'000LL;
    } else if (ns > 1'000'000'000LL) {
        ++s;
        ns -= 1'000'000'000LL;
    }

    return __gthread_time_t { s, static_cast<long>(ns) };
}

static inline int __gthread_mutex_timedlock(__gthread_mutex_t* m, const __gthread_time_t* abs_timeout) {
    timeval now {};
    gettimeofday(&now, NULL);

    auto t_us { (*abs_timeout - now).microseconds() };
    if (t_us < 0) {
        t_us = 0;
    }
    // add 2 ticks to avoid rounding error because of tick resolution
    return (::xSemaphoreTake(*m, pdUS_TO_TICKS(t_us) + 2) == pdTRUE) ? 0 : 1;
}

static inline int __gthread_recursive_mutex_timedlock(__gthread_recursive_mutex_t* m, const __gthread_time_t* abs_timeout) {
    timeval now {};
    gettimeofday(&now, NULL);

    auto t_us { (*abs_timeout - now).microseconds() };
    if (t_us < 0) {
        t_us = 0;
    }
    // add 2 ticks to avoid rounding error because of tick resolution
    return (::xSemaphoreTakeRecursive(*m, pdUS_TO_TICKS(t_us) + 2) == pdTRUE) ? 0 : 1;
}

// All functions returning int should return zero on success or the error number.  If the operation is not supported, -1 is returned.

static inline int __gthread_create(__gthread_t* thread, void (*func)(void*), void* args) {
    return thread->create_thread(func, args) ? 0 : 1;
}
static inline int __gthread_join(__gthread_t& thread, void** value_ptr) {
    thread.join();
    return 0;
}
static inline int __gthread_detach(__gthread_t& thread) {
    thread.detach();
    return 0;
}
static inline int __gthread_equal(const __gthread_t& t1, const __gthread_t& t2) {
    return t1 == t2 ? 0 : 1;
}
static inline __gthread_t __gthread_self(void) {
    return __gthread_t::self();
}


static inline int __gthread_yield(void) {
    taskYIELD();
    return 0;
}


int __gthread_cond_timedwait(__gthread_cond_t*, __gthread_mutex_t*, const __gthread_time_t*);
int __gthread_cond_wait(__gthread_cond_t*, __gthread_mutex_t*);
int __gthread_cond_signal(__gthread_cond_t*);
int __gthread_cond_broadcast(__gthread_cond_t*);


static inline int __gthread_cond_destroy(__gthread_cond_t*) {
    return 0;
}

} // extern "C"
#elif !defined PLATFORMIO
#warning "std::thread support not available with Teensyduino."
#undef _GLIBCXX_HAS_GTHREADS
#undef __GTHREADS
typedef SemaphoreHandle_t __gthread_mutex_t;
typedef SemaphoreHandle_t __gthread_recursive_mutex_t;
#else
#warning "Compiler too old for std::thread support with FreeRTOS."
#undef _GLIBCXX_HAS_GTHREADS
#undef __GTHREADS
typedef SemaphoreHandle_t __gthread_mutex_t;
typedef SemaphoreHandle_t __gthread_recursive_mutex_t;
#endif // _GCC_VERSION
