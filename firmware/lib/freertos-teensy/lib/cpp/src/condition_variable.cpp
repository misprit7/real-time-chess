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

#if __GNUC__ < 11
#include "arduino_freertos.h"
#include "semphr.h"

#if _GCC_VERSION >= 60100
#include <condition_variable>

namespace std {

condition_variable::condition_variable() = default;

condition_variable::~condition_variable() { // It is only safe to invoke the destructor if all threads have been notified.
    // if (!_M_cond.empty()) {
    //     std::__throw_system_error(117); // POSIX error: structure needs cleaning
    // }
    configASSERT(_M_cond.empty());
}

void condition_variable::wait(std::unique_lock<std::mutex>& m) { // pre-condition: m is taken!!
    _M_cond.lock();
    _M_cond.push(__gthread_t::native_task_handle());
    _M_cond.unlock();

    m.unlock();

    ::ulTaskNotifyTakeIndexed(configTASK_NOTIFICATION_ARRAY_ENTRIES - 1, pdTRUE, portMAX_DELAY);

    m.lock();
}

void condition_variable::notify_one() {
    _M_cond.lock();
    if (!_M_cond.empty()) {
        auto t = _M_cond.front();
        _M_cond.pop();
        ::xTaskNotifyGiveIndexed(t, configTASK_NOTIFICATION_ARRAY_ENTRIES - 1);
    }
    _M_cond.unlock();
}

void condition_variable::notify_all() {
    _M_cond.lock();
    while (!_M_cond.empty()) {
        auto t = _M_cond.front();
        _M_cond.pop();
        ::xTaskNotifyGiveIndexed(t, configTASK_NOTIFICATION_ARRAY_ENTRIES - 1);
    }
    _M_cond.unlock();
}

} // namespace std
#endif // _GCC_VERSION >= 60100
#endif // __GNUC__ < 11
