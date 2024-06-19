C++ FreeRTOS GCC
================

# Introduction

Included library implements an interface enabling C++ multithreading
in FreeRTOS. That is:

* Creating threads - std::thread,
* Locking - std::mutex, std::condition_variable, etc.
* Time - std::chrono, std::sleep_for, etc.

Missing parts are related to 'futures':

* std::assync
* std::launch
* std::promise
* etc.

This implementation is for GNU C Compiler (GCC) only. Tested with:

* GCC 7.2 and 8.2 for ARM 32bit (cmake generates Eclipse project)
* FreeRTOS 10.1.1
* Windows 10
* NXP MCUXpresso v10.3.0 (ARM CM4 & CM7)

Although I have not tried any platforms other than ARM, I believe it should work.
The dependency is on FreeRTOS only. If FreeRTOS runs on your target then I believe
this library will too.

This library is not intended to be accessed directly from your application.
It is an interface between C++ and FreeRTOS. Your application should use the
STL directly. STL will use the provided library under the hood. Saying that,
none of the files should be included in your application's source files -
except one (`freertos_time.h` to set system time).

# Background

The C++11 standard introduced unified multithreading interface. The standard
defines the interface only. It is up to the compiler vendors how to implement it.
Multithreading requires a tasks sheduler running at low level which implicates
an operating system is present. Both, scheduler and OS are beyond of
the C++ standard definition. Obviously, it is natural that the implementations
from compiler vendors would cover most popular OS's only, like Windows or Linux.

What about embedded world, microcontrollers and limited resources
systems? Well... there is so many embedded OS's that it is
certainly impossible to provide implementation for all of them. Should OS
vendor provide an implementation for different compilers? Maybe. Unfortumately,
C++ is not popular in embedded world. Vendors focus on plain C. It is expected
that when C++ compiler is used, code will compile too. Nothing more is needed
to deliver. With the multithreading library is different. There is an additional
layer needed to interface OS with C++.

FreeRTOS is a small real time operarting system. The core library is more like
a task scheduler with few tools to synchronize access to resources. It has few
extension libraries, like TCP/IP stack and a file system. This OS is very
popular in embedded world of small microcontrollers. It is
for free and delivered as a source code. Strong points of this small RTOS are
good performance, small footprint and simple API. Although it is implemented
in C, there are many programmers that create their own API wrappers in C++.

C++ language is not popular in embedded world. I believe it is a mistake.
C++ has got everything what standard C has, plus many nice features that make
the code easier to express algorithm, is safer and fast. Working with FreeRTOS is
about managing resources. Mainly creating and releasing handles, passing correct
types as arguments, etc. I found that often, instead of focusing on an
algorithm, I am checking for memory leaks or incorrect data types. Having code
wrapped in C++ classes brings the development to a different level.

Multithreading interface in C++ is very clean and simple to use. On the negative
side it is little bit heavy under the hood. It might not be the best if an
embedded application has to create and destroy new tasks often or control stack
size and priorities. C++ interface does not provide this features.
However, if it is about starting a worker thread now and then or implementing
a dispatch queue that is snoozing somewhere in the system waiting for tasks to
be processed, this interface will do the job.
Finally, not every embedded application is a hard real time application.

So, how to make FreeRTOS working with the C++ multithreading interface?

# Hello World!

Building a project is not much different than the regular way
the project for ARM is built.  It is not needed
to understand how the library is implemented to use it. The source code must
not be accessed directly from the application. It is called by GCC
implementation itself. That is, user application uses components from std
namespace only.

As usual, FreeRTOS source and a startup code for a processor will be needed.
The following definitions should also be placed in `FreeRTOSConfig.h` file:

```C++
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1

#define pdMS_TO_TICKS( xTimeInMs ) \
     ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * \
     ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000 ) )
```

And then the following files need to be included in the project:

```
condition_variable.cpp  --> Definitions required by std::condition_variable
condition_variable.h    --> Helper class to implement std::condition_variable
critical_section.h      --> Helper class wrap FreeRTOS citical section
                            (it is for the internal use only)
freertos_time.cpp       --> Setting and reading system wall/clock time
freertos_time.h         --> Declaration
gthr-FreeRTOS.h         --> GCC Hook (thread and mutex)
thread_gthread.h        --> Helper class to integrate FreeRTOS with std::thread
thread.cpp              --> Definitions required by std::thread class
```

Simple example application can be like that:

```C++
#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>
#include <chrono>

int main()
{
  std::queue<int> q;
  std::mutex m;
  std::condition_variable cv;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::thread processor{[&]() {
    std::unique_lock<std::mutex> lock{m};

    while (1)
    {
      cv.wait(lock, [&q] { return q.size() > 0; });
      int i = q.front();
      q.pop();
      lock.unlock();

      if (i == 0)
        return;

      lock.lock();
    }
  }};

  for (int i = 100; i >= 0; i--)
  {
    m.lock();
    q.push(i);
    m.unlock();
    cv.notify_one();
  }

  processor.join();
}
```

However, this program will not build yet!

## GCC Hook

The problem is that GCC cannot see functions that implement the threading
interface using FreeRTOS. Required implementation is in `gthr-FreeRTOS.h`.

The interesting file is the `gthr.h` located in a GCC instalation directory.
This is what I have got in my ARM distribution:

```bash
./include/c++/8.2.1/arm-none-eabi/arm/v5te/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/arm/v5te/softfp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v6-m/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7+fp/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7+fp/softfp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7-m/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7e-m/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7e-m+dp/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7e-m+dp/softfp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7e-m+fp/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v7e-m+fp/softfp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.base/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.main/nofp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.main+dp/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.main+dp/softfp/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.main+fp/hard/bits/gthr.h
./include/c++/8.2.1/arm-none-eabi/thumb/v8-m.main+fp/softfp/bits/gthr.h
```

Each file is the same. Different directories are related to different ARM cores.
For example, CortexM4 should be linked with v7e-m+xx, xx - depending on floating
point configuration. The file itself has lots of code commented out. This is an
instruction for implementers. It tells which functions must be implemented to
provide multithreading in a system.

The end of the file looks like this:

```C++
...

#ifndef _GLIBCXX_GTHREAD_USE_WEAK
#define _GLIBCXX_GTHREAD_USE_WEAK 1
#endif
#endif
#include <bits/gthr-default.h>

#ifndef _GLIBCXX_HIDE_EXPORTS
#pragma GCC visibility pop
#endif

...
```

The file includes a default implementation from `gthr-default.h`. What would be
a default implementation for a system without a system? Yes, empty functions.
So, how to replace the default implementation with the one from the library?

Unfortunately, the only way I have found to hook to a compiler is to modify
the file `gthr.h` in the compiler's distribution itself. It sounds little bit
scary, doesn't it?

The easiest and the least intrusive way is to replace the single include
with the code:

```C++
#ifdef _GCC_MULTITHREAD_FREERTOS_ENABLE
#include "gthr-FreeRTOS.h"
#else
#include <bits/gthr-default.h>
#endif
```

Then `_GCC_MULTITHREAD_FREERTOS_ENABLE` must be defined during the application
project build. Otherwise, the default header will be included. Might be handy
when the multithread features of C++ are not required.

Probably the easiest way is to edit all of the files. If you know the
architecture of your processor, you can change only the related one.

Now, project should build.

# Library Implementation

## Mutex

Implementation of mutex is probably the simplest one because FreeRTOS API
contains all the functions that almost directly translate to the GCC interface.
Full implementation is in `gthr-FreeRTOS.h`. Here is just a sample:

```C++
typedef xSemaphoreHandle __gthread_mutex_t;

static inline void __GTHREAD_MUTEX_INIT_FUNCTION(__gthread_mutex_t *mutex){
  *mutex = xSemaphoreCreateMutex(); }

static inline int __gthread_mutex_destroy(__gthread_mutex_t *mutex){
  vSemaphoreDelete(*mutex);  return 0; }

static inline int __gthread_mutex_lock(__gthread_mutex_t *mutex){
  return (xSemaphoreTake(*mutex, portMAX_DELAY) == pdTRUE) ? 0 : 1; }

static inline int __gthread_mutex_unlock(__gthread_mutex_t *mutex){
  return (xSemaphoreGive(*mutex) == pdTRUE) ? 0 : 1; }
```

Once these functions are defined it is possible to use all different variants
of mutex from the std namespace (e.g. unique_mutex, lock_guard,
etc.). Except timed_mutex. This one requires access to system time which will
be described later in this article.

## Condition Variable

It is little bit tricky to implement a condition variable with FreeRTOS
to match the `std` interface. First of all it is good to understand what a
condition variable is and how it is (or should be) implemented in a system.
Good article is
[here](https://www.microsoft.com/en-us/research/wp-content/uploads/2004/12/ImplementingCVs.pdf).

Without going into much detail, implementation is a collection of threads waiting for
a condition that would let them to exit that waiting state. It is a form of
an event - a thread is waiting for an event, a module sends a notification
and the thread wakes up. The interface provides a function to notify just one
thread or all of them.

FreeRTOS has few different ways of suspending and resuming a task (thread).
The `Event Groups` looks promising. It maintains a list of waiting threads
and wakes them all when an event has been notified. However, this interface
seems does not provide a way to wake up a single task. Another one
is `Direct To Task Notifications`. This one, on the other hand, requires an
implementation handling a list of threads. This method is less efficient but
at least is possible to meet the `std::condition_variable` interface.

Have a closer look at the `std::condition_variable` class.
The implementation is in `condition_variable` header. The snippet below
is not a full class. Just an interesting part of it:

```C++
  /// condition_variable
  class condition_variable
  {
    typedef __gthread_cond_t native_type;

    __native_type M_cond;

  ...
  public:

    condition_variable() noexcept;
    ~condition_variable() noexcept;

    void
    notify_one() noexcept;

    void
    notify_all() noexcept;

    void
    wait(unique_lock<mutex>& __lock) noexcept;

    template<typename _Predicate>
    void
    wait(unique_lock<mutex>& __lock, _Predicate __p)
    {
      while (!__p())
      wait(__lock);
    }
  ...
  };
```

Class has a single member variable `_M_cond`, which is a handle to a native OS
interface - FreeRTOS interface in this case. There are also few member functions
that have to be implemented by an external library.
That is a back door to provide operations on the native handle. The `wait` with
a predicate is implemented. Just shown here because it will be needed later to explain
one detail.

Two things are needed. A queue of waiting tasks and a semaphore to synchronise
the access to that queue. Both have to be stored in a single handle inside of
the `condition_variable` class.

The single handle is implemented as `free_rtos_std::cv_task_list` class in the
`condition_variable.h` file of this library. It is a wrapper to `std::queue`
and a FreeRTOS semaphore (the `semaphore` class is in the same file).

```C++
class cv_task_list
{
public:
  using __gthread_t = free_rtos_std::gthr_freertos;
  using thrd_type = __gthread_t::native_task_type;
  using queue_type = std::queue<thrd_type>;

  cv_task_list() = default;

  void push(thrd_type thrd) { _que.push(thrd); }
  void pop() { _que.pop(); }
  bool empty() const { return _que.empty(); }

  ~cv_task_list()
  {
    lock();
    _que = queue_type{};
    unlock();
  }

  // no copy and no move
  cv_task_list &operator=(const cv_task_list &r) = delete;
  cv_task_list &operator=(cv_task_list &&r) = delete;
  cv_task_list(cv_task_list &&) = delete;
  cv_task_list(const cv_task_list &) = delete;

  thrd_type &front() { return _que.front(); }
  const thrd_type &front() const { return _que.front(); }
  thrd_type &back() { return _que.back(); }
  const thrd_type &back() const { return _que.back(); }

  void lock() { _sem.lock(); }
  void unlock() { _sem.unlock(); }

private:
  queue_type _que;
  semaphore _sem;
};
```

Once this class is defined the native handler needs to be defined too.
It is done in `gthr-FreeRTOS.h`, together with mutexes.

```C++
typedef free_rtos_std::cv_task_list __gthread_cond_t;
```

Now, class `std::condition_variable` can see the `cv_task_list` class as a native
handler. Great! Time for the missing functions.

The implementation is in `condition_variable.cpp` file. The functions are:

* constructor
* destructor
* notify_all
* notify_one
* and wait

Constructor has nothing to do so, keep a default implementation. Destructor
will throw a system error if there are waiting threads. The cppreference website
says that it is not safe to call a destructor in a such case. For that reason
the condition is tested.

```C++
namespace std{

condition_variable::condition_variable() = default;

condition_variable::~condition_variable()
{ // It is only safe to invoke the destructor if all threads have been notified.
  if (!_M_cond.empty())
    std::__throw_system_error(117); // POSIX error: structure needs cleaning
}
...
}
```

The `wait` function is the one which keeps the secret of a condition
variable (snippet below). It saves a handle of the current thread to the queue while the
both mutexes are taken! The first one is taken outside the `wait` call and
is protecting the condition (have a look at implementation of
`condition_variable::wait` with a predicate). This is important - this is a
contract that guarantees that only one thread is checking the condition at one
time. The second mutex protects the threads' queue. It makes sure that
a different thread that calls notify_one/all does not modify the queue at the
same time.

Once the thread's handle has been pushed to the queue, the thread is ready to
suspend. Suspend might block the execution so, the two mutexes must be unlocked
and give a chance for other threads to execute.
The `ulTaskNotifyTake` is a FreeRTOS function that will switch a task to a
waiting state until the `xTaskNotifyGive` function is called.
It is worth making a comment that when the second unlock returns,
context can be switched. It is possible that a different thread calls
notify_one/all in that time. In that case the task that has been pushed to the
queue will be removed from that queue before even starting being suspended.
This is correct behaviour. Accordingly to FreeRTOS documentation a call to
`ulTaskNotifyTake` will not suspend the task in that case.

Regardles whether the task got suspended or not, when `ulTaskNotifyTake` returns,
it means that the `xTaskNotifyGive` has been called at least once. That means
the condition must be tested again and that means the mutex protecting
the condition must be taken again. However, it could be that some other thread
got access to the condition in the meantime. So, the immediate lock can lock
the thread again.

The remaining two functions `notify_all` and `notify_one` are almost the same.
Both lock the access to the queue, remove a task from the queue and wake that
task. Difference is that `notify_one` wakes only one task and the `notify_all`
wakes all of them in a loop.

```C++
namespace std{

...

void condition_variable::wait(std::unique_lock<std::mutex> &m)
{ // pre-condition: m is taken!!
  _M_cond.lock();
  _M_cond.push(__gthread_t::native_task_handle());
  _M_cond.unlock();

  m.unlock();
  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  m.lock();
}

void condition_variable::notify_one()
{
  _M_cond.lock();
  if (!_M_cond.empty())
  {
    auto t = _M_cond.front();
    _M_cond.pop();
    xTaskNotifyGive(t);
  }
  _M_cond.unlock();
}

void condition_variable::notify_all()
{
  _M_cond.lock();
  while (!_M_cond.empty())
  {
    auto t = _M_cond.front();
    _M_cond.pop();
    xTaskNotifyGive(t);
  }
  _M_cond.unlock();
}

} // namespace std

```

## Thread

C++11 standard defines threading interface as in a snippet bellow. Important
part to notice is that `id` is defined as part of the `thread` class.

```C++
namespace std {
  class thread;

  ...

  namespace this_thread {

    thread::id get_id() noexcept;
    void yield() noexcept;
    template <class Clock, class Duration>
        void sleep_until(const chrono::time_point<Clock, Duration>& abs_time);
    template <class Rep, class Period>
        void sleep_for(const chrono::duration<Rep, Period>& rel_time);

  }
}
```

[Source: cppreference.com](https://en.cppreference.com/w/cpp/header/thread)

Now, have a look at `<thread>` header file in your GCC. The file is quiet long
so, the snippet has only important parts.

```C++
class thread
{
public:
  // Abstract base class for types that wrap arbitrary functors to be
  // invoked in the new thread of execution.
  struct _State
  {
    virtual ~_State();
    virtual void _M_run() = 0;
  };
  using _State_ptr = unique_ptr<_State>;

  typedef __gthread_t native_handle_type;

  /// thread::id
  class id
  {
    native_handle_type _M_thread;

    ...
  };

  void
  join();

  void
  detach();

  // Returns a value that hints at the number of hardware thread contexts.
  static unsigned int
  hardware_concurrency() noexcept;

private:
  id _M_id;
  ...
  void
    _M_start_thread(_State_ptr, void (*)());
  ...
};
```

The `_State` class is used for passing a user
thread function. The `native_handle_type` is an underlying thread data holder
type. The code in my library must define exactly this type to hook to the GCC
implementation. Easy to notice, this is the same approach as in the condition
variable interface. The `id` is the place where the thread's handle is kept
(`_M_thread`). And at last few functions of which definitions are missing:

* `thread::_State::~_State()`
* `thread::hardware_concurency`
* `thread::join`
* `thread::detach`
* `thread::_M_start_thread`

Implementation is in `thread.cpp`. There is nothing special to do for the first
two so:

```C++
namespace std{

  thread::_State::~_State() = default;

  // Returns the number of concurrent threads supported by the implementation.
  // The value should be considered only a hint.
  //
  // Return value
  //    Number of concurrent threads supported. If the value is not well defined
  //    or not computable, returns ​0​.
  unsigned int thread::hardware_concurrency() noexcept
  {
    return 0; // not computable
  }

}
```

Remember the `gthr-FreeRTOS.h` file? The same one where the mutex interface
is implemented? This file has number of function definitions to support threads.
They can be used now to implement missing definitions of `std::thread`
class. Task's function is visible for the first time
(`__execute_native_thread_routine`). This is an internal thread function.
User thread function is called inside. Definition will be described little
bit later.

Have a closer look at the `_M_start_thread`. Interesting here is the `state`
argument. `_State_ptr` is a `unique_pointer<T>` and is intended to keep user's
thread function. The raw pointer kept inside the unique_pointer is passed to
the native thread function. This is important. It means the ownership
is passed. Now, the native thread function is responsible for releasing it.
For that reason, the thread function must execute! The `join` would block by
definition. The `detach` must wait for the thread to start.

```C++
namespace std{

  void thread::_M_start_thread(_State_ptr state, void (*)())
  {
    const int err = __gthread_create(
        &_M_id._M_thread, __execute_native_thread_routine, state.get());

    if (err)
      __throw_system_error(err);

    state.release();
  }
```

Both, `join` and `detach` are simple. One note on comparing threads. In the
typical implementation of these two functions, `_M_id`'s (thread::id type) are
compared directly. However, the overloaded compare operator makes copies of its
arguments. That is OK if the thread handle is just a pointer. Not so good if
the handle is a class with few members. So, to optimise it the threads are
compared directly instead. Less copies and assembler looks better as well.

```C++
  void thread::join()
  {
    id invalid;
    if (_M_id._M_thread != invalid._M_thread)
      __gthread_join(_M_id._M_thread, nullptr);
    else
      __throw_system_error(EINVAL);

    // destroy the handle explicitly - next call to join/detach will throw
    _M_id = std::move(invalid);
  }

  void thread::detach()
  {
    id invalid;
    if (_M_id._M_thread != invalid._M_thread)
      __gthread_detach(_M_id._M_thread);
    else
      __throw_system_error(EINVAL);

    // destroy the handle explicitly - next call to join/detach will throw
    _M_id = std::move(invalid);
  }

```

So, how is FreeRTOS attached to the thread handle?
Two features of the rtos are needed - a rtos task handle itself and an event
group handle. The join function must block until the thread function executes.
The events group matches this requirement perfectly. As in the case
of the condition variable, both handles must be stored in one generic handle.

## Thread function vs std::thread instance

Everything would be beautiful if not the detach function. There is an issue that
must be solved. The generic handle keeps both handles. Just for the clarity of
this explanation forget about the generic one for the moment.
So, there are two handles - a thread's handle and an event's handle.

The resources are allocated when a new thread starts. When should the resources
be released? If the std::thread instance exists as long as the thread executes
then the destructor should be the right place. However, due to the detach function
the thread execution can outlive the std::thread instance. Should the handles
be released in the thread function itself? Then what if the thread function finishes first?
The join function must have access to the event handle. The handle must exist.
Although, it should be possible to check if the handle is valid. I tried to go
that way and I run into a race condition - who gets first - the thread function
destroys the handle or join gets the handle. Because join must block on that
handle then synchronisation becomes a challange. I think simpler solution exists.

The solution is that the two handles have different living time. The ugly part
is that, two handles must be kept in the same generic handle. The rtos task
handle is released at the end of the thread function. The events handle
is released at the end of join/detach call. Here:

```C++
_M_id = std::move(invalid);
```

The native thread function is here:

```C++
namespace std{

  static void __execute_native_thread_routine(void *__p)
  {
    __gthread_t local{*static_cast<__gthread_t *>(__p)}; //copy

    { // we own the arg now; it must be deleted after run() returns
      thread::_State_ptr __t{static_cast<thread::_State *>(local.arg())};
      local.notify_started(); // copy has been made; tell we are running
      __t->_M_run();
    }

    local.notify_joined(); // finished; release joined threads
  }

}
```

The handle is passed as a void pointer so, casting is needed and at the same
time a copy is made. Also, the state is put back into the unique_pointer `__t`.
Now, it is time to notify that the thread has started execution and then
call the user's task. When the user's task function returns, the state
will be deleted (by the scope) and that means the thread has finished its
function. Notify the joined thread and that is it.

## Native Handle Implementation

The native thread handle is defined as `__gthread`. The definition comes
from the  `gthr-FreeRTOS.h` file:

```C++
typedef free_rtos_std::gthr_freertos __gthread_t;
```

The `gthr_freertos` class is the generic handle, the one that holds both handles
inside. The rtos task and the event handle. The class is defined
in `thrad_gthread.h` file and included in `gthr-FreeRTOS.h`.

```C++
class gthr_freertos
{
  friend std::thread;

  enum
  {
    eEvStoragePos = 0,
    eStartedEv = 1 << 22,
    eJoinEv = 1 << 23
  };

public:
  typedef void (*task_foo)(void *);
  typedef TaskHandle_t native_task_type;

  gthr_freertos(const gthr_freertos &r);
  ~gthr_freertos() = default;

  bool create_thread(task_foo foo, void *arg);

  void join();
  void detach();

  void notify_started();
  void notify_joined();

  static gthr_freertos self();
  static native_task_type native_task_handle();

  bool operator==(const gthr_freertos &r) const;
  bool operator!=(const gthr_freertos &r) const;
  bool operator<(const gthr_freertos &r) const;

  void *arg();

private:
  gthr_freertos() = default;
  gthr_freertos(native_task_type thnd, EventGroupHandle_t ehnd);
  gthr_freertos(gthr_freertos &&r) = delete;
  gthr_freertos &operator=(const gthr_freertos &r) = delete;

  gthr_freertos &operator=(gthr_freertos &&r);

  void move(gthr_freertos &&r);

  void wait_for_start();

  native_task_type _taskHandle{nullptr};
  EventGroupHandle_t _evHandle{nullptr};
  void *_arg{nullptr};
  bool _fOwner{false};
};
```

There is no point describing all of the functions. Below is a description,
in my opinion, the most important ones.

### Critical Section

Critical section is used in `gthr_freertos` class functions. This simple
implementation is in reality disabling and enabling interrupts. If this is
not acceptable in your application, implementation of this class should
be changed.

```C++
namespace free_rtos_std
{
struct critical_section
{
  critical_section() { taskENTER_CRITICAL(); }
  ~critical_section() { taskEXIT_CRITICAL(); }
};
}
```

Class is defined in `critical_section.h` file.

### Creating Thread

Creating the FreeRTOS task requires allocating two handles. They are not created
in a constructor but in create_thread function. Program will
terminate if there is no resources. Alternatively, the function
could return false instead. By default only 256 words will be allocated
for the stack. This would be 1kB on ARM. Standard C++ interface does not let
define the stack size. So, if application requires more then this code has to
be modified. Note, that the change will apply to all threads. Critical
section disables interrupts. As described earlier, the native thread function
will delete thread's handle when finished. So here, critical section makes sure
the thread does not start before the event's handle is stored in the thread's
local storage.

```C++
bool gthr_freertos::create_thread(task_foo foo, void *arg)
{
  _arg = arg;

  _evHandle = xEventGroupCreate();
  if (!_evHandle)
    std::terminate();

  {
    critical_section critical;

    xTaskCreate(foo, "Task", 256, this, tskIDLE_PRIORITY + 1, &_taskHandle);
    if (!_taskHandle)
      std::terminate();

    vTaskSetThreadLocalStoragePointer(_taskHandle, eEvStoragePos, _evHandle);
    _fOwner = true;
  }

  return true;
}
```

### Join

Join waits for events to be notified by the native thread function. The 'while'
loop makes sure it is not a spurious event. There is no need to synchronise
anything. The thread function will not release the event handle even if the
thread has finished execution.

```C++
void gthr_freertos::join()
{
  while (0 == xEventGroupWaitBits(_evHandle,
                                  eJoinEv | eStartedEv,
                                  pdFALSE,
                                  pdTRUE,
                                  portMAX_DELAY))
    ;
}
```

### Detach

Detaching will remove the event handle. This can be done only if the thread has
started execution. Functions `std::detach` or `std::~thread` will destroy the handle.
Native thread function must make a copy of this instance first to preserve
the state pointer stored in `_arg`.

Event handle is stored in the task's local storage. It must be set to an invalid
handle now. Critical section is used to make sure that the task is not deleted
while accessing the storage. However, task could not exist already.
It must be tested if it is the case so.

```C++
void gthr_freertos::detach()
{
  wait_for_start();

  {
    critical_section critical;

    if (eDeleted != eTaskGetState(_taskHandle))
    {
      vTaskSetThreadLocalStoragePointer(_taskHandle, eEvStoragePos, nullptr);
      vEventGroupDelete(_evHandle);
      _fOwner = false;
    }
  }
}
```

### Sending Notifications

Both notifications are sent from the native thread functions. First one is to
tell that the thread has started and all necessary copies have been made.
Second notification is to tell that the user's thread function has finished
and two threads can be joined now.

There is not much to do for start notification. Just setting a bit in the event
group.

To notify the joining thread is more difficult. There is a possibility that
the thread has been detached and no one is waiting to join. That means the
event handle is deleted. The event handle in 'this' instance is just a copy
and can point to a released memory. In this case, valid information is stored
in the local storage. If the handle is invalid, this indicates the thread
has been detached and it is safe to exit without sending a notification.

Finally, the task can be deleted. FreeRTOS allows to pass nullptr as an argument
to remove 'this' task. Because the task is deleted, function will not return.
For that reason, the critical section is in its own scope - it must be destroyed
before deleting the task. From that moment any task handle, in any copies
is invalid. It can be tested using FreeRTOS API `eTaskGetState` function.

```C++
void gthr_freertos::notify_started()
{
  xEventGroupSetBits(_evHandle, eStartedEv);
}

void notify_joined()
{
  {
    critical_section critical;

    auto evHnd = static_cast<EventGroupHandle_t>(
        pvTaskGetThreadLocalStoragePointer(_taskHandle, eEvStoragePos));

    if (evHnd)
      xEventGroupSetBits(evHnd, eJoinEv);
  }

  // vTaskDelete does not return
  vTaskDelete(nullptr);
}
```

### Moving Ownership

`std::thread` is passing the handle between functions quiet few times. Frequent
copies are made. The ownership is passed together with an ownership flag.
The code makes sure that the handles are destroyed only if the class is
the owner. `gthr_freertos` class has a default destructor that does
not touch the handles. The handles are destroyed in the move operator.
It happens in the last line of the join/detach functions.

```C++
gthr_freertos::gthr_freertos(const gthr_freertos &r)
{
  critical_section critical;

  _taskHandle = r._taskHandle;
  _evHandle = r._evHandle;
  _arg = r._arg;
  _fOwner = false;
}

gthr_freertos &gthr_freertos::operator=(gthr_freertos &&r)
{
  if (this == &r)
    return *this;

  taskENTER_CRITICAL();

  if (_fOwner)
  {
    if (eDeleted != eTaskGetState(_taskHandle))
      vTaskDelete(_taskHandle);
    if (_evHandle)
      vEventGroupDelete(_evHandle);
    _fOwner = false;
  }
  else if (r._fOwner)
  {
    taskEXIT_CRITICAL();
    r.wait_for_start();
    taskENTER_CRITICAL();
  }

  move(std::forward<gthr_freertos>(r));
  taskEXIT_CRITICAL();
  return *this;
}
```

## System Time

Last bit of c++ threading is `sleep_for` and `sleep_until` functions.
The first one is simple and requires just one function which is defined in
`thread.cpp` file. It assumes that one tick in FreeRTOS is
one millisecond. Time is converted to ticks and FreeRTOS API `vTaskDelay` does
the job.

```C++
void this_thread::__sleep_for(chrono::seconds sec, chrono::nanoseconds nsec)
{
  long ms = nsec.count() / 1'000'000;
  if (sec.count() == 0 && ms == 0 && nsec.count() > 0)
    ms = 1; // round up to 1 ms => if sleep time != 0, sleep at least 1ms

  vTaskDelay(pdMS_TO_TICKS(chrono::milliseconds(sec).count() + ms));
}
```

Second function is, in fact, already implemented. However, it requires system
time to operate. `sleep_until` calls `gettimeofday`, which then calls
`_gettimeofday`. This one must be implemented using FreeRTOS API.

In order to get time of day, it would be nice to be able to set time of day
first. For this reason an additional function to set time is provided. As far
as I am aware `ctime` header does not provide a standard function for setting
time. My own implementation is provided instead. Both functions are in
`freertos_time.cpp` file.

Algorithm is very simple. System ticks is a time counter. Then a global
variable is needed to keep an offset between the real time and ticks.
The variable must be thread safe so:

```C++
namespace free_rtos_std
{

class wall_clock
{
public:
  struct time_data
  {
    timeval offset;
    TickType_t ticks;
  };

  static time_data time()
  { //atomic
    critical_section critical;
    return time_data{_timeOffset, xTaskGetTickCount()};
  }

  static void time(const timeval &time)
  { //atomic
    critical_section critical;
    _timeOffset = time;
  }

private:
  static timeval _timeOffset;
};

timeval wall_clock::_timeOffset;

}
```

Setting time becomes easy. Just storing the difference between ticks and
the time:

```C++
using namespace std::chrono;
void SetSystemClockTime(
    const time_point<system_clock, system_clock::duration> &time)
{
  auto delta = time - time_point<system_clock>(
                          milliseconds(pdTICKS_TO_MS(xTaskGetTickCount())));
  long long sec = duration_cast<seconds>(delta).count();
  long usec = duration_cast<microseconds>(delta).count() - sec * 1'000'000;

  free_rtos_std::wall_clock::time({sec, usec});
}
```

Reading time is a reversed operation - add the offset and ticks:

```C++
timeval operator+(const timeval &l, const timeval &r);


extern "C" int _gettimeofday(timeval *tv, void *tzvp)
{
  (void)tzvp;

  auto t = free_rtos_std::wall_clock::time();

  long long sec = pdTICKS_TO_MS(t.ticks) / 1000;
  long usec = pdTICKS_TO_MS(t.ticks - (sec * configTICK_RATE_HZ)) * 1000;
  *tv = t.offset + timeval{sec, usec};

  return 0; // return non-zero for error
}
```

# Summary

There is few clever things in this library to manage hiding FreeRTOS behind
generic handles but in general I believe it is a clean solution. I have doubts
about performance. There is some copying involved. Also interrupts are disabled
in few places. However, as I mentioned at the beginning, not every embedded
application is a safety critical or (hard) real time one. I could be wrong
but I believe, someone who wants a real time application would not use std::thread
in the first place anyway.

I believe that the main advantage of this library is the same generic
C++ interface. I find it handy to implement and debug certain algorithms in
Visual Studio and then port it to a target board painlessly.

My target was to make C++ multithreading available over FreeRTOS API. So, I did
not bother to make POSIX C interface working. For that reason I believe
code in `gthr-FreeRTOS.h` would not compile in plain C project (have not even
tried it).

I have not tried to implement any of the `futures` of the multithreading
interface. Although, by looking how the thread or conditional variable interface
is designed, I do not expect much surprises in `futures` either.
