#pragma once

// `sleep_for(async_context_t*, std::chrono::microseconds)` returns an object
// that can be awaited upon by a coroutine using the `co_await` operator.
// Doing so suspends the invoking coroutine and resumes it after the specified
// number of microseconds. The resumption is scheduled using the specified
// `async_context_t`.
//
// Example usage:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/event_loop.h>
//     #include <picoro/sleep.h>
//
//     #include <pico/async_context_poll.h>
//     #include <pico/stdlib.h>
//
//     #include <cassert>
//     #include <chrono>
//     #include <coroutine>
//     #include <iostream>
//
//     Coroutine<void> every_second(async_context_t *context) {
//       for (int i = 0; i < 10; ++i) {
//         co_await picoro::sleep_for(context, std::chrono::seconds(1));
//         std::cout << i << ": here I am\n";
//       }
//     }
//
//     int main() {
//       stdio_init_all();
//       async_context_poll_t context = {};
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//       picoro::run_event_loop(&context.core, every_second(&context.core));
//       // unreachable
//     }

#include <pico/async_context.h>
#include <pico/time.h>

#include <chrono>
#include <coroutine>
#include <cstdlib>

namespace picoro {
//              _.---..._
//           ./^         ^-._
//         ./^C===.         ^\.   /\.
//        .|'     \\        _ ^|.^.|
//   ___.--'_     ( )  .      ./ /||
//  /.---^T\      ,     |     / /|||
// C'   ._`|  ._ /  __,-/    / /-,||
//      \ \/    ;  /O  / _    |) )|,
//       i \./^O\./_,-^/^    ,;-^,'
//        \ |`--/ ..-^^      |_-^
//         `|  \^-           /|:
//          i.  .--         / '|.
//           i   =='       /'  |\._
//         _./`._        //    |.  ^-ooo.._
//  _.oo../'  |  ^-.__./X/   . `|    |#######b
// d####     |'      ^^^^   /   |    _\#######
// #####b ^^^^^^^^--. ...--^--^^^^^^^_.d######
// ######b._         Y            _.d#########
// ##########b._     |        _.d#############
//
//                     --- Steven J. Simmons

struct Sleep {
  async_context_t *context;
  absolute_time_t deadline;
  async_at_time_worker_t worker = {};
  std::coroutine_handle<> continuation = nullptr;

  bool await_ready();
  void await_suspend(std::coroutine_handle<> continuation);
  void await_resume();

  static void on_expire(async_context_t *, async_at_time_worker_t *);
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Sleep sleep_for(async_context_t *context, std::chrono::microseconds delay);

// Implementations
// ===============

// struct Sleep
// ------------
inline bool Sleep::await_ready() { return false; }

inline void Sleep::await_suspend(std::coroutine_handle<> coroutine) {
  continuation = coroutine;
  worker.do_work = &Sleep::on_expire;
  worker.user_data = this;
  (void)async_context_add_at_time_worker_at(context, &worker, deadline);
}

inline void Sleep::await_resume() {}

inline void Sleep::on_expire(async_context_t *,
                             async_at_time_worker_t *worker) {
  auto *sleep = static_cast<Sleep *>(worker->user_data);
  sleep->continuation.resume();
}

inline Sleep sleep_for(async_context_t *context,
                       std::chrono::microseconds delay) {
  const uint64_t delay_us = delay / std::chrono::microseconds(1);
  return Sleep{context, make_timeout_time_us(delay_us)};
}

}  // namespace picoro
