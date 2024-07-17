#pragma once

// `run_event_loop(async_context_t*, Coroutines...)` polls the specified
// `async_context_t` in an infinite loop.  Zero or more `Coroutines...` may
// also be specified, but `run_event_loop` doesn't do anything with them
// besides keep them alive for the duration of the loop.
//
// `run_event_loop` does not return.
//
// For example:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/event_loop.h>
//
//     #include <pico/async_context_poll.h>
//
//     Coroutine<void> some_coroutine();
//     Coroutine<void> some_other_coroutine(async_context_t*);
//
//     int main() {
//       async_context_poll_t context = {};
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//       picoro::run_event_loop(
//           &context.core,
//           some_coroutine(),
//           some_other_coroutine(&context.core));
//     }

#include <pico/async_context.h>
#include <picoro/debug.h>

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

template <typename... Coroutines>
void run_event_loop(async_context_t *context,
                    Coroutines... toplevel_coroutines);

// Implementations
// ===============

// void run_event_loop
// -------------------
template <typename... Coroutines>
void run_event_loop(async_context_t *context, Coroutines...) {
  // The `Coroutines...` don't need names.  `picoro::Coroutine` is eagerly
  // started, so the `Coroutines...` parameters serve only to provide a place
  // for those coroutine objects to live while the event loop runs.
  for (;;) {
    debug("{");
    async_context_poll(context);
    debug("}");
    async_context_wait_for_work_ms(context, 10 * 1000);
  }
}

}  // namespace picoro
