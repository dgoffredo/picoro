#pragma once

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
