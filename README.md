Picoro
======
```
              _.---..._
           ./^         ^-._
         ./^C===.         ^\.   /\
        .|'     \\        _ ^|.^.|
   ___.--'_     ( )  .      ./ /||
  /.---^T\      ,     |     / /|||
 C'   ._`|  ._ /  __,-/    / /-,||
      \ \/    ;  /O  / _    |) )|,
       i \./^O\./_,-^/^    ,;-^,'
        \ |`--/ ..-^^      |_-^
         `|  \^-           /|:
          i.  .--         / '|.
           i   =='       /'  |\._
         _./`._        //    |.  ^-ooo.._
  _.oo../'  |  ^-.__./X/   . `|    |#######b
 d####     |'      ^^^^   /   |    _\#######
 #####b ^^^^^^^^--. ...--^--^^^^^^^_.d######
 ######b._         Y            _.d#########
 ##########b._     |        _.d#############
```
Picoro is a header-only library of C++20 coroutines for the Raspberry Pi Pico W
microcontroller board.

Why
---
I've been playing around with the [Raspberry Pi Pico W][1] ([datasheet][2])
[microcontroller board][12].  I wanted to have an accurate CO₂ sensor for less
than $100, and the only way was to buy the bare sensor module and learn how to
connect it to a computer.

The [SCD41][3] ([datasheet][4]) is a [nondispersive infrared sensor][4] (NDIR)
of CO₂.  It speaks [I²C][5], so if you want to use it you need either an
adapter for your computer or a microcontroller board.

The "W" in Raspberry Pi Pico W refers to its onboard 2.4 GHz WiFi and Bluetooth
chip, the [CYW43439][6].  My plan was to wire the SCD41 to the Pico, write some
code, and then query the current CO₂ concentration over the WiFi network.

And it works! My [initial solution][11] was written in [MicroPython][7], which
is an implementation of Python specifically for microcontrollers.  It even
implements the [asyncio][8] module.

The initial solution had periodic latency spikes that I attributed to
MicroPython's garbage collector. I was wrong — the spikes are unrelated to
MicroPython.  Still, I began rewriting the sensor server in C++ using the
Pico's excellent [C SDK][9] ([GitHub][10]).

The C++ standard library does not have an equivalent to Python's asyncio
module.  This presents an obstacle when you're writing programs for a platform
that does not have an operating system.  If your program needs to sleep (wait)
for a few milliseconds as part of the CO₂  sensor's communication protocol, the
CPU cannot do anything else during that time.  The CPU will still handle
interrupts, but your program cannot do any more real work until the wait time
has elapsed.  For example, the program cannot serve an incoming HTTP request.

There are multiple ways to work around this problem.  The most straightforward
approach is to use the Pico's second CPU core.  One part of the program can run
on the first core, waiting for the sensor to complete its measurement, while
another part of the program running on the second core can manage the WiFi chip
and serve HTTP requests.  That's cheating, though.  My problem is not a lack of
compute, but an inappropriate programming model.

So, change the programming model!  I could design the program as a state
machine.  When the program sends a "take a measurement" request to the CO₂
sensor, it could then transition into the "I'm waiting for the sensor" state.
From that state, it could transition into the "I'm replying to an HTTP request"
state.  Only later will the program enter the "I'm reading the response from
the sensor" state.

Writing programs this way is not fun.  It's easy enough to get right, but woe
to the future you trying to understand the program again just a few months
later.  Our human minds do not think of "poll the sensor for data, and also
serve HTTP requests as they come in" as a set of program states with
appropriate transitions between them that are equivalent to performing both
tasks.  That's crazy talk.  Instead, we think of the program as being composed
of two routines that execute concurrently.  _Coroutines_.  I need conceptual
machinery that allows me to write the program that way.

In MicroPython, the asyncio module provides the necessary machinery.  C++ does
not have an equivalent facility.

_Except it does_.  C++20 introduced the [coroutines][13] library, which does
not contain any coroutines.  What it does contain, though, are tools for
interacting with a new feature of the programming language: suspendible
functions.  Using these tools, you can implement your own coroutines, which you
can then use to write your program as a composition of concurrently executing
procedures.

I'm not the first to take this approach.  [FunMiles][14] went deep implementing
DMA channels, UART, sleep, and other async facilities of the Pico in terms of
C++20 coroutines.

My implementation is less powerful, solely focused on the task of writing a CO₂
sensing HTTP server on a single core.

What
----
Picoro is a header-only library of C++20 coroutine-compatible facilities for
use with the Raspberry Pi Pico W.  Coroutines that use this library are
intended to be scheduled by the Pico C SDK's `pico_async_context_poll` library.

Picoro consists of the following, most of which live in `namespace picoro`:

- [#include <picoro/coroutine.h>][15] defines `class Coroutine<Value>`, a
  coroutine that `co_return`s a `Value`.  Use this as the return type of any
  function containing `co_await` expressions or `co_return` statements.
- [#include <picoro/sleep.h>][16] defines
  `sleep_for(async_context_t*, std::chrono::microseconds)`.
  `co_await sleep_for(context, delay)` will suspend the invoking coroutine for
  the specified `delay` amount of time and then resume it using the specified
  `context`.
- [#include <picoro/event_loop.h>][17] defines
  `run_event_loop(async_context_t*, Coroutines...)`.  `run_event_loop` is an
  infinite loop that repeatedly polls the `async_context_t` for work to do,
  sleeping when there is no work to do.  The `Coroutines...` are ignored; those
  parameters exist as a convenient place for coroutine objects that must
  outlive the event loop.
- [#include <picoro/tcp.h>][18] defines coroutine adapters for the
  callback-based lwIP library included in the Pico's C SDK.  It provides only
  those facilities
  needed to run a socket server:
  - `class Listener` listens on all interfaces on a specified port, with a
    specified backlog, and has an `accept()` member function that can be
    `co_await`ed to obtain a tuple `(Connection, err_t)`.
  - `class Connection` is an `accept()`ed connection that has
    `send(std::string_view)` and `recv(char*, int)` member functions that can
    be `co_await`ed to send and receive data, respectively, to and from the
    client.
- [#include <picoro/debug.h>][19] defines `debug`, which is a wrapper around
  `printf` in debug builds, and a no-op in release builds.
- [#include <picoro/drivers/scd4x.h>][20] is a driver for the Sensirion SCD40
  and SCD41 CO₂ sensors.  It defines `struct sensirion::SCD4x`, whose member
  functions can be `co_await`ed to interact with the sensor.
- [examples/co2-server/][21] is an example program that motivated the writing
  of this library.  It's an HTTP server that responds to all requests with the latest data read from an SCD41 CO₂ sensor.

How
---
Include the [include/picoro][22] headers directly into your source code.
See [examples/co2-server/README.md][23].

Your `CMakeLists.txt` might need to depend on the following libraries:

- `pico_stdlib` for standard C library functionality
- `hardware_i2c` for the SCD4x sensor driver
- `pico_async_context_poll` for scheduling suspended coroutines
- `pico_cyw43_arch_lwip_poll` for scheduling network event handlers

More
----
Each header file is documented in a comment block at the beginning of the file.

[1]: https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html
[2]: https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf
[3]: https://sensirion.com/products/catalog/SCD41/
[4]: https://en.wikipedia.org/wiki/Nondispersive_infrared_sensor
[5]: https://en.wikipedia.org/wiki/I2c
[6]: https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-4-802.11n/cyw43439/
[7]: https://micropython.org/
[8]: https://docs.micropython.org/en/latest/library/asyncio.html
[9]: https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf
[10]: https://github.com/raspberrypi/pico-sdk
[11]: https://github.com/dgoffredo/raspberry-pi-pico-w/blob/main/co2-server/co2-server.py
[12]: https://en.wikipedia.org/wiki/Single-board_microcontroller
[13]: https://en.cppreference.com/w/cpp/language/coroutines
[14]: https://github.com/FunMiles/PicoAsync
[15]: include/picoro/coroutine.h
[16]: include/picoro/sleep.h
[17]: include/picoro/event_loop.h
[18]: include/picoro/tcp.h
[19]: include/picoro/debug.h
[20]: include/picoro/drivers/scd4x.h
[21]: examples/co2-server/
[22]: include/picoro
[23]: examples/co2-server/README.md
