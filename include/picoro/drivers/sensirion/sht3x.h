#pragma once

// This file contains a driver for the [SHT30][1] temperature and humidity
// sensor.  It's based on Sensirion's [generic driver][2], but has been
// modified for use with C++20 coroutines.
//
// `struct sensirion::SHT3x` represents a sensor.  It must be initialized with
// an `async_context_t*`, which is used for sleeps.  Other properties may be
// set on its `device` data member, such as the `ic2_inst_t*` to which the
// sensor is connected (`i2c0` or `i2c1`), and the I²C blocking `read_timeout`
// and `write_timeout`.
//
// `SHT3x` member functions can be `co_await`ed upon by a coroutine.  Each
// member function returns a `uint16_t` error code that is zero on success and
// nonzero if an error occurred.
//
// When including this header file, also name `picoro_devices_sensirion_sht3x`
// as a link library in your build configuration.
//
// Example usage:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/drivers/sensirion/sht3x.h>
//     #include <picoro/event_loop.h>
//     #include <picoro/sleep.h>
//
//     #include <pico/async_context_poll.h>
//     #include <pico/stdio.h>
//     #include <hardware/i2c.h>
//
//     #include <cassert>
//     #include <chrono>
//
//     picoro::Coroutine<void> monitor_weather(async_context_t *ctx) {
//       i2c_inst_t *const instance = i2c0;
//       const uint desired_clock_hz = 400 * 1000;
//       const uint sda_pin = 4;
//       const uint scl_pin = 5;
//
//       const uint actual_baudrate = i2c_init(instance, desired_clock_hz);
//       printf("The I2C baudrate is %u Hz\n", actual_baudrate);
//       gpio_set_function(sda_pin, GPIO_FUNC_I2C);
//       gpio_set_function(scl_pin, GPIO_FUNC_I2C);
//       gpio_pull_up(sda_pin);
//       gpio_pull_up(scl_pin);
//
//       picoro::sensirion::SHT3x sensor{ctx};
//       sensor.device.instance = i2c.instance;
//       for (;;) {
//         co_await picoro::sleep_for(ctx, std::chrono::seconds(2));
//         float celsius, percent;
//         if (int rc = co_await sensor.measure_single_shot_high_repeatability(
//                 &celsius, &percent)) {
//           printf("error: code %d\n", rc);
//           continue;
//         }
//         printf("temperature: %.1f C, humidity: %.1f%%\n", celsius, percent);
//       }
//     }
//
//     int main() {
//       stdio_init_all();
//
//       async_context_poll_t context;
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//       async_context_t *const ctx = &context.core;
//
//       picoro::run_event_loop(ctx, monitor_weather(ctx));
//
//       // unreachable
//       cyw43_arch_deinit();
//       async_context_deinit(ctx);
//     }
//
// [1]: https://cdn-shop.adafruit.com/product-files/5064/5064_Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital.pdf
// [2]: https://github.com/Sensirion/embedded-i2c-sht3x

/*
 * This code is originally based on the following file:
 *
 * - <https://github.com/Sensirion/embedded-i2c-sht3x/blob/b207314e143e9c1bcf1ebd2c1c3124c368be9868/sht3x_i2c.c>
 *
 * The copyright notice for that file is reproduced below.
 *
 * --------------------------------------------------------------------------
 *
 * Copyright (c) 2023, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <picoro/coroutine.h>
#include <picoro/drivers/sensirion/i2c.h>

namespace picoro {
namespace sensirion {

struct SHT3x {
  async_context_t* context;
  i2c::Device device;

  explicit SHT3x(async_context_t*, uint8_t i2c_address = 0x44);

  Coroutine<int16_t> measure_single_shot_high_repeatability(
        float* temperature_celsius,
        float* humidity_percent);
};

inline
SHT3x::SHT3x(async_context_t *context, uint8_t i2c_address) : context(context) {
  device.address = i2c_address;
}

inline
Coroutine<int16_t> SHT3x::measure_single_shot_high_repeatability(
      float* temperature_celsius,
      float* humidity_percent) {
    uint8_t buffer[6] = {};
    const uint16_t offset =
        i2c::add_command_to_buffer(buffer, 0, 0x2400);
    int16_t error =
        i2c::write_data(device, buffer, offset);
    if (error) {
        co_return error;
    }

    co_await sleep_for(context, std::chrono::milliseconds(16));

    error = i2c::read_data_inplace(device, buffer, 4);
    if (error) {
        co_return error;
    }

    const uint16_t temperature_ticks = i2c::bytes_to_uint16_t(&buffer[0]);
    const uint16_t humidity_ticks = i2c::bytes_to_uint16_t(&buffer[2]);
    // Pass through `double` for precision, and then narrow to `float`.
    *temperature_celsius = -45 + 175 * (temperature_ticks / 65535.0);
    *humidity_percent = 100 * (humidity_ticks / 65535.0);
    co_return error;
}

} // namespace sensirion
} // namespace picoro