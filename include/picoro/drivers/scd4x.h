#pragma once

// This file contains a driver for the [Sensirion SCD40 and SCD41][1] CO₂
// sensors.  It's based on Sensirion's [generic driver][2], but has been
// modified for use with C++20 coroutines.
//
// `struct sensirion::SCD4x` represents a sensor.  It must be initialized with
// an `async_context_t*`, which is used for sleeps, and other properties may be
// set on its `device` data member, such as the `ic2_inst_t*` to which the
// sensor is connected (`i2c0` or `i2c1`), the I²C address of the sensor, and
// the I²C blocking `read_timeout` and `write_timeout`.
//
// `SCD4x` member functions can be `co_await`ed upon by a coroutine.  Each
// member function returns a `uint16_t` error code that is zero (`NO_ERROR`) on
// success and nonzero if an error occurred.
//
// Example usage:
//
//     #include <picoro/coroutine.h>
//     #include <picoro/event_loop.h>
//     #include <picoro/sleep.h>
//     #include <picoro/drivers/scd4x.h>
//     #include <pico/async_context_poll.h>
//     #include <pico/stdlib.h>
//     #include <stdio.h>
//     #include <cassert>
//     #include <coroutine>
//
//     picoro::Coroutine<bool> data_ready(const sensirion::SCD4x &sensor) {
//       bool result;
//       int rc = co_await sensor.get_data_ready_flag(&result);
//       if (rc) {
//         printf("Unable to query whether the sensor has data ready. Error code %d.\n", rc);
//         co_return false;
//       }
//       co_return result;
//     }
//
//     picoro::Coroutine<void> monitor_scd4x(async_context_t *context) {
//       // I²C GPIO pins
//       const uint sda_pin = 12;
//       const uint scl_pin = 13;
//       // I²C clock rate
//       const uint clock_hz = 400 * 1000;
//
//       i2c_inst_t *const instance = i2c0;
//       const uint actual_baudrate = i2c_init(instance, clock_hz);
//       struct Guard {
//         i2c_inst_t *const instance;
//         ~Guard() {
//           i2c_deinit(instance);
//         }
//       } guard{instance};
//       printf("The actual I2C baudrate is %u Hz\n", actual_baudrate);
//       gpio_set_function(sda_pin, GPIO_FUNC_I2C);
//       gpio_set_function(scl_pin, GPIO_FUNC_I2C);
//       gpio_pull_up(sda_pin);
//       gpio_pull_up(scl_pin);
//
//       sensirion::SCD4x sensor{context};
//       sensor.device.instance = instance;
//
//       int rc = co_await sensor.set_automatic_self_calibration(0);
//       if (rc) {
//         printf("Unable to disable automatic self-calibration. Error code %d.\n", rc);
//         co_return;
//       }
//
//       rc = co_await sensor.start_periodic_measurement();
//       if (rc) {
//         printf("Unable to start periodic measurement mode. Error code %d.\n", rc);
//         co_return;
//       }
//
//       for (;;) {
//         co_await picoro::sleep_for(context, std::chrono::seconds(5));
//         while (!co_await data_ready(sensor)) {
//           co_await picoro::sleep_for(context, std::chrono::seconds(1));
//         }
//
//         uint16_t co2_ppm;
//         int32_t temperature_millicelsius;
//         int32_t relative_humidity_millipercent;
//         rc = co_await sensor.read_measurement(&co2_ppm, &temperature_millicelsius,
//                                               &relative_humidity_millipercent);
//         if (rc) {
//           printf("Unable to read sensor measurement. Error code %d.\n", rc);
//         } else {
//           printf("CO2: %hu ppm\ttemperature: %d millicelsius\thumidity: %d millipercent\n",
//             co2_ppm,
//             temperature_millicelsius,
//             relative_humidity_millipercent);
//         }
//       }
//     }
//
//     int main() {
//       stdio_init_all();
//
//       async_context_poll_t context = {};
//       const bool ok = async_context_poll_init_with_defaults(&context);
//       assert(ok);
//
//       picoro::run_event_loop(&context.core, monitor_scd4x(&context.core));
//
//       // unreachable
//       async_context_deinit(&context.core);
//     }
//
// [1]: https://developer.sensirion.com/products-support/scd4x-co2-sensor
// [2]: https://github.com/Sensirion/embedded-i2c-scd4x/

/*
 * This code is originally based on the following files:
 *
 * - <https://github.com/Sensirion/embedded-i2c-scd4x/blob/3b413b9c804dd7c4df11178e9cf2501b6f61fa2e/scd4x_i2c.c>
 * - <https://github.com/Sensirion/embedded-i2c-scd4x/blob/3b413b9c804dd7c4df11178e9cf2501b6f61fa2e/sensirion_common.c>
 * - <https://github.com/Sensirion/embedded-i2c-scd4x/blob/3b413b9c804dd7c4df11178e9cf2501b6f61fa2e/sensirion_i2c.c>
 * - <https://github.com/Sensirion/embedded-i2c-scd4x/blob/3b413b9c804dd7c4df11178e9cf2501b6f61fa2e/sensirion_i2c_hal.c>
 *
 * The copyright notice for those files is reproduced below.
 *
 * --------------------------------------------------------------------------
 *
 * Copyright (c) 2018, 2021, Sensirion AG
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

#include <hardware/i2c.h>
#include <picoro/coroutine.h>
#include <picoro/sleep.h>
#include <stdint.h>

#include <chrono>

namespace sensirion {

constexpr uint16_t NO_ERROR = 0;

namespace i2c {

constexpr int8_t CRC_ERROR = 1;
constexpr uint16_t BYTE_NUM_ERROR = 4;

constexpr uint8_t CRC8_POLYNOMIAL = 0x31;
constexpr uint8_t CRC8_INIT = 0xFF;
constexpr uint16_t CRC8_LEN = 1;

constexpr unsigned COMMAND_SIZE = 2;
constexpr unsigned WORD_SIZE = 2;
constexpr unsigned MAX_BUFFER_WORDS = 32;

struct Device {
  i2c_inst_t* instance = i2c0;
  uint8_t address = 0x62;
  std::chrono::microseconds read_timeout = std::chrono::microseconds(1000);
  std::chrono::microseconds write_timeout = std::chrono::microseconds(1000);

  /**
   * Execute one read transaction on the I2C bus, reading a given number of
   * bytes. If the device does not acknowledge the read command, return an
   * error.
   *
   * @param data    pointer to the buffer where the data is to be stored
   * @param count   number of bytes to read from I2C and store in the buffer
   * @returns 0 on success, error code otherwise
   */
  int8_t read(uint8_t* data, uint16_t count) const;

  /**
   * Execute one write transaction on the I2C bus, sending a given number of
   * bytes. The bytes in the supplied buffer must be sent to the given address.
   * If the slave device does not acknowledge any of the bytes, return an error.
   *
   * @param data    pointer to the buffer containing the data to write
   * @param count   number of bytes to read from the buffer and send over I2C
   * @returns 0 on success, error code otherwise
   */
  int8_t write(const uint8_t* data, uint16_t count) const;
};

inline int8_t Device::read(uint8_t* data, uint16_t count) const {
  const bool nostop = true;  // master retains control of the bus after the read
  const unsigned timeout_μs = read_timeout / std::chrono::microseconds(1);
  const int rc =
      i2c_read_timeout_us(instance, address, data, count, nostop, timeout_μs);
  switch (rc) {
    case PICO_ERROR_GENERIC:  // address not acknowledged
    case PICO_ERROR_TIMEOUT:  // device didn't respond in time
      return rc;
    default:
      // `rc` is the number of bytes read. If it's what we wanted (`count`),
      // good. If not, bad.
      if (rc == count) {
        return NO_ERROR;
      }
      return PICO_ERROR_NO_DATA;
  }
}

inline int8_t Device::write(const uint8_t* data, uint16_t count) const {
  const bool nostop =
      true;  // master retains control of the bus after the write
  const unsigned timeout_μs = write_timeout / std::chrono::microseconds(1);
  const int rc =
      i2c_write_timeout_us(instance, address, data, count, nostop, timeout_μs);
  switch (rc) {
    case PICO_ERROR_GENERIC:  // address not acknowledged
    case PICO_ERROR_TIMEOUT:  // device didn't respond in time
      return rc;
    default:
      // `rc` is the number of bytes written. If it's what we wanted (`count`),
      // good. If not, bad.
      if (rc == count) {
        return NO_ERROR;
      }
      return PICO_ERROR_IO;
  }
}

inline uint8_t generate_crc(const uint8_t* data, uint16_t count) {
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for (current_byte = 0; current_byte < count; ++current_byte) {
    crc ^= (data[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

inline int8_t check_crc(const uint8_t* data, uint16_t count, uint8_t checksum) {
  if (generate_crc(data, count) != checksum) return CRC_ERROR;
  return NO_ERROR;
}

inline uint16_t add_command_to_buffer(uint8_t* buffer, uint16_t offset,
                                      uint16_t command) {
  buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
  return offset;
}

inline uint16_t add_uint16_t_to_buffer(uint8_t* buffer, uint16_t offset,
                                       uint16_t data) {
  buffer[offset++] = (uint8_t)((data & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((data & 0x00FF) >> 0);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;

  return offset;
}

inline int16_t write_data(const Device& device, const uint8_t* data,
                          uint16_t data_length) {
  return device.write(data, data_length);
}

inline int16_t read_data_inplace(const Device& device, uint8_t* buffer,
                                 uint16_t expected_data_length) {
  int16_t error;
  uint16_t i, j;
  uint16_t size = (expected_data_length / WORD_SIZE) * (WORD_SIZE + CRC8_LEN);

  if (expected_data_length % WORD_SIZE != 0) {
    return BYTE_NUM_ERROR;
  }

  error = device.read(buffer, size);
  if (error) {
    return error;
  }

  for (i = 0, j = 0; i < size; i += WORD_SIZE + CRC8_LEN) {
    error = check_crc(&buffer[i], WORD_SIZE, buffer[i + WORD_SIZE]);
    if (error) {
      return error;
    }
    buffer[j++] = buffer[i];
    buffer[j++] = buffer[i + 1];
  }

  return NO_ERROR;
}

}  // namespace i2c

struct SCD4x {
  async_context_t* context;
  i2c::Device device;

  explicit SCD4x(async_context_t*);
  SCD4x() = delete;

  picoro::Coroutine<int16_t> start_periodic_measurement() const;
  picoro::Coroutine<int16_t> read_measurement_ticks(uint16_t* co2,
                                                    uint16_t* temperature,
                                                    uint16_t* humidity) const;
  picoro::Coroutine<int16_t> read_measurement(
      uint16_t* co2, int32_t* temperature_m_deg_c,
      int32_t* humidity_m_percent_rh) const;
  picoro::Coroutine<int16_t> stop_periodic_measurement() const;
  picoro::Coroutine<int16_t> get_temperature_offset_ticks(
      uint16_t* t_offset) const;
  picoro::Coroutine<int16_t> get_temperature_offset(
      int32_t* t_offset_m_deg_c) const;
  picoro::Coroutine<int16_t> set_temperature_offset_ticks(
      uint16_t t_offset) const;
  picoro::Coroutine<int16_t> set_temperature_offset(
      int32_t t_offset_m_deg_c) const;
  picoro::Coroutine<int16_t> get_sensor_altitude(
      uint16_t* sensor_altitude) const;
  picoro::Coroutine<int16_t> set_sensor_altitude(
      uint16_t sensor_altitude) const;
  picoro::Coroutine<int16_t> set_ambient_pressure(
      uint16_t ambient_pressure) const;
  picoro::Coroutine<int16_t> perform_forced_recalibration(
      uint16_t target_co2_concentration, uint16_t* frc_correction) const;
  picoro::Coroutine<int16_t> get_automatic_self_calibration(
      uint16_t* asc_enabled) const;
  picoro::Coroutine<int16_t> set_automatic_self_calibration(
      uint16_t asc_enabled) const;
  picoro::Coroutine<int16_t> start_low_power_periodic_measurement() const;
  picoro::Coroutine<int16_t> get_data_ready_flag(bool* data_ready_flag) const;
  picoro::Coroutine<int16_t> persist_settings() const;
  picoro::Coroutine<int16_t> get_serial_number(uint16_t* serial_0,
                                               uint16_t* serial_1,
                                               uint16_t* serial_2) const;
  picoro::Coroutine<int16_t> perform_self_test(uint16_t* sensor_status) const;
  picoro::Coroutine<int16_t> perform_factory_reset() const;
  picoro::Coroutine<int16_t> reinit() const;
  picoro::Coroutine<int16_t> measure_single_shot() const;
  picoro::Coroutine<int16_t> measure_single_shot_rht_only() const;
  picoro::Coroutine<int16_t> power_down() const;
  picoro::Coroutine<int16_t> wake_up() const;

  static uint16_t bytes_to_uint16_t(const uint8_t*);
};

inline SCD4x::SCD4x(async_context_t* context) : context(context) {}

inline picoro::Coroutine<int16_t> SCD4x::start_periodic_measurement() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x21B1);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::read_measurement_ticks(
    uint16_t* co2, uint16_t* temperature, uint16_t* humidity) const {
  int16_t error;
  uint8_t buffer[9];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0xEC05);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 6);
  if (error) {
    co_return error;
  }
  *co2 = bytes_to_uint16_t(&buffer[0]);
  *temperature = bytes_to_uint16_t(&buffer[2]);
  *humidity = bytes_to_uint16_t(&buffer[4]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::read_measurement(
    uint16_t* co2, int32_t* temperature_m_deg_c,
    int32_t* humidity_m_percent_rh) const {
  int16_t error;
  uint16_t temperature;
  uint16_t humidity;

  error = co_await read_measurement_ticks(co2, &temperature, &humidity);
  if (error) {
    co_return error;
  }
  *temperature_m_deg_c = ((21875 * (int32_t)temperature) >> 13) - 45000;
  *humidity_m_percent_rh = ((12500 * (int32_t)humidity) >> 13);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::stop_periodic_measurement() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3F86);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(500000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::get_temperature_offset_ticks(
    uint16_t* t_offset) const {
  int16_t error;
  uint8_t buffer[3];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2318);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  *t_offset = bytes_to_uint16_t(&buffer[0]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::get_temperature_offset(
    int32_t* t_offset_m_deg_c) const {
  int16_t error;
  uint16_t t_offset;

  error = co_await get_temperature_offset_ticks(&t_offset);
  if (error) {
    co_return error;
  }
  *t_offset_m_deg_c = ((21875 * (int32_t)t_offset) >> 13);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::set_temperature_offset_ticks(
    uint16_t t_offset) const {
  int16_t error;
  uint8_t buffer[5];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x241D);

  offset = i2c::add_uint16_t_to_buffer(&buffer[0], offset, t_offset);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::set_temperature_offset(
    int32_t t_offset_m_deg_c) const {
  uint16_t t_offset = (uint16_t)((t_offset_m_deg_c * 12271) >> 15);
  co_return co_await set_temperature_offset_ticks(t_offset);
}

inline picoro::Coroutine<int16_t> SCD4x::get_sensor_altitude(
    uint16_t* sensor_altitude) const {
  int16_t error;
  uint8_t buffer[3];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2322);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  *sensor_altitude = bytes_to_uint16_t(&buffer[0]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::set_sensor_altitude(
    uint16_t sensor_altitude) const {
  int16_t error;
  uint8_t buffer[5];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2427);

  offset = i2c::add_uint16_t_to_buffer(&buffer[0], offset, sensor_altitude);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::set_ambient_pressure(
    uint16_t ambient_pressure) const {
  int16_t error;
  uint8_t buffer[5];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0xE000);

  offset = i2c::add_uint16_t_to_buffer(&buffer[0], offset, ambient_pressure);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::perform_forced_recalibration(
    uint16_t target_co2_concentration, uint16_t* frc_correction) const {
  int16_t error;
  uint8_t buffer[5];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x362F);

  offset =
      i2c::add_uint16_t_to_buffer(&buffer[0], offset, target_co2_concentration);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(400000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  *frc_correction = bytes_to_uint16_t(&buffer[0]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::get_automatic_self_calibration(
    uint16_t* asc_enabled) const {
  int16_t error;
  uint8_t buffer[3];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2313);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  *asc_enabled = bytes_to_uint16_t(&buffer[0]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::set_automatic_self_calibration(
    uint16_t asc_enabled) const {
  int16_t error;
  uint8_t buffer[5];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2416);

  offset = i2c::add_uint16_t_to_buffer(&buffer[0], offset, asc_enabled);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::start_low_power_periodic_measurement()
    const {
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x21AC);

  co_return i2c::write_data(device, &buffer[0], offset);
}

inline picoro::Coroutine<int16_t> SCD4x::get_data_ready_flag(
    bool* data_ready_flag) const {
  int16_t error;
  uint8_t buffer[3];
  uint16_t offset = 0;
  uint16_t local_data_ready = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0xE4B8);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  local_data_ready = bytes_to_uint16_t(&buffer[0]);
  *data_ready_flag = (local_data_ready & 0x07FF) != 0;
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::persist_settings() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3615);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(800000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::get_serial_number(
    uint16_t* serial_0, uint16_t* serial_1, uint16_t* serial_2) const {
  int16_t error;
  uint8_t buffer[9];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3682);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));

  error = i2c::read_data_inplace(device, &buffer[0], 6);
  if (error) {
    co_return error;
  }
  *serial_0 = bytes_to_uint16_t(&buffer[0]);
  *serial_1 = bytes_to_uint16_t(&buffer[2]);
  *serial_2 = bytes_to_uint16_t(&buffer[4]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::perform_self_test(
    uint16_t* sensor_status) const {
  int16_t error;
  uint8_t buffer[3];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3639);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }

  co_await picoro::sleep_for(context, std::chrono::microseconds(10000000));

  error = i2c::read_data_inplace(device, &buffer[0], 2);
  if (error) {
    co_return error;
  }
  *sensor_status = bytes_to_uint16_t(&buffer[0]);
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::perform_factory_reset() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3632);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(800000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::reinit() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x3646);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(20000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::measure_single_shot() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x219D);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(5000000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::measure_single_shot_rht_only() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x2196);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(50000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::power_down() const {
  int16_t error;
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x36E0);

  error = i2c::write_data(device, &buffer[0], offset);
  if (error) {
    co_return error;
  }
  co_await picoro::sleep_for(context, std::chrono::microseconds(1000));
  co_return NO_ERROR;
}

inline picoro::Coroutine<int16_t> SCD4x::wake_up() const {
  uint8_t buffer[2];
  uint16_t offset = 0;
  offset = i2c::add_command_to_buffer(&buffer[0], offset, 0x36F6);

  // Sensor does not acknowledge the wake-up call, error is ignored
  (void)i2c::write_data(device, &buffer[0], offset);
  co_await picoro::sleep_for(context, std::chrono::microseconds(20000));
  co_return NO_ERROR;
}

inline uint16_t SCD4x::bytes_to_uint16_t(const uint8_t* bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

}  // namespace sensirion
