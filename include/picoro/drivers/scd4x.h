#pragma once

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
#include <stdlib.h>

#include <chrono>

namespace sensirion {

constexpr uint16_t NO_ERROR = 0;

namespace common {

inline uint16_t bytes_to_uint16_t(const uint8_t* bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

inline uint32_t bytes_to_uint32_t(const uint8_t* bytes) {
  return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
         (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

inline int16_t bytes_to_int16_t(const uint8_t* bytes) {
  return (int16_t)common::bytes_to_uint16_t(bytes);
}

inline int32_t bytes_to_int32_t(const uint8_t* bytes) {
  return (int32_t)common::bytes_to_uint32_t(bytes);
}

inline float bytes_to_float(const uint8_t* bytes) {
  union {
    uint32_t u32_value;
    float float32;
  } tmp;

  tmp.u32_value = common::bytes_to_uint32_t(bytes);
  return tmp.float32;
}

inline void uint32_t_to_bytes(const uint32_t value, uint8_t* bytes) {
  bytes[0] = value >> 24;
  bytes[1] = value >> 16;
  bytes[2] = value >> 8;
  bytes[3] = value;
}

inline void uint16_t_to_bytes(const uint16_t value, uint8_t* bytes) {
  bytes[0] = value >> 8;
  bytes[1] = value;
}

inline void int32_t_to_bytes(const int32_t value, uint8_t* bytes) {
  bytes[0] = value >> 24;
  bytes[1] = value >> 16;
  bytes[2] = value >> 8;
  bytes[3] = value;
}

inline void int16_t_to_bytes(const int16_t value, uint8_t* bytes) {
  bytes[0] = value >> 8;
  bytes[1] = value;
}

inline void float_to_bytes(const float value, uint8_t* bytes) {
  union {
    uint32_t u32_value;
    float float32;
  } tmp;
  tmp.float32 = value;
  common::uint32_t_to_bytes(tmp.u32_value, bytes);
}

inline void copy_bytes(const uint8_t* source, uint8_t* destination,
                       uint16_t data_length) {
  uint16_t i;
  for (i = 0; i < data_length; i++) {
    destination[i] = source[i];
  }
}

}  // namespace common

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

inline uint16_t fill_cmd_send_buf(uint8_t* buf, uint16_t cmd,
                                  const uint16_t* args, uint8_t num_args) {
  uint8_t i;
  uint16_t idx = 0;

  buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
  buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

  for (i = 0; i < num_args; ++i) {
    buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

    uint8_t crc = generate_crc((uint8_t*)&buf[idx - 2], WORD_SIZE);
    buf[idx++] = crc;
  }
  return idx;
}

inline int16_t read_words_as_bytes(const Device& device, uint8_t* data,
                                   uint16_t num_words) {
  int16_t ret;
  uint16_t i, j;
  uint16_t size = num_words * (WORD_SIZE + CRC8_LEN);
  uint16_t word_buf[MAX_BUFFER_WORDS];
  uint8_t* const buf8 = (uint8_t*)word_buf;

  ret = device.read(buf8, size);
  if (ret != NO_ERROR) return ret;

  /* check the CRC for each word */
  for (i = 0, j = 0; i < size; i += WORD_SIZE + CRC8_LEN) {
    ret = check_crc(&buf8[i], WORD_SIZE, buf8[i + WORD_SIZE]);
    if (ret != NO_ERROR) return ret;

    data[j++] = buf8[i];
    data[j++] = buf8[i + 1];
  }

  return NO_ERROR;
}

inline int16_t read_words(const Device& device, uint16_t* data_words,
                          uint16_t num_words) {
  int16_t ret;
  uint8_t i;

  ret = read_words_as_bytes(device, (uint8_t*)data_words, num_words);
  if (ret != NO_ERROR) return ret;

  for (i = 0; i < num_words; ++i) {
    const uint8_t* word_bytes = (uint8_t*)&data_words[i];
    data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
  }

  return NO_ERROR;
}

inline int16_t write_cmd(const Device& device, uint16_t command) {
  uint8_t buf[COMMAND_SIZE];

  fill_cmd_send_buf(buf, command, NULL, 0);
  return device.write(buf, COMMAND_SIZE);
}

inline int16_t write_cmd_with_args(const Device& device, uint16_t command,
                                   const uint16_t* data_words,
                                   uint16_t num_words) {
  uint8_t buf[MAX_BUFFER_WORDS];
  uint16_t buf_size;

  buf_size = fill_cmd_send_buf(buf, command, data_words, num_words);
  return device.write(buf, buf_size);
}

inline int16_t read_cmd(const Device& device, uint16_t cmd,
                        uint16_t* data_words, uint16_t num_words) {
  int16_t ret;
  uint8_t buf[COMMAND_SIZE];

  fill_cmd_send_buf(buf, cmd, NULL, 0);
  ret = device.write(buf, COMMAND_SIZE);
  if (ret != NO_ERROR) return ret;

  return read_words(device, data_words, num_words);
}

inline uint16_t add_command_to_buffer(uint8_t* buffer, uint16_t offset,
                                      uint16_t command) {
  buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
  return offset;
}

inline uint16_t add_uint32_t_to_buffer(uint8_t* buffer, uint16_t offset,
                                       uint32_t data) {
  buffer[offset++] = (uint8_t)((data & 0xFF000000) >> 24);
  buffer[offset++] = (uint8_t)((data & 0x00FF0000) >> 16);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;
  buffer[offset++] = (uint8_t)((data & 0x0000FF00) >> 8);
  buffer[offset++] = (uint8_t)((data & 0x000000FF) >> 0);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;

  return offset;
}

inline uint16_t add_int32_t_to_buffer(uint8_t* buffer, uint16_t offset,
                                      int32_t data) {
  return add_uint32_t_to_buffer(buffer, offset, (uint32_t)data);
}

inline uint16_t add_uint16_t_to_buffer(uint8_t* buffer, uint16_t offset,
                                       uint16_t data) {
  buffer[offset++] = (uint8_t)((data & 0xFF00) >> 8);
  buffer[offset++] = (uint8_t)((data & 0x00FF) >> 0);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;

  return offset;
}

inline uint16_t add_int16_t_to_buffer(uint8_t* buffer, uint16_t offset,
                                      int16_t data) {
  return add_uint16_t_to_buffer(buffer, offset, (uint16_t)data);
}

inline uint16_t add_float_to_buffer(uint8_t* buffer, uint16_t offset,
                                    float data) {
  union {
    uint32_t uint32_data;
    float float_data;
  } convert;

  convert.float_data = data;

  buffer[offset++] = (uint8_t)((convert.uint32_data & 0xFF000000) >> 24);
  buffer[offset++] = (uint8_t)((convert.uint32_data & 0x00FF0000) >> 16);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;
  buffer[offset++] = (uint8_t)((convert.uint32_data & 0x0000FF00) >> 8);
  buffer[offset++] = (uint8_t)((convert.uint32_data & 0x000000FF) >> 0);
  buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
  offset++;

  return offset;
}

inline uint16_t add_bytes_to_buffer(uint8_t* buffer, uint16_t offset,
                                    uint8_t* data, uint16_t data_length) {
  uint16_t i;

  if (data_length % WORD_SIZE != 0) {
    return BYTE_NUM_ERROR;
  }

  for (i = 0; i < data_length; i += 2) {
    buffer[offset++] = data[i];
    buffer[offset++] = data[i + 1];

    buffer[offset] = generate_crc(&buffer[offset - WORD_SIZE], WORD_SIZE);
    offset++;
  }

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
  *co2 = common::bytes_to_uint16_t(&buffer[0]);
  *temperature = common::bytes_to_uint16_t(&buffer[2]);
  *humidity = common::bytes_to_uint16_t(&buffer[4]);
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
  *t_offset = common::bytes_to_uint16_t(&buffer[0]);
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
  *sensor_altitude = common::bytes_to_uint16_t(&buffer[0]);
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
  *frc_correction = common::bytes_to_uint16_t(&buffer[0]);
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
  *asc_enabled = common::bytes_to_uint16_t(&buffer[0]);
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
  local_data_ready = common::bytes_to_uint16_t(&buffer[0]);
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
  *serial_0 = common::bytes_to_uint16_t(&buffer[0]);
  *serial_1 = common::bytes_to_uint16_t(&buffer[2]);
  *serial_2 = common::bytes_to_uint16_t(&buffer[4]);
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
  *sensor_status = common::bytes_to_uint16_t(&buffer[0]);
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

}  // namespace sensirion
