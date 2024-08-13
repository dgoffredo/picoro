#pragma once

// This file provides I2C related utilities for use by Sensirion sensor
// devices.  Definitions within this file live in the `picoro::sensirion::i2c`
// namespace.
//
// `struct i2c::Device` provides blocking-with-timeout reads and writes to and
// from an I2C address on a particular I2C instance (`i2c_inst_t*`).
//
// Both `struct SCD4x` (in `scd4x.h`) and `struct SHT3x` (in `sht3x.h`) contain
// an `i2c::Device` data member.
//
// This file also contains utilities for performing cyclic redundancy checks
// (CRCs), as well as for marshalling bytes to and from I2C command words.

/*
 * This code is originally based on the following files:
 *
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

#include <chrono>

#include <stdint.h>

namespace picoro {
namespace sensirion {
namespace i2c {

constexpr int8_t CRC_ERROR = 1;
constexpr uint16_t BYTE_NUM_ERROR = 4;

constexpr uint16_t CRC8_LEN = 1;
constexpr unsigned WORD_SIZE = 2;

struct Device {
  i2c_inst_t* instance = i2c0;
  uint8_t address;
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
        return 0;
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
        return 0;
      }
      return PICO_ERROR_IO;
  }
}

inline uint8_t generate_crc(const uint8_t* data, uint16_t count) {
  constexpr uint8_t CRC8_POLYNOMIAL = 0x31;
  constexpr uint8_t CRC8_INIT = 0xFF;

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
  return 0;
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

  return 0;
}

inline uint16_t bytes_to_uint16_t(const uint8_t* bytes) {
  return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

}  // namespace i2c
}  // namespace sensirion
}  // namespace picoro
