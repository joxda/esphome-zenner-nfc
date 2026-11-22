// SPDX-License-Identifier: LGPL-2.1-or-later
// Cooperative adaptation of the PN5180 work of Andreas Trappmann (2018)
// and Mark Hermann (2025). Original attribution and license: NOTICE.md / LICENSE.
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
// Copyright (c) 2025 by Mark Hermann. All rights reserved.
//
// This file is part of the Zenner NFC component. See NOTICE.md for upstream attribution.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
#pragma once
#include <cstdint>
#include <initializer_list>

#include "nfc_reader.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace zenner_nfc {
// NSS is managed separately from the short SPI acquisitions: PN5180 holds BUSY
// high until NSS rises. This requires a dedicated, validated ESPHome SPI bus.
class PN5180 : public NfcReader,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_2MHZ>,
               public i2c::I2CDevice {
 public:
  void set_cs_pin(GPIOPin *pin) { nss_ = pin; }
  void set_busy_pin(GPIOPin *pin) { busy_ = pin; }
  void set_reset_pin(GPIOPin *pin) { rst_ = pin; }
  void set_pn532_mode(bool enabled) { pn532_mode_ = enabled; }
  void set_pn532_i2c(bool enabled) { pn532_i2c_ = enabled; }
  void set_busy_timeout(uint8_t ms) override { busy_timeout_ = ms; }
  void begin() override;
  void tick() override;
  void abort() override;
  void hold_reset() override;
  bool active() const override { return active_; }
  bool ok() const override { return ok_; }
  bool transport_failed() const override { return transport_failed_; }
  const char *error() const override { return error_; }
  bool is_pn5180() const override { return !pn532_mode_; }
  uint8_t selection_stage() const override { return selection_stage_; }
  const uint8_t *data() const override { return rx_; }
  const uint8_t *uid() const override { return uid_; }
  uint8_t uid_length() const override { return uid_length_; }
  void start_reset() override;
  void start_rf(bool on) override;
  void start_select() override;
  void start_session() override;
  void start_request(const uint8_t request[6]) override;
  void start_response() override;
  void start_eeprom_read(uint8_t address, uint8_t length) override;
  void start_dpc_write(uint8_t value) override;

 protected:
  enum class Kind : uint8_t { COMMAND, PAUSE, RESET_LOW, RESET_HIGH, IRQ, RF_STATE, RX_SHAPE, ACK };
  struct Action {
    Kind kind{Kind::COMMAND};
    uint8_t bytes[10]{};
    uint8_t length{0}, receive{0};
    uint32_t value{0};
  };
  enum class Host : uint8_t {
    IDLE,
    WAIT_LOW,
    COMMAND_SETUP,
    COMMAND_HIGH,
    COMMAND_LOW,
    RESPONSE_SETUP,
    RESPONSE_HIGH,
    RESPONSE_LOW
  };
  Action actions_[40]{};
  uint8_t count_{0}, index_{0};
  Host host_{Host::IDLE};
  uint8_t host_tx_[10]{}, host_length_{0}, host_receive_{0};
  uint8_t rx_[64]{}, uid_[7]{}, uid_length_{0}, select_cmd_[7]{};
  uint8_t selection_stage_{0};
  bool selecting_{false}, active_{false}, ok_{false}, transport_failed_{false};
  bool action_started_{false}, irq_reading_{false};
  uint32_t phase_at_{0}, action_at_{0};
  uint8_t busy_timeout_{50};
  GPIOPin *nss_{nullptr}, *busy_{nullptr}, *rst_{nullptr};
  const char *error_{"none"};
  enum class PN532Operation : uint8_t { NONE, RESET, RF_ON, RF_OFF, SELECT, SESSION, REQUEST, RESPONSE };
  PN532Operation pn532_operation_{PN532Operation::NONE};
  uint8_t pn532_firmware_[4]{};
  uint8_t request_[6]{};
  bool pn532_mode_{false}, pn532_i2c_{true}, pn532_initialized_{false};
  void start_();
  void fail_(const char *error, bool transport = false);
  void next_();
  void command_(std::initializer_list<uint8_t> bytes, uint8_t receive = 0);
  void reg_(uint8_t opcode, uint8_t address, uint32_t value);
  void action_(Kind kind, uint32_t value = 0, uint8_t receive = 0);
  void exchange_(const uint8_t *bytes, uint8_t length, uint8_t expected, bool short_ack = false,
                 uint8_t valid_bits = 0);
  void selection_next_();
  void host_start_(const uint8_t *bytes, uint8_t length, uint8_t receive);
  bool host_tick_();
  uint32_t word_() const;
  void start_pn532_(PN532Operation operation);
  void tick_pn532_();
  bool pn532_command_(uint8_t command, const uint8_t *params, uint8_t param_length, uint8_t *response,
                      uint8_t response_capacity, uint8_t &response_length);
  bool pn532_write_(const uint8_t *data, size_t length);
  bool pn532_read_ready_();
  bool pn532_read_(uint8_t *data, size_t length);
  bool pn532_exchange_(const uint8_t *command, uint8_t command_length, uint8_t *response,
                       uint8_t expected_length);
  bool pn532_wait_ready_(uint16_t timeout_ms);
};
}  // namespace zenner_nfc
}  // namespace esphome
