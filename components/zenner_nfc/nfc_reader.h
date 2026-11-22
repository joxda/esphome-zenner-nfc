// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once
#include <cstdint>

namespace esphome {
namespace zenner_nfc {
class NfcReader {
 public:
  virtual ~NfcReader() = default;
  virtual void begin() = 0;
  virtual void tick() = 0;
  virtual void abort() = 0;
  virtual void hold_reset() = 0;
  virtual bool active() const = 0;
  virtual bool ok() const = 0;
  virtual bool transport_failed() const = 0;
  virtual const char *error() const = 0;
  virtual bool is_pn5180() const = 0;
  virtual uint8_t selection_stage() const = 0;
  virtual const uint8_t *data() const = 0;
  virtual const uint8_t *uid() const = 0;
  virtual uint8_t uid_length() const = 0;
  virtual void start_reset() = 0;
  virtual void start_rf(bool on) = 0;
  virtual void start_select() = 0;
  virtual void start_session() = 0;
  virtual void start_request(const uint8_t request[6]) = 0;
  virtual void start_response() = 0;
  virtual void start_eeprom_read(uint8_t address, uint8_t length) = 0;
  virtual void start_dpc_write(uint8_t value) = 0;
  virtual void set_busy_timeout(uint8_t ms) = 0;
};
}  // namespace zenner_nfc
}  // namespace esphome