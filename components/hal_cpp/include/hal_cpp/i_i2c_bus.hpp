// II2cBus — master-side I2C contract.
//
// `write_read` performs a write followed by a repeated-start read in a single
// transaction (the most common register-read pattern). `timeout_ms` is per-call
// so tests can simulate slow devices.

#pragma once

#include <cstddef>
#include <cstdint>

#include "hal_cpp/result.hpp"

namespace espid::hal_cpp {

class II2cBus {
   public:
    virtual ~II2cBus() = default;

    virtual Status write(std::uint8_t device_addr,
                         const std::uint8_t* data, std::size_t len,
                         std::uint32_t timeout_ms) noexcept = 0;

    virtual Status read(std::uint8_t device_addr,
                        std::uint8_t* out, std::size_t len,
                        std::uint32_t timeout_ms) noexcept = 0;

    virtual Status write_read(std::uint8_t device_addr,
                              const std::uint8_t* tx, std::size_t tx_len,
                              std::uint8_t* rx,       std::size_t rx_len,
                              std::uint32_t timeout_ms) noexcept = 0;
};

}  // namespace espid::hal_cpp
