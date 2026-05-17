// ESPID-Coffee — TSIC 306 RMT capture driver (ESP-IDF only).
//
// Implements IZacwireCapture using the RMT RX peripheral. The sensor's
// ZACwire bitstream on a single GPIO is captured into symbol pairs (low/high
// durations), then parsed into two 10-bit packets (start + 8 data + parity)
// which are concatenated into a 16-bit raw value (top 5 bits zero for the
// 11-bit TSIC 306).
//
// Wire ordering on each packet (MSB-first, 10 bits total):
//     [ start(0) | d7 d6 d5 d4 d3 d2 d1 d0 | parity_even ]
//
// raw = (packet1_byte << 8) | packet2_byte
//
// The decoder (components/tsic306) then masks to 11 bits and applies the
// transfer function.

#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "hal_cpp/i_zacwire_capture.hpp"
#include "hal_cpp/result.hpp"

namespace espid::tsic306 {

class Tsic306Driver final : public hal_cpp::IZacwireCapture {
   public:
    struct Config {
        gpio_num_t gpio_num;          // ZACwire data line.
        int        intr_priority = 0; // 0 = let driver pick (low priority).
    };

    Tsic306Driver()                                  = default;
    Tsic306Driver(const Tsic306Driver&)              = delete;
    Tsic306Driver& operator=(const Tsic306Driver&)   = delete;
    Tsic306Driver(Tsic306Driver&&)                   = delete;
    Tsic306Driver& operator=(Tsic306Driver&&)        = delete;
    ~Tsic306Driver() override;

    // One-shot init. Allocates the RMT channel, queues, callbacks, and starts
    // receiving. Safe to call once; subsequent calls return kInvalidState.
    hal_cpp::Status init(const Config& cfg) noexcept;

    // IZacwireCapture
    hal_cpp::Result<hal_cpp::ZacwireFrame> next_frame(std::uint32_t timeout_ms) noexcept override;
    hal_cpp::Status                        flush() noexcept override;

   private:
    static bool IRAM_ATTR on_recv_done_(rmt_channel_handle_t              chan,
                                        const rmt_rx_done_event_data_t*   edata,
                                        void*                             user_ctx);

    hal_cpp::Status start_receive_() noexcept;

    // RMT resolution = 1 MHz (1 tick = 1 µs).
    static constexpr std::uint32_t kResolutionHz       = 1'000'000;
    // Glitch filter: anything shorter than 1 µs is noise.
    static constexpr std::uint32_t kSignalRangeMinNs   = 1'000;
    // End-of-frame if line is idle longer than 1 ms.
    static constexpr std::uint32_t kSignalRangeMaxNs   = 1'000'000;
    // Max symbols per receive — TSIC frame is 20 symbols, leave headroom.
    static constexpr std::size_t   kSymbolBufferLen    = 64;
    // RMT memory block size (symbols); must be even.
    static constexpr std::size_t   kRmtMemBlockSymbols = 64;

    rmt_channel_handle_t chan_{nullptr};
    QueueHandle_t        done_queue_{nullptr};       // payload: std::size_t (num_symbols)
    rmt_symbol_word_t    sym_buf_[kSymbolBufferLen]{};
    bool                 initialised_{false};
};

}  // namespace espid::tsic306
