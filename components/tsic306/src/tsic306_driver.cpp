// ESPID-Coffee — TSIC 306 RMT driver implementation.

#include "tsic306/tsic306_driver.hpp"

#include <cstring>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

namespace espid::tsic306 {

namespace {

constexpr const char* kTag = "tsic306_drv";

// Decide bit value by comparing the LOW portion of one bit period against an
// adaptive threshold derived from the start bit. We deliberately do NOT use
// the symbol's HIGH portion: the last symbol of each packet has its
// `duration1` inflated by either the inter-packet idle (packet 1) or the
// end-of-frame idle (packet 2), which flips the naïve low-vs-high decision
// roughly 50 % of the time and would manifest as a parity failure.
//
// ZACwire timing (TSIC 306, t_strobe ≈ 31 µs, bit period = 4·t_strobe):
//   '0' bit: LOW for ~3·t_strobe (≈93 µs)
//   '1' bit: LOW for ~1·t_strobe (≈31 µs)
//
// The start bit is always '0', so its `duration0` is the long-low reference;
// `2/3 × duration0` sits midway between the '0' and '1' low-times and is a
// robust per-packet threshold.
inline std::uint8_t bit_from_low(std::uint16_t low, std::uint16_t threshold) noexcept {
    return (low < threshold) ? 1U : 0U;
}

// Parse 10 consecutive RMT symbols (start + 8 data + parity, MSB-first).
// Returns the 8 data bits as a byte, or -1 if start bit fails its sanity
// check or even parity is wrong.
int parse_packet(const rmt_symbol_word_t* syms) noexcept {
    const std::uint16_t low_start = syms[0].duration0;
    // Reject garbage: start bit's low must be at least ~2·t_strobe (60 µs).
    if (low_start < 50U) {
        return -1;
    }
    const std::uint16_t threshold = static_cast<std::uint16_t>((low_start * 2U) / 3U);
    // Sanity-check that the start bit reads as '0' under the same rule
    // (low_start should be well above threshold by construction).
    if (bit_from_low(low_start, threshold) != 0U) {
        return -1;
    }
    std::uint8_t data   = 0;
    std::uint8_t parity = 0;
    for (int i = 0; i < 8; ++i) {
        const std::uint8_t b = bit_from_low(syms[1 + i].duration0, threshold);
        data    = static_cast<std::uint8_t>((data << 1) | b);
        parity ^= b;
    }
    const std::uint8_t parity_bit = bit_from_low(syms[9].duration0, threshold);
    if (parity_bit != parity) {
        return -1;  // even-parity mismatch
    }
    return data;
}

}  // namespace

Tsic306Driver::~Tsic306Driver() {
    if (chan_ != nullptr) {
        rmt_disable(chan_);
        rmt_del_channel(chan_);
        chan_ = nullptr;
    }
    if (done_queue_ != nullptr) {
        vQueueDelete(done_queue_);
        done_queue_ = nullptr;
    }
}

hal_cpp::Status Tsic306Driver::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Error::kAlreadyInit;
    }

    done_queue_ = xQueueCreate(2, sizeof(std::size_t));
    if (done_queue_ == nullptr) {
        return hal_cpp::Error::kHardwareFault;
    }

    rmt_rx_channel_config_t chan_cfg = {};
    chan_cfg.gpio_num                = cfg.gpio_num;
    chan_cfg.clk_src                 = RMT_CLK_SRC_DEFAULT;
    chan_cfg.resolution_hz           = kResolutionHz;
    chan_cfg.mem_block_symbols       = kRmtMemBlockSymbols;
    chan_cfg.intr_priority           = cfg.intr_priority;
    chan_cfg.flags.invert_in         = 0;
    chan_cfg.flags.with_dma          = 0;
    chan_cfg.flags.allow_pd          = 0;

    if (rmt_new_rx_channel(&chan_cfg, &chan_) != ESP_OK) {
        ESP_LOGE(kTag, "rmt_new_rx_channel failed");
        vQueueDelete(done_queue_);
        done_queue_ = nullptr;
        return hal_cpp::Error::kBusError;
    }

    const rmt_rx_event_callbacks_t cbs = { .on_recv_done = &Tsic306Driver::on_recv_done_ };
    if (rmt_rx_register_event_callbacks(chan_, &cbs, this) != ESP_OK) {
        ESP_LOGE(kTag, "rmt_rx_register_event_callbacks failed");
        rmt_del_channel(chan_);
        chan_ = nullptr;
        vQueueDelete(done_queue_);
        done_queue_ = nullptr;
        return hal_cpp::Error::kBusError;
    }

    if (rmt_enable(chan_) != ESP_OK) {
        ESP_LOGE(kTag, "rmt_enable failed");
        rmt_del_channel(chan_);
        chan_ = nullptr;
        vQueueDelete(done_queue_);
        done_queue_ = nullptr;
        return hal_cpp::Error::kBusError;
    }

    initialised_ = true;

    const hal_cpp::Status s = start_receive_();
    if (!s.ok()) {
        rmt_disable(chan_);
        rmt_del_channel(chan_);
        chan_ = nullptr;
        vQueueDelete(done_queue_);
        done_queue_  = nullptr;
        initialised_ = false;
        return s;
    }

    ESP_LOGI(kTag, "TSIC 306 driver initialised on GPIO %d", static_cast<int>(cfg.gpio_num));
    return hal_cpp::Status{};
}

hal_cpp::Status Tsic306Driver::start_receive_() noexcept {
    rmt_receive_config_t rcv_cfg  = {};
    rcv_cfg.signal_range_min_ns   = kSignalRangeMinNs;
    rcv_cfg.signal_range_max_ns   = kSignalRangeMaxNs;
    rcv_cfg.flags.en_partial_rx   = 0;

    const esp_err_t err = rmt_receive(chan_, sym_buf_, sizeof(sym_buf_), &rcv_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "rmt_receive failed: %d", err);
        return hal_cpp::Error::kBusError;
    }
    return hal_cpp::Status{};
}

bool IRAM_ATTR Tsic306Driver::on_recv_done_(rmt_channel_handle_t /*chan*/,
                                            const rmt_rx_done_event_data_t* edata,
                                            void*                           user_ctx) {
    auto*        self = static_cast<Tsic306Driver*>(user_ctx);
    BaseType_t   higher_woken = pdFALSE;
    const std::size_t n = edata->num_symbols;
    xQueueSendFromISR(self->done_queue_, &n, &higher_woken);
    return higher_woken == pdTRUE;
}

hal_cpp::Result<hal_cpp::ZacwireFrame> Tsic306Driver::next_frame(std::uint32_t timeout_ms) noexcept {
    if (!initialised_) {
        return hal_cpp::Error::kNotInitialised;
    }

    std::size_t num_symbols = 0;
    const TickType_t to     = pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(done_queue_, &num_symbols, to) != pdTRUE) {
        return hal_cpp::Error::kTimeout;
    }

    // Re-arm for the next frame regardless of parse outcome below.
    // We capture the parse first because parsing reads from sym_buf_ and
    // start_receive_() hands the buffer back to RMT.
    hal_cpp::ZacwireFrame frame{};
    bool                  frame_ok = false;
    bool                  short_frame = false;

    if (num_symbols >= 20U) {
        const int p1 = parse_packet(&sym_buf_[0]);
        const int p2 = parse_packet(&sym_buf_[10]);
        frame.timestamp_us = static_cast<std::uint64_t>(esp_timer_get_time());
        frame.raw          = static_cast<std::uint16_t>(
            ((p1 < 0 ? 0 : p1) << 8) | (p2 < 0 ? 0 : p2));
        frame.parity_mask  = static_cast<std::uint8_t>(((p1 >= 0) ? 0x1 : 0x0) |
                                                      ((p2 >= 0) ? 0x2 : 0x0));
        frame_ok           = true;
    } else {
        short_frame = true;
    }

    const hal_cpp::Status rs = start_receive_();
    if (!rs.ok()) {
        return rs.error();
    }
    if (short_frame) {
        ESP_LOGW(kTag, "short frame: %u symbols", static_cast<unsigned>(num_symbols));
        return hal_cpp::Error::kBusError;
    }
    if (!frame_ok) {
        return hal_cpp::Error::kBusError;
    }
    return frame;
}

hal_cpp::Status Tsic306Driver::flush() noexcept {
    if (!initialised_) {
        return hal_cpp::Error::kNotInitialised;
    }
    // Drain pending notifications. The buffer ownership flips back to us as
    // soon as the queue is empty.
    std::size_t scratch = 0;
    while (xQueueReceive(done_queue_, &scratch, 0) == pdTRUE) {
        // discard
    }
    return hal_cpp::Status{};
}

}  // namespace espid::tsic306
