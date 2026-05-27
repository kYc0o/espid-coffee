#include "abp2/abp2_driver.hpp"

#include <cstring>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace espid::abp2 {

namespace {

constexpr const char* kTag = "abp2_drv";
constexpr std::uint8_t kReadMeasurementCmd[3] = {0xAAU, 0x00U, 0x00U};

hal_cpp::Error map_i2c_error(esp_err_t err) noexcept {
    if (err == ESP_ERR_TIMEOUT) {
        return hal_cpp::Error::kTimeout;
    }
    return hal_cpp::Error::kBusError;
}

}  // namespace

hal_cpp::Status Abp2Driver::init(const Config& cfg) noexcept {
    if (initialised_) {
        return hal_cpp::Error::kAlreadyInit;
    }
    if (cfg.i2c_port < -1 ||
        cfg.i2c_addr_7bit == 0 || cfg.i2c_addr_7bit > 0x7FU ||
        cfg.i2c_freq_hz == 0 || cfg.timeout_ms == 0 ||
        cfg.command_delay_ms == 0) {
        return hal_cpp::Error::kInvalidArg;
    }

    cfg_ = cfg;
    has_last_raw_ = false;

    const i2c_port_num_t port = static_cast<i2c_port_num_t>(cfg_.i2c_port);
    esp_err_t err = i2c_master_get_bus_handle(port, &bus_);
    if (err == ESP_OK) {
        owns_bus_ = false;
    } else if (err == ESP_ERR_INVALID_STATE) {
        i2c_master_bus_config_t bus_cfg{};
        bus_cfg.i2c_port = port;
        bus_cfg.sda_io_num = cfg_.sda;
        bus_cfg.scl_io_num = cfg_.scl;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.intr_priority = 0;
        bus_cfg.trans_queue_depth = 0;
        bus_cfg.flags.enable_internal_pullup = 1;
        bus_cfg.flags.allow_pd = 0;

        err = i2c_new_master_bus(&bus_cfg, &bus_);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
            return hal_cpp::Error::kBusError;
        }
        owns_bus_ = true;
    } else {
        ESP_LOGE(kTag, "i2c_master_get_bus_handle failed: %s", esp_err_to_name(err));
        return hal_cpp::Error::kBusError;
    }

    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = cfg_.i2c_addr_7bit;
    dev_cfg.scl_speed_hz = cfg_.i2c_freq_hz;
    dev_cfg.scl_wait_us = 0;
    dev_cfg.flags.disable_ack_check = 0;

    err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        if (owns_bus_) {
            (void)i2c_del_master_bus(bus_);
            owns_bus_ = false;
        }
        bus_ = nullptr;
        return hal_cpp::Error::kBusError;
    }

    initialised_ = true;
    ESP_LOGI(kTag, "ABP2 ready (i2c=%d addr=0x%02X sda=%d scl=%d freq=%u)",
             cfg_.i2c_port,
             static_cast<unsigned>(cfg_.i2c_addr_7bit),
             static_cast<int>(cfg_.sda),
             static_cast<int>(cfg_.scl),
             static_cast<unsigned>(cfg_.i2c_freq_hz));
    return hal_cpp::Status{};
}

hal_cpp::Result<DecodedSample> Abp2Driver::read_sample() noexcept {
    if (!initialised_) {
        return hal_cpp::Error::kNotInitialised;
    }

    std::uint8_t raw[7]{};

    esp_err_t err = i2c_master_transmit(
        dev_,
        kReadMeasurementCmd,
        sizeof(kReadMeasurementCmd),
        static_cast<int>(cfg_.timeout_ms));
    if (err != ESP_OK) {
        return map_i2c_error(err);
    }

    vTaskDelay(pdMS_TO_TICKS(cfg_.command_delay_ms));

    err = i2c_master_receive(
        dev_,
        raw,
        sizeof(raw),
        static_cast<int>(cfg_.timeout_ms));
    if (err != ESP_OK) {
        return map_i2c_error(err);
    }

    std::memcpy(last_raw_, raw, sizeof(raw));
    has_last_raw_ = true;

    return decode_frame(raw, cfg_.transfer);
}

bool Abp2Driver::last_raw_frame(std::uint8_t out[7]) const noexcept {
    if (out == nullptr || !has_last_raw_) {
        return false;
    }
    std::memcpy(out, last_raw_, sizeof(last_raw_));
    return true;
}

}  // namespace espid::abp2
