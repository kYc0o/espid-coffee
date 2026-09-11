// HawkBit DDI client implementation.

#include "hawkbit/hawkbit_client.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "esp_heap_caps.h"

namespace espid::hawkbit {

namespace {

constexpr const char* kTag       = "hawkbit";
constexpr std::size_t kJsonBuf   = 4096;
constexpr std::size_t kChunkSize = 4096;

// Scan forward from |start| for the first JSON string value whose key
// matches |key|.  Extracts the value into |dst| (NUL-terminated).
// Very minimal: handles only "key":"value" and "key": "value" patterns.
// Returns true on success.
bool json_str_after_key(const char* start, const char* key,
                        char* dst, std::size_t dst_size) noexcept {
    char pat[64];
    std::snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(start, pat);
    if (!p) return false;
    p += std::strlen(pat);
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    if (*p != '"') return false;
    ++p;
    std::size_t i = 0;
    while (*p && *p != '"' && i < dst_size - 1) {
        if (*p == '\\' && *(p + 1)) { ++p; }  // skip escape
        dst[i++] = *p++;
    }
    dst[i] = '\0';
    return (*p == '"') && (i > 0);
}

void set_auth(esp_http_client_handle_t c, const char* token) noexcept {
    if (!token || token[0] == '\0') {
        return;
    }
    char hdr[128];
    std::snprintf(hdr, sizeof(hdr), "TargetToken %s", token);
    esp_http_client_set_header(c, "Authorization", hdr);
}

// Perform a GET and read the response body into |buf| (NUL-terminated).
// Returns HTTP status code, or <0 on transport error.
// Uses the manual open/fetch_headers/read/close flow — perform() discards
// the body when no event_handler is registered.
int http_get(const char* url, const char* token,
             char* buf, std::size_t buf_size) noexcept {
    esp_http_client_config_t cfg{};
    cfg.url               = url;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return -1;
    }
    set_auth(client, token);

    buf[0] = '\0';
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "http_get open: %d", static_cast<int>(err));
        esp_http_client_cleanup(client);
        return -1;
    }

    esp_http_client_fetch_headers(client);
    const int status = esp_http_client_get_status_code(client);

    if (status == 200) {
        const int rd = esp_http_client_read_response(
            client, buf, static_cast<int>(buf_size) - 1);
        buf[rd > 0 ? rd : 0] = '\0';
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

// POST a JSON body; returns HTTP status code or <0 on error.
int http_post_json(const char* url, const char* token,
                   const char* body) noexcept {
    esp_http_client_config_t cfg{};
    cfg.url               = url;
    cfg.method            = HTTP_METHOD_POST;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return -1;
    }
    set_auth(client, token);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, static_cast<int>(std::strlen(body)));

    const esp_err_t err = esp_http_client_perform(client);
    const int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return (err == ESP_OK) ? status : -1;
}

// Download artifact and flash it to the next OTA partition.
// Uses a manual streaming download so we can set the Authorization header.
esp_err_t download_and_flash(const char* url, const char* token) noexcept {
    const esp_partition_t* update_part =
        esp_ota_get_next_update_partition(nullptr);
    if (!update_part) {
        ESP_LOGE(kTag, "no OTA partition available");
        return ESP_FAIL;
    }
    ESP_LOGI(kTag, "target partition: %s", update_part->label);

    esp_http_client_config_t cfg{};
    cfg.url               = url;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.buffer_size       = static_cast<int>(kChunkSize);

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        return ESP_ERR_NO_MEM;
    }
    set_auth(client, token);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "HTTP open failed: %d", static_cast<int>(err));
        esp_http_client_cleanup(client);
        return err;
    }

    const std::int64_t content_len = esp_http_client_fetch_headers(client);
    ESP_LOGI(kTag, "firmware size: %" PRId64 " bytes", content_len);

    esp_ota_handle_t ota = 0;
    err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return err;
    }

    char* chunk = static_cast<char*>(heap_caps_malloc(kChunkSize, MALLOC_CAP_DEFAULT));
    if (!chunk) {
        esp_ota_abort(ota);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    int total = 0;
    int rd    = 0;
    bool ok   = true;
    while ((rd = esp_http_client_read(client, chunk, static_cast<int>(kChunkSize))) > 0) {
        if (esp_ota_write(ota, chunk, static_cast<std::size_t>(rd)) != ESP_OK) {
            ok = false;
            break;
        }
        total += rd;
        if ((total & 0xFFFF) == 0) {
            ESP_LOGI(kTag, "flashing: %d KB", total / 1024);
        }
    }

    heap_caps_free(chunk);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (!ok || rd < 0) {
        esp_ota_abort(ota);
        return ESP_FAIL;
    }

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_ota_end failed: %d", static_cast<int>(err));
        return err;
    }

    ESP_LOGI(kTag, "flashed %d bytes, activating partition", total);
    return esp_ota_set_boot_partition(update_part);
}

void send_feedback(const char* dep_url_base, const char* token,
                   const char* action_id, bool success) noexcept {
    char fb_url[256];
    std::snprintf(fb_url, sizeof(fb_url), "%s/feedback", dep_url_base);

    char body[256];
    std::snprintf(body, sizeof(body),
        "{\"id\":\"%s\",\"status\":{"
        "\"execution\":\"closed\","
        "\"result\":{\"finished\":\"%s\"},"
        "\"details\":[\"%s\"]}}",
        action_id,
        success ? "success" : "failure",
        success ? "Firmware applied" : "Flash failed");

    const int st = http_post_json(fb_url, token, body);
    if (st != 200 && st != 201) {
        ESP_LOGW(kTag, "feedback POST returned %d", st);
    } else {
        ESP_LOGI(kTag, "feedback sent: %s", success ? "success" : "failure");
    }
}

}  // namespace

bool poll_and_apply(const Config& cfg) noexcept {
    if (!cfg.server_url || cfg.server_url[0] == '\0') {
        ESP_LOGW(kTag, "server URL not configured");
        return false;
    }

    // 1 — Poll base URL.
    char base_url[256];
    std::snprintf(base_url, sizeof(base_url), "%s/%s/controller/v1/%s",
                  cfg.server_url, cfg.tenant ? cfg.tenant : "DEFAULT",
                  cfg.device_id ? cfg.device_id : "unknown");

    ESP_LOGI(kTag, "polling %s", base_url);

    char* json = static_cast<char*>(heap_caps_malloc(kJsonBuf, MALLOC_CAP_DEFAULT));
    if (!json) {
        return false;
    }

    const int status = http_get(base_url, cfg.security_token, json, kJsonBuf);
    if (status != 200) {
        ESP_LOGW(kTag, "poll returned %d", status);
        heap_caps_free(json);
        return false;
    }

    // 2 — Check for deploymentBase link.
    // Extract the href string value that follows the "deploymentBase" key.
    // HawkBit poll response: {"_links":{"deploymentBase":{"href":"http://..."}}}
    char dep_url[256];
    {
        const char* p = strstr(json, "\"deploymentBase\"");
        if (!p || !json_str_after_key(p, "href", dep_url, sizeof(dep_url))) {
            ESP_LOGI(kTag, "no pending deployment");
            heap_caps_free(json);
            return false;
        }
    }

    ESP_LOGI(kTag, "deployment: %s", dep_url);

    // 3 — GET deployment details.
    const int ds = http_get(dep_url, cfg.security_token, json, kJsonBuf);
    if (ds != 200) {
        ESP_LOGW(kTag, "deployment GET returned %d", ds);
        heap_caps_free(json);
        return false;
    }

    // 4 — Extract action ID (last path segment of dep_url) and download URL.
    // Download URL follows the "download-http" key (plain HTTP, no TLS cert needed).
    // Fall back to "download" key (HTTPS) if not found.
    char action_id[32];
    char dl_url[256];
    // dep_url_base strips any "?c=..." query string for feedback URL construction.
    char dep_url_base[256];
    {
        std::strncpy(dep_url_base, dep_url, sizeof(dep_url_base) - 1);
        dep_url_base[sizeof(dep_url_base) - 1] = '\0';
        char* q = std::strchr(dep_url_base, '?');
        if (q) *q = '\0';
    }
    {
        // action_id = last '/' segment of dep_url_base (query string already stripped)
        const char* last_slash = std::strrchr(dep_url_base, '/');
        const char* id_src     = last_slash ? last_slash + 1 : dep_url_base;
        std::strncpy(action_id, id_src, sizeof(action_id) - 1);
        action_id[sizeof(action_id) - 1] = '\0';

        // download URL: try download-http first, then download
        const char* p = strstr(json, "\"download-http\"");
        if (!p) p = strstr(json, "\"download\"");
        if (!p || !json_str_after_key(p, "href", dl_url, sizeof(dl_url))) {
            ESP_LOGE(kTag, "deployment payload: no download URL");
            heap_caps_free(json);
            return false;
        }
    }
    heap_caps_free(json);

    ESP_LOGI(kTag, "action=%s  download=%s", action_id, dl_url);

    // 5 — Download and flash.
    const esp_err_t ota_err = download_and_flash(dl_url, cfg.security_token);
    const bool ok = (ota_err == ESP_OK);
    ESP_LOGI(kTag, "OTA %s", ok ? "SUCCESS — rebooting" : "FAILED");

    send_feedback(dep_url_base, cfg.security_token, action_id, ok);

    return ok;
}

}  // namespace espid::hawkbit
