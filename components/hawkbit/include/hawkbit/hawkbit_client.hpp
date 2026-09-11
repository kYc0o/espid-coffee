// HawkBit DDI client — polls for firmware updates and applies them.
//
// Implements the minimal HawkBit Direct Device Integration (DDI) API:
//   GET  /{tenant}/controller/v1/{device}           — poll for actions
//   GET  /{tenant}/controller/v1/{device}/deploymentBase/{id}
//   POST /{tenant}/controller/v1/{device}/deploymentBase/{id}/feedback
//   HTTP GET artifact download with Authorization: TargetToken header
//
// Threading: drive from a single low-priority task; not thread-safe.

#pragma once

namespace espid::hawkbit {

struct Config {
    const char* server_url;     // e.g. "https://hawkbit.example.com"
    const char* tenant;         // usually "DEFAULT"
    const char* device_id;      // unique per device, e.g. MQTT client ID
    const char* security_token; // HawkBit target security token (may be "")
};

// Poll HawkBit for a pending deployment and apply it if found.
// Returns true when firmware was flashed; caller should reboot.
bool poll_and_apply(const Config& cfg) noexcept;

}  // namespace espid::hawkbit
