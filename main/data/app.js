(function () {
  'use strict';

  // --- Show / hide password toggle ---
  var pwd   = document.getElementById('pwd');
  var showBtn = document.getElementById('show-pw');
  showBtn.addEventListener('click', function () {
    var visible = pwd.type === 'text';
    pwd.type        = visible ? 'password' : 'text';
    showBtn.textContent = visible ? 'Show' : 'Hide';
  });

  // --- Pre-populate form fields from /api/settings ---
  var FIELDS = [
    'pid_kp', 'pid_tn', 'pid_tv',
    'pump_kp', 'pump_ki', 'pump_kd',
    'brew_sp', 'steam_sp',
    'brew_t', 'preinf_t', 'preinf_pause_t',
    'mqtt_host', 'mqtt_port',
    'mqtt_user', 'mqtt_pass', 'mqtt_client'
  ];

  fetch('/api/settings')
    .then(function (r) {
      if (!r.ok) { throw new Error('HTTP ' + r.status); }
      return r.json();
    })
    .then(function (cfg) {
      // Show which AP we are connected to
      var apEl = document.getElementById('ap-ssid');
      if (apEl && cfg.ap_ssid) { apEl.textContent = cfg.ap_ssid; }

      // Hint the SSID field with the currently saved STA SSID
      var ssidEl = document.getElementById('ssid');
      if (ssidEl && cfg.wifi_ssid) {
        ssidEl.placeholder = 'current: ' + cfg.wifi_ssid;
      }

      // Populate all numeric / text settings
      FIELDS.forEach(function (f) {
        var el = document.getElementById(f);
        if (el && cfg[f] !== undefined && cfg[f] !== '') {
          el.value = cfg[f];
        }
      });
    })
    .catch(function (e) {
      console.warn('Failed to load settings from device:', e);
    });
}());
