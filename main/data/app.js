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
    'brew_kp', 'brew_tn', 'brew_tv', 'brew_trail', 'brew_pulse',
    'pump_kp', 'pump_ki', 'pump_kd',
    'brew_sp', 'steam_sp',
    'brew_t', 'preinf_t', 'preinf_pause_t',
    'mqtt_host', 'mqtt_port',
    'mqtt_user', 'mqtt_pass', 'mqtt_client',
    'hb_server', 'hb_token'
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

  // --- Standby / wake schedule from /api/power ---
  function text(id, value) {
    var el = document.getElementById(id);
    if (el) { el.textContent = value; }
  }

  fetch('/api/power')
    .then(function (r) {
      if (!r.ok) { throw new Error('HTTP ' + r.status); }
      return r.json();
    })
    .then(function (p) {
      ['sb_en', 'sb_min', 'wk_en', 'wk_time'].forEach(function (f) {
        var el = document.getElementById(f);
        if (el && p[f] !== undefined) { el.value = p[f]; }
      });
      text('pw-clock', p.clock);
      text('pw-next', p.next_wake);
      text('pw-left', p.sb_en === '1'
        ? Math.ceil(p.sleep_in_s / 60) + ' min'
        : 'off');
    })
    .catch(function (e) {
      console.warn('Failed to load standby settings from device:', e);
    });
}());
