(() => {
  "use strict";

  const THEME_KEY = "health-station-theme";
  const METRIC_META = {
    spo2_pct: { label: "Oxigenación (SpO2)", unit: "%", range: "Normal ≥ 95%" },
    bpm: { label: "Pulso Cardíaco", unit: "bpm", range: "Normal 50–100 bpm" },
    air_quality_ppm: { label: "Calidad Aire / CO", unit: "ppm", range: "Normal ≤ 10 ppm" },
    temperature_c: { label: "Temperatura", unit: "°C", range: "Confort 18–28 °C" },
    humidity_pct: { label: "Humedad", unit: "% RH", range: "Ideal 30–60%" },
  };

  let csrfToken = "";
  let devices = [];
  let selectedDeviceId = null;
  let historyMinutes = 60;
  let selectedMetric = "spo2_pct";
  let historyPoints = [];
  let includeAcked = false;

  const esc = (str) =>
    String(str ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");

  // Theme management (follows OS prefers-color-scheme unless overridden)
  const sysDark = window.matchMedia ? window.matchMedia("(prefers-color-scheme: dark)") : null;
  const applyTheme = (dark) => {
    if (dark) document.documentElement.setAttribute("data-theme", "dark");
    else document.documentElement.removeAttribute("data-theme");
    const btn = document.getElementById("theme-toggle");
    if (btn) btn.textContent = dark ? "Claro" : "Oscuro";
  };

  let storedTheme = null;
  try { storedTheme = localStorage.getItem(THEME_KEY); } catch (_) {}
  let darkMode = storedTheme ? storedTheme === "dark" : Boolean(sysDark && sysDark.matches);
  applyTheme(darkMode);
  sysDark?.addEventListener?.("change", (ev) => {
    try { if (localStorage.getItem(THEME_KEY)) return; } catch (_) {}
    darkMode = ev.matches;
    applyTheme(darkMode);
  });

  document.getElementById("theme-toggle")?.addEventListener("click", () => {
    darkMode = document.documentElement.getAttribute("data-theme") !== "dark";
    applyTheme(darkMode);
    try {
      if (sysDark && darkMode === sysDark.matches) localStorage.removeItem(THEME_KEY);
      else localStorage.setItem(THEME_KEY, darkMode ? "dark" : "light");
    } catch (_) {}
  });

  const sevPillClass = (sev) => {
    if (sev === "critical") return "pill-crit";
    if (sev === "warning") return "pill-warn";
    return "pill-ok";
  };

  const sevLabel = (sev) => {
    if (sev === "critical") return "CRÍTICO";
    if (sev === "warning") return "ALERTA";
    return "NORMAL";
  };

  const statusPillClass = (st) => {
    if (st === "online") return "pill-ok";
    if (st === "stale") return "pill-warn";
    return "pill-crit";
  };

  const formatTime = (iso) => {
    if (!iso) return "—";
    const d = new Date(iso);
    return Number.isNaN(d.getTime()) ? iso : d.toLocaleTimeString();
  };

  const buildSparklineSvg = (vals, strokeColor) => {
    if (!vals || vals.length < 2) {
      return '<svg class="sparkline" viewBox="0 0 200 36"></svg>';
    }
    const min = Math.min(...vals);
    const max = Math.max(...vals);
    const span = max - min || 1;
    const coords = vals.map((v, i) => {
      const x = (i / (vals.length - 1)) * 196 + 2;
      const y = 32 - ((v - min) / span) * 28;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    });
    return `<svg class="sparkline" viewBox="0 0 200 36" preserveAspectRatio="none">
      <polyline fill="none" stroke="${strokeColor}" stroke-width="2" points="${coords.join(" ")}" />
    </svg>`;
  };

  const renderSnapshot = (snap) => {
    devices = snap.devices || [];
    if (!selectedDeviceId && devices.length > 0) {
      selectedDeviceId = devices[0].device_id;
    }
    if (selectedDeviceId && !devices.some((d) => d.device_id === selectedDeviceId) && devices.length > 0) {
      selectedDeviceId = devices[0].device_id;
    }

    const totals = snap.totals || {};
    const totalsEl = document.getElementById("totals-summary");
    if (totalsEl) {
      totalsEl.textContent = `Online: ${totals.online ?? 0}/${totals.devices ?? 0} · Alertas: ${totals.active_alerts ?? 0} · DLQ: ${totals.dlq_rows ?? 0}`;
    }

    const tabsEl = document.getElementById("station-tabs");
    if (tabsEl) {
      tabsEl.innerHTML = devices.length
        ? devices
            .map((d) => {
              const sel = d.device_id === selectedDeviceId ? "is-selected" : "";
              const badge = d.active_alerts > 0 ? `<span class="pill pill-crit">${d.active_alerts}</span>` : "";
              return `<button class="station-tab ${sel}" data-device="${esc(d.device_id)}" type="button">
                <span class="dot ${esc(d.status)}"></span>
                <span>${esc(d.device_id)}</span>
                ${badge}
              </button>`;
            })
            .join("")
        : '<span class="meta-detail">Esperando telemetría de estaciones…</span>';
    }

    renderActiveStation();
  };

  const renderActiveStation = () => {
    const dev = devices.find((d) => d.device_id === selectedDeviceId);
    const nameEl = document.getElementById("active-station-name");
    const statusEl = document.getElementById("active-station-status");
    const ageEl = document.getElementById("active-station-age");
    const healthEl = document.getElementById("sensor-health");
    const gridEl = document.getElementById("metrics-grid");

    if (!dev) {
      if (nameEl) nameEl.textContent = "Sin estación seleccionada";
      if (gridEl) gridEl.innerHTML = "";
      return;
    }

    if (nameEl) nameEl.textContent = dev.device_id;
    if (statusEl) {
      statusEl.className = `pill ${statusPillClass(dev.status)}`;
      statusEl.textContent = dev.status.toUpperCase();
    }
    if (ageEl) {
      ageEl.textContent = `Último paquete: hace ${dev.age_sec}s · seq #${dev.last_seq ?? "—"}`;
    }

    if (healthEl) {
      const ss = dev.sensor_status || {};
      const chips = [
        ["MAX30102", ss.max30102],
        ["DHT11", ss.dht11],
        ["MQ-135", ss.mq135],
      ];
      healthEl.innerHTML = chips
        .map(([label, ok]) => {
          const cls = ok === true ? "pill-ok" : ok === false ? "pill-crit" : "pill-muted";
          const state = ok === true ? "OK" : ok === false ? "FALLA" : "N/A";
          return `<span class="pill ${cls}">${label}: ${state}</span>`;
        })
        .join("");
    }

    const sqiMetric = dev.metrics?.signal_quality;
    const sqiText = sqiMetric ? `SQI ${(sqiMetric.value * 100).toFixed(0)}%` : "SQI —";

    if (gridEl) {
      gridEl.innerHTML = Object.entries(METRIC_META)
        .map(([key, meta]) => {
          const m = dev.metrics?.[key];
          const val = m ? Number(m.value) : null;
          const sev = m ? m.severity : "normal";
          const sparkSeries = historyPoints
            .map((pt) => pt[key])
            .filter((v) => typeof v === "number")
            .slice(-24);
          const stroke = sev === "critical" ? "var(--crit)" : sev === "warning" ? "var(--warn)" : "var(--accent)";
          const footLeft = key === "bpm" || key === "spo2_pct" ? sqiText : meta.range;
          const footRight = m ? formatTime(m.updated_at) : "—";

          return `<article class="metric-card sev-${esc(sev)}">
            <div class="metric-top">
              <span class="metric-label">${esc(meta.label)}</span>
              <span class="pill ${sevPillClass(sev)}">${sevLabel(sev)}</span>
            </div>
            <div class="metric-value-row">
              <span class="metric-value num">${val !== null ? val.toFixed(key === "bpm" ? 0 : 1) : "—"}</span>
              <span class="metric-unit">${esc(meta.unit)}</span>
            </div>
            ${buildSparklineSvg(sparkSeries, stroke)}
            <div class="metric-foot">
              <span>${esc(footLeft)}</span>
              <span class="num">${esc(footRight)}</span>
            </div>
          </article>`;
        })
        .join("");
    }
  };

  const renderHistoryChart = () => {
    const svg = document.getElementById("history-chart");
    const readout = document.getElementById("chart-readout");
    if (!svg) return;

    const series = historyPoints
      .map((p) => ({ ts: p.recorded_at, val: p[selectedMetric] }))
      .filter((p) => typeof p.val === "number");

    if (series.length < 2) {
      svg.innerHTML = '<text x="50%" y="50%" text-anchor="middle" fill="currentColor" opacity="0.5" font-size="13">Sin suficientes puntos en esta ventana</text>';
      if (readout) readout.textContent = "";
      return;
    }

    const w = 760, h = 240, padX = 48, padY = 22;
    const min = Math.min(...series.map((s) => s.val));
    const max = Math.max(...series.map((s) => s.val));
    const span = max - min || 1;

    const coords = series.map((s, i) => {
      const x = padX + (i / (series.length - 1)) * (w - padX * 1.4);
      const y = h - padY - ((s.val - min) / span) * (h - padY * 2);
      return { x, y, ...s };
    });

    const linePts = coords.map((c) => `${c.x.toFixed(1)},${c.y.toFixed(1)}`).join(" ");
    const unit = METRIC_META[selectedMetric]?.unit || "";

    svg.innerHTML = `
      <line x1="${padX}" y1="${padY}" x2="${padX}" y2="${h - padY}" stroke="var(--border)" />
      <line x1="${padX}" y1="${h - padY}" x2="${w - 16}" y2="${h - padY}" stroke="var(--border)" />
      <text x="6" y="${padY + 6}" fill="var(--muted)" font-size="11" class="num">${max.toFixed(1)}</text>
      <text x="6" y="${h - padY}" fill="var(--muted)" font-size="11" class="num">${min.toFixed(1)}</text>
      <polyline fill="none" stroke="var(--accent)" stroke-width="2.5" points="${linePts}" />
      ${coords
        .map(
          (c, idx) =>
            `<circle cx="${c.x.toFixed(1)}" cy="${c.y.toFixed(1)}" r="3.5" fill="var(--accent)" data-idx="${idx}" />`
        )
        .join("")}
    `;

    const last = series[series.length - 1];
    if (readout) {
      readout.textContent = `${series.length} muestras · Último: ${last.val.toFixed(1)} ${unit} (${formatTime(last.ts)}) · Mín: ${min.toFixed(1)} · Máx: ${max.toFixed(1)}`;
    }
  };

  const renderAlerts = (alerts) => {
    const listEl = document.getElementById("alerts-list");
    if (!listEl) return;
    if (!alerts || alerts.length === 0) {
      listEl.innerHTML = '<p class="meta-detail">Sin alertas pendientes.</p>';
      return;
    }

    listEl.innerHTML = alerts
      .map((a) => {
        const ackCls = a.acknowledged ? "is-acked" : "";
        const btn = a.acknowledged
          ? `<span class="pill pill-muted">Atendida</span>`
          : `<button class="btn" data-ack-id="${a.id}" type="button">Atender</button>`;
        return `<div class="alert-item sev-${esc(a.severity)} ${ackCls}">
          <div class="alert-head">
            <span class="pill ${sevPillClass(a.severity)}">${sevLabel(a.severity)} · ${esc(a.device_id)}</span>
            ${btn}
          </div>
          <div class="alert-msg">${esc(a.message)}</div>
          <div class="alert-meta num">
            <span>${esc(a.code)}</span>
            <span>${esc(formatTime(a.created_at))}</span>
          </div>
        </div>`;
      })
      .join("");
  };

  const refreshHistory = async () => {
    if (!selectedDeviceId) return;
    const res = await fetch(`/api/devices/${encodeURIComponent(selectedDeviceId)}/history?minutes=${historyMinutes}`);
    if (!res.ok) return;
    const data = await res.json();
    historyPoints = data.points || [];
    renderActiveStation();
    renderHistoryChart();
  };

  const refreshAlerts = async () => {
    const res = await fetch(`/api/alerts?include_acked=${includeAcked ? "true" : "false"}`);
    if (!res.ok) return;
    const data = await res.json();
    renderAlerts(data.alerts || []);
  };

  const refreshAll = async () => {
    const snapRes = await fetch("/api/snapshot");
    if (snapRes.ok) {
      renderSnapshot(await snapRes.json());
    }
    await Promise.all([refreshHistory(), refreshAlerts()]);
  };

  const acknowledgeAlert = async (alertId) => {
    const idemKey = `ack-${alertId}-${Date.now()}`;
    const res = await fetch(`/api/alerts/${alertId}/ack`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-CSRF-Token": csrfToken,
        "Idempotency-Key": idemKey,
      },
      body: JSON.stringify({ actor: "dashboard-operator" }),
    });
    if (res.ok) {
      await refreshAlerts();
      await refreshAll();
    }
  };

  // Event delegation (CSP-safe: zero inline handlers)
  document.getElementById("station-tabs")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-device]");
    if (!btn) return;
    selectedDeviceId = btn.getAttribute("data-device");
    renderSnapshot({ devices, totals: { devices: devices.length, online: devices.filter((d) => d.status === "online").length } });
    refreshAll();
  });

  document.getElementById("metric-select")?.addEventListener("change", (ev) => {
    selectedMetric = ev.target.value;
    renderHistoryChart();
  });

  document.querySelector(".window-group")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-minutes]");
    if (!btn) return;
    historyMinutes = Number(btn.getAttribute("data-minutes")) || 60;
    document.querySelectorAll(".window-group .btn-tab").forEach((b) => b.classList.remove("is-active"));
    btn.classList.add("is-active");
    refreshHistory();
  });

  document.getElementById("show-acked")?.addEventListener("change", (ev) => {
    includeAcked = Boolean(ev.target.checked);
    refreshAlerts();
  });

  document.getElementById("alerts-list")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-ack-id]");
    if (!btn) return;
    const id = Number(btn.getAttribute("data-ack-id"));
    if (id > 0) acknowledgeAlert(id);
  });

  const connectSSE = () => {
    const badge = document.getElementById("conn-badge");
    const es = new EventSource("/api/stream");
    es.addEventListener("open", () => {
      if (badge) {
        badge.className = "pill pill-ok";
        badge.textContent = "SSE en vivo";
      }
    });
    es.addEventListener("snapshot", (ev) => {
      renderSnapshot(JSON.parse(ev.data));
      refreshHistory();
    });
    es.addEventListener("telemetry", () => {
      refreshAll();
    });
    es.addEventListener("alert", () => {
      refreshAlerts();
    });
    es.addEventListener("alert_ack", () => {
      refreshAlerts();
    });
    es.addEventListener("status", () => {
      refreshAll();
    });
    es.addEventListener("error", () => {
      if (badge) {
        badge.className = "pill pill-warn";
        badge.textContent = "Reconectando…";
      }
    });
  };

  const boot = async () => {
    try {
      const sessRes = await fetch("/api/session");
      if (sessRes.ok) {
        const sess = await sessRes.json();
        csrfToken = sess.csrf_token || "";
        const disc = document.getElementById("disclaimer-text");
        if (disc && sess.disclaimer) disc.textContent = sess.disclaimer;
        const demoBadge = document.getElementById("demo-badge");
        if (demoBadge && sess.demo_mode) demoBadge.classList.remove("hidden");
      }
    } catch (_) {}

    await refreshAll();
    connectSSE();
    setInterval(refreshAll, 15000);
  };

  document.addEventListener("DOMContentLoaded", boot);
})();
