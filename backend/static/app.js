(() => {
  "use strict";

  const THEME_KEY = "health-station-theme";
  const METRIC_META = {
    spo2_pct: {
      label: "SpO2",
      unit: "%",
      range: "Normal ≥ 95%",
      thresholds: [
        { val: 95.0, label: "95% min", color: "var(--warn)" },
        { val: 90.0, label: "90% crit", color: "var(--crit)" },
      ],
    },
    bpm: {
      label: "Heart Rate",
      unit: "bpm",
      range: "Normal 50–100",
      thresholds: [
        { val: 100.0, label: "100 bpm", color: "var(--warn)" },
        { val: 120.0, label: "120 crit", color: "var(--crit)" },
      ],
    },
    air_quality_ppm: {
      label: "Air / CO",
      unit: "ppm",
      range: "Normal ≤ 10 ppm",
      thresholds: [
        { val: 10.0, label: "10 ppm", color: "var(--warn)" },
        { val: 35.0, label: "35 crit", color: "var(--crit)" },
      ],
    },
    temperature_c: {
      label: "Temperature",
      unit: "°C",
      range: "Comfort 18–28 °C",
      thresholds: [
        { val: 28.0, label: "28 °C", color: "var(--warn)" },
        { val: 18.0, label: "18 °C", color: "var(--warn)" },
      ],
    },
    humidity_pct: {
      label: "Humidity",
      unit: "% RH",
      range: "Ideal 30–60%",
      thresholds: [
        { val: 60.0, label: "60%", color: "var(--warn)" },
        { val: 30.0, label: "30%", color: "var(--warn)" },
      ],
    },
  };

  let csrfToken = "";
  let devices = [];
  let totals = {};
  let selectedDeviceId = null;
  let historyMinutes = 15;
  let selectedMetric = "spo2_pct";
  let historyPoints = [];
  let includeAcked = false;
  let filterCurrentStation = true;
  let chartCoords = [];
  let defaultReadoutText = "";

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
    if (btn) btn.textContent = dark ? "Light" : "Dark";
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
    if (sev === "critical") return "CRITICAL";
    if (sev === "warning") return "WARNING";
    return "NORMAL";
  };

  const statusPillClass = (st) => {
    if (st === "online") return "pill-ok";
    if (st === "stale") return "pill-warn";
    return "pill-crit";
  };

  const statusLabel = (st) => {
    if (st === "online") return "ONLINE";
    if (st === "stale") return "STALE";
    return "OFFLINE";
  };

  const formatTime = (iso, withSeconds = true) => {
    if (!iso) return "—";
    const d = new Date(iso);
    if (Number.isNaN(d.getTime())) return iso;
    return d.toLocaleTimeString([], {
      hour: "2-digit",
      minute: "2-digit",
      ...(withSeconds ? { second: "2-digit" } : {}),
      hour12: false,
    });
  };

  const buildSparklineSvg = (vals, strokeColor) => {
    if (!vals || vals.length < 2) {
      return '<svg class="sparkline" viewBox="0 0 200 30"></svg>';
    }
    const min = Math.min(...vals);
    const max = Math.max(...vals);
    const span = max - min || 1;
    const coords = vals.map((v, i) => {
      const x = (i / (vals.length - 1)) * 196 + 2;
      const y = 27 - ((v - min) / span) * 24;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    });
    return `<svg class="sparkline" viewBox="0 0 200 30" preserveAspectRatio="none">
      <polyline fill="none" stroke="${strokeColor}" stroke-width="1.75" points="${coords.join(" ")}" />
    </svg>`;
  };

  const renderTotals = () => {
    const totalsEl = document.getElementById("totals-summary");
    if (!totalsEl) return;
    const onlineCount = devices.filter((d) => d.status === "online").length;
    const activeAlerts = devices.reduce((acc, d) => acc + (d.active_alerts || 0), 0);
    totalsEl.textContent = `Online: ${onlineCount}/${devices.length} · Alerts: ${activeAlerts} · DLQ: ${totals.dlq_rows ?? 0}`;
  };

  const renderTabs = () => {
    const tabsEl = document.getElementById("station-tabs");
    if (!tabsEl) return;
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
      : '<span class="meta-detail">Waiting for station telemetry…</span>';
  };

  const renderSnapshot = (snap) => {
    devices = snap.devices || [];
    totals = snap.totals || {};
    if (!selectedDeviceId && devices.length > 0) {
      selectedDeviceId = devices[0].device_id;
    }
    if (selectedDeviceId && !devices.some((d) => d.device_id === selectedDeviceId) && devices.length > 0) {
      selectedDeviceId = devices[0].device_id;
    }
    renderTotals();
    renderTabs();
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
      if (nameEl) nameEl.textContent = "No station selected";
      if (gridEl) gridEl.innerHTML = "";
      return;
    }

    const isOnline = dev.status === "online";
    if (nameEl) nameEl.textContent = dev.device_id;
    if (statusEl) {
      statusEl.className = `pill ${statusPillClass(dev.status)}`;
      statusEl.textContent = statusLabel(dev.status);
    }
    if (ageEl) {
      ageEl.textContent = `Last packet: ${dev.age_sec ?? 0}s ago · seq #${dev.last_seq ?? "—"}`;
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
          if (!isOnline) {
            return `<span class="pill pill-muted">${label}: OFFLINE</span>`;
          }
          const cls = ok === true ? "pill-ok" : ok === false ? "pill-crit" : "pill-muted";
          const state = ok === true ? "OK" : ok === false ? "FAULT" : "N/A";
          return `<span class="pill ${cls}">${label}: ${state}</span>`;
        })
        .join("");
    }

    const sqiMetric = dev.metrics?.signal_quality;
    const sqiText = isOnline && sqiMetric ? `SQI ${(sqiMetric.value * 100).toFixed(0)}%` : "SQI —";

    if (gridEl) {
      gridEl.innerHTML = Object.entries(METRIC_META)
        .map(([key, meta]) => {
          const m = dev.metrics?.[key];
          const val = m ? Number(m.value) : null;
          const sev = isOnline ? (m ? m.severity : "normal") : "muted";
          const cardStateCls = dev.status === "offline" ? "is-offline" : dev.status === "stale" ? "is-stale" : "";
          const badgeCls = isOnline ? sevPillClass(sev) : "pill-muted";
          const badgeTxt = isOnline ? sevLabel(sev) : statusLabel(dev.status);

          const sparkSeries = historyPoints
            .map((pt) => pt[key])
            .filter((v) => typeof v === "number")
            .slice(-24);

          const stroke = !isOnline
            ? "var(--muted)"
            : sev === "critical"
            ? "var(--crit)"
            : sev === "warning"
            ? "var(--warn)"
            : "var(--accent)";
          const footLeft = key === "bpm" || key === "spo2_pct" ? sqiText : meta.range;
          const footRight = m ? formatTime(m.updated_at) : "—";

          return `<article class="metric-card sev-${esc(sev)} ${cardStateCls}">
            <div class="metric-top">
              <span class="metric-label">${esc(meta.label)}</span>
              <span class="pill ${badgeCls}">${badgeTxt}</span>
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
      chartCoords = [];
      svg.innerHTML = '<text x="50%" y="50%" text-anchor="middle" fill="var(--muted)" font-size="12">Insufficient samples in this window</text>';
      if (readout) readout.textContent = "";
      return;
    }

    const w = 760, h = 220, padL = 48, padR = 62, padT = 16, padB = 26;
    const plotW = w - padL - padR;
    const plotH = h - padT - padB;

    const meta = METRIC_META[selectedMetric] || { unit: "", thresholds: [] };
    const vals = series.map((s) => s.val);
    let min = Math.min(...vals);
    let max = Math.max(...vals);
    // Pad domain slightly so flat lines stay readable
    if (max - min < 1.0) {
      min -= 0.5;
      max += 0.5;
    }
    const span = max - min;

    const yFor = (v) => padT + plotH - ((v - min) / span) * plotH;

    chartCoords = series.map((s, i) => {
      const x = padL + (i / (series.length - 1)) * plotW;
      const y = yFor(s.val);
      return { x, y, ...s };
    });

    // 4 horizontal gridlines + Y ticks
    const gridLines = [];
    for (let g = 0; g <= 3; g++) {
      const frac = g / 3;
      const y = padT + plotH * (1 - frac);
      const tickVal = min + span * frac;
      gridLines.push(
        `<line x1="${padL}" y1="${y.toFixed(1)}" x2="${w - padR}" y2="${y.toFixed(1)}" stroke="var(--border)" stroke-width="1" />`,
        `<text x="${padL - 6}" y="${(y + 4).toFixed(1)}" text-anchor="end" fill="var(--muted)" font-size="10.5" class="num">${tickVal.toFixed(1)}</text>`
      );
    }

    // 4 X-axis time ticks
    const xTicks = [];
    const tickCount = Math.min(4, series.length);
    for (let t = 0; t < tickCount; t++) {
      const idx = Math.round((t / Math.max(1, tickCount - 1)) * (series.length - 1));
      const pt = chartCoords[idx];
      xTicks.push(
        `<text x="${pt.x.toFixed(1)}" y="${h - 7}" text-anchor="middle" fill="var(--muted)" font-size="10.5" class="num">${esc(formatTime(pt.ts, false))}</text>`
      );
    }

    // Clinical threshold reference lines (drawn when inside visible range)
    const refLines = [];
    for (const th of meta.thresholds || []) {
      if (th.val >= min && th.val <= max) {
        const ty = yFor(th.val);
        refLines.push(
          `<line x1="${padL}" y1="${ty.toFixed(1)}" x2="${w - padR}" y2="${ty.toFixed(1)}" stroke="${th.color}" stroke-width="1" stroke-dasharray="4 3" opacity="0.85" />`,
          `<text x="${w - padR + 4}" y="${(ty + 3).toFixed(1)}" fill="${th.color}" font-size="10" class="num">${esc(th.label)}</text>`
        );
      }
    }

    const linePts = chartCoords.map((c) => `${c.x.toFixed(1)},${c.y.toFixed(1)}`).join(" ");
    const baseLineY = (padT + plotH).toFixed(1);
    const areaPts = `${chartCoords[0].x.toFixed(1)},${baseLineY} ${linePts} ${chartCoords[chartCoords.length - 1].x.toFixed(1)},${baseLineY}`;

    // Only draw point markers when sparse (<= 25 points) so dense series stay crisp
    const markers = chartCoords.length <= 25
      ? chartCoords.map((c) => `<circle cx="${c.x.toFixed(1)}" cy="${c.y.toFixed(1)}" r="2.5" fill="var(--accent)" />`).join("")
      : "";

    svg.innerHTML = `
      ${gridLines.join("")}
      ${refLines.join("")}
      ${xTicks.join("")}
      <polygon fill="var(--accent)" opacity="0.10" points="${areaPts}" />
      <polyline fill="none" stroke="var(--accent)" stroke-width="2" stroke-linejoin="round" points="${linePts}" />
      ${markers}
      <line id="chart-crosshair" x1="0" y1="${padT}" x2="0" y2="${padT + plotH}" stroke="var(--ink)" stroke-width="1" stroke-dasharray="3 3" opacity="0" />
      <circle id="chart-focus-dot" cx="0" cy="0" r="4" fill="var(--accent)" stroke="var(--surface)" stroke-width="1.5" opacity="0" />
    `;

    const last = series[series.length - 1];
    defaultReadoutText = `${series.length} samples · Latest: ${last.val.toFixed(1)} ${meta.unit} (${formatTime(last.ts)}) · Min: ${Math.min(...vals).toFixed(1)} · Max: ${Math.max(...vals).toFixed(1)}`;
    if (readout) readout.textContent = defaultReadoutText;
  };

  const renderAlerts = (alerts) => {
    const listEl = document.getElementById("alerts-list");
    if (!listEl) return;
    const prevScroll = listEl.scrollTop;

    const visible = filterCurrentStation && selectedDeviceId
      ? (alerts || []).filter((a) => a.device_id === selectedDeviceId)
      : (alerts || []);

    if (visible.length === 0) {
      listEl.innerHTML = '<p class="meta-detail">No pending alerts in this view.</p>';
      return;
    }

    listEl.innerHTML = visible
      .map((a) => {
        const ackCls = a.acknowledged ? "is-acked" : "";
        const btn = a.acknowledged
          ? `<span class="pill pill-muted">Acknowledged</span>`
          : `<button class="btn" data-ack-id="${a.id}" type="button">Acknowledge</button>`;
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
    listEl.scrollTop = prevScroll;
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
      await Promise.all([refreshAlerts(), refreshAll()]);
    }
  };

  // Interactive SVG hover crosshair
  const chartSvg = document.getElementById("history-chart");
  chartSvg?.addEventListener("mousemove", (ev) => {
    if (chartCoords.length < 2) return;
    const rect = chartSvg.getBoundingClientRect();
    const svgX = ((ev.clientX - rect.left) / rect.width) * 760;
    let nearest = chartCoords[0];
    let bestDist = Math.abs(svgX - nearest.x);
    for (let i = 1; i < chartCoords.length; i++) {
      const dist = Math.abs(svgX - chartCoords[i].x);
      if (dist < bestDist) {
        bestDist = dist;
        nearest = chartCoords[i];
      }
    }
    const cross = document.getElementById("chart-crosshair");
    const dot = document.getElementById("chart-focus-dot");
    const readout = document.getElementById("chart-readout");
    if (cross) {
      cross.setAttribute("x1", nearest.x.toFixed(1));
      cross.setAttribute("x2", nearest.x.toFixed(1));
      cross.setAttribute("opacity", "0.75");
    }
    if (dot) {
      dot.setAttribute("cx", nearest.x.toFixed(1));
      dot.setAttribute("cy", nearest.y.toFixed(1));
      dot.setAttribute("opacity", "1");
    }
    if (readout) {
      const unit = METRIC_META[selectedMetric]?.unit || "";
      readout.textContent = `Cursor: ${nearest.val.toFixed(1)} ${unit} · ${formatTime(nearest.ts)}`;
    }
  });

  chartSvg?.addEventListener("mouseleave", () => {
    document.getElementById("chart-crosshair")?.setAttribute("opacity", "0");
    document.getElementById("chart-focus-dot")?.setAttribute("opacity", "0");
    const readout = document.getElementById("chart-readout");
    if (readout && defaultReadoutText) readout.textContent = defaultReadoutText;
  });

  // Event delegation (CSP-safe: zero inline handlers)
  document.getElementById("station-tabs")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-device]");
    if (!btn) return;
    selectedDeviceId = btn.getAttribute("data-device");
    renderTabs();
    renderActiveStation();
    Promise.all([refreshHistory(), refreshAlerts()]);
  });

  document.getElementById("metric-select")?.addEventListener("change", (ev) => {
    selectedMetric = ev.target.value;
    renderHistoryChart();
  });

  document.querySelector(".window-group")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-minutes]");
    if (!btn) return;
    historyMinutes = Number(btn.getAttribute("data-minutes")) || 15;
    document.querySelectorAll(".window-group .btn-tab").forEach((b) => b.classList.remove("is-active"));
    btn.classList.add("is-active");
    refreshHistory();
  });

  document.getElementById("filter-current-station")?.addEventListener("change", (ev) => {
    filterCurrentStation = Boolean(ev.target.checked);
    refreshAlerts();
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

  // Incremental SSE state updates (avoids redundant GET polling on every tick)
  const handleSseTelemetry = (payload) => {
    const dev = devices.find((d) => d.device_id === payload.device_id);
    if (!dev) {
      refreshAll();
      return;
    }
    dev.status = "online";
    dev.age_sec = 0;
    dev.sensor_status = { ...(dev.sensor_status || {}), ...(payload.sensor_status || {}) };
    const updated = { ...(dev.metrics || {}) };
    for (const [k, v] of Object.entries(payload.metrics || {})) {
      updated[k] = { ...v, updated_at: payload.recorded_at };
    }
    dev.metrics = updated;

    renderTotals();
    renderTabs();

    if (payload.device_id === selectedDeviceId) {
      const point = { recorded_at: payload.recorded_at };
      for (const [k, v] of Object.entries(payload.metrics || {})) {
        if (typeof v?.value === "number") point[k] = v.value;
      }
      historyPoints.push(point);
      const cutoffMs = Date.now() - historyMinutes * 60 * 1000;
      historyPoints = historyPoints.filter((pt) => {
        const t = Date.parse(pt.recorded_at);
        return Number.isNaN(t) || t >= cutoffMs;
      });
      renderActiveStation();
      renderHistoryChart();
    }
  };

  const handleSseStatus = (payload) => {
    const dev = devices.find((d) => d.device_id === payload.device_id);
    if (!dev) {
      refreshAll();
      return;
    }
    dev.status = payload.status;
    renderTotals();
    renderTabs();
    if (payload.device_id === selectedDeviceId) {
      renderActiveStation();
    }
  };

  const connectSSE = () => {
    const badge = document.getElementById("conn-badge");
    const es = new EventSource("/api/stream");
    es.addEventListener("open", () => {
      if (badge) {
        badge.className = "pill pill-ok";
        badge.textContent = "SSE LIVE";
      }
    });
    es.addEventListener("snapshot", (ev) => {
      renderSnapshot(JSON.parse(ev.data));
      refreshHistory();
    });
    es.addEventListener("telemetry", (ev) => {
      handleSseTelemetry(JSON.parse(ev.data));
    });
    es.addEventListener("status", (ev) => {
      handleSseStatus(JSON.parse(ev.data));
    });
    es.addEventListener("alert", (ev) => {
      const alertObj = JSON.parse(ev.data);
      const dev = devices.find((d) => d.device_id === alertObj.device_id);
      if (dev) {
        dev.active_alerts = (dev.active_alerts || 0) + 1;
        renderTotals();
        renderTabs();
      }
      refreshAlerts();
    });
    es.addEventListener("alert_ack", () => {
      refreshAll();
    });
    es.addEventListener("error", () => {
      if (badge) {
        badge.className = "pill pill-warn";
        badge.textContent = "RECONNECTING…";
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
    setInterval(refreshAll, 30000);
  };

  document.addEventListener("DOMContentLoaded", boot);
})();
