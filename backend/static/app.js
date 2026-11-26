(() => {
  "use strict";

  const THEME_KEY = "health-station-theme";

  const VITALS_META = {
    spo2_pct: {
      label: "Oxygen Saturation (SpO2)",
      unit: "%",
      min: 80,
      max: 100,
      safeLo: 95,
      safeHi: 100,
      minLabel: "80%",
      safeLabel: "Normal ≥ 95%",
      maxLabel: "100%",
      decimals: 1,
    },
    bpm: {
      label: "Heart Rate",
      unit: "bpm",
      min: 30,
      max: 160,
      safeLo: 50,
      safeHi: 100,
      minLabel: "30",
      safeLabel: "Normal 50–100",
      maxLabel: "160",
      decimals: 0,
    },
  };

  const ENV_META = {
    air_quality_ppm: { label: "Air Quality / CO", unit: "ppm", range: "Normal ≤ 10 ppm", decimals: 1 },
    temperature_c: { label: "Ambient Temperature", unit: "°C", range: "Comfort 18–28 °C", decimals: 1 },
    humidity_pct: { label: "Relative Humidity", unit: "% RH", range: "Ideal 30–60%", decimals: 1 },
  };

  // Clinical Y-axis floors prevent auto-zooming into ±0.8% healthy noise
  const METRIC_META = {
    spo2_pct: {
      label: "SpO2",
      unit: "%",
      minFloor: 88.0,
      maxCeil: 100.0,
      thresholds: [
        { val: 95.0, label: "95% min", color: "var(--warn)" },
        { val: 90.0, label: "90% crit", color: "var(--crit)" },
      ],
    },
    bpm: {
      label: "Heart Rate",
      unit: "bpm",
      minFloor: 45.0,
      maxCeil: 135.0,
      thresholds: [
        { val: 100.0, label: "100 bpm", color: "var(--warn)" },
        { val: 120.0, label: "120 crit", color: "var(--crit)" },
      ],
    },
    air_quality_ppm: {
      label: "Air / CO",
      unit: "ppm",
      minFloor: 0.0,
      maxCeil: 40.0,
      thresholds: [
        { val: 10.0, label: "10 ppm", color: "var(--warn)" },
        { val: 35.0, label: "35 crit", color: "var(--crit)" },
      ],
    },
    temperature_c: {
      label: "Temperature",
      unit: "°C",
      minFloor: 14.0,
      maxCeil: 34.0,
      thresholds: [
        { val: 28.0, label: "28 °C", color: "var(--warn)" },
        { val: 18.0, label: "18 °C", color: "var(--warn)" },
      ],
    },
    humidity_pct: {
      label: "Humidity",
      unit: "% RH",
      minFloor: 20.0,
      maxCeil: 75.0,
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
  let lastAlerts = [];
  let includeAcked = false;
  let filterCurrentStation = true;
  let chartCoords = [];
  let chartViewWidth = 860;
  let defaultReadoutText = "";

  const esc = (str) =>
    String(str ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");

  // Segmented Theme Management (follows OS prefers-color-scheme unless manually overridden)
  const sysDark = window.matchMedia ? window.matchMedia("(prefers-color-scheme: dark)") : null;
  const applyTheme = (dark) => {
    if (dark) document.documentElement.setAttribute("data-theme", "dark");
    else document.documentElement.removeAttribute("data-theme");
    document.querySelectorAll("#theme-seg .tseg-btn").forEach((btn) => {
      const choice = btn.getAttribute("data-theme-choice");
      btn.classList.toggle("is-active", (dark && choice === "dark") || (!dark && choice === "light"));
    });
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

  document.getElementById("theme-seg")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-theme-choice]");
    if (!btn) return;
    darkMode = btn.getAttribute("data-theme-choice") === "dark";
    applyTheme(darkMode);
    try {
      if (sysDark && darkMode === sysDark.matches) localStorage.removeItem(THEME_KEY);
      else localStorage.setItem(THEME_KEY, darkMode ? "dark" : "light");
    } catch (_) {}
  });

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

  const buildSparklineSvg = (vals, strokeColor, cssClass, width = 84, height = 22) => {
    if (!vals || vals.length < 2) {
      return `<svg class="${cssClass}" viewBox="0 0 ${width} ${height}"></svg>`;
    }
    const min = Math.min(...vals);
    const max = Math.max(...vals);
    const span = max - min || 1;
    const padY = 3;
    const coords = vals.map((v, i) => {
      const x = (i / (vals.length - 1)) * (width - 4) + 2;
      const y = height - padY - ((v - min) / span) * (height - padY * 2);
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    });
    return `<svg class="${cssClass}" viewBox="0 0 ${width} ${height}" preserveAspectRatio="none">
      <polyline fill="none" stroke="${strokeColor}" stroke-width="1.5" points="${coords.join(" ")}" />
    </svg>`;
  };

  const renderTotals = () => {
    const totalsEl = document.getElementById("totals-summary");
    const dlqEl = document.getElementById("hw-dlq");
    const onlineCount = devices.filter((d) => d.status === "online").length;
    const activeAlerts = devices.reduce((acc, d) => acc + (d.active_alerts || 0), 0);
    if (totalsEl) {
      totalsEl.textContent = `${onlineCount}/${devices.length} ONLINE`;
    }
    if (dlqEl) {
      dlqEl.textContent = `${totals.dlq_rows ?? 0} / ${activeAlerts}`;
    }
  };

  const renderStationIndex = () => {
    const tabsEl = document.getElementById("station-tabs");
    if (!tabsEl) return;
    if (!devices.length) {
      tabsEl.innerHTML = '<p class="empty-note-rail">Waiting for telemetry…</p>';
      return;
    }
    tabsEl.innerHTML = devices
      .map((d) => {
        const sel = d.device_id === selectedDeviceId ? "is-selected" : "";
        const spo2 = d.metrics?.spo2_pct?.value;
        const bpm = d.metrics?.bpm?.value;
        const preview = d.status === "online" && spo2 != null && bpm != null
          ? `SpO2 ${Number(spo2).toFixed(1)}% · ${Number(bpm).toFixed(0)} bpm`
          : d.status.toUpperCase();
        const alertBadge = d.active_alerts > 0
          ? `<span class="srow-alert-ct">${d.active_alerts}</span>`
          : "";
        return `<button class="station-row ${sel}" data-device="${esc(d.device_id)}" type="button">
          <div class="srow-top">
            <span class="srow-name"><span class="dot ${esc(d.status)}"></span>${esc(d.device_id)}</span>
            ${alertBadge}
          </div>
          <span class="srow-sub">${esc(preview)}</span>
        </button>`;
      })
      .join("");
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
    renderStationIndex();
    renderActiveStation();
  };

  const setHardwareCell = (id, isOnline, okFlag) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (!isOnline) {
      el.textContent = "OFFLINE";
      el.className = "is-muted";
      return;
    }
    if (okFlag === true) {
      el.textContent = "OK";
      el.className = "is-ok";
    } else if (okFlag === false) {
      el.textContent = "FAULT";
      el.className = "is-fault";
    } else {
      el.textContent = "N/A";
      el.className = "is-muted";
    }
  };

  const renderActiveStation = () => {
    const dev = devices.find((d) => d.device_id === selectedDeviceId);
    const nameEl = document.getElementById("active-station-name");
    const statusEl = document.getElementById("active-station-status");
    const ageEl = document.getElementById("active-station-age");
    const sqiMetaEl = document.getElementById("vitals-sqi-meta");
    const vitalsEl = document.getElementById("vitals-grid");
    const envEl = document.getElementById("env-ledger");

    if (!dev) {
      if (nameEl) nameEl.textContent = "—";
      if (vitalsEl) vitalsEl.innerHTML = "";
      if (envEl) envEl.innerHTML = "";
      return;
    }

    const isOnline = dev.status === "online";
    if (nameEl) nameEl.textContent = dev.device_id;
    if (statusEl) {
      statusEl.textContent = dev.status.toUpperCase();
      statusEl.className = `hw-status mono is-${dev.status}`;
    }
    if (ageEl) {
      ageEl.textContent = `${dev.age_sec ?? 0}s ago · #${dev.last_seq ?? "—"}`;
    }

    const ss = dev.sensor_status || {};
    setHardwareCell("hw-max", isOnline, ss.max30102);
    setHardwareCell("hw-dht", isOnline, ss.dht11);
    setHardwareCell("hw-mq", isOnline, ss.mq135);

    const sqiMetric = dev.metrics?.signal_quality;
    if (sqiMetaEl) {
      sqiMetaEl.textContent = isOnline && sqiMetric
        ? `SQI ${(sqiMetric.value * 100).toFixed(0)}% · ${formatTime(sqiMetric.updated_at)}`
        : "SQI —";
    }

    // Zone 1: Patient Vitals (Hero Numerals + Window Delta + SVG Range Gauge)
    if (vitalsEl) {
      vitalsEl.innerHTML = Object.entries(VITALS_META)
        .map(([key, meta]) => {
          const m = dev.metrics?.[key];
          const val = m ? Number(m.value) : null;
          const sev = isOnline ? (m ? m.severity : "normal") : dev.status;
          const stateText = isOnline
            ? (sev === "normal" ? "IN RANGE" : sev.toUpperCase())
            : dev.status.toUpperCase();

          const safeLeftX = ((meta.safeLo - meta.min) / (meta.max - meta.min)) * 200;
          const safeWidthX = ((meta.safeHi - meta.safeLo) / (meta.max - meta.min)) * 200;
          const needleX = val !== null
            ? Math.max(0, Math.min(197.5, ((val - meta.min) / (meta.max - meta.min)) * 197.5))
            : null;

          const series = historyPoints
            .map((pt) => pt[key])
            .filter((v) => typeof v === "number");
          let deltaMain = "—";
          let deltaSub = `Window ${historyMinutes}m`;
          if (series.length >= 2) {
            const diff = series[series.length - 1] - series[0];
            const sign = diff > 0 ? "+" : "";
            deltaMain = Math.abs(diff) < 0.05 ? "Steady" : `${sign}${diff.toFixed(meta.decimals)} ${meta.unit}`;
            deltaSub = `Min ${Math.min(...series).toFixed(meta.decimals)} · Max ${Math.max(...series).toFixed(meta.decimals)}`;
          }

          return `<div class="vital-cell is-${esc(sev)}">
            <div class="vital-head">
              <span class="vital-label">${esc(meta.label)}</span>
              <span class="vital-state is-${esc(sev)}">${esc(stateText)}</span>
            </div>
            <div class="vital-reading">
              <div class="vital-num-group">
                <span class="vital-num mono">${val !== null ? val.toFixed(meta.decimals) : "—"}</span>
                <span class="vital-unit mono">${esc(meta.unit)}</span>
              </div>
              <div class="vital-delta">
                <span class="vital-delta-main">${esc(deltaMain)}</span>
                <span>${esc(deltaSub)}</span>
              </div>
            </div>
            <div class="range-gauge">
              <svg class="gauge-svg" viewBox="0 0 200 10" preserveAspectRatio="none" aria-hidden="true">
                <line x1="0" y1="5" x2="200" y2="5" class="gauge-bg-line" />
                <rect x="${safeLeftX.toFixed(1)}" y="4" width="${safeWidthX.toFixed(1)}" height="2" class="gauge-safe-rect" />
                ${needleX !== null ? `<rect x="${needleX.toFixed(1)}" y="0" width="2.5" height="10" class="gauge-needle-rect" />` : ""}
              </svg>
              <div class="gauge-labels">
                <span>${esc(meta.minLabel)}</span>
                <span>${esc(meta.safeLabel)}</span>
                <span>${esc(meta.maxLabel)}</span>
              </div>
            </div>
          </div>`;
        })
        .join("");
    }

    // Zone 3: Room Environment Hairline Ledger
    if (envEl) {
      envEl.innerHTML = Object.entries(ENV_META)
        .map(([key, meta]) => {
          const m = dev.metrics?.[key];
          const val = m ? Number(m.value) : null;
          const sev = isOnline ? (m ? m.severity : "normal") : dev.status;
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
            : "var(--trace)";

          return `<div class="env-row is-${esc(sev)}">
            <div class="env-info">
              <span class="env-name">${esc(meta.label)}</span>
              <span class="env-range">${esc(meta.range)}</span>
            </div>
            ${buildSparklineSvg(sparkSeries, stroke, "env-spark", 84, 22)}
            <div class="env-val-col">
              <span class="env-val">${val !== null ? val.toFixed(meta.decimals) : "—"}</span>
              <span class="env-unit">${esc(meta.unit)}</span>
            </div>
          </div>`;
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
      if (readout) readout.textContent = "—";
      return;
    }

    const w = Math.max(320, Math.round(svg.clientWidth || 860));
    const h = 205;
    chartViewWidth = w;
    svg.setAttribute("viewBox", `0 0 ${w} ${h}`);
    const padL = 44, padR = 58, padT = 14, padB = 24;
    const plotW = w - padL - padR;
    const plotH = h - padT - padB;

    const meta = METRIC_META[selectedMetric] || { unit: "", thresholds: [] };
    const vals = series.map((s) => s.val);
    const dataMin = Math.min(...vals);
    const dataMax = Math.max(...vals);
    const min = meta.minFloor !== undefined ? Math.min(dataMin, meta.minFloor) : dataMin;
    const max = meta.maxCeil !== undefined ? Math.max(dataMax, meta.maxCeil) : dataMax;
    const span = (max - min) || 1.0;
    const yFor = (v) => padT + plotH - ((v - min) / span) * plotH;

    chartCoords = series.map((s, i) => {
      const x = padL + (i / (series.length - 1)) * plotW;
      const y = yFor(s.val);
      return { x, y, ...s };
    });

    const gridLines = [];
    for (let g = 0; g <= 3; g++) {
      const frac = g / 3;
      const y = padT + plotH * (1 - frac);
      const tickVal = min + span * frac;
      gridLines.push(
        `<line x1="${padL}" y1="${y.toFixed(1)}" x2="${w - padR}" y2="${y.toFixed(1)}" stroke="var(--rule)" stroke-width="1" />`,
        `<text x="${padL - 8}" y="${(y + 3.5).toFixed(1)}" text-anchor="end" fill="var(--subtle)" font-size="10.5" class="mono">${tickVal.toFixed(1)}</text>`
      );
    }

    const xTicks = [];
    const tickCount = Math.min(5, series.length);
    for (let t = 0; t < tickCount; t++) {
      const idx = Math.round((t / Math.max(1, tickCount - 1)) * (series.length - 1));
      const pt = chartCoords[idx];
      xTicks.push(
        `<text x="${pt.x.toFixed(1)}" y="${h - 5}" text-anchor="middle" fill="var(--subtle)" font-size="10.5" class="mono">${esc(formatTime(pt.ts, false))}</text>`
      );
    }

    const refLines = [];
    for (const th of meta.thresholds || []) {
      if (th.val >= min && th.val <= max) {
        const ty = yFor(th.val);
        refLines.push(
          `<line x1="${padL}" y1="${ty.toFixed(1)}" x2="${w - padR}" y2="${ty.toFixed(1)}" stroke="${th.color}" stroke-width="1" stroke-dasharray="4 3" opacity="0.8" />`,
          `<text x="${w - padR + 6}" y="${(ty + 3.5).toFixed(1)}" fill="${th.color}" font-size="10.5" class="mono">${esc(th.label)}</text>`
        );
      }
    }

    const linePts = chartCoords.map((c) => `${c.x.toFixed(1)},${c.y.toFixed(1)}`).join(" ");
    const baseLineY = (padT + plotH).toFixed(1);
    const areaPts = `${chartCoords[0].x.toFixed(1)},${baseLineY} ${linePts} ${chartCoords[chartCoords.length - 1].x.toFixed(1)},${baseLineY}`;
    const markers = chartCoords.length <= 25
      ? chartCoords.map((c) => `<circle cx="${c.x.toFixed(1)}" cy="${c.y.toFixed(1)}" r="2" fill="var(--trace)" />`).join("")
      : "";

    svg.innerHTML = `
      <defs>
        <linearGradient id="trace-area-grad" x1="0" y1="0" x2="0" y2="1">
          <stop offset="0%" stop-color="var(--trace)" stop-opacity="0.14" />
          <stop offset="100%" stop-color="var(--trace)" stop-opacity="0" />
        </linearGradient>
      </defs>
      ${gridLines.join("")}
      ${refLines.join("")}
      ${xTicks.join("")}
      <polygon fill="url(#trace-area-grad)" points="${areaPts}" />
      <polyline fill="none" stroke="var(--trace)" stroke-width="1.5" stroke-linejoin="round" points="${linePts}" />
      ${markers}
      <line id="chart-crosshair" x1="0" y1="${padT}" x2="0" y2="${padT + plotH}" stroke="var(--ink)" stroke-width="1" stroke-dasharray="2 2" opacity="0" />
      <circle id="chart-focus-dot" cx="0" cy="0" r="3.5" fill="var(--ink)" stroke="var(--bg)" stroke-width="1.5" opacity="0" />
    `;

    const last = series[series.length - 1];
    defaultReadoutText = `${series.length} samples · Latest ${last.val.toFixed(1)} ${meta.unit} (${formatTime(last.ts)}) · Min ${dataMin.toFixed(1)} · Max ${dataMax.toFixed(1)}`;
    if (readout) readout.textContent = defaultReadoutText;
  };

  const renderAlerts = (alerts) => {
    lastAlerts = alerts || [];
    const listEl = document.getElementById("alerts-list");
    if (!listEl) return;
    const prevScroll = listEl.scrollTop;

    const visible = filterCurrentStation && selectedDeviceId
      ? lastAlerts.filter((a) => a.device_id === selectedDeviceId)
      : lastAlerts;

    if (visible.length === 0) {
      const otherCount = filterCurrentStation ? lastAlerts.length : 0;
      const actionBtn = otherCount > 0
        ? `<button class="empty-action-btn" data-action="show-all-alerts" type="button">Show ${otherCount} alerts from other stations →</button>`
        : "";
      listEl.innerHTML = `<div class="event-empty">
        <span class="event-empty-title">All vitals within thresholds for ${esc(selectedDeviceId || "station")}.</span>
        ${actionBtn}
      </div>`;
      return;
    }

    listEl.innerHTML = visible
      .map((a) => {
        const ackCls = a.acknowledged ? "is-acked" : "";
        const action = a.acknowledged
          ? `<span class="ev-acked-lbl">ACK</span>`
          : `<button class="ev-ack-btn" data-ack-id="${a.id}" type="button">Acknowledge</button>`;
        return `<div class="ev-row sev-${esc(a.severity)} ${ackCls}">
          <span class="ev-bar" aria-hidden="true"></span>
          <div class="ev-body">
            <span class="ev-msg">${esc(a.message)}</span>
            <span class="ev-meta">${esc(a.device_id)} · ${esc(a.code)} · ${esc(formatTime(a.created_at))}</span>
          </div>
          ${action}
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

  // Interactive SVG hover crosshair + responsive ResizeObserver
  const chartSvg = document.getElementById("history-chart");
  if (chartSvg && "ResizeObserver" in window) {
    let lastObservedW = 0;
    new ResizeObserver(() => {
      const nextW = Math.round(chartSvg.clientWidth);
      if (nextW > 0 && nextW !== lastObservedW) {
        lastObservedW = nextW;
        if (historyPoints.length >= 2) renderHistoryChart();
      }
    }).observe(chartSvg);
  }
  chartSvg?.addEventListener("mousemove", (ev) => {
    if (chartCoords.length < 2) return;
    const rect = chartSvg.getBoundingClientRect();
    const svgX = ((ev.clientX - rect.left) / rect.width) * chartViewWidth;
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
      cross.setAttribute("opacity", "0.7");
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
    renderStationIndex();
    renderActiveStation();
    Promise.all([refreshHistory(), refreshAlerts()]);
  });

  document.getElementById("metric-tabs")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-metric]");
    if (!btn) return;
    selectedMetric = btn.getAttribute("data-metric");
    document.querySelectorAll("#metric-tabs .mtab").forEach((b) => b.classList.remove("is-active"));
    btn.classList.add("is-active");
    renderHistoryChart();
  });

  document.querySelector(".window-group")?.addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-minutes]");
    if (!btn) return;
    historyMinutes = Number(btn.getAttribute("data-minutes")) || 15;
    document.querySelectorAll(".window-group .wtab").forEach((b) => b.classList.remove("is-active"));
    btn.classList.add("is-active");
    refreshHistory();
  });

  document.getElementById("btn-export-csv")?.addEventListener("click", () => {
    if (!historyPoints || historyPoints.length === 0) return;
    const cols = ["recorded_at", "bpm", "spo2_pct", "temperature_c", "humidity_pct", "air_quality_ppm", "signal_quality"];
    const escapeCsv = (val) => {
      if (val === null || val === undefined) return "";
      const s = String(val);
      return /[",\r\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
    };
    const lines = [cols.join(",")];
    for (const pt of historyPoints) {
      lines.push(cols.map((c) => escapeCsv(pt[c])).join(","));
    }
    const blob = new Blob([lines.join("\n") + "\n"], { type: "text/csv;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    const devSlug = (selectedDeviceId || "station").replace(/[^A-Za-z0-9._-]/g, "_");
    const day = new Date().toISOString().slice(0, 10);
    a.href = url;
    a.download = `${devSlug}_${historyMinutes}m_${day}.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  });

  document.getElementById("filter-current-station")?.addEventListener("change", (ev) => {
    filterCurrentStation = Boolean(ev.target.checked);
    renderAlerts(lastAlerts);
  });

  document.getElementById("show-acked")?.addEventListener("change", (ev) => {
    includeAcked = Boolean(ev.target.checked);
    refreshAlerts();
  });

  document.getElementById("alerts-list")?.addEventListener("click", (ev) => {
    const showAllBtn = ev.target.closest('[data-action="show-all-alerts"]');
    if (showAllBtn) {
      filterCurrentStation = false;
      const chk = document.getElementById("filter-current-station");
      if (chk) chk.checked = false;
      renderAlerts(lastAlerts);
      return;
    }
    const btn = ev.target.closest("[data-ack-id]");
    if (!btn) return;
    const id = Number(btn.getAttribute("data-ack-id"));
    if (id > 0) acknowledgeAlert(id);
  });

  // Incremental SSE state updates
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
    renderStationIndex();

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
    renderStationIndex();
    if (payload.device_id === selectedDeviceId) {
      renderActiveStation();
    }
  };

  const connectSSE = () => {
    const badge = document.getElementById("conn-badge");
    const es = new EventSource("/api/stream");
    es.addEventListener("open", () => {
      if (badge) {
        badge.className = "stream-pill is-live";
        const span = badge.querySelector("span");
        if (span) span.textContent = "LIVE";
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
        renderStationIndex();
      }
      refreshAlerts();
    });
    es.addEventListener("alert_ack", () => {
      refreshAll();
    });
    es.addEventListener("error", () => {
      if (badge) {
        badge.className = "stream-pill is-warn";
        const span = badge.querySelector("span");
        if (span) span.textContent = "RECONNECTING";
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
