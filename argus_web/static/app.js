const view = document.getElementById("view");
const clock = document.getElementById("clock");

const timers = new Set();

function addTimer(id) {
  timers.add(id);
  return id;
}

function clearTimers() {
  timers.forEach((id) => {
    clearTimeout(id);
    clearInterval(id);
  });
  timers.clear();
}

const STATUS = {
  CLEAR: "ПУСТО",
  WARNING: "ВНИМАНИЕ",
  BLOCKED: "ПРЕПЯТСТВИЕ",
  DEGRADED: "НЕТ ДАННЫХ",
  ready: "ГОТОВО",
  incomplete: "НЕ СОБРАНА",
  holdout: "ОТЛОЖЕНА",
  parsing: "РАЗБИРАЕТСЯ",
  running: "ИДЁТ ПРОГОН",
  starting: "ЗАПУСК",
  error: "ОШИБКА",
};

function fmtNum(value, digits = 1) {
  if (value == null || Number.isNaN(value)) return "—";
  return Number(value).toLocaleString("ru-RU", {
    maximumFractionDigits: digits,
    minimumFractionDigits: digits,
  });
}

function fmtInt(value) {
  if (value == null) return "—";
  return Number(value).toLocaleString("ru-RU");
}

function fmtSize(bytes) {
  if (!bytes) return "—";
  const units = ["Б", "КБ", "МБ", "ГБ"];
  let n = bytes;
  let i = 0;
  while (n >= 1024 && i < units.length - 1) {
    n /= 1024;
    i += 1;
  }
  return `${fmtNum(n, i === 0 ? 0 : 1)} ${units[i]}`;
}

function fmtDuration(seconds) {
  if (!seconds) return "—";
  if (seconds < 90) return `${fmtNum(seconds, 0)} с`;
  const min = seconds / 60;
  if (min < 90) return `${fmtNum(min, 0)} мин`;
  return `${fmtNum(min / 60, 1)} ч`;
}

function fmtMeters(value) {
  if (value == null || Number.isNaN(value)) return "—";
  return `${fmtNum(value, 1)} м`;
}

function fmtClock(stampNs) {
  if (!stampNs) return "—";
  const ms = Number(stampNs) / 1e6;
  const d = new Date(ms);
  if (Number.isNaN(d.getTime())) return "—";
  const p = (n, w = 2) => String(n).padStart(w, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
}

function proofLine(report) {
  if (!report || report.latency_ms == null || report.latency_ms.median == null) return "";
  const ms = fmtNum(report.latency_ms.median, 0);
  const hz = report.fps != null ? ` · ${fmtNum(report.fps, 0)} Гц` : "";
  return `${ms} мс${hz}`;
}

function reportForBag(reports, bagId) {
  if (!bagId) return null;
  const items = (reports || []).filter((item) => item.name === bagId || item.name.startsWith(`${bagId}_`));
  return items.find((item) => item.blocked > 0) || items[0] || null;
}

function verdictOf(report, row, view) {
  if (!report) {
    return { code: "not_run", chip: "НЕТ ОТЧЁТА", cls: "", phrase: "Детектор ещё не прогнан", meters: null };
  }
  if (view && view.geom === false && view.noReturn === false && view.freeSpace === false) {
    return { code: "unsure", chip: "ВЫКЛ", cls: "warn-text", phrase: "Признаки обнаружения выключены", meters: null };
  }
  if (!row) {
    return { code: "not_covered", chip: "НЕТ РЕЗУЛЬТАТА", cls: "warn-text", phrase: "Для этого кадра нет результата детектора", meters: null };
  }
  if (row.status === "DEGRADED") {
    return { code: "unsure", chip: "НЕ УВЕРЕН", cls: "warn-text", phrase: "Детектор не уверен: кадр обработан не полностью", meters: null };
  }
  if (row.status === "BLOCKED" && row.nearest_m != null) {
    return {
      code: "blocked",
      chip: "ПРЕПЯТСТВИЕ",
      cls: "bad-text",
      phrase: `Препятствие, ${fmtNum(row.nearest_m, 1)} м`,
      meters: row.nearest_m,
    };
  }
  if (row.status === "CLEAR") {
    return { code: "clear", chip: "ПУСТО", cls: "ok-text", phrase: "В габарите пусто", meters: null };
  }
  return { code: "unsure", chip: "НЕ УВЕРЕН", cls: "warn-text", phrase: "Детектор не уверен", meters: null };
}

function decisionPhrase(report) {
  if (!report) return "Детектор ещё не прогнан";
  if (report.blocked > 0 && report.nearest_m != null) return `Препятствие, ${fmtNum(report.nearest_m, 1)} м`;
  if (report.clear > 0 && !report.blocked) return "В габарите пусто";
  return "Детектор не уверен";
}

function tick() {
  clock.textContent = new Date().toLocaleTimeString("ru-RU", { hour12: false });
}

setInterval(tick, 1000);
tick();

async function api(path, opts) {
  const res = await fetch(path, opts);
  if (!res.ok) {
    let message = `Ошибка ${res.status}`;
    try {
      const data = await res.json();
      if (data.error) message = data.error;
    } catch (_err) {
      void _err;
    }
    throw new Error(message);
  }
  const type = res.headers.get("content-type") || "";
  if (type.includes("json")) return res.json();
  if (type.includes("octet-stream")) return res;
  return res.text();
}

function setNav(route) {
  document.querySelectorAll(".nav a").forEach((el) => {
    el.classList.toggle("active", el.dataset.route === route);
  });
}

function setSystem() {}

const ICO = {
  doc: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M6 3h8l4 4v14H6z"/><path d="M14 3v4h4"/></svg>',
  play: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M7 4.5v15l13-7.5z"/></svg>',
  pause: '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4.5" width="4" height="15"/><rect x="14" y="4.5" width="4" height="15"/></svg>',
  prev: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M18 5v14L8.5 12z"/><rect x="5" y="5" width="2.4" height="14"/></svg>',
  next: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M6 5v14l9.5-7z"/><rect x="16.6" y="5" width="2.4" height="14"/></svg>',
  up: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M12 16V5"/><path d="M6.5 10.5L12 5l5.5 5.5"/><path d="M4 19h16"/></svg>',
  lock: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><rect x="5.5" y="10.5" width="13" height="9.5" rx="1.5"/><path d="M8.5 10.5V8a3.5 3.5 0 0 1 7 0v2.5"/></svg>',
  chevL: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M14.5 5.5L8 12l6.5 6.5"/></svg>',
  chevR: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M9.5 5.5L16 12l-6.5 6.5"/></svg>',
};

function chip(text, kind = "", dot = true) {
  return `<span class="chip ${kind}">${dot ? "<i></i>" : ""}${text}</span>`;
}

function sparkline(values, color = "#8a9199", w = 108, h = 26) {
  const id = `sp${Math.random().toString(36).slice(2, 8)}`;
  addTimer(
    setTimeout(() => {
      const canvas = document.getElementById(id);
      if (!canvas) return;
      const dpr = window.devicePixelRatio || 1;
      canvas.width = w * dpr;
      canvas.height = h * dpr;
      const ctx = canvas.getContext("2d");
      ctx.scale(dpr, dpr);
      const pts = (values || []).filter((v) => v != null && !Number.isNaN(v));
      ctx.strokeStyle = "#2c333a";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, h - 1);
      ctx.lineTo(w, h - 1);
      ctx.stroke();
      if (pts.length > 1) {
        const lo = Math.min(...pts);
        const hi = Math.max(...pts);
        const span = hi - lo || 1;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        pts.forEach((v, i) => {
          const x = (i / (pts.length - 1)) * (w - 2) + 1;
          const y = h - 4 - ((v - lo) / span) * (h - 8);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
      }
    }, 0)
  );
  return `<canvas id="${id}" style="width:${w}px;height:${h}px"></canvas>`;
}

function metricCard(label, valueHtml, series, color) {
  return `
    <div class="metric">
      <div>
        <div class="label">${label}</div>
        <div class="val">${valueHtml}</div>
      </div>
      ${sparkline(series, color)}
    </div>`;
}

function scaleSvg(value, max = 50) {
  const w = 320;
  const h = 46;
  const x = (v) => 14 + (Math.min(v, max) / max) * (w - 28);
  let ticks = "";
  for (let v = 0; v <= max; v += 10) {
    ticks += `<line x1="${x(v)}" y1="14" x2="${x(v)}" y2="20" stroke="#3a424b"/>`;
    ticks += `<text x="${x(v)}" y="11" fill="#8a9199" font-size="9" text-anchor="middle">${v}</text>`;
  }
  const marker = value != null
    ? `<line x1="${x(value)}" y1="8" x2="${x(value)}" y2="27" stroke="#c05a4c" stroke-width="2"/>
       <text x="${x(value)}" y="42" fill="#c05a4c" font-size="11" text-anchor="middle">${fmtNum(value, 1)}</text>`
    : "";
  return `
    <svg viewBox="0 0 ${w} ${h}" role="img" aria-label="Шкала расстояния">
      <line x1="14" y1="17" x2="${w - 14}" y2="17" stroke="#3a424b"/>
      <line x1="${x(max)}" y1="14" x2="${x(max)}" y2="20" stroke="#3a424b"/>
      <text x="${w - 12}" y="11" fill="#8a9199" font-size="9" text-anchor="start">м</text>
      ${ticks}${marker}
    </svg>`;
}

function tunnelSvg() {
  return `
    <svg viewBox="0 0 230 128" role="img" aria-label="Сечение тоннеля">
      <path d="M28 118 L28 56 Q28 18 115 18 Q202 18 202 56 L202 118" fill="none" stroke="#4a525b" stroke-width="1.2"/>
      <path d="M40 118 L40 60 Q40 30 115 30 Q190 30 190 60 L190 118" fill="none" stroke="#333b43" stroke-width="1"/>
      <line x1="20" y1="118" x2="210" y2="118" stroke="#333b43"/>
      <line x1="97" y1="118" x2="93" y2="104" stroke="#4a525b"/>
      <line x1="133" y1="118" x2="137" y2="104" stroke="#4a525b"/>
      <line x1="82" y1="104" x2="148" y2="104" stroke="#333b43"/>
      <line x1="60" y1="118" x2="46" y2="96" stroke="#2c333a"/>
      <line x1="170" y1="118" x2="184" y2="96" stroke="#2c333a"/>
      <rect x="121" y="72" width="15" height="24" fill="rgba(160,82,72,0.25)" stroke="#c05a4c" stroke-width="1.2"/>
    </svg>`;
}

function gizmoSvg() {
  return `
    <svg width="72" height="62" viewBox="0 0 72 62" aria-hidden="true">
      <line x1="36" y1="38" x2="36" y2="8" stroke="#8a9199" stroke-width="1.3"/>
      <path d="M36 6l-3.4 5h6.8z" fill="#8a9199"/>
      <text x="42" y="12" fill="#8a9199" font-size="9">Z</text>
      <line x1="36" y1="38" x2="66" y2="38" stroke="#8a9199" stroke-width="1.3"/>
      <path d="M68 38l-5-3.4v6.8z" fill="#8a9199"/>
      <text x="62" y="50" fill="#8a9199" font-size="9">X</text>
      <line x1="36" y1="38" x2="12" y2="52" stroke="#4f88a8" stroke-width="1.3"/>
      <path d="M10 53.5l6 .4-2.6-5.4z" fill="#4f88a8"/>
      <text x="4" y="60" fill="#4f88a8" font-size="9">Y</text>
    </svg>`;
}

function ringSvg(pct) {
  const r = 17;
  const c = 2 * Math.PI * r;
  const p = Math.max(0, Math.min(1, pct || 0));
  return `
    <div class="ring" role="img" aria-label="Готовность ${Math.round(p * 100)}%">
      <svg width="44" height="44" viewBox="0 0 44 44">
        <circle cx="22" cy="22" r="${r}" fill="none" stroke="#262d34" stroke-width="3.5"/>
        <circle cx="22" cy="22" r="${r}" fill="none" stroke="#4f88a8" stroke-width="3.5"
          stroke-dasharray="${(c * p).toFixed(1)} ${c.toFixed(1)}" stroke-linecap="butt"
          transform="rotate(-90 22 22)"/>
      </svg>
      <span>${Math.round(p * 100)}%</span>
    </div>`;
}

function checkRow(id, checked) {
  return `
    <label class="check">
      <input id="${id}" type="checkbox" ${checked ? "checked" : ""}>
      <i aria-hidden="true"></i>
    </label>`;
}

function switchRow(id, label, on, disabled = false) {
  return `
    <label class="switch-row ${on ? "" : "off"}">
      <span class="name">${label}</span>
      <span class="switch">
        <input id="${id}" type="checkbox" ${on ? "checked" : ""} ${disabled ? "disabled" : ""}>
        <i aria-hidden="true"></i>
      </span>
    </label>`;
}

function pair(k, v, cls = "") {
  return `<div class="pair"><div class="k">${k}</div><div class="v ${cls}">${v}</div></div>`;
}

function banner(text) {
  return `<div class="banner" role="alert">${text}</div>`;
}

function empty(text, action = "") {
  return `<div class="empty"><p style="margin:0">${text}</p>${action}</div>`;
}

function planOf(x, y, gauge) {
  const axis = (gauge && gauge.forward_axis) || "-y";
  if (axis === "-y") return [-y, x];
  if (axis === "+y" || axis === "y") return [y, -x];
  if (axis === "-x") return [-x, -y];
  return [x, y];
}

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = 1;
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    return true;
  }
  return false;
}

function groundFloor(gauge) {
  const sh = gauge && gauge.sensor_height != null ? gauge.sensor_height : 1.2;
  return -sh + 0.6;
}

function clusterBounds(sides, zs, half) {
  if (!sides.length) {
    return { side0: -0.6, side1: 0.6, z0: -1.0, z1: 0.4 };
  }
  const sorted = (arr) => [...arr].sort((a, b) => a - b);
  const q = (arr, p) => arr[Math.min(arr.length - 1, Math.max(0, Math.floor(p * (arr.length - 1))))];
  const ss = sorted(sides);
  const zz = sorted(zs);
  return {
    side0: Math.max(-half, q(ss, 0.05)),
    side1: Math.min(half, q(ss, 0.95)),
    z0: q(zz, 0.05),
    z1: q(zz, 0.95),
  };
}

function paintPerspective(ctx, canvas, xyz, gauge, row, layers, focusSide) {
  const w = canvas.width;
  const h = canvas.height;
  ctx.fillStyle = "#0b0d10";
  ctx.fillRect(0, 0, w, h);
  const hw = gauge ? gauge.half_width : 1.5;
  const floor = groundFloor(gauge);
  const cx = w / 2;
  const cy = h * 0.48;
  const F = Math.min(w * 0.85, h * 1.5);
  const d0 = 2.2;
  const project = (fwd, side, z) => {
    const depth = fwd + d0;
    return [cx + (side - focusSide) * (F / depth), cy - (z * F) / depth];
  };
  const sides = [];
  const zs = [];
  const blocked = layers.box && row && row.status === "BLOCKED" && row.nearest_m != null;
  const candidates = layers.box && row && row.status !== "BLOCKED" ? (row.candidates || []) : [];
  const near = blocked ? row.nearest_m : null;
  const n = xyz.length / 3;
  const image = ctx.createImageData(w, h);
  const pixels = image.data;
  for (let i = 0; i < n; i += 1) {
    const [fwd, side] = planOf(xyz[i * 3], xyz[i * 3 + 1], gauge);
    if (fwd < 0.3 || fwd > 90) continue;
    const z = xyz[i * 3 + 2];
    const view = window.__view;
    if (view && (fwd < view.minRange || fwd > view.maxRange)) continue;
    if (layers.ground && (!view || view.ground !== false) && z < floor) continue;
    if (Math.abs(side) > hw * 2.4) continue;
    const [px, py] = project(fwd, side, z);
    const x = px | 0;
    const y = py | 0;
    if (x < 0 || y < 0 || x >= w || y >= h) continue;
    const offset = (y * w + x) * 4;
    const b = Math.max(46, Math.min(215, 215 - fwd * 2.3));
    pixels[offset] = b;
    pixels[offset + 1] = b + 6;
    pixels[offset + 2] = b + 12;
    pixels[offset + 3] = 255;
    if (near != null && fwd > near - 1.5 && fwd < near + 1.5 && Math.abs(side) <= hw) {
      sides.push(side);
      zs.push(z);
    }
  }
  ctx.putImageData(image, 0, 0);
  if (gauge && gauge.near && gauge.far && layers.gauge) {
    ctx.strokeStyle = "rgba(91,122,140,0.9)";
    ctx.lineWidth = Math.max(1, w / 1400);
    const loop = (poly) => {
      ctx.beginPath();
      poly.forEach((p, i) => {
        const [fwd, side] = planOf(p[0], p[1], gauge);
        const [px, py] = project(Math.max(0.6, fwd), side, p[2]);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.closePath();
      ctx.stroke();
    };
    loop(gauge.far);
    loop(gauge.near);
    for (let k = 0; k < 6; k += 1) {
      const a = gauge.near[k];
      const b = gauge.far[k];
      ctx.beginPath();
      const [f1, s1] = planOf(a[0], a[1], gauge);
      const [f2, s2] = planOf(b[0], b[1], gauge);
      const [x1, y1] = project(Math.max(0.6, f1), s1, a[2]);
      const [x2, y2] = project(Math.max(0.6, f2), s2, b[2]);
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }
  }
  if (blocked) {
    const bb = clusterBounds(sides, zs, hw);
    const f0 = Math.max(0.5, near - 1.2);
    const f1 = near + 1.2;
    const corners = [];
    [f0, f1].forEach((f) => {
      [[bb.side0, bb.z0], [bb.side1, bb.z0], [bb.side1, bb.z1], [bb.side0, bb.z1]].forEach(([s, z]) => {
        corners.push(project(f, s, z));
      });
    });
    ctx.strokeStyle = "#c05a4c";
    ctx.lineWidth = Math.max(1, w / 1300);
    ctx.beginPath();
    corners.slice(0, 4).forEach(([px, py], i) => {
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    corners.slice(4).forEach(([px, py], i) => {
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.stroke();
    for (let i = 0; i < 4; i += 1) {
      ctx.beginPath();
      ctx.moveTo(corners[i][0], corners[i][1]);
      ctx.lineTo(corners[i + 4][0], corners[i + 4][1]);
      ctx.stroke();
    }
  }
  if (candidates.length) {
    ctx.strokeStyle = "#d5a84f";
    ctx.lineWidth = Math.max(1, w / 1500);
    candidates.forEach((candidate) => {
      const position = candidate.position || [];
      const extent = candidate.extent || [];
      if (position.length !== 3 || extent.length !== 3) return;
      const fwd = -Number(position[1]);
      const side = Number(position[0]);
      const halfFwd = Number(extent[1]) / 2;
      const halfSide = Number(extent[0]) / 2;
      const a = project(fwd - halfFwd, side - halfSide, Number(position[2]));
      const b = project(fwd + halfFwd, side + halfSide, Number(position[2]) + Number(extent[2]));
      ctx.setLineDash([6, 4]);
      ctx.strokeRect(a[0], b[1], b[0] - a[0], a[1] - b[1]);
      ctx.setLineDash([]);
      ctx.fillStyle = "#f1d48a";
      ctx.font = `${Math.round(h / 42)}px Consolas, monospace`;
      ctx.fillText("КАНДИДАТ", a[0] + 6, Math.max(16, b[1] - 6));
    });
  }
}

function paintPlan(ctx, canvas, xyz, gauge, row, layers, focusSide) {
  const w = canvas.width;
  const h = canvas.height;
  ctx.fillStyle = "#0b0d10";
  ctx.fillRect(0, 0, w, h);
  const span = 40;
  const hw = gauge ? gauge.half_width : 1.5;
  const lateral = Math.max(8, hw * 4);
  const floor = groundFloor(gauge);
  const bins = new Map();
  const sides = [];
  const zs = [];
  const blocked = layers.box && row && row.status === "BLOCKED" && row.nearest_m != null;
  const candidates = layers.box && row && row.status !== "BLOCKED" ? (row.candidates || []) : [];
  const near = blocked ? row.nearest_m : null;
  const n = xyz.length / 3;
  const yOf = (side) => Math.round(h / 2 - ((side - focusSide) / lateral) * (h * 0.42));
  const image = ctx.createImageData(w, h);
  const pixels = image.data;
  for (let i = 0; i < n; i += 1) {
    const [fwd, side] = planOf(xyz[i * 3], xyz[i * 3 + 1], gauge);
    if (fwd < 0 || fwd > span) continue;
    if (Math.abs(side) > lateral) continue;
    const z = xyz[i * 3 + 2];
    const view = window.__view;
    if (view && (fwd < view.minRange || fwd > view.maxRange)) continue;
    if (layers.ground && (!view || view.ground !== false) && z < floor) continue;
    const px = Math.round((fwd / span) * (w - 24) + 12);
    const py = yOf(side);
    if (px < 0 || py < 0 || px >= w || py >= h) continue;
    const offset = (py * w + px) * 4;
    const inside = Math.abs(side) <= hw;
    pixels[offset] = inside ? 215 : 89;
    pixels[offset + 1] = inside ? 228 : 98;
    pixels[offset + 2] = inside ? 236 : 107;
    pixels[offset + 3] = 255;
    if (near != null && fwd > near - 1.5 && fwd < near + 1.5 && inside) {
      sides.push(side);
      zs.push(z);
    }
  }
  ctx.putImageData(image, 0, 0);
  if (gauge && gauge.left && gauge.right && layers.gauge) {
    ctx.strokeStyle = "rgba(91,122,140,0.95)";
    ctx.lineWidth = Math.max(1, w / 1200);
    [gauge.left, gauge.right].forEach((line) => {
      ctx.beginPath();
      line.forEach((p, i) => {
        const [fwd, side] = planOf(p[0], p[1], gauge);
        const px = (fwd / span) * (w - 24) + 12;
        const py = yOf(side);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    });
  }
  if (blocked) {
    const bb = clusterBounds(sides, zs, hw);
    const x0 = ((near - 1.2) / span) * (w - 24) + 12;
    const x1 = ((near + 1.2) / span) * (w - 24) + 12;
    const y0 = yOf(bb.side1);
    const y1 = yOf(bb.side0);
    ctx.strokeStyle = "#c05a4c";
    ctx.lineWidth = Math.max(1, w / 1100);
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    const mx = (near / span) * (w - 24) + 12;
    const my = (y0 + y1) / 2;
    ctx.fillStyle = "#e6e9ec";
    const label = `${fmtNum(near, 1)} м`;
    ctx.font = `${Math.round(h / 34)}px Consolas, monospace`;
    ctx.fillText(label, Math.min(w - 60, mx + 8), Math.max(14, y0 - 6));
    void my;
  }
  if (candidates.length) {
    ctx.strokeStyle = "#d5a84f";
    ctx.lineWidth = Math.max(1, w / 1300);
    ctx.setLineDash([6, 4]);
    candidates.forEach((candidate) => {
      const position = candidate.position || [];
      const extent = candidate.extent || [];
      if (position.length !== 3 || extent.length !== 3) return;
      const fwd = -Number(position[1]);
      const side = Number(position[0]);
      const x0 = ((fwd - Number(extent[1]) / 2) / span) * (w - 24) + 12;
      const x1 = ((fwd + Number(extent[1]) / 2) / span) * (w - 24) + 12;
      const y0 = yOf(side + Number(extent[0]) / 2);
      const y1 = yOf(side - Number(extent[0]) / 2);
      ctx.strokeRect(x0, Math.min(y0, y1), x1 - x0, Math.abs(y1 - y0));
      ctx.fillStyle = "#f1d48a";
      ctx.font = `${Math.round(h / 34)}px Consolas, monospace`;
      ctx.fillText("КАНДИДАТ", Math.min(w - 90, x0 + 6), Math.max(14, Math.min(y0, y1) - 6));
    });
    ctx.setLineDash([]);
  }
}

function paintScene(mode) {
  return (ctx, canvas, xyz, gauge, row, layers, focusSide) => {
    if (mode === "plan") paintPlan(ctx, canvas, xyz, gauge, row, layers, focusSide);
    else paintPerspective(ctx, canvas, xyz, gauge, row, layers, focusSide);
  };
}

const cloudCache = new Map();

function fetchCloud(bagId, frame, synthetic = false) {
  const url = synthetic
    ? `/api/synthetic/${encodeURIComponent(bagId)}?frame=${frame}`
    : `/api/cloud?bag=${encodeURIComponent(bagId)}&frame=${frame}`;
  const key = `${bagId}:${frame}:${synthetic ? 1 : 0}`;
  const cached = cloudCache.get(key);
  if (cached) return cached;
  return fetch(url).then(async (res) => {
    if (!res.ok) throw new Error("Нет кадра");
    const buf = await res.arrayBuffer();
    const cloud = {
      xyz: new Float32Array(buf),
      points: Number(res.headers.get("X-Argus-Points") || 0),
      raw: Number(res.headers.get("X-Argus-Raw") || 0),
      total: Number(res.headers.get("X-Argus-Total") || 0),
      stamp: res.headers.get("X-Argus-Stamp") || "",
    };
    cloudCache.set(key, cloud);
    if (cloudCache.size > 40) cloudCache.delete(cloudCache.keys().next().value);
    return cloud;
  });
}

function wireUpload({ drop, file, browse, name, notes, msg, after }) {
  browse.addEventListener("click", () => file.click());
  drop.addEventListener("click", () => file.click());
  file.addEventListener("change", () => {
    if (file.files[0]) runUpload(file.files[0], name ? name.value : "", notes ? notes.value : "", msg, after);
  });
  drop.addEventListener("dragover", (ev) => {
    ev.preventDefault();
    drop.classList.add("over");
  });
  drop.addEventListener("dragleave", () => drop.classList.remove("over"));
  drop.addEventListener("drop", (ev) => {
    ev.preventDefault();
    drop.classList.remove("over");
    const item = ev.dataTransfer.files[0];
    if (item) runUpload(item, name ? name.value : "", notes ? notes.value : "", msg, after);
  });
}

async function runUpload(file, name, notes, msg, after) {
  msg.textContent = "Загрузка…";
  msg.classList.add("muted");
  try {
    const res = await fetch("/api/upload", {
      method: "POST",
      headers: {
        "X-Filename": encodeURIComponent(file.name),
        "X-Name": encodeURIComponent(name || file.name.replace(/\.[^.]+$/, "")),
        "X-Notes": encodeURIComponent(notes || ""),
      },
      body: file,
    });
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || "Не удалось загрузить");
    msg.textContent = "Файл принят, разбирается.";
    addTimer(setTimeout(() => route(), 1500));
    if (after) after();
  } catch (err) {
    msg.textContent = err.message;
    msg.classList.remove("muted");
    msg.classList.add("bad-text");
  }
}

function uploadProgressHtml(job) {
  if (!job) return "";
  const pct = Math.round((job.progress || 0) * 100);
  return `
    <div class="uprog">
      <div class="row"><span>Разбор <b>${job.name}</b></span><span>${pct}%</span></div>
      <div class="progress"><span style="width:${pct}%"></span></div>
    </div>`;
}

async function pageOverview() {
  const [data, bagData, repData] = await Promise.all([
    api("/api/overview"),
    api("/api/bags"),
    api("/api/reports"),
  ]);
  setSystem(data.run);
  const latest = data.latest;
  const rows = (latest && latest.rows) || [];
  const fpsSeries = rows.map((r) => r.fps).filter((v) => v != null);
  const latSeries = rows.map((r) => r.latency_ms).filter((v) => v != null);
  let acc = 0;
  const alertSeries = rows.map((r) => {
    if (r.status === "BLOCKED") acc += 1;
    return acc;
  });
  const hz = latest && latest.fps ? latest.fps : 10;
  const uptime = latest ? latest.frames / hz : null;
  const blocked = latest && latest.blocked > 0;
  const phrase = decisionPhrase(latest);
  const job = (bagData.jobs || []).find((j) => j.status === "parsing");
  const bags = bagData.items || [];
  const ready = bags.filter((b) => b.status === "ready" && !b.holdout);
  const demo = ready.find((b) => b.id === "doubleT_obstacle") || ready[0];
  const allReports = repData.items || [];
  const demoReport = reportForBag(allReports, demo && demo.id);
  const alertFrame = demoReport && demoReport.first_blocked_frame != null ? demoReport.first_blocked_frame : 0;
  const reports = allReports.slice(0, 3);
  const openBtn = demo
    ? `<a href="#/efir/${encodeURIComponent(demo.id)}"><button class="btn accent" type="button">Открыть эфир</button></a>`
    : `<a href="#/zapis"><button class="btn accent" type="button">Загрузить запись</button></a>`;
  view.innerHTML = `
    <div class="metrics">
      ${metricCard("ЧАСТОТА", latest && latest.fps != null ? fmtNum(latest.fps, 1) : "—", fpsSeries, "#8a9199")}
      ${metricCard("ЗАДЕРЖКА", latest && latest.latency_ms && latest.latency_ms.median != null ? `${fmtNum(latest.latency_ms.median, 0)} <small>мс</small>` : "—", latSeries, "#8a9199")}
      ${metricCard("ТРЕВОГИ", latest ? fmtInt(latest.blocked) : "—", alertSeries, "#b95c4f")}
      ${metricCard("ДЛИТЕЛЬНОСТЬ", uptime ? fmtDuration(uptime) : "—", rows.map((_, i) => i), "#8a9199")}
    </div>
    <div class="ov-grid">
      <div class="stack">
        <section class="card">
          <h2>Загрузка записи</h2>
          <div id="ov-drop" class="drop">
            <div>
              ${ICO.up}
              <p style="margin:8px 0 0">Перетащите .zst или папку bag</p>
            </div>
          </div>
          <div class="actions" style="margin-top:12px">
            <button id="ov-browse" class="btn accent" type="button">Выбрать файлы</button>
          </div>
          <input id="ov-file" type="file" accept=".zst,.tar,.yaml" hidden>
          <div id="ov-uprog">${uploadProgressHtml(job)}</div>
          <p id="ov-msg" class="hint"></p>
        </section>
        <section class="card">
          <h2>Состояние обнаружения</h2>
          <div class="scale-wrap">${scaleSvg(latest && latest.nearest_m != null ? latest.nearest_m : null)}</div>
          <div class="det-line">
            ${latest ? chip(blocked ? "ПРЕПЯТСТВИЕ" : latest.blocked ? "НЕ УВЕРЕН" : "ПУСТО", blocked ? "bad" : latest.blocked ? "warn" : "ok") : chip("НЕТ ОТЧЁТА", "")}
            <span class="phrase">${phrase}</span>
          </div>
          <div class="tunnel">${tunnelSvg()}</div>
          <div class="actions" style="margin-top:10px">${openBtn}</div>
        </section>
      </div>
      <div class="stack">
        <section class="card vp-card">
          <div class="card-head">
            <h2>Эфир</h2>
            <span id="ov-vpmeta" class="muted" style="font-size:11px"></span>
          </div>
          <div class="vp chipset" id="ov-vp" style="height:330px">
            <canvas id="ov-canvas"></canvas>
            <div class="vp-chips" id="ov-chips"></div>
            ${demo ? "" : '<div class="vp-note">Загрузите запись лидара</div>'}
          </div>
        </section>
        <section class="card">
          <h2>Последние отчёты</h2>
          ${reports.length ? `
          <table>
            <thead><tr><th>Отчёт</th><th>Дата</th><th>Размер</th><th>Статус</th><th></th></tr></thead>
            <tbody>
              ${reports.map((r) => `
                <tr>
                  <td><span class="td-flex"><span class="td-ico">${ICO.doc}</span>${r.name}</span></td>
                  <td class="muted">${new Date((r.mtime || 0) * 1000).toLocaleString("ru-RU", { day: "2-digit", month: "2-digit", hour: "2-digit", minute: "2-digit" })}</td>
                  <td class="muted">${fmtSize(r.size_bytes)}</td>
                  <td>${chip("ГОТОВО", "ok")}</td>
                  <td class="actions-cell"><a href="#/otchet/${encodeURIComponent(r.id)}"><button class="btn tiny" type="button">Открыть</button></a></td>
                </tr>
              `).join("")}
            </tbody>
          </table>` : empty("Отчётов нет. Загрузите запись и откройте её эфир.", "")}
        </section>
      </div>
    </div>
  `;
  wireUpload({
    drop: document.getElementById("ov-drop"),
    file: document.getElementById("ov-file"),
    browse: document.getElementById("ov-browse"),
    msg: document.getElementById("ov-msg"),
  });
  if (demo) {
    document.getElementById("ov-vp").addEventListener("click", () => {
      location.hash = `#/efir/${encodeURIComponent(demo.id)}`;
    });
    try {
      const cloud = await fetchCloud(demo.id, alertFrame);
      const canvas = document.getElementById("ov-canvas");
      fitCanvas(canvas);
      const ctx = canvas.getContext("2d");
      let gauge = window.__gaugeCache;
      if (!gauge) {
        try {
          gauge = await api("/api/gauge");
          window.__gaugeCache = gauge;
        } catch (err) {
          gauge = null;
        }
      }
      const teaserRow =
        demoReport && demoReport.blocked > 0 && demoReport.nearest_m != null
          ? { status: "BLOCKED", nearest_m: demoReport.nearest_m }
          : null;
      paintScene("perspective")(ctx, canvas, cloud.xyz, gauge, teaserRow, { cloud: 1, gauge: 1, box: 1, ground: 1 }, 0);
      const chips = document.getElementById("ov-chips");
      chips.innerHTML = `
        <span>кадр ${fmtInt(alertFrame + 1)} / ${fmtInt(cloud.total || (demo.frames || 0))}</span>
        <span class="sep">|</span>
        <span>${fmtNum(hz, 0)} Гц</span>
        ${teaserRow ? `<span class="sep">|</span><span class="st-bad">ПРЕПЯТСТВИЕ</span>` : ""}
      `;
      document.getElementById("ov-vpmeta").textContent = demo.name;
    } catch (err) {
      void err;
    }
  }
  if (job) addTimer(setTimeout(() => route(), 3000));
}

async function pageBags() {
  const data = await api("/api/bags");
  const jobs = data.jobs || [];
  const items = data.items || [];
  const jobRows = jobs.length
    ? jobs.map((j) => {
        if (["queued", "parsing", "analyzing", "starting", "running"].includes(j.status)) {
          const pct = Math.round((j.progress || 0) * 100);
          return `
            <div class="job">
              <span class="ico">${ICO.doc}</span>
              ${ringSvg(j.progress)}
              <div class="info">
                <div class="name">${j.name}</div>
                <div class="sub">${j.status === "queued" ? "Файл принят" : j.status === "parsing" ? "Распаковка архива" : "Детектор обрабатывает запись"}${j.notes ? ` · ${j.notes}` : ""}</div>
                <div class="bar"><div class="progress"><span style="width:${pct}%"></span></div></div>
              </div>
              ${chip(STATUS[j.status] || "ОБРАБОТКА", "warn")}
              <span></span>
            </div>`;
        }
        const kind = j.status === "ready" ? "ok" : j.status === "error" ? "bad" : "";
        return `
          <div class="job">
            <span class="ico">${ICO.doc}</span>
            <div class="info">
              <div class="name">${j.name}</div>
              <div class="sub">${j.error ? j.error : j.bag ? `${fmtSize(j.bag.size_bytes)} · ${fmtInt(j.bag.frames)} кадров` : ""}</div>
            </div>
            ${chip(STATUS[j.status] || (j.status === "ready" ? "ГОТОВО" : j.status === "incomplete" ? "НЕ СОБРАНА" : j.status === "error" ? "ОШИБКА" : j.status), kind)}
            ${j.report_id ? `<a href="#/otchet/${encodeURIComponent(j.report_id)}"><button class="btn tiny accent" type="button">Отчёт</button></a>` : j.status === "ready" && j.bag ? `<a href="#/efir/${encodeURIComponent(j.bag.id)}"><button class="btn sq" type="button" title="Эфир">${ICO.play}</button></a>` : "<span></span>"}
          </div>`;
      }).join("")
    : '<p class="muted" style="margin:6px 0">Очередь пуста.</p>';
  view.innerHTML = `
    <section class="card">
      <h2>Загрузка новой записи</h2>
      <div class="grid-2">
        <div id="dz-drop" class="drop drop-tall">
          <div>
            ${ICO.up}
            <p style="margin:8px 0 0">Перетащите архив .zst<br>или папку bag</p>
          </div>
        </div>
        <div class="stack" style="gap:14px">
          <label class="field">Имя источника
            <input id="dz-name" value="new_data" autocomplete="off" spellcheck="false">
          </label>
          <label class="field">Примечание
            <input id="dz-notes" value="" autocomplete="off" spellcheck="false" placeholder="например: LCT-2026 основной набор">
          </label>
          <div class="actions">
            <button id="dz-browse" class="btn accent" type="button">Начать загрузку</button>
            <button id="dz-cancel" class="btn ghost" type="button">Отмена</button>
          </div>
        </div>
      </div>
      <input id="dz-file" type="file" accept=".zst,.tar,.yaml" hidden>
      <p id="dz-msg" class="hint"></p>
    </section>
    <section class="card" style="margin-top:16px">
      <h2>Очередь разбора</h2>
      ${jobRows}
    </section>
    <section class="card" style="margin-top:16px">
      <h2>Список записей</h2>
      ${items.length ? `
      <table>
        <thead><tr><th>Запись</th><th class="num">Размер</th><th class="num">Кадры</th><th>Длительность</th><th>Топик</th><th>Статус</th><th></th></tr></thead>
        <tbody>
          ${items.map((b) => `
            <tr>
              <td><span class="td-flex"><span class="td-ico">${ICO.doc}</span>${b.name}</span></td>
              <td class="muted">${fmtSize(b.size_bytes)}</td>
              <td class="muted">${fmtInt(b.frames)}</td>
              <td class="muted">${fmtDuration(b.duration_s)}</td>
              <td class="muted">${b.topic}</td>
              <td>${b.holdout ? chip("ОТЛОЖЕНА", "warn") : b.status === "ready" ? chip("ГОТОВО", "ok") : chip("НЕ СОБРАНА", "")}</td>
              <td class="actions-cell">
                ${b.holdout
                  ? `<span class="muted" title="Закрыта до проверки">${ICO.lock}</span>`
                  : b.status === "ready"
                    ? `<a href="#/efir/${encodeURIComponent(b.id)}"><button class="btn tiny accent" type="button">Эфир</button></a>`
                    : '<span class="muted">—</span>'}
              </td>
            </tr>
          `).join("")}
        </tbody>
      </table>` : empty("Загрузите запись лидара.")}
    </section>
  `;
  const msg = document.getElementById("dz-msg");
  wireUpload({
    drop: document.getElementById("dz-drop"),
    file: document.getElementById("dz-file"),
    browse: document.getElementById("dz-browse"),
    name: document.getElementById("dz-name"),
    notes: document.getElementById("dz-notes"),
    msg,
  });
  document.getElementById("dz-cancel").addEventListener("click", () => {
    document.getElementById("dz-file").value = "";
    msg.textContent = "";
  });
  if (jobs.some((j) => ["queued", "parsing", "analyzing", "starting", "running"].includes(j.status))) addTimer(setTimeout(() => route(), 3000));
}

async function pageLive(bagId) {
  const [bagData, repData] = await Promise.all([api("/api/bags"), api("/api/reports")]);
  const bags = bagData.items || [];
  const ready = bags.filter((b) => b.status === "ready" && !b.holdout);
  const preferred = ready.find((b) => b.id === "doubleT_obstacle") || ready[0];
  const selected = ready.find((b) => b.id === bagId) || preferred;
  const reports = repData.items || [];
  const report = selected && selected.synthetic ? null : reportForBag(reports, selected && selected.id);
  let gauge = null;
  try {
    gauge = await api("/api/gauge");
    window.__gaugeCache = gauge;
  } catch (err) {
    void err;
  }
  let groundOn = false;
  try {
    const yaml = await api("/api/params");
    groundOn = /method:\s*z_threshold/.test(yaml);
  } catch (err) {
    void err;
  }
  if (!selected) {
    view.innerHTML = `<h1 style="margin-bottom:14px">Эфир</h1>${empty(
      "Загрузите запись лидара.",
      '<a href="#/zapis"><button class="btn accent" type="button">Загрузить запись</button></a>'
    )}`;
    return;
  }
  const firstAlert = report && report.first_blocked_frame != null ? report.first_blocked_frame : -1;
  const requestedParams = new URLSearchParams(location.hash.split("?")[1] || "");
  const requestedFrame = requestedParams.get("frame");
  const requestedStamp = requestedParams.get("stamp");
  view.innerHTML = `
    <div class="live-grid">
      <section class="card vp-card">
        <div class="card-head">
          <h2>Эфир</h2>
          <div class="actions">
            <button id="view-plan" class="btn tiny" type="button">Сверху</button>
            <button id="view-ahead" class="btn tiny accent" type="button">Вперёд</button>
            <select id="bag-select" style="width:auto" aria-label="Запись">
              ${ready.map((b) => `<option value="${b.id}" ${b.id === selected.id ? "selected" : ""}>${b.name}</option>`).join("")}
            </select>
          </div>
        </div>
        <div class="vp live-vp">
          <canvas id="cloud"></canvas>
          <div class="vp-chips" id="vp-chips"></div>
          <div class="gizmo">${gizmoSvg()}</div>
        </div>
        <div class="player">
          <button id="play" class="btn sq" type="button" aria-label="Играть">${ICO.play}</button>
          <span class="count" id="count">0 / ${fmtInt(selected.frames)}</span>
          <input id="scrub" type="range" min="0" max="${Math.max(0, (selected.frames || 1) - 1)}" value="0" aria-label="Кадр">
          <button id="prev" class="btn sq" type="button" aria-label="Кадр назад">${ICO.prev}</button>
          <button id="next" class="btn sq" type="button" aria-label="Кадр вперёд">${ICO.next}</button>
          <select id="speed" aria-label="Скорость">
            <option value="0.5">0.5x</option>
            <option value="1">1.0x</option>
            <option value="2">2.0x</option>
            <option value="4">4.0x</option>
          </select>
          <label class="follow">${checkRow("follow", false)}Следить за препятствием</label>
          ${firstAlert >= 0 ? '<button id="to-alert" class="btn tiny" type="button">К тревоге</button>' : ""}
        </div>
      </section>
      <div class="stack">
        <section class="card">
          <h2>Обнаружение</h2>
          <div class="big-det">
            <div>
              <div class="num" id="det-num">—</div>
              <div class="cap">ближайшее препятствие</div>
            </div>
            <span id="det-badge"></span>
          </div>
          <div class="scale-wrap" style="margin-top:12px">${scaleSvg(null)}</div>
        </section>
        <section class="card">
          <h2>Препятствие</h2>
          <div class="kv2" id="obst-kv">
            ${pair("Дальность", "—")}
          </div>
        </section>
        <section class="card">
          <h2>Кадр</h2>
          <div class="kv2" id="frame-kv"></div>
        </section>
        <section class="card">
          <h2>Слои</h2>
          <div id="layers"></div>
        </section>
      </div>
    </div>
  `;
  document.getElementById("bag-select").addEventListener("change", (ev) => {
    location.hash = `#/efir/${encodeURIComponent(ev.target.value)}`;
  });
  const canvas = document.getElementById("cloud");
  const ctx = canvas.getContext("2d");
  const scrub = document.getElementById("scrub");
  const playBtn = document.getElementById("play");
  const speedSel = document.getElementById("speed");
  const byStamp = new Map();
  (report && report.rows ? report.rows : []).forEach((row) => {
    if (row.stamp_ns != null) byStamp.set(String(row.stamp_ns), row);
  });
  let stamps = [];
  let frame = requestedFrame != null ? Number(requestedFrame) : firstAlert >= 0 ? firstAlert : 0;
  scrub.value = String(frame);
  let playing = false;
  let loading = false;
  let viewMode = "ahead";
  const layers = { cloud: true, gauge: true, box: true, ground: true };
  let focusSide = 0;
  const play = window.__play || { speed: 1, loop: false };
  speedSel.value = String(play.speed);

  const layerDefs = [
    ["lay-cloud", "Облако точек", "cloud", false],
    ["lay-gauge", "Границы габарита", "gauge", false],
    ["lay-box", "Бокс препятствия", "box", false],
    ["lay-ground", "Маска пола", "ground", false],
  ];
  document.getElementById("layers").innerHTML = layerDefs
    .map(([id, label, key, dis]) => switchRow(id, label, layers[key], dis))
    .join("");
  layerDefs.forEach(([id, , key, dis]) => {
    const el = document.getElementById(id);
    if (dis) return;
    el.addEventListener("change", () => {
      layers[key] = el.checked;
      drawFrame();
    });
  });

  document.getElementById("view-ahead").addEventListener("click", () => setViewMode("ahead"));
  document.getElementById("view-plan").addEventListener("click", () => setViewMode("plan"));

  function setViewMode(mode) {
    viewMode = mode;
    document.getElementById("view-ahead").classList.toggle("accent", mode === "ahead");
    document.getElementById("view-plan").classList.toggle("accent", mode === "plan");
    drawFrame();
  }

  function setPlaying(on) {
    playing = on;
    playBtn.innerHTML = on ? ICO.pause : ICO.play;
    playBtn.setAttribute("aria-label", on ? "Пауза" : "Играть");
  }

  playBtn.addEventListener("click", () => setPlaying(!playing));
  scrub.addEventListener("input", () => {
    frame = Number(scrub.value);
    setPlaying(false);
    drawFrame();
  });
  document.getElementById("prev").addEventListener("click", () => {
    frame = Math.max(0, frame - 1);
    setPlaying(false);
    drawFrame();
  });
  document.getElementById("next").addEventListener("click", () => {
    frame = Math.min(Number(scrub.max), frame + 1);
    setPlaying(false);
    drawFrame();
  });
  speedSel.addEventListener("change", () => {
    play.speed = Number(speedSel.value) || 1;
    window.__play = play;
    restartInterval();
  });
  const jump = document.getElementById("to-alert");
  if (jump) {
    jump.addEventListener("click", () => {
      frame = firstAlert;
      setPlaying(false);
      drawFrame();
    });
  }

  let loopTimer = null;
  function restartInterval() {
    if (loopTimer) clearInterval(loopTimer);
    const speed = Number(speedSel.value) || 1;
    loopTimer = addTimer(
      setInterval(() => {
        if (!playing || loading) return;
        frame += 1;
        if (frame > Number(scrub.max)) {
          if (!play.loop) {
            frame = Number(scrub.max);
            setPlaying(false);
            return;
          }
          frame = 0;
        }
        drawFrame();
      }, Math.round(100 / speed))
    );
  }

  async function drawFrame() {
    if (loading) return;
    loading = true;
    try {
      const cloud = await fetchCloud(selected.id, frame, selected.synthetic);
      const total = cloud.total || selected.frames || 1;
      const stamp = stamps[frame] || cloud.stamp || "";
      const row = stamp && stamp !== "0" ? byStamp.get(String(stamp)) || null : null;
      const v = verdictOf(report, row, window.__view);
      if (layers.box && v.code === "blocked") {
        focusSide = obstacleFocus(cloud.xyz, gauge, v.meters);
      } else {
        focusSide = 0;
      }
      fitCanvas(canvas);
      paintScene(viewMode)(ctx, canvas, cloud.xyz, gauge, row, layers, focusSide);
      document.getElementById("count").textContent = `${fmtInt(frame + 1)} / ${fmtInt(total)}`;
      scrub.max = String(Math.max(0, total - 1));
      scrub.value = String(frame);
      const hz = (report && report.fps) || selected.rate_hz || 10;
      document.getElementById("vp-chips").innerHTML = `
        <span>кадр ${fmtInt(frame + 1)} / ${fmtInt(total)}</span>
        <span class="sep">|</span>
        <span>${fmtNum(hz, 0)} Гц</span>
        ${v.code !== "not_run" ? `<span class="sep">|</span><span class="st-${v.code === "blocked" ? "bad" : v.code === "clear" ? "ok" : "warn"}">${v.chip}</span>` : ""}
      `;
      const badge = document.getElementById("det-badge");
      badge.innerHTML = chip(v.chip, v.code === "blocked" ? "bad" : v.code === "clear" ? "ok" : v.code === "unsure" ? "warn" : "", false);
      document.getElementById("det-num").textContent = v.meters != null ? `${fmtNum(v.meters, 1)} м` : "—";
      document.getElementById("det-num").className = `num ${v.code === "blocked" ? "bad-text" : v.code === "clear" ? "ok-text" : v.code === "unsure" ? "warn-text" : "muted"}`;
      document.querySelector(".scale-wrap").innerHTML = scaleSvg(v.meters);
      document.getElementById("obst-kv").innerHTML = [
        pair("Дальность", v.meters != null ? fmtMeters(v.meters) : "—"),
        pair("Вперёд", row && row.forward_m != null ? fmtMeters(row.forward_m) : "—"),
        pair("Объектов", row ? fmtInt(row.objects) : "—"),
        ...(row && row.obstacles ? row.obstacles.map((obstacle) => pair(`Габариты трека ${obstacle.track_id}`, obstacle.extent.map((value) => fmtNum(value, 2)).join(" × ") + " м")) : []),
        ...(row && row.candidates ? row.candidates.map((candidate) => pair(`Кандидат ${candidate.candidate_id}`, candidate.extent.map((value) => fmtNum(value, 2)).join(" × ") + " м")) : []),
        pair("Статус", v.code === "blocked" ? '<span class="ok-text">ПОДТВЕРЖДЁН</span>' : v.code === "not_covered" ? '<span class="warn-text">НЕТ РЕЗУЛЬТАТА</span>' : v.code === "unsure" ? '<span class="warn-text">НЕ УВЕРЕН</span>' : v.code === "not_run" ? "—" : '<span class="muted">—</span>'),
      ].join("");
      document.getElementById("frame-kv").innerHTML = [
        pair("Частота", (row && row.fps != null ? fmtNum(row.fps, 1) : fmtNum(hz, 1))),
        pair("Задержка", row && row.latency_ms != null ? `${fmtNum(row.latency_ms, 0)} мс` : "—"),
        pair("Точек валидных", fmtInt(cloud.points)),
        pair("Точек сырых", fmtInt(cloud.raw)),
        pair("Маска пола", groundOn ? '<span class="ok-text">ВКЛ</span>' : "ВЫКЛ"),
      ].join("");
    } catch (err) {
      document.getElementById("vp-chips").innerHTML = `<span class="st-warn">${err.message}</span>`;
    } finally {
      loading = false;
    }
  }

  function obstacleFocus(xyz, g, nearest) {
    const hw = g ? g.half_width : 1.5;
    const sides = [];
    const n = xyz.length / 3;
    for (let i = 0; i < n; i += 2) {
      const [fwd, side] = planOf(xyz[i * 3], xyz[i * 3 + 1], g);
      if (fwd > nearest - 1.5 && fwd < nearest + 1.5 && Math.abs(side) <= hw) sides.push(side);
    }
    if (!sides.length) return 0;
    const mean = sides.reduce((a, b) => a + b, 0) / sides.length;
    return Math.max(-hw, Math.min(hw, mean)) * 0.8;
  }

  clearTimers();
  restartInterval();
  setPlaying(false);
  drawFrame();
  api(`/api/stamps?bag=${encodeURIComponent(selected.id)}`)
    .then((data2) => {
      stamps = data2.stamps || [];
      if (requestedStamp) {
        const stampFrame = stamps.indexOf(requestedStamp);
        if (stampFrame >= 0) {
          frame = stampFrame;
          drawFrame();
          return;
        }
      }
      const matched = stamps.findIndex((stamp) => {
        const row = byStamp.get(String(stamp));
        return row && row.status === "BLOCKED";
      });
      if (!playing && matched >= 0 && frame === (firstAlert >= 0 ? firstAlert : 0)) {
        frame = matched;
        drawFrame();
      }
    })
    .catch(() => {});
}

function timelineSvg(rows, frames, nearest) {
  const w = 560;
  const h = 168;
  const padL = 40;
  const padR = 20;
  const total = Math.max(1, frames / 10);
  const x = (sec) => padL + (sec / total) * (w - padL - padR);
  const blockedIdx = rows.map((r, i) => (r.status === "BLOCKED" ? i : -1)).filter((i) => i >= 0);
  let bands = "";
  if (blockedIdx.length) {
    let start = blockedIdx[0];
    let prev = blockedIdx[0];
    const segs = [];
    for (let k = 1; k < blockedIdx.length; k += 1) {
      if (blockedIdx[k] !== prev + 1) {
        segs.push([start, prev]);
        start = blockedIdx[k];
      }
      prev = blockedIdx[k];
    }
    segs.push([start, prev]);
    bands = segs
      .map(([a, b]) => `<rect x="${x(a / 10)}" y="52" width="${Math.max(2, x((b + 1) / 10) - x(a / 10))}" height="22" fill="#a05248" opacity="0.85"/>`)
      .join("");
  }
  const nearRow = nearest != null ? rows.find((r) => r.nearest_m != null && Math.abs(r.nearest_m - nearest) < 0.005) : null;
  const tick = nearRow ? `<rect x="${x(rows.indexOf(nearRow) / 10) - 1.5}" y="44" width="3" height="38" fill="#9aa4ad"/>` : "";
  let axis = `<line x1="${padL}" y1="96" x2="${w - padR}" y2="96" stroke="#3a424b"/>`;
  const step = total <= 24 ? 2 : 5;
  for (let s = 0; s <= total + 0.001; s += step) {
    axis += `<line x1="${x(s)}" y1="92" x2="${x(s)}" y2="100" stroke="#3a424b"/>`;
    axis += `<text x="${x(s)}" y="114" fill="#8a9199" font-size="9" text-anchor="middle">${Math.round(s)}</text>`;
  }
  axis += `<text x="${w - padR}" y="132" fill="#8a9199" font-size="9" text-anchor="end">Время (с)</text>`;
  const first = blockedIdx.length ? blockedIdx[0] + 1 : null;
  return `<svg viewBox="0 0 ${w} ${h}">${axis}${bands}${tick}</svg>
    <div class="legend">
      <span class="key"><span class="swatch" style="background:#a05248"></span>${first ? `Препятствие с кадра ${fmtInt(first)}` : "Тревог нет"}</span>
      ${nearRow ? `<span class="key"><span class="tick"></span>Ближайшее ${fmtNum(nearest, 1)} м</span>` : ""}
    </div>`;
}

function histogramSvg(values, med, p95) {
  const w = 560;
  const h = 168;
  const padL = 34;
  const padR = 20;
  const base = 140;
  const top = 26;
  if (!values.length) {
    return `<svg viewBox="0 0 ${w} ${h}"><text x="${w / 2}" y="${h / 2}" fill="#8a9199" font-size="11" text-anchor="middle">Нет данных о задержке</text></svg>`;
  }
  const hi = Math.max(20, Math.ceil(Math.max(...values, p95 || 0, med || 0) / 10) * 10);
  const bins = new Array(Math.ceil(hi / 2)).fill(0);
  values.forEach((v) => {
    const b = Math.min(bins.length - 1, Math.max(0, Math.floor(v / 2)));
    bins[b] += 1;
  });
  const peak = Math.max(...bins, 1);
  const bw = (w - padL - padR) / bins.length;
  const x = (ms) => padL + (ms / hi) * (w - padL - padR);
  let bars = "";
  bins.forEach((cnt, i) => {
    if (!cnt) return;
    const bh = ((base - top) * cnt) / peak;
    bars += `<rect x="${(padL + i * bw).toFixed(1)}" y="${(base - bh).toFixed(1)}" width="${(bw - 1.2).toFixed(1)}" height="${bh.toFixed(1)}" fill="#5c6c78" opacity="0.85"/>`;
  });
  let axis = `<line x1="${padL}" y1="${base}" x2="${w - padR}" y2="${base}" stroke="#3a424b"/>`;
  for (let ms = 0; ms <= hi; ms += 10) {
    axis += `<line x1="${x(ms)}" y1="${base - 4}" x2="${x(ms)}" y2="${base}" stroke="#3a424b"/>`;
    axis += `<text x="${x(ms)}" y="${base + 14}" fill="#8a9199" font-size="9" text-anchor="middle">${ms}</text>`;
  }
  axis += `<text x="${w - padR}" y="${base + 26}" fill="#8a9199" font-size="9" text-anchor="end">Задержка (мс)</text>`;
  const lines =
    med != null
      ? `<line x1="${x(med)}" y1="${top - 8}" x2="${x(med)}" y2="${base}" stroke="#4f88a8" stroke-dasharray="4 3"/>
         ${p95 != null ? `<line x1="${x(p95)}" y1="${top - 8}" x2="${x(p95)}" y2="${base}" stroke="#c05a4c" stroke-dasharray="4 3"/>` : ""}`
      : "";
  const legend = `
    <div class="legend">
      ${med != null ? `<span class="key"><span style="width:14px;border-top:2px dashed #4f88a8"></span>медиана ${fmtNum(med, 0)} мс</span>` : ""}
      ${p95 != null ? `<span class="key"><span style="width:14px;border-top:2px dashed #c05a4c"></span>p95 ${fmtNum(p95, 0)} мс</span>` : ""}
    </div>`;
  return `<svg viewBox="0 0 ${w} ${h}">${axis}${bars}${lines}</svg>${legend}`;
}

async function pageReports(reportId) {
  const data = await api("/api/reports");
  const items = data.items || [];
  if (!items.length) {
    view.innerHTML = `<h1 style="margin-bottom:14px">Отчёт</h1>${empty(
      "Отчётов нет. Загрузите запись и откройте её эфир.",
      '<a href="#/zapis"><button class="btn accent" type="button">Загрузить запись</button></a>'
    )}`;
    return;
  }
  if (!reportId) {
    view.innerHTML = `
      <section class="card">
        <h2>Отчёты</h2>
        <table>
          <thead><tr><th>Запись</th><th class="num">Кадры</th><th class="num">Препятствий</th><th>Ближайшее</th><th>Дата</th><th></th></tr></thead>
          <tbody>
            ${items.map((r) => `
              <tr>
                <td><span class="td-flex"><span class="td-ico">${ICO.doc}</span>${r.name}</span></td>
                <td class="muted">${fmtInt(r.frames)}</td>
                <td class="${r.blocked ? "bad-text" : "ok-text"}">${fmtInt(r.blocked)}</td>
                <td>${r.nearest_m != null ? fmtMeters(r.nearest_m) : "—"}</td>
                <td class="muted">${new Date((r.mtime || 0) * 1000).toLocaleString("ru-RU", { day: "2-digit", month: "2-digit", hour: "2-digit", minute: "2-digit" })}</td>
                <td class="actions-cell"><a href="#/otchet/${encodeURIComponent(r.id)}"><button class="btn tiny accent" type="button">Открыть</button></a></td>
              </tr>
            `).join("")}
          </tbody>
        </table>
      </section>`;
    return;
  }
  const current = items.find((r) => r.id === reportId) || items[0];
  const rows = current.rows || [];
  const latSeries = rows.map((r) => r.latency_ms).filter((v) => v != null);
  let acc = 0;
  const blockedSeries = rows.map((r) => {
    if (r.status === "BLOCKED") acc += 1;
    return acc;
  });
  const clearSeries = rows.map((r, i) => i + 1 - blockedSeries[i]);
  const nearestSeries = rows.map((r) => r.nearest_m).filter((v) => v != null);
  const alertRow = rows.find((r) => r.status === "BLOCKED");
  const liveLink = current.bag_id ? `#/efir/${encodeURIComponent(current.bag_id)}` : "#/efir";
  const detectedObstacles = (alertRow && alertRow.obstacles) || [];
  const degradedRows = rows.filter((row) => row.status === "DEGRADED");
  const obstacleText = detectedObstacles.length
    ? detectedObstacles.map((obstacle) => `${obstacle.track_id}: ${obstacle.extent.map((value) => fmtNum(value, 2)).join(" × ")} м`).join(" · ")
    : alertRow
      ? "Для кадра детектор не передал геометрию препятствия"
      : "В отчёте нет подтверждённого препятствия";
  view.innerHTML = `
    <div class="report-top">
      <div class="title"><span>ОТЧЁТ</span>${current.name}</div>
      <div class="actions">
        <a href="/api/reports/${encodeURIComponent(current.id)}.csv"><button class="btn" type="button">Скачать CSV</button></a>
        <a href="${liveLink}"><button class="btn accent" type="button">${alertRow ? "К тревоге" : "К эфиру"}</button></a>
        <a href="#/otchet"><button class="btn ghost" type="button">К списку</button></a>
      </div>
    </div>
    <div class="summary-row">
      ${metricCard("КАДРЫ", fmtInt(current.frames), rows.map((_, i) => i), "#8a9199")}
      ${metricCard("ПРЕПЯТСТВИЯ", fmtInt(current.blocked), blockedSeries, "#b95c4f")}
      ${metricCard("ПУСТО", fmtInt(current.clear), clearSeries, "#7fa97f")}
      ${metricCard("БЛИЖАЙШЕЕ", current.nearest_m != null ? fmtNum(current.nearest_m, 1) : "—", nearestSeries, "#b95c4f")}
      ${metricCard("P95 ЗАДЕРЖКА", current.latency_ms && current.latency_ms.p95 != null ? `${fmtNum(current.latency_ms.p95, 0)} <small>мс</small>` : "—", latSeries, "#8a9199")}
    </div>
    <div class="charts">
      <section class="card">
        <h2>График тревог</h2>
        <div class="chart">${timelineSvg(rows, current.frames, current.nearest_m)}</div>
      </section>
      <section class="card">
        <h2>Задержка</h2>
        <div class="chart">${histogramSvg(latSeries, (current.latency_ms || {}).median, (current.latency_ms || {}).p95)}</div>
      </section>
    </div>
    <section class="card">
      <h2>${alertRow ? "Кадр тревоги" : "Кадр без подтверждённой тревоги"}</h2>
      <p class="muted">${alertRow ? `Кадр ${fmtInt((alertRow.index || 0) + 1)} · ${fmtMeters(alertRow.nearest_m)}` : "В отчёте нет кадра с препятствием"}</p>
      <p>${obstacleText}</p>
      ${degradedRows.length ? `<p class="warn-text">Не полностью обработано кадров: ${fmtInt(degradedRows.length)}. По ним детектор не подтвердил отсутствие препятствия.</p>` : ""}
      ${alertRow && current.bag_id ? `<a href="#/efir/${encodeURIComponent(current.bag_id)}?stamp=${encodeURIComponent(alertRow.stamp_ns || "")}"><button class="btn accent" type="button">Открыть кадр с облаком</button></a><img class="report-frame" src="/api/reports/${encodeURIComponent(current.id)}/frame.png?frame=${encodeURIComponent(alertRow.index || 0)}&stamp=${encodeURIComponent(alertRow.stamp_ns || "")}" alt="Облако точек в кадре тревоги">` : ""}
    </section>
    <section class="card">
      <h2>Таблица кадров</h2>
      <div style="overflow-x:auto">
      <table>
        <thead><tr><th class="num">Кадр</th><th>Время</th><th>Состояние</th><th class="num">Дальность, м</th><th class="num">Вперёд, м</th><th class="num">Объекты</th><th class="num">Задержка, мс</th></tr></thead>
        <tbody id="frame-rows"></tbody>
      </table>
      </div>
      <div class="pager">
        <span class="info" id="pager-info"></span>
        <span class="btns">
          <button id="pg-prev" class="btn sq" type="button" aria-label="Назад">${ICO.chevL}</button>
          <button id="pg-next" class="btn sq" type="button" aria-label="Вперёд">${ICO.chevR}</button>
        </span>
      </div>
      <details class="more">
        <summary>Подробности</summary>
        <p class="muted" style="font-size:11px">
          Медиана задержки ${current.latency_ms && current.latency_ms.median != null ? `${fmtNum(current.latency_ms.median, 0)} мс` : "—"} ·
          максимум ${current.latency_ms && current.latency_ms.max != null ? `${fmtNum(current.latency_ms.max, 0)} мс` : "—"} ·
          файл ${current.file_name}
        </p>
      </details>
    </section>
  `;
  const pageSize = 8;
  let page = 0;
  const pages = Math.max(1, Math.ceil(rows.length / pageSize));
  function renderRows() {
    const slice = rows.slice(page * pageSize, page * pageSize + pageSize);
    document.getElementById("frame-rows").innerHTML = slice
      .map((r) => {
        const st = r.status === "BLOCKED" ? chip("ПРЕПЯТСТВИЕ", "bad") : r.status === "CLEAR" ? chip("ПУСТО", "ok") : chip("НЕТ ДАННЫХ", "");
        return `
          <tr>
            <td>${fmtInt((r.index || 0) + 1)}</td>
            <td class="muted">${fmtClock(r.stamp_ns)}</td>
            <td>${st}</td>
            <td>${r.nearest_m != null ? fmtNum(r.nearest_m, 1) : "—"}</td>
            <td>${r.forward_m != null ? fmtNum(r.forward_m, 2) : "—"}</td>
            <td>${r.objects ? fmtInt(r.objects) : "—"}</td>
            <td>${r.latency_ms != null ? fmtNum(r.latency_ms, 0) : "—"}</td>
          </tr>`;
      })
      .join("");
    document.getElementById("pager-info").textContent = `показаны ${page * pageSize + 1}–${Math.min(rows.length, (page + 1) * pageSize)} из ${fmtInt(rows.length)}`;
    document.getElementById("pg-prev").disabled = page === 0;
    document.getElementById("pg-next").disabled = page >= pages - 1;
  }
  document.getElementById("pg-prev").addEventListener("click", () => {
    page = Math.max(0, page - 1);
    renderRows();
  });
  document.getElementById("pg-next").addEventListener("click", () => {
    page = Math.min(pages - 1, page + 1);
    renderRows();
  });
  renderRows();
}

function yamlSection(text, name) {
  const out = {};
  let cur = null;
  text.split("\n").forEach((line) => {
    const sec = line.match(/^ {4}([a-z_]+):\s*$/);
    if (sec) {
      cur = sec[1];
      return;
    }
    if (cur !== name) return;
    const kv = line.match(/^ {6}([a-z_]+):\s*(.+?)\s*$/);
    if (kv) out[kv[1]] = kv[2].replace(/^["']|["']$/g, "");
  });
  return out;
}

async function pageParams() {
  const [gauge, yaml, bagData] = await Promise.all([api("/api/gauge"), api("/api/params"), api("/api/bags")]);
  const ground = yamlSection(yaml, "ground_segmentation");
  const geom = yamlSection(yaml, "detector_geometry");
  const noRet = yamlSection(yaml, "detector_no_return");
  const useFs = (yamlSection(yaml, "fusion").use_free_space || "false") === "true";
  const bags = bagData.items || [];
  const play = window.__play || { speed: 1, loop: false, bag: "" };
  const chosen = bags.find((b) => b.id === play.bag && b.status === "ready") || bags.find((b) => b.status === "ready");
  const axes = [
    ["-y", "−Y"],
    ["+y", "+Y"],
    ["-x", "−X"],
    ["+x", "+X"],
  ];
  const fields = [
    ["half_width", "Полуширина, м", gauge.half_width],
    ["height", "Высота, м", gauge.height],
    ["sensor_height", "Высота сенсора, м", gauge.sensor_height],
    ["base_offset", "Смещение от рельса, м", gauge.base_offset],
    ["safety_margin", "Запас по высоте и перед носом, м", gauge.safety_margin],
  ];
  view.innerHTML = `
    <div class="settings-grid">
      <div class="stack">
        <section class="card">
          <h2>Габарит</h2>
          <form id="gauge-form">
            <div class="gauge-grid">
              ${fields
                .map(
                  ([key, label, value]) => `
                  <label class="field">${label}
                    <input id="${key}" name="${key}" type="number" step="0.01" min="0.05" value="${Number(value).toFixed(2)}">
                  </label>`
                )
                .join("")}
              <label class="field">Ось вперёд
                <select id="forward_axis">
                  ${axes.map(([v, l]) => `<option value="${v}" ${v === gauge.forward_axis ? "selected" : ""}>${l}</option>`).join("")}
                </select>
              </label>
            </div>
          </form>
          <p class="settings-note">от головки рельса · габарит ${fmtNum(gauge.half_width * 2, 1)} × ${fmtNum(gauge.height, 1)} м · этими линиями эфир рисует границы</p>
        </section>
        <section class="card">
          <h2>Детектирование</h2>
          ${switchRow("ro-geom", "Геометрический остаток", (geom.enabled || "false") === "true", false)}
          ${switchRow("ro-nr", "Пропуски возвратов", (noRet.enabled || "false") === "true", false)}
          ${switchRow("ro-fs", "Голос свободного пространства", useFs, false)}
          ${switchRow("ro-ground", "Маска пола (порог по Z)", (ground.enabled || "false") === "true", false)}
          <div class="gauge-grid" style="margin-top:12px">
            <label class="field">Дальность от, м
              <input id="det-min" type="number" step="0.1" min="0" value="${geom.min_range || "4.0"}">
            </label>
            <label class="field">Дальность до, м
              <input id="det-max" type="number" step="0.1" min="0" value="${geom.max_target_range_m || "45.0"}">
            </label>
          </div>
          <p class="settings-note">эфир скрывает точки вне этой дальности и отключает выбранные признаки</p>
        </section>
      </div>
      <div class="stack">
        <section class="card">
          <h2>Воспроизведение</h2>
          <div class="gauge-grid">
            <label class="field">Скорость
              <select id="play-speed">
                ${[0.5, 1, 2, 4].map((v) => `<option value="${v}" ${Number(play.speed) === v ? "selected" : ""}>${fmtNum(v, 1)}x</option>`).join("")}
              </select>
            </label>
            <label class="field">Запись
              <select id="play-bag">
                ${bags.filter((b) => b.status === "ready").map((b) => `<option value="${b.id}" ${chosen && b.id === chosen.id ? "selected" : ""}>${b.name}</option>`).join("")}
              </select>
            </label>
          </div>
          <label class="field" style="margin-top:12px">Топик лидара
            <input id="play-topic" type="text" value="${chosen ? chosen.topic : "/lidar_points"}" readonly>
          </label>
          ${switchRow("play-loop", "Цикл", Boolean(play.loop), false)}
          <p class="settings-note">скорость и цикл применяются к эфиру выбранной записи</p>
        </section>
      </div>
    </div>
    <div class="settings-actions">
      <button id="save-gauge" class="btn accent" type="button">Сохранить</button>
      <button id="reset-gauge" class="btn ghost" type="button">Сбросить</button>
      <span id="gauge-msg" class="hint" style="margin:0"></span>
    </div>
  `;
  const msg = document.getElementById("gauge-msg");
  document.getElementById("save-gauge").addEventListener("click", async () => {
    const body = {};
    fields.forEach(([key]) => {
      body[key] = Number(document.getElementById(key).value);
    });
    body.forward_axis = document.getElementById("forward_axis").value;
    window.__view = {
      geom: document.getElementById("ro-geom").checked,
      noReturn: document.getElementById("ro-nr").checked,
      freeSpace: document.getElementById("ro-fs").checked,
      ground: document.getElementById("ro-ground").checked,
      minRange: Number(document.getElementById("det-min").value),
      maxRange: Number(document.getElementById("det-max").value),
    };
    const bagId = document.getElementById("play-bag").value;
    const bag = bags.find((b) => b.id === bagId);
    window.__play = {
      speed: Number(document.getElementById("play-speed").value) || 1,
      loop: document.getElementById("play-loop").checked,
      bag: bagId,
    };
    try {
      await api("/api/gauge", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      msg.textContent = bag ? `Сохранено. Эфир: ${bag.name}.` : "Сохранено.";
      window.__gaugeCache = null;
    } catch (err) {
      msg.textContent = err.message;
    }
  });
  document.getElementById("play-bag").addEventListener("change", (ev) => {
    const bag = bags.find((b) => b.id === ev.target.value);
    document.getElementById("play-topic").value = bag ? bag.topic : "";
  });
  document.getElementById("reset-gauge").addEventListener("click", async () => {
    try {
      const fresh = await api("/api/gauge", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reset: true }),
      });
      fields.forEach(([key]) => {
        document.getElementById(key).value = Number(fresh[key]).toFixed(2);
      });
      document.getElementById("forward_axis").value = fresh.forward_axis;
      msg.textContent = "Показаны числа сдачи.";
    } catch (err) {
      msg.textContent = err.message;
    }
  });
}

async function route() {
  clearTimers();
  const hash = (location.hash || "#/").replace(/^#/, "") || "/";
  const [path] = hash.split("?");
  const parts = path.split("/").filter(Boolean);
  const head = `/${parts[0] || ""}`;
  setNav(head === "/" ? "/" : head);
  try {
    if (head === "/" || head === "") await pageOverview();
    else if (head === "/zapis") await pageBags();
    else if (head === "/efir") await pageLive(decodeURIComponent(parts[1] || ""));
    else if (head === "/otchet") await pageReports(decodeURIComponent(parts[1] || ""));
    else if (head === "/parametry") await pageParams();
    else view.innerHTML = empty("Нет такой страницы.");
  } catch (err) {
    view.innerHTML = banner(err.message);
  }
}

window.addEventListener("hashchange", route);
route();
