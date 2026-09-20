const view = document.getElementById("view");
const clock = document.getElementById("clock");
const pill = document.getElementById("system-pill");

const STATUS = {
  CLEAR: "свободно",
  WARNING: "внимание",
  BLOCKED: "препятствие",
  DEGRADED: "нет данных",
  ready: "готово",
  incomplete: "не собрана",
  holdout: "отложена",
  parsing: "разбирается",
  running: "идёт прогон",
  starting: "запуск",
  idle: "ожидает",
  error: "ошибка",
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
      /* keep */
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

function setSystem(run) {
  if (run && run.status === "running") {
    pill.textContent = "Идёт прогон";
    pill.className = "pill warn";
    return;
  }
  if (run && run.status === "error") {
    pill.textContent = "Прогон не удался";
    pill.className = "pill bad";
    return;
  }
  pill.textContent = "Система ожидает";
  pill.className = "pill";
}

function banner(text) {
  return `<div class="banner" role="alert">${text}</div>`;
}

function empty(text, action = "") {
  return `<div class="empty"><p>${text}</p>${action}</div>`;
}

async function pageOverview() {
  const data = await api("/api/overview");
  setSystem(data.run);
  const latest = data.latest;
  const blocked = latest && latest.blocked > 0;
  view.innerHTML = `
    <div class="grid metrics">
      <div class="panel"><h2>Кадр/с</h2><div class="num">${fmtNum(latest?.fps, 1)}</div></div>
      <div class="panel"><h2>Задержка</h2><div class="num">${fmtNum(latest?.latency_ms?.median, 0)} <small>мс</small></div></div>
      <div class="panel"><h2>Тревоги</h2><div class="num ${blocked ? "bad" : ""}">${latest ? fmtInt(latest.blocked) : "—"}</div></div>
      <div class="panel"><h2>Записей</h2><div class="num">${fmtInt(data.ready)} <small class="muted">/ ${fmtInt(data.bags)}</small></div></div>
    </div>
    <div class="grid two" style="margin-top:16px">
      <div class="stack">
        <div class="panel">
          <h2>Загрузить запись</h2>
          <p class="muted">Архив .zst или папка bag. После разбора можно смотреть эфир и отчёт.</p>
          <div class="actions" style="margin-top:12px">
            <a href="#/zapis"><button class="primary" type="button">Открыть загрузку</button></a>
          </div>
        </div>
        <div class="panel">
          <h2>Состояние габарита</h2>
          ${latest ? `
            <p class="${blocked ? "bad" : "ok"}">${blocked ? "Препятствие" : "Свободно"}</p>
            <p>${blocked ? `Ближайшее ${fmtNum(latest.nearest_m, 1)} м` : "В габарите ничего нет"}</p>
            <p class="muted">${latest.name}, кадров ${fmtInt(latest.frames)}</p>
          ` : empty("Ещё не было прогона.", `<a href="#/zapis"><button type="button">Загрузить запись</button></a>`)}
        </div>
      </div>
      <div class="panel">
        <h2>Последние отчёты</h2>
        ${await reportsTable(3)}
      </div>
    </div>
  `;
}

async function reportsTable(limit) {
  const data = await api("/api/reports");
  const items = (data.items || []).slice(0, limit || 20);
  if (!items.length) return empty("Отчётов нет. Загрузите запись и запустите прогон.");
  return `
    <table>
      <thead><tr><th>Запись</th><th>Кадры</th><th>Препятствие</th><th>Дальность</th><th></th></tr></thead>
      <tbody>
        ${items.map((r) => `
          <tr>
            <td>${r.name}</td>
            <td>${fmtInt(r.frames)}</td>
            <td class="${r.blocked ? "bad" : "ok"}">${fmtInt(r.blocked)}</td>
            <td>${r.nearest_m != null ? fmtNum(r.nearest_m, 1) + " м" : "—"}</td>
            <td><a href="#/otchet/${encodeURIComponent(r.id)}"><button type="button">Открыть</button></a></td>
          </tr>
        `).join("")}
      </tbody>
    </table>
  `;
}

async function pageBags() {
  const data = await api("/api/bags");
  const jobs = data.jobs || [];
  const items = data.items || [];
  view.innerHTML = `
    <h1>Запись</h1>
    <div class="grid two">
      <div class="panel">
        <h2>Новая запись</h2>
        <div id="drop" class="drop">Перетащите .zst или папку bag сюда</div>
        <div class="actions" style="margin-top:12px">
          <label class="muted">Имя источника
            <input id="src-name" value="new_data" autocomplete="off">
          </label>
          <button id="browse" class="primary" type="button">Выбрать файл</button>
          <input id="file" type="file" accept=".zst,.tar,.yaml" hidden>
        </div>
        <p id="upload-msg" class="muted" style="margin-top:10px"></p>
      </div>
      <div class="panel">
        <h2>Очередь</h2>
        ${jobs.length ? jobs.map((j) => `
          <p>${j.name} — ${STATUS[j.status] || j.status}
            ${j.progress != null ? ` (${fmtInt(Math.round(j.progress * 100))} %)` : ""}
          </p>
          <div class="progress"><span style="width:${Math.round((j.progress || 0) * 100)}%"></span></div>
        `).join("") : `<p class="muted">Очередь пуста.</p>`}
      </div>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Доступные записи</h2>
      ${items.length ? `
      <table>
        <thead><tr><th>Имя</th><th>Размер</th><th>Кадры</th><th>Длительность</th><th>Состояние</th><th></th></tr></thead>
        <tbody>
          ${items.map((b) => `
            <tr>
              <td>${b.name}</td>
              <td>${fmtSize(b.size_bytes)}</td>
              <td>${fmtInt(b.frames)}</td>
              <td>${fmtDuration(b.duration_s)}</td>
              <td class="${b.holdout ? "warn" : b.status === "ready" ? "ok" : "muted"}">${STATUS[b.status] || b.status}</td>
              <td>
                ${b.holdout ? "закрыта до проверки" : b.status === "ready"
                  ? `<a href="#/efir/${encodeURIComponent(b.id)}"><button type="button">Эфир</button></a>
                     <button data-run="${b.id}" type="button">Прогнать</button>`
                  : "неполная"}
              </td>
            </tr>
          `).join("")}
        </tbody>
      </table>` : empty("Записей нет. Положите bag в data или загрузите .zst.")}
    </div>
  `;
  const drop = document.getElementById("drop");
  const file = document.getElementById("file");
  const name = document.getElementById("src-name");
  const msg = document.getElementById("upload-msg");
  document.getElementById("browse").onclick = () => file.click();
  file.onchange = () => {
    if (file.files[0]) upload(file.files[0], name.value, msg);
  };
  drop.ondragover = (ev) => { ev.preventDefault(); };
  drop.ondrop = (ev) => {
    ev.preventDefault();
    const item = ev.dataTransfer.files[0];
    if (item) upload(item, name.value, msg);
  };
  view.querySelectorAll("[data-run]").forEach((btn) => {
    btn.onclick = async () => {
      btn.disabled = true;
      try {
        await api("/api/run", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ bag: btn.dataset.run, rate: 1 }),
        });
        location.hash = "#/efir/" + encodeURIComponent(btn.dataset.run);
      } catch (err) {
        msg.textContent = err.message;
      }
    };
  });
}

async function upload(file, name, msg) {
  msg.textContent = "Загрузка…";
  try {
    const res = await fetch("/api/upload", {
      method: "POST",
      headers: {
        "X-Filename": encodeURIComponent(file.name),
        "X-Name": encodeURIComponent(name || file.name.replace(/\.[^.]+$/, "")),
      },
      body: file,
    });
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || "Не удалось загрузить");
    msg.textContent = "Файл принят, разбирается.";
    setTimeout(() => route(), 1500);
  } catch (err) {
    msg.textContent = err.message;
  }
}

let liveTimer = null;

function stopLive() {
  if (liveTimer) {
    clearInterval(liveTimer);
    liveTimer = null;
  }
}

async function pageLive(bagId) {
  const bags = (await api("/api/bags")).items || [];
  const ready = bags.filter((b) => b.status === "ready" && !b.holdout);
  const selected = ready.find((b) => b.id === bagId) || ready[0];
  const reports = (await api("/api/reports")).items || [];
  const report = reports.find((r) => r.name === (selected && selected.id)) || reports[0];
  if (!selected) {
    view.innerHTML = `<h1>Эфир</h1>${empty("Нет готовой записи.", `<a href="#/zapis"><button type="button">Загрузить запись</button></a>`)}`;
    return;
  }
  view.innerHTML = `
    <div class="live">
      <div class="panel" style="display:flex;flex-direction:column">
        <div style="display:flex;justify-content:space-between;align-items:center">
          <h2>Эфир · ${selected.name}</h2>
          <span class="muted" id="live-meta">кадр 0 / ${fmtInt(selected.frames)}</span>
        </div>
        <canvas id="cloud" width="960" height="640"></canvas>
        <div class="player">
          <button id="play" type="button">Пауза</button>
          <input id="scrub" type="range" min="0" max="${Math.max(0, (selected.frames || 1) - 1)}" value="0">
        </div>
      </div>
      <div class="stack">
        <div class="panel">
          <h2>Габарит</h2>
          <p id="det-status" class="num">—</p>
          <p id="det-range" class="muted">нет кадра</p>
        </div>
        <div class="panel">
          <h2>Кадр</h2>
          <div class="kv" id="frame-kv"></div>
        </div>
        <div class="panel">
          <label class="muted">Запись
            <select id="bag-select">
              ${ready.map((b) => `<option value="${b.id}" ${b.id === selected.id ? "selected" : ""}>${b.name}</option>`).join("")}
            </select>
          </label>
        </div>
      </div>
    </div>
  `;
  document.getElementById("bag-select").onchange = (ev) => {
    location.hash = "#/efir/" + encodeURIComponent(ev.target.value);
  };
  const canvas = document.getElementById("cloud");
  const ctx = canvas.getContext("2d");
  const scrub = document.getElementById("scrub");
  const play = document.getElementById("play");
  let frame = 0;
  let playing = true;
  let loading = false;
  play.onclick = () => {
    playing = !playing;
    play.textContent = playing ? "Пауза" : "Играть";
  };
  scrub.oninput = () => {
    frame = Number(scrub.value);
    playing = false;
    play.textContent = "Играть";
    drawFrame();
  };

  async function drawFrame() {
    if (loading) return;
    loading = true;
    try {
      const res = await fetch(`/api/cloud?bag=${encodeURIComponent(selected.id)}&frame=${frame}`);
      if (!res.ok) throw new Error("Нет кадра");
      const buf = await res.arrayBuffer();
      const xyz = new Float32Array(buf);
      const points = Number(res.headers.get("X-Argus-Points") || xyz.length / 3);
      const raw = Number(res.headers.get("X-Argus-Raw") || points);
      const total = Number(res.headers.get("X-Argus-Total") || selected.frames);
      paintCloud(ctx, canvas, xyz);
      document.getElementById("live-meta").textContent =
        `кадр ${fmtInt(frame + 1)} / ${fmtInt(total)} · ${fmtInt(raw)} точек`;
      const row = report && report.rows ? report.rows[frame] : null;
      const blocked = row && row.status === "BLOCKED";
      document.getElementById("det-status").textContent = row ? row.status_label : "облако";
      document.getElementById("det-status").className = `num ${blocked ? "bad" : "ok"}`;
      document.getElementById("det-range").textContent = blocked
        ? `${fmtNum(row.nearest_m, 1)} м впереди`
        : "в габарите пусто";
      document.getElementById("frame-kv").innerHTML = `
        <span>Точек</span><span>${fmtInt(raw)}</span>
        <span>На экране</span><span>${fmtInt(points)}</span>
        <span>Задержка</span><span>${row && row.latency_ms != null ? fmtNum(row.latency_ms, 0) + " мс" : "—"}</span>
      `;
      scrub.max = Math.max(0, total - 1);
      scrub.value = String(frame);
    } catch (err) {
      document.getElementById("det-range").textContent = err.message;
    } finally {
      loading = false;
    }
  }

  stopLive();
  liveTimer = setInterval(() => {
    if (!playing || loading) return;
    frame += 1;
    if (frame > Number(scrub.max)) frame = 0;
    drawFrame();
  }, 100);
  drawFrame();
}

function paintCloud(ctx, canvas, xyz) {
  const w = canvas.width;
  const h = canvas.height;
  ctx.fillStyle = "#0b0d10";
  ctx.fillRect(0, 0, w, h);
  const n = xyz.length / 3;
  const img = ctx.createImageData(w, h);
  const data = img.data;
  for (let i = 0; i < n; i += 1) {
    const x = xyz[i * 3];
    const y = xyz[i * 3 + 1];
    const z = xyz[i * 3 + 2];
    const depth = -y;
    if (depth < 1.5 || depth > 80) continue;
    const u = 0.5 + (x / depth) * 1.15;
    const v = 0.62 - (z / depth) * 1.15;
    const px = Math.floor(u * w);
    const py = Math.floor(v * h);
    if (px < 0 || py < 0 || px >= w || py >= h) continue;
    const shade = Math.max(40, Math.min(210, 220 - depth * 2.2));
    const idx = (py * w + px) * 4;
    data[idx] = shade;
    data[idx + 1] = shade + 8;
    data[idx + 2] = shade + 14;
    data[idx + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

async function pageReports(reportId) {
  const data = await api("/api/reports");
  const items = data.items || [];
  const current = items.find((r) => r.id === reportId) || items[0];
  if (!current) {
    view.innerHTML = `<h1>Отчёт</h1>${empty("Отчётов нет.", `<a href="#/zapis"><button type="button">Загрузить запись</button></a>`)}`;
    return;
  }
  const lat = current.latency_ms || {};
  const rows = (current.rows || []).slice(0, 12);
  view.innerHTML = `
    <div class="panel" style="display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap">
      <h1 style="margin:0">Отчёт · ${current.name}</h1>
      <div class="actions">
        <a href="/api/reports/${encodeURIComponent(current.id)}.csv"><button type="button">Скачать таблицу</button></a>
        <label class="muted">Другой отчёт
          <select id="rep-select">
            ${items.map((r) => `<option value="${r.id}" ${r.id === current.id ? "selected" : ""}>${r.name}</option>`).join("")}
          </select>
        </label>
      </div>
    </div>
    <div class="grid metrics" style="margin-top:16px">
      <div class="panel"><h2>Кадры</h2><div class="num">${fmtInt(current.frames)}</div></div>
      <div class="panel"><h2>Препятствие</h2><div class="num bad">${fmtInt(current.blocked)}</div></div>
      <div class="panel"><h2>Свободно</h2><div class="num ok">${fmtInt(current.clear)}</div></div>
      <div class="panel"><h2>Ближайшее</h2><div class="num">${fmtNum(current.nearest_m, 1)} <small>м</small></div></div>
      <div class="panel"><h2>Задержка 95%</h2><div class="num">${fmtNum(lat.p95, 0)} <small>мс</small></div></div>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Кадры</h2>
      <table>
        <thead><tr><th>Кадр</th><th>Состояние</th><th>Дальность</th><th>Задержка</th></tr></thead>
        <tbody>
          ${rows.map((r) => `
            <tr>
              <td>${fmtInt(r.index)}</td>
              <td class="${r.status === "BLOCKED" ? "bad" : "ok"}">${r.status_label}</td>
              <td>${r.nearest_m != null ? fmtNum(r.nearest_m, 1) + " м" : "—"}</td>
              <td>${r.latency_ms != null ? fmtNum(r.latency_ms, 0) + " мс" : "—"}</td>
            </tr>
          `).join("")}
        </tbody>
      </table>
    </div>
  `;
  document.getElementById("rep-select").onchange = (ev) => {
    location.hash = "#/otchet/" + encodeURIComponent(ev.target.value);
  };
}

async function pageParams() {
  const text = await api("/api/params");
  view.innerHTML = `
    <h1>Параметры</h1>
    <p class="muted">Копия для панели. Текущий прогон detect пока всегда берёт файл сдачи.</p>
    <div class="panel">
      <textarea id="yaml">${text}</textarea>
      <div class="actions" style="margin-top:12px">
        <button id="reset" type="button">Вернуть сдачу</button>
        <button id="save" class="primary" type="button">Сохранить</button>
        <span id="params-msg" class="muted"></span>
      </div>
    </div>
  `;
  const box = document.getElementById("yaml");
  const msg = document.getElementById("params-msg");
  document.getElementById("save").onclick = async () => {
    await fetch("/api/params", { method: "POST", body: box.value });
    msg.textContent = "Сохранено для следующих прогонов панели.";
  };
  document.getElementById("reset").onclick = async () => {
    box.value = await api("/api/params?factory=1");
    msg.textContent = "Показан файл сдачи. Нажмите «Сохранить», чтобы им пользоваться.";
  };
}

async function route() {
  stopLive();
  const hash = (location.hash || "#/").replace(/^#/, "") || "/";
  const parts = hash.split("/").filter(Boolean);
  const head = "/" + (parts[0] || "");
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
