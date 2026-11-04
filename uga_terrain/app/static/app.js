"use strict";
// UGA Terrain Toolkit UI. Plain JS; talks to the local server under /api.

const $ = (id) => document.getElementById(id);
const S = {
  recordings: [], ticked: new Set(), maps: [], map: null, status: null,
  previews: {}, images: {}, roads: [], drives: [], summary: null,
  view: null, road: null, job: null, settings: null, three: null,
};

// --- helpers ---------------------------------------------------------------------
async function api(path, opts = {}) {
  const o = Object.assign({ headers: {} }, opts);
  if (o.body && typeof o.body !== "string") {
    o.body = JSON.stringify(o.body);
    o.headers["Content-Type"] = "application/json";
  }
  const r = await fetch(path, o);
  if (!r.ok) {
    let msg = r.statusText;
    try { msg = (await r.json()).detail || msg; } catch (e) { /* not JSON */ }
    throw new Error(msg);
  }
  const ct = r.headers.get("content-type") || "";
  return ct.includes("json") ? r.json() : r.arrayBuffer();
}
function toast(msg, bad = false) {
  const t = $("toast");
  t.textContent = msg;
  t.className = "toast" + (bad ? " bad" : "");
  t.hidden = false;
  clearTimeout(toast._t);
  toast._t = setTimeout(() => (t.hidden = true), bad ? 9000 : 4000);
}
const fmtSize = (b) => b > 1e9 ? (b / 1e9).toFixed(2) + " GB" : b > 1e6 ? (b / 1e6).toFixed(1) + " MB" : (b / 1e3).toFixed(0) + " kB";
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? "–" : Number(v).toFixed(d);
function el(tag, attrs = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null) e.setAttribute(k, v);
  }
  for (const k of kids) e.append(k instanceof Node ? k : document.createTextNode(String(k)));
  return e;
}
function guard(fn) {
  return async (...a) => { try { await fn(...a); } catch (e) { toast(e.message, true); } };
}

// --- tabs ------------------------------------------------------------------------
document.querySelectorAll(".tabs button").forEach((b) => b.addEventListener("click", () => {
  document.querySelectorAll(".tabs button").forEach((x) => x.classList.toggle("active", x === b));
  document.querySelectorAll(".tab").forEach((t) => t.classList.toggle("active", t.id === "tab-" + b.dataset.tab));
  if (b.dataset.tab === "map") drawMap();
  if (b.dataset.tab === "three") guard(open3D)();
  if (b.dataset.tab === "profile") drawProfile();
  if (b.dataset.tab === "settings") guard(loadSettings)();
}));

// --- recordings --------------------------------------------------------------------
async function loadRecordings() {
  const d = await api("/api/recordings");
  S.recordings = d.recordings;
  $("folders").textContent = "From " + (d.folders.join(", ") || "(no folders set, see Settings)");
  const list = $("rec-list");
  list.innerHTML = "";
  if (!S.recordings.length) list.append(el("div", { class: "muted" }, "No .mcap files found. Put recordings in the folder above, or change it in Settings."));
  for (const r of S.recordings) {
    const cb = el("input", { type: "checkbox" });
    cb.checked = S.ticked.has(r.path);
    cb.addEventListener("change", () => { cb.checked ? S.ticked.add(r.path) : S.ticked.delete(r.path); });
    const info = el("button", { class: "small", title: "Show topics and duration",
      onclick: guard(async (ev) => { ev.stopPropagation(); await inspectRec(r.path); }) }, "Info");
    const item = el("label", { class: "item" }, cb,
      el("div", { class: "grow" }, el("div", {}, r.rel), el("div", { class: "sub" }, fmtSize(r.size))), info);
    list.append(item);
  }
}
async function inspectRec(path) {
  toast("Reading " + path.split("/").pop() + " ...");
  const d = await api("/api/recordings/inspect", { method: "POST", body: { path } });
  const need = Object.entries(d.needed).map(([k, v]) => `${v.present ? "✓" : "✗"} ${k}: ${v.topic}`).join("\n");
  const dur = d.duration ? d.duration.toFixed(1) + " s" : "unknown length";
  toast(`${path.split("/").pop()}: ${dur}, ${d.topics.length} topics${d.indexed ? "" : " (no index; file was cut off?)"}\n${need}`);
}

// --- maps --------------------------------------------------------------------------
async function loadMaps(keep = true) {
  const d = await api("/api/maps");
  S.maps = d.maps;
  const list = $("map-list");
  list.innerHTML = "";
  if (!S.maps.length) list.append(el("div", { class: "muted" }, "No maps yet."));
  for (const m of S.maps) {
    const built = m.steps && m.steps.export;
    const item = el("div", { class: "item" + (S.map === m.slug ? " selected" : ""), onclick: guard(() => selectMap(m.slug)) },
      el("div", { class: "grow" }, el("div", {}, m.name), el("div", { class: "sub" }, `${m.recordings.length} recording(s)`)),
      el("span", { class: "pill" + (built ? " ok" : "") }, built ? "built" : "not built"));
    list.append(item);
  }
  if (keep && S.map && S.maps.find((m) => m.slug === S.map)) await selectMap(S.map, false);
  else if (!S.map && S.maps.length) await selectMap(S.maps[S.maps.length - 1].slug);
}
async function selectMap(slug, reload = true) {
  S.map = slug;
  document.querySelectorAll("#map-list .item").forEach((x, i) => x.classList.toggle("selected", S.maps[i] && S.maps[i].slug === slug));
  const st = await api(`/api/maps/${slug}`);
  S.status = st;
  $("map-actions").hidden = false;
  $("map-title").textContent = st.name;
  $("map-recs").innerHTML = "";
  st.recordings.forEach((r) => $("map-recs").append(el("div", {}, "• " + r.split("/").pop())));
  const steps = ["scan", "map", "roads", "export"].map((s) => (st.steps[s] ? "✓ " : "○ ") + s).join("   ");
  $("map-steps").textContent = (st.built ? "Last built " + st.built + "   " : "") + steps;
  [...$("rebuild-from").options].forEach((o) => (o.disabled = !st.can_start.includes(o.value)));
  $("btn-rebuild").disabled = st.can_start.length < 2;
  await loadSnapshots();
  if (reload) await loadResults();
}

// --- snapshots, export, import ------------------------------------------------------------
async function loadSnapshots() {
  const L = $("snap-list");
  L.innerHTML = "";
  if (!S.map) return;
  const { snapshots } = await api(`/api/maps/${S.map}/snapshots`);
  if (!snapshots.length) { L.append(el("div", { class: "muted" }, "No snapshots yet.")); return; }
  for (const sn of snapshots) {
    L.append(el("div", { class: "item", title: `Saved ${sn.saved}` + (sn.built ? `, map built ${sn.built}` : "") },
      el("div", { class: "grow" }, el("div", {}, sn.label), el("div", { class: "sub" }, `${sn.saved} · ${fmtSize(sn.size)}`)),
      el("button", { class: "small", title: "Put the map and settings back the way they were", onclick: guard(async (e) => {
        e.stopPropagation();
        if (!confirm(`Restore "${sn.label}" from ${sn.saved}?\n\nThe map's current results and settings are replaced. Save a snapshot first if you want to keep them.`)) return;
        await api(`/api/maps/${S.map}/snapshots/${sn.id}/restore`, { method: "POST", body: { settings: true } });
        toast(`Restored "${sn.label}".`);
        await loadMaps();
        if ($("tab-settings").classList.contains("active")) await loadSettings();
      }) }, "Restore"),
      el("button", { class: "small", title: "Delete this snapshot", onclick: guard(async (e) => {
        e.stopPropagation();
        if (!confirm(`Delete the snapshot "${sn.label}" from ${sn.saved}?`)) return;
        await api(`/api/maps/${S.map}/snapshots/${sn.id}`, { method: "DELETE" });
        await loadSnapshots();
      }) }, "✕")));
  }
}
async function saveSnapshot() {
  if (!S.map) throw new Error("Pick a map first.");
  const label = prompt("Name this snapshot (optional):", "");
  if (label === null) return;
  const sn = await api(`/api/maps/${S.map}/snapshots`, { method: "POST", body: { label } });
  toast(`Saved snapshot "${sn.label}" (${fmtSize(sn.size)}).`);
  await loadSnapshots();
}
$("btn-snapshot").addEventListener("click", guard(saveSnapshot));
window.addEventListener("keydown", (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s") { e.preventDefault(); guard(saveSnapshot)(); }
});
$("btn-export").addEventListener("click", () => {
  if (!S.map) return;
  const a = el("a", { href: `/api/maps/${S.map}/export?cache=${$("export-cache").checked}`, download: "" });
  document.body.append(a); a.click(); a.remove();
  toast("Exporting the map; your browser will save the .zip when it is ready.");
});
$("btn-import").addEventListener("click", () => $("import-file").click());
$("import-file").addEventListener("change", guard(async (e) => {
  const f = e.target.files[0];
  e.target.value = "";
  if (!f) return;
  toast(`Importing ${f.name} ...`);
  const r = await fetch(`/api/maps/import?filename=${encodeURIComponent(f.name)}`, { method: "POST", body: f,
    headers: { "Content-Type": "application/zip" } });
  if (!r.ok) { let m = r.statusText; try { m = (await r.json()).detail || m; } catch (_) { /* */ } throw new Error(m); }
  const st = await r.json();
  S.map = st.slug;
  await loadMaps();
  toast(`Imported "${st.name}".` + (st.steps && st.steps.export ? "" : " Build it to see results."));
}));
async function loadResults() {
  S.summary = S.status && S.status.summary;
  S.previews = {}; S.images = {}; S.roads = []; S.drives = []; S.road = null;
  if (S.three) { S.three.dispose(); S.three = null; }
  if (S.summary) {
    S.previews = await api(`/api/maps/${S.map}/previews`);
    S.roads = await api(`/api/maps/${S.map}/roads`);
    S.drives = await api(`/api/maps/${S.map}/drives`);
    if (S.roads.length) S.road = S.roads[0].id;
  }
  S.view = null;
  fillRoadSelect();
  await loadFiles();
  drawMap();
  drawProfile();
}

$("btn-create").addEventListener("click", guard(async () => {
  const name = $("new-name").value.trim();
  if (!name) throw new Error("Type a name for the map first.");
  if (!S.ticked.size) throw new Error("Tick at least one recording first.");
  const recs = S.recordings.filter((r) => S.ticked.has(r.path)).map((r) => r.path);
  const st = await api("/api/maps", { method: "POST", body: { name, recordings: recs } });
  $("new-name").value = "";
  S.map = st.slug;
  await loadMaps();
  toast(`Map "${name}" created with ${recs.length} recording(s). Press Build map.`);
}));
$("btn-add").addEventListener("click", guard(async () => {
  if (!S.ticked.size) throw new Error("Tick the recordings to add first.");
  const add = S.recordings.filter((r) => S.ticked.has(r.path) && !S.status.recordings.includes(r.path)).map((r) => r.path);
  if (!add.length) throw new Error("Those recordings are already in this map.");
  await api(`/api/maps/${S.map}`, { method: "PUT", body: { recordings: S.status.recordings.concat(add) } });
  await loadMaps();
  toast(`Added ${add.length} recording(s). Build map to merge them in.`);
}));
$("btn-build").addEventListener("click", guard(() => startBuild("scan")));
$("btn-rebuild").addEventListener("click", guard(() => startBuild($("rebuild-from").value)));
$("btn-refresh").addEventListener("click", guard(loadRecordings));

async function startBuild(start) {
  const d = await api(`/api/maps/${S.map}/build`, { method: "POST", body: { start } });
  S.job = d;
  pollJob();
}

// --- job -----------------------------------------------------------------------------
let polling = false;
async function pollJob() {
  if (polling) return;
  polling = true;
  try {
    while (true) {
      const { job } = await api("/api/job");
      showJob(job);
      if (!job || job.state !== "running") {
        if (job && S.job && S.job.state === "running") await jobFinished(job);
        S.job = job;
        break;
      }
      S.job = job;
      await new Promise((r) => setTimeout(r, 800));
    }
  } finally { polling = false; }
}
function showJob(j) {
  const bar = $("job-bar");
  $("btn-stop").hidden = !(j && j.state === "running");
  ["btn-build", "btn-rebuild", "btn-add", "btn-snapshot", "btn-import"].forEach((id) => ($(id).disabled = !!(j && j.state === "running")));
  if (!j) { bar.className = "job idle"; $("job-text").textContent = "No job running"; return; }
  const pct = Math.round(100 * j.done / Math.max(j.total, 1));
  $("job-fill").style.width = pct + "%";
  if (j.state === "running") {
    bar.className = "job";
    $("job-text").textContent = `${j.phase}: ${j.detail || ""} (${pct}%, ${Math.round(j.elapsed)} s)`;
  } else {
    bar.className = "job idle " + (j.state === "error" ? "error" : j.state === "done" ? "done" : "");
    $("job-text").textContent = j.state === "done" ? `Finished in ${Math.round(j.elapsed)} s`
      : j.state === "stopped" ? "Stopped" : "Failed: " + (j.error || "");
  }
  $("log").textContent = (j.log || []).join("\n");
}
async function jobFinished(j) {
  if (j.state === "done") toast("Map built.");
  else if (j.state === "error") toast("The build failed: " + j.error, true);
  await loadMaps();
}
$("btn-stop").addEventListener("click", guard(async () => { await api("/api/job/stop", { method: "POST" }); }));

// --- 2D map ------------------------------------------------------------------------------
const cv = $("map-canvas");
const ctx = cv.getContext("2d");
function getImage(name) {
  if (S.images[name]) return S.images[name];
  const img = new Image();
  img.onload = () => drawMap();
  img.src = `/api/maps/${S.map}/preview/${S.previews[name].file}?t=${S.status.built || ""}`;
  S.images[name] = img;
  return img;
}
function fitView() {
  const p = Object.values(S.previews)[0];
  if (!p) return;
  const [x0, y0, x1, y1] = dataBounds(p.extent);
  const w = cv.clientWidth, h = cv.clientHeight;
  const sc = Math.min(w / Math.max(x1 - x0, 1), h / Math.max(y1 - y0, 1)) * 0.92;
  S.view = { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, s: sc, rot: 0 };
}
function dataBounds(ext) {
  let xs = [], ys = [];
  for (const d of S.drives) { xs = xs.concat(d.x); ys = ys.concat(d.y); }
  if (!xs.length) return ext;
  const pad = 40;
  return [Math.min(...xs) - pad, Math.min(...ys) - pad, Math.max(...xs) + pad, Math.max(...ys) + pad];
}
// world (x east, y north) <-> canvas pixels; the view can be rotated by S.view.rot (rad, CCW)
function toPx(x, y) {
  const c = Math.cos(S.view.rot), s = Math.sin(S.view.rot), k = S.view.s * devicePixelRatio;
  const dx = x - S.view.cx, dy = y - S.view.cy;
  return [cv.width / 2 + (c * dx - s * dy) * k, cv.height / 2 - (s * dx + c * dy) * k];
}
function toW(px, py) {      // px, py in CSS pixels
  const c = Math.cos(S.view.rot), s = Math.sin(S.view.rot), k = S.view.s * devicePixelRatio;
  const rx = (px * devicePixelRatio - cv.width / 2) / k, ry = -(py * devicePixelRatio - cv.height / 2) / k;
  return [S.view.cx + c * rx + s * ry, S.view.cy - s * rx + c * ry];
}
function drawMap() {
  if (!$("tab-map").classList.contains("active")) return;
  const w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w * devicePixelRatio; cv.height = h * devicePixelRatio;
  ctx.clearRect(0, 0, cv.width, cv.height);
  const layer = $("layer").value;
  const p = S.previews[layer] || S.previews.elevation;
  $("map-empty").hidden = !!p;
  if (!p) return;
  if (!S.view) fitView();
  const img = getImage(layer in S.previews ? layer : "elevation");
  if (img.complete && img.naturalWidth) {
    const [x0, y0, x1, y1] = p.extent;
    const [a, b] = toPx(x0, y1);
    const k = S.view.s * devicePixelRatio;
    ctx.imageSmoothingEnabled = S.view.s * (x1 - x0) / p.width < 2;
    ctx.save();
    ctx.translate(a, b);
    ctx.rotate(-S.view.rot);
    ctx.drawImage(img, 0, 0, (x1 - x0) * k, (y1 - y0) * k);
    ctx.restore();
  }
  if ($("show-drives").checked) {
    ctx.lineWidth = 1.5 * devicePixelRatio;
    ctx.strokeStyle = "rgba(43,108,176,.9)";
    ctx.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio]);
    for (const d of S.drives) polyline(d.x, d.y);
    ctx.setLineDash([]);
  }
  if ($("show-roads").checked) {
    for (const r of S.roads) {
      ctx.lineWidth = (r.id === S.road ? 4 : 2.5) * devicePixelRatio;
      ctx.strokeStyle = r.id === S.road ? "#b5121b" : "rgba(181,18,27,.6)";
      polyline(r.x, r.y);
      const m = Math.floor(r.x.length / 2);
      const [px, py] = toPx(r.x[m], r.y[m]);
      ctx.fillStyle = "#b5121b";
      ctx.font = `${12 * devicePixelRatio}px system-ui`;
      ctx.fillText("road " + r.id, px + 6, py - 6);
    }
  }
  scaleBar();
  northArrow();
  mapLegend(p, layer);
}
function northArrow() {
  const d = devicePixelRatio, R = 26 * d, r = 12 * d, x = 38 * d, y = 38 * d;
  const ang = -S.view.rot;           // screen angle of north, clockwise from straight up
  ctx.save();
  ctx.translate(x, y);
  ctx.fillStyle = "rgba(255,255,255,.85)";
  ctx.beginPath(); ctx.arc(0, 0, R, 0, 2 * Math.PI); ctx.fill();
  ctx.save();
  ctx.rotate(ang);
  ctx.translate(0, 5 * d);
  ctx.fillStyle = "#b5121b";
  ctx.beginPath(); ctx.moveTo(0, -r); ctx.lineTo(4.5 * d, 0); ctx.lineTo(-4.5 * d, 0); ctx.fill();
  ctx.fillStyle = "#777";
  ctx.beginPath(); ctx.moveTo(0, r); ctx.lineTo(4.5 * d, 0); ctx.lineTo(-4.5 * d, 0); ctx.fill();
  ctx.restore();
  ctx.fillStyle = "#222"; ctx.font = `bold ${11 * d}px system-ui`;
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillText("N", Math.sin(ang) * 17 * d, -Math.cos(ang) * 17 * d);
  ctx.restore();
}
function polyline(xs, ys) {
  ctx.beginPath();
  for (let i = 0; i < xs.length; i++) {
    const [px, py] = toPx(xs[i], ys[i]);
    i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
  }
  ctx.stroke();
}
function scaleBar() {
  const target = 120 / S.view.s;
  const nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000].find((v) => v >= target / 2) || 5000;
  const px = nice * S.view.s * devicePixelRatio;
  const x = cv.width - px - 16 * devicePixelRatio, y = cv.height - 16 * devicePixelRatio;
  ctx.fillStyle = "rgba(0,0,0,.6)";
  ctx.fillRect(x, y - 3 * devicePixelRatio, px, 3 * devicePixelRatio);
  ctx.font = `${11 * devicePixelRatio}px system-ui`;
  ctx.fillText(nice + " m", x, y - 6 * devicePixelRatio);
}
// colour stops of the preview images (uga_terrain/export.py _STOPS)
const STOPS = {
  terrain: [[0, [40, 70, 160]], [0.25, [60, 150, 200]], [0.45, [90, 180, 110]], [0.65, [220, 210, 120]], [0.85, [170, 110, 70]], [1, [250, 250, 250]]],
  heat: [[0, [20, 40, 90]], [0.35, [40, 150, 160]], [0.65, [240, 200, 70]], [1, [220, 50, 40]]],
  gray: [[0, [20, 20, 20]], [1, [245, 245, 245]]],
};
const LAYER_MAP = { elevation: "terrain", slope: "heat", roughness: "heat", intensity: "gray" };
function rampColor(name, f) {
  const st = STOPS[name];
  f = Math.min(Math.max(f, 0), 1);
  for (let i = 1; i < st.length; i++) {
    if (f <= st[i][0]) {
      const [f0, c0] = st[i - 1], [f1, c1] = st[i], t = (f - f0) / Math.max(f1 - f0, 1e-9);
      return c0.map((v, k) => Math.round(v + (c1[k] - v) * t));
    }
  }
  return st[st.length - 1][1];
}
function niceSteps(lo, hi, n = 8) {
  const raw = (hi - lo) / n, p = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-9))));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * p).find((v) => v >= raw) || raw;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6));
  return out.length >= 2 ? out : [lo, hi];
}
// legend box: one colour swatch per value; elevation also in metres above sea level
function legendBox(box, { title, ramp, lo, hi, unit, digits, asl, note }) {
  box.innerHTML = "";
  box.append(el("div", { class: "t" }, title));
  if (asl !== undefined) box.append(el("div", { class: "st hdr" }, el("span", { class: "sw" }), el("span", { class: "v" }, "relative"), el("span", { class: "asl" }, "GPS altitude")));
  const vals = niceSteps(lo, hi).reverse();
  for (const v of vals) {
    const c = rampColor(ramp, (v - lo) / Math.max(hi - lo, 1e-9));
    const row = el("div", { class: "st" }, el("span", { class: "sw", style: `background: rgb(${c.join(",")})` }),
      el("span", { class: "v" }, `${v > 0 && asl !== undefined ? "+" : ""}${v.toFixed(digits)} ${unit}`));
    if (asl !== undefined) row.append(el("span", { class: "asl" }, `${(asl + v).toFixed(1)} m`));
    box.append(row);
  }
  if (note) box.append(el("div", { class: "note" }, note));
  box.hidden = false;
}
function mapLegend(p, layer) {
  const box = $("map-legend");
  const ramp = LAYER_MAP[layer];
  if (!p || !$("show-legend").checked) { box.hidden = true; return; }
  if (!ramp) {          // surface classes
    box.innerHTML = "";
    box.append(el("div", { class: "t" }, "Surface"));
    for (const c of Object.values(p.classes || {})) {
      box.append(el("div", { class: "st" }, el("span", { class: "sw", style: `background: rgb(${c.rgb.join(",")})` }), el("span", { class: "v" }, c.name)));
    }
    box.append(el("div", { class: "note" }, "Switch Layer to Elevation for heights."));
    box.hidden = false;
    return;
  }
  const origin = S.status && S.status.origin;
  if (layer === "elevation") {
    legendBox(box, { title: "Elevation", ramp, lo: p.min, hi: p.max, unit: "m",
      digits: p.max - p.min < 5 ? 2 : 1, asl: origin ? origin[2] : undefined,
      note: "Relative = height compared with where the first drive starts." });
  } else if (layer === "slope") legendBox(box, { title: "Slope", ramp, lo: p.min, hi: p.max, unit: "°", digits: 0 });
  else if (layer === "roughness") legendBox(box, { title: "Roughness (RMS)", ramp, lo: 0, hi: 6, unit: "cm", digits: 1 });
  else legendBox(box, { title: "Lidar intensity", ramp, lo: p.min, hi: p.max, unit: "", digits: 0 });
}
let drag = null;
const centreAngle = (x, y) => Math.atan2(y - cv.clientHeight / 2, x - cv.clientWidth / 2);
cv.addEventListener("contextmenu", (e) => e.preventDefault());
cv.addEventListener("mousedown", (e) => {
  if (!S.view) return;
  e.preventDefault();
  drag = { x: e.offsetX, y: e.offsetY, moved: false, rotate: e.button === 2,
           anchor: toW(e.offsetX, e.offsetY), a0: centreAngle(e.offsetX, e.offsetY), rot0: S.view.rot };
  cv.style.cursor = drag.rotate ? "alias" : "grabbing";
});
window.addEventListener("mouseup", (e) => {
  if (drag && !drag.moved && !drag.rotate && e.target === cv) pickRoad(e.offsetX, e.offsetY);
  drag = null;
  cv.style.cursor = "";
});
cv.addEventListener("mousemove", (e) => {
  if (!S.view) return;
  if (drag) {
    const dx = e.offsetX - drag.x, dy = e.offsetY - drag.y;
    if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
    if (drag.rotate) {
      // turn the map about the centre of the view, following the mouse
      S.view.rot = drag.rot0 - (centreAngle(e.offsetX, e.offsetY) - drag.a0);
    } else {
      const [wx, wy] = toW(e.offsetX, e.offsetY);   // keep the grabbed point under the mouse
      S.view.cx += drag.anchor[0] - wx; S.view.cy += drag.anchor[1] - wy;
    }
    drawMap();
    return;
  }
  hover(e.offsetX, e.offsetY);
});
cv.addEventListener("wheel", (e) => {
  if (!S.view) return;
  e.preventDefault();
  const [wx, wy] = toW(e.offsetX, e.offsetY);
  const f = Math.exp(-e.deltaY * 0.0015);
  S.view.s = Math.min(Math.max(S.view.s * f, 0.02), 200);
  const [nx, ny] = toW(e.offsetX, e.offsetY);
  S.view.cx += wx - nx; S.view.cy += wy - ny;
  drawMap();
}, { passive: false });
let hoverT = null;
function hover(px, py) {
  const [x, y] = toW(px, py);
  clearTimeout(hoverT);
  hoverT = setTimeout(guard(async () => {
    const v = await api(`/api/maps/${S.map}/sample?x=${x.toFixed(2)}&y=${y.toFixed(2)}`);
    $("readout").textContent = v.z === null ? `x ${x.toFixed(1)}  y ${y.toFixed(1)} m\nno ground here`
      : `x ${x.toFixed(1)}  y ${y.toFixed(1)} m   ${v.surface}${v.filled ? " (filled gap)" : ""}\n`
        + `elevation ${fmt(v.z, 2)} m   slope ${fmt(v.slope_deg, 1)}°   roughness ${fmt(100 * v.roughness, 1)} cm`;
  }), 120);
}
function pickRoad(px, py) {
  const [x, y] = toW(px, py);
  let best = null, bd = 15 / S.view.s;
  for (const r of S.roads) {
    for (let i = 0; i < r.x.length; i += 2) {
      const d = Math.hypot(r.x[i] - x, r.y[i] - y);
      if (d < bd) { bd = d; best = r.id; }
    }
  }
  if (best !== null) { S.road = best; $("road-select").value = best; drawMap(); drawProfile(); toast(`Road ${best} selected; see the Road profile tab.`); }
}
["layer", "show-roads", "show-drives", "show-legend"].forEach((id) => $(id).addEventListener("change", drawMap));
$("btn-fit").addEventListener("click", () => { S.view = null; drawMap(); });
window.addEventListener("resize", () => { drawMap(); drawProfile(); if (S.three) S.three.resize(); });

// --- road profile ---------------------------------------------------------------------
function fillRoadSelect() {
  const s = $("road-select");
  s.innerHTML = "";
  for (const r of S.roads) s.append(el("option", { value: r.id }, `Road ${r.id} (${r.length.toFixed(0)} m)`));
  if (S.road !== null) s.value = S.road;
  const T = $("road-table");
  T.innerHTML = "";
  const list = (S.summary && S.summary.road_list) || [];
  if (!list.length) { T.append(el("div", { class: "muted" }, S.summary ? "No roads found in this map." : "Build a map to see its roads.")); return; }
  const tbl = el("table");
  tbl.append(el("tr", {}, ...["Road", "Length m", "Mean grade %", "Max |grade| %", "Mean bank °", "Max |bank| °", "Width m", "Sidewalk L %", "Sidewalk R %"].map((h) => el("th", {}, h))));
  for (const r of list) {
    const tr = el("tr", { class: r.id === S.road ? "sel" : "", onclick: () => { S.road = r.id; s.value = r.id; fillRoadSelect(); drawProfile(); drawMap(); } },
      ...[r.id, fmt(r.length_m, 0), fmt(r.mean_grade_pct, 2), fmt(r.max_abs_grade_pct, 1), fmt(r.mean_bank_deg, 2),
        fmt(r.max_abs_bank_deg, 1), fmt(r.median_width_m, 1), fmt(r.sidewalk_left_pct, 0), fmt(r.sidewalk_right_pct, 0)].map((v) => el("td", {}, v)));
    tbl.append(tr);
  }
  T.append(tbl);
}
$("road-select").addEventListener("change", () => { S.road = Number($("road-select").value); fillRoadSelect(); drawProfile(); drawMap(); });

function chart(id, s, series, opts = {}) {
  const c = $(id);
  const w = c.clientWidth, h = c.clientHeight;
  c.width = w * devicePixelRatio; c.height = h * devicePixelRatio;
  const g = c.getContext("2d");
  g.scale(devicePixelRatio, devicePixelRatio);
  g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.body);
  const muted = css.getPropertyValue("--muted"), line = css.getPropertyValue("--line");
  let lo = Infinity, hi = -Infinity;
  for (const se of series) for (const v of se.v) if (v !== null && Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (!Number.isFinite(lo)) { g.fillStyle = muted; g.fillText("no data", 10, 20); return; }
  if (opts.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
  if (hi - lo < (opts.minSpan || 1e-6)) { const m = (hi + lo) / 2; lo = m - (opts.minSpan || 1) / 2; hi = m + (opts.minSpan || 1) / 2; }
  const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const L = 46, R = 8, T = 6, B = 20;
  const s0 = s[0], s1 = s[s.length - 1] || 1;
  const X = (v) => L + (v - s0) / Math.max(s1 - s0, 1e-9) * (w - L - R);
  const Y = (v) => T + (hi - v) / (hi - lo) * (h - T - B);
  g.strokeStyle = line; g.fillStyle = muted; g.font = "11px system-ui"; g.lineWidth = 1;
  for (let k = 0; k <= 4; k++) {
    const v = lo + (hi - lo) * k / 4, y = Y(v);
    g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke();
    g.fillText(v.toFixed(hi - lo < 0.1 ? 3 : hi - lo < 1 ? 2 : 1), 2, y + 4);
  }
  for (let k = 0; k <= 5; k++) {
    const v = s0 + (s1 - s0) * k / 5;
    g.fillText(v.toFixed(0) + " m", X(v) - 12, h - 5);
  }
  for (const se of series) {
    g.strokeStyle = se.c; g.lineWidth = se.w || 1.6;
    g.beginPath();
    let on = false;
    for (let i = 0; i < s.length; i++) {
      const v = se.v[i];
      if (v === null || !Number.isFinite(v)) { on = false; continue; }
      on ? g.lineTo(X(s[i]), Y(v)) : g.moveTo(X(s[i]), Y(v));
      on = true;
    }
    g.stroke();
  }
}
function drawProfile() {
  if (!$("tab-profile").classList.contains("active")) return;
  const r = S.roads.find((x) => x.id === S.road);
  if (!r) { $("road-info").textContent = S.summary ? "No roads in this map." : ""; return; }
  $("road-info").textContent = `${r.source.split(":").slice(1).join(":")}  ·  ${r.length.toFixed(0)} m`;
  const neg = (a) => a.map((v) => (v === null ? null : -v));
  chart("ch-elev", r.s, [{ v: r.z, c: "#2b6cb0" }]);
  chart("ch-grade", r.s, [{ v: r.vehicle_grade_pct || [], c: "#dd6b20", w: 1.2 }, { v: r.grade_pct, c: "#2b6cb0" }], { zero: true, minSpan: 2 });
  chart("ch-bank", r.s, [{ v: r.vehicle_bank_deg || [], c: "#dd6b20", w: 1.2 }, { v: r.bank_deg, c: "#2b6cb0" }], { zero: true, minSpan: 2 });
  chart("ch-width", r.s, [{ v: r.road_l, c: "#4a5568" }, { v: neg(r.road_r), c: "#4a5568" },
    { v: r.walk_l.map((v, i) => (v ? r.road_l[i] + r.verge_l[i] + v : null)), c: "#d69e2e" },
    { v: r.walk_r.map((v, i) => (v ? -(r.road_r[i] + r.verge_r[i] + v) : null)), c: "#d69e2e" }], { zero: true });
  chart("ch-curb", r.s, [{ v: r.curb_h_l, c: "#2b6cb0" }, { v: r.curb_h_r, c: "#dd6b20" }], { zero: true, minSpan: 0.1 });
  chart("ch-rough", r.s, [{ v: r.roughness, c: "#2b6cb0" }], { zero: true, minSpan: 0.01 });
}

// --- 3D -------------------------------------------------------------------------------
async function open3D() {
  if (!S.summary) { $("three-msg").hidden = false; $("three-msg").textContent = "Build a map first."; return; }
  if (S.three) { S.three.resize(); return; }
  $("three-msg").hidden = false;
  $("three-msg").textContent = "Loading the ground point cloud ...";
  let THREE, OrbitControls;
  try {
    THREE = await import("three");
    ({ OrbitControls } = await import("three/addons/controls/OrbitControls.js"));
  } catch (e) {
    $("three-msg").textContent = "The 3D viewer (three.js) could not load: " + e.message;
    return;
  }
  const buf = await api(`/api/maps/${S.map}/cloud?max_points=800000`);
  const n = new DataView(buf).getUint32(0, true);
  const xyz = new Float32Array(buf, 4, n * 3);
  const lab = new Uint8Array(buf, 4 + n * 12, n);
  let cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < n; i++) { cx += xyz[3 * i]; cy += xyz[3 * i + 1]; cz += xyz[3 * i + 2]; }
  cx /= n; cy /= n; cz /= n;
  // colour range: 1st to 99th percentile of height, like the map
  const zs = Float32Array.from({ length: Math.min(n, 20000) }, (_, k) => xyz[3 * Math.floor(k * n / Math.min(n, 20000)) + 2]).sort();
  const zmin = zs[Math.floor(0.01 * (zs.length - 1))], zmax = zs[Math.floor(0.99 * (zs.length - 1))];
  const pos = new Float32Array(n * 3);
  const wrap = $("three-wrap");
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  wrap.append(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(getComputedStyle(document.body).getPropertyValue("--map-bg").trim() || "#111");
  const cam = new THREE.PerspectiveCamera(55, 1, 0.5, 20000);
  cam.up.set(0, 0, 1);
  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  const col = new Float32Array(n * 3);
  geom.setAttribute("color", new THREE.BufferAttribute(col, 3));
  const mat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
  scene.add(new THREE.Points(geom, mat));
  const controls = new OrbitControls(cam, renderer.domElement);
  // right button turns the view, left button moves it, wheel zooms
  controls.mouseButtons = { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };
  controls.screenSpacePanning = false;
  renderer.domElement.addEventListener("contextmenu", (e) => e.preventDefault());
  let span = 0;
  for (let i = 0; i < n; i++) span = Math.max(span, Math.abs(xyz[3 * i] - cx), Math.abs(xyz[3 * i + 1] - cy));
  cam.position.set(0, -span * 1.1, span * 0.8);
  controls.target.set(0, 0, 0);
  controls.update();
  const LUT = { 0: [0, 0, 0], 1: [0.35, 0.35, 0.38], 2: [0.9, 0.78, 0.35], 3: [0.9, 0.31, 0.24], 4: [0.43, 0.67, 0.35] };
  const update = () => {
    const ex = Number($("three-exag").value);
    const byZ = $("three-color").value === "z";
    for (let i = 0; i < n; i++) {
      pos[3 * i] = xyz[3 * i] - cx; pos[3 * i + 1] = xyz[3 * i + 1] - cy; pos[3 * i + 2] = (xyz[3 * i + 2] - cz) * ex;
      let c;
      if (byZ) {
        c = rampColor("terrain", (xyz[3 * i + 2] - zmin) / Math.max(zmax - zmin, 1e-6)).map((v) => v / 255);
      } else c = LUT[lab[i]] || LUT[0];
      col[3 * i] = c[0]; col[3 * i + 1] = c[1]; col[3 * i + 2] = c[2];
    }
    geom.attributes.position.needsUpdate = true;
    geom.attributes.color.needsUpdate = true;
    geom.computeBoundingSphere();
    mat.size = Number($("three-size").value);
    const origin = S.status && S.status.origin;
    if (byZ) legendBox($("three-legend"), { title: "Elevation", ramp: "terrain", lo: zmin, hi: zmax, unit: "m",
      digits: zmax - zmin < 5 ? 2 : 1, asl: origin ? origin[2] : undefined });
    else {
      const box = $("three-legend");
      box.innerHTML = "";
      box.append(el("div", { class: "t" }, "Surface"));
      [["road", 1], ["sidewalk", 2], ["curb", 3], ["other ground", 4]].forEach(([name, k]) =>
        box.append(el("div", { class: "st" }, el("span", { class: "sw", style: `background: rgb(${LUT[k].map((v) => Math.round(v * 255)).join(",")})` }), el("span", { class: "v" }, name))));
      box.hidden = false;
    }
  };
  update();
  ["three-color", "three-exag", "three-size"].forEach((id) => $(id).addEventListener("input", update));
  const resize = () => {
    const w = wrap.clientWidth, h = wrap.clientHeight;
    renderer.setSize(w, h); cam.aspect = w / Math.max(h, 1); cam.updateProjectionMatrix();
  };
  let alive = true;
  const loop = () => { if (!alive) return; controls.update(); renderer.render(scene, cam); requestAnimationFrame(loop); };
  resize(); loop();
  $("three-msg").hidden = true;
  S.three = { resize, dispose: () => { alive = false; $("three-legend").hidden = true; renderer.dispose(); renderer.domElement.remove(); } };
}

// --- files -------------------------------------------------------------------------------
async function loadFiles() {
  const L = $("file-list");
  L.innerHTML = "";
  if (!S.map) return;
  const d = await api(`/api/maps/${S.map}/files`);
  $("out-folder").textContent = d.folder;
  if (!d.files.length) { L.append(el("div", { class: "muted" }, "Nothing yet; build the map first.")); return; }
  for (const f of d.files) {
    L.append(el("div", { class: "item" }, el("a", { class: "grow", href: `/api/maps/${S.map}/files/${encodeURIComponent(f.name)}`, download: f.name }, f.name),
      el("span", { class: "sub" }, fmtSize(f.size))));
  }
  if (d.rasters) L.append(el("div", { class: "item" }, el("a", { class: "grow", href: `/api/maps/${S.map}/rasters.zip` }, "rasters.zip"), el("span", { class: "sub" }, "elevation, slope, roughness, labels")));
}

// --- settings ------------------------------------------------------------------------------
const FIELDS = [
  ["Where things are", [
    ["data_folders", "Recording folders", "list", "One folder per line; .mcap files inside (and in sub-folders) show up on the left."],
    ["work_dir", "Work folder", "text", "Maps, caches and output files go here."],
  ]],
  ["Topics and frames", [
    ["topics.lidar", "Lidar topic", "text"], ["topics.navsat", "GNSS topic (NavSatFix)", "text"],
    ["topics.imu", "INS topic (Imu)", "text"], ["topics.tf_static", "Static transforms", "text"],
    ["frames.base", "Vehicle frame (on the ground)", "text"], ["frames.lidar", "Lidar frame", "text"],
    ["pose.imu_convention", "INS orientation convention", ["enu_frd", "enu_flu"], "enu_frd for the OXTS ROS 2 driver."],
  ]],
  ["Poses and ground", [
    ["odometry.method", "Lidar odometry", ["auto", "kiss_icp", "none"], "auto uses KISS-ICP when installed, checked against GNSS/INS every frame."],
    ["odometry.fuse_sigma_s", "GNSS pull-back time (s)", "number"],
    ["ground.method", "Ground segmentation", ["auto", "patchworkpp", "builtin"]],
    ["lidar.ground_max_range", "Max ground range (m)", "number", "Ground farther than this from the car is not used."],
    ["lidar.min_motion_m", "Skip frames until moved (m)", "number"],
  ]],
  ["Elevation map", [
    ["grid.cell", "Cell size (m)", "number"], ["grid.fill_radius", "Fill gaps up to (cells)", "number"],
    ["grid.object_height", "Object height (m)", "number", "Cells this far above the surrounding surface are treated as objects and removed."],
  ]],
  ["Roads and sidewalks", [
    ["roads.curb_min", "Smallest curb (m)", "number"], ["roads.curb_max", "Largest curb (m)", "number"],
    ["roads.rough_road_max", "Max road roughness (m)", "number", "Rougher than this (grass, gravel) ends the road."],
    ["roads.rough_walk_max", "Max sidewalk roughness (m)", "number"],
    ["roads.sidewalk_min_width", "Min sidewalk width (m)", "number"],
    ["roads.sidewalk_search", "Look for sidewalk up to (m past curb)", "number"],
    ["roads.lane_width", "Lane width (m)", "number", "Paved width is split into lanes of about this size."],
    ["roads.half_width", "Search each side of the drive (m)", "number"],
  ]],
  ["OpenDRIVE", [
    ["xodr.segment_len", "Reference line piece length (m)", "number"],
    ["xodr.profile_step", "Profile record spacing (m)", "number"],
    ["xodr.relative_elevation", "Elevation relative to map origin", "bool", "Off: absolute altitude (m above the ellipsoid)."],
  ]],
];
const getp = (o, p) => p.split(".").reduce((a, k) => (a == null ? a : a[k]), o);
function setp(o, p, v) { const ks = p.split("."); let a = o; ks.slice(0, -1).forEach((k) => (a = a[k] = a[k] || {})); a[ks[ks.length - 1]] = v; }
async function loadSettings() {
  const d = await api("/api/settings");
  S.settings = d.config;
  $("settings-yaml").value = d.yaml;
  const F = $("settings-form");
  F.innerHTML = "";
  for (const [title, fields] of FIELDS) {
    const fs = el("fieldset", {}, el("legend", {}, title));
    for (const [path, label, kind, hint] of fields) {
      const v = getp(S.settings, path);
      let input;
      if (Array.isArray(kind)) { input = el("select", { "data-path": path }); kind.forEach((o) => input.append(el("option", { value: o }, o))); input.value = v; }
      else if (kind === "bool") { input = el("input", { type: "checkbox", "data-path": path }); input.checked = !!v; }
      else if (kind === "list") { input = el("textarea", { "data-path": path, rows: 2, style: "min-height: 50px" }); input.value = (v || []).join("\n"); }
      else { input = el("input", { type: kind === "number" ? "number" : "text", step: "any", "data-path": path }); input.value = v ?? ""; }
      input.dataset.kind = Array.isArray(kind) ? "choice" : kind;
      const f = el("div", { class: "field" + (kind === "list" || kind === "text" ? " wide" : "") }, el("label", {}, label), input);
      if (hint) f.append(el("div", { class: "hint" }, hint));
      fs.append(f);
    }
    F.append(fs);
  }
}
$("btn-save-settings").addEventListener("click", guard(async () => {
  const changes = {};
  document.querySelectorAll("#settings-form [data-path]").forEach((i) => {
    const k = i.dataset.kind;
    let v = k === "bool" ? i.checked : k === "number" ? Number(i.value) : k === "list" ? i.value.split("\n").map((s) => s.trim()).filter(Boolean) : i.value;
    setp(changes, i.dataset.path, v);
  });
  await api("/api/settings", { method: "PUT", body: { changes } });
  $("settings-msg").textContent = "Saved. Re-run the map (left panel) for the changes to take effect.";
  await loadSettings(); await loadRecordings();
}));
$("btn-export-settings").addEventListener("click", () => {
  const a = el("a", { href: "/api/settings/export", download: "uga_terrain_settings.yaml" });
  document.body.append(a); a.click(); a.remove();
});
$("btn-import-settings").addEventListener("click", () => $("import-settings-file").click());
$("import-settings-file").addEventListener("change", guard(async (e) => {
  const f = e.target.files[0];
  e.target.value = "";
  if (!f) return;
  await api("/api/settings", { method: "PUT", body: { yaml: await f.text(), keep_work_dir: true } });
  toast(`Settings loaded from ${f.name}. Re-run the map for them to take effect.`);
  await loadSettings(); await loadRecordings();
}));
$("btn-save-yaml").addEventListener("click", guard(async () => {
  await api("/api/settings", { method: "PUT", body: { yaml: $("settings-yaml").value } });
  toast("Settings saved.");
  await loadSettings(); await loadRecordings();
}));

// --- start -------------------------------------------------------------------------------
(guard(async () => {
  const st = await api("/api/state");
  if (!st.kiss_icp) $("log").textContent += "KISS-ICP is not installed: poses come from GNSS/INS only.\n";
  if (!st.patchworkpp) $("log").textContent += "Patchwork++ is not installed: using the built-in ground segmentation.\n";
  await loadRecordings();
  await loadMaps(false);
  if (st.job && st.job.state === "running") { S.job = st.job; pollJob(); } else showJob(st.job);
}))();
