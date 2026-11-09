"use strict";
// UGA Terrain Toolkit UI. Plain JS; talks to the local server under /api.

const $ = (id) => document.getElementById(id);
const S = {
  recordings: [], ticked: new Set(), maps: [], map: null, status: null,
  previews: {}, images: {}, roads: [], drives: [], summary: null,
  view: null, road: null, job: null, settings: null, three: null,
};

// --- helpers ---------------------------------------------------------------------
const roadName = (id) => (S.status && S.status.road_names && S.status.road_names[id]) || `Road ${id}`;
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
  if (b.dataset.tab === "accuracy") guard(loadAccuracy)();
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
  st.recordings.forEach((r, i) => $("map-recs").append(recordingRow(st, r, (st.ranges || [])[i])));
  const steps = ["scan", "map", "roads", "export"].map((s) => (st.steps[s] ? "✓ " : "○ ") + s).join("   ");
  $("map-steps").textContent = (st.built ? "Last built " + st.built + "   " : "") + steps;
  [...$("rebuild-from").options].forEach((o) => (o.disabled = !st.can_start.includes(o.value)));
  $("btn-rebuild").disabled = st.can_start.length < 2;
  await loadSnapshots();
  if (reload) await loadResults();
}

// --- part of a recording ----------------------------------------------------------------
function rangeText(rg) {
  if (!rg) return "whole recording";
  const f = (v, end) => v === null || v === undefined ? (end ? "end" : "start")
    : rg.unit === "frame" ? `frame ${v}` : `${v} s`;
  return `${f(rg.start, false)} to ${f(rg.end, true)}`;
}
function recordingRow(st, path, rg) {
  const name = path.split("/").pop();
  const row = el("div", { class: "rec-row" });
  const head = el("div", { class: "row" },
    el("span", { class: "grow", title: path }, "• " + name),
    el("span", { class: "pill" + (rg ? " ok" : "") }, rangeText(rg)),
    el("button", { class: "small", title: "Map only part of this recording", onclick: () => { form.hidden = !form.hidden; if (!form.hidden) guard(fillInfo)(); } }, "Part…"));
  const unit = el("select", {}, el("option", { value: "s" }, "seconds"), el("option", { value: "frame" }, "lidar frames"));
  const a = el("input", { type: "number", step: "any", min: "0", placeholder: "start", style: "width: 70px" });
  const b = el("input", { type: "number", step: "any", min: "0", placeholder: "end", style: "width: 70px" });
  if (rg) { unit.value = rg.unit; a.value = rg.start ?? ""; b.value = rg.end ?? ""; }
  const info = el("div", { class: "muted" }, "");
  const save = async (r) => {
    await api(`/api/maps/${S.map}`, { method: "PUT", body: { ranges: { [path]: r } } });
    toast(r ? `Only ${rangeText(r)} of ${name} will be mapped. Press Build map.` : `${name}: the whole recording will be mapped. Press Build map.`);
    await loadMaps();
  };
  const form = el("div", { class: "range-form" },
    el("div", { class: "row" }, "From", a, "to", b, unit),
    info,
    el("div", { class: "row" },
      el("button", { class: "small primary", onclick: guard(() => save({ unit: unit.value, start: a.value === "" ? null : Number(a.value), end: b.value === "" ? null : Number(b.value) })) }, "Use this part"),
      el("button", { class: "small", onclick: guard(() => save(null)) }, "Whole recording")));
  form.hidden = true;
  async function fillInfo() {
    if (info.dataset.done) return;
    const d = await api("/api/recordings/inspect", { method: "POST", body: { path, scan: false } });
    const lid = (d.topics || []).find((t) => t.topic === (d.needed && d.needed.lidar && d.needed.lidar.topic));
    const parts = [];
    if (d.duration) parts.push(`${d.duration.toFixed(1)} s long`);
    if (lid && lid.count) parts.push(`${lid.count} lidar frames (0 to ${lid.count - 1})`);
    info.textContent = (parts.length ? "This recording is " + parts.join(", ") + ". " : "")
      + "Leave a box empty for the start or end of the recording.";
    info.dataset.done = "1";
  }
  row.append(head, form);
  return row;
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
  S.previews = {}; S.images = {}; S.roads = []; S.drives = []; S.junctions = []; S.signals = []; S.lanes = null; S.road = null; S.ed = null; S.edits = null; S.pick = null;
  if (S.three) { S.three.dispose(); S.three = null; }
  if (S.summary) {
    S.previews = await api(`/api/maps/${S.map}/previews`);
    S.roads = await api(`/api/maps/${S.map}/roads`);
    S.drives = await api(`/api/maps/${S.map}/drives`);
    S.junctions = (await api(`/api/maps/${S.map}/junctions`)) || [];
    S.signals = (await api(`/api/maps/${S.map}/signals`)) || [];
    S.lanes = await api(`/api/maps/${S.map}/lanes`);
    S.ed = await api(`/api/maps/${S.map}/edits`);
    S.edits = JSON.parse(JSON.stringify(S.ed.edits)); S.dirty = false; S.pick = null;
    if (S.roads.length) S.road = S.roads[0].id;
  }
  S.view = null;
  S.acc = null; S.section = null;
  fillRoadSelect();
  markCopy();
  if ($("tab-accuracy").classList.contains("active")) await loadAccuracy();
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

$("btn-delete-map").addEventListener("click", guard(async () => {
  const name = S.status ? S.status.name : S.map;
  const n = $("snap-list").querySelectorAll(".item").length;
  if (!confirm(`Delete the map "${name}"?\n\nIts outputs, scan cache` + (n ? ` and ${n} snapshot(s)` : "") +
    ` are deleted for good. Your .mcap recordings are not touched.\n\nUse Export map first if you may want it back.`)) return;
  await api(`/api/maps/${S.map}`, { method: "DELETE" });
  S.map = null; S.status = null;
  $("map-actions").hidden = true;
  await loadResults();
  await loadMaps(false);
  toast(`Deleted "${name}".`);
}));

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
  ["btn-build", "btn-rebuild", "btn-add", "btn-snapshot", "btn-import", "btn-accuracy"].forEach((id) => ($(id).disabled = !!(j && j.state === "running")));
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
  if (j.kind === "accuracy") {
    if (j.state === "done") toast("Accuracy check finished.");
    else if (j.state === "error") toast("The accuracy check failed: " + j.error, true);
    S.acc = null;
    await loadAccuracy();
    return;
  }
  if (j.state === "done") toast("Map built.");
  else if (j.state === "error") toast("The build failed: " + j.error, true);
  await loadMaps();
  await selectMap(S.map);       // reload the new results
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
    ctx.imageSmoothingEnabled = layer === "relief" || S.view.s * (x1 - x0) / p.width < 2;
    ctx.imageSmoothingQuality = "high";
    ctx.save();
    ctx.translate(a, b);
    ctx.rotate(-S.view.rot);
    ctx.drawImage(img, 0, 0, (x1 - x0) * k, (y1 - y0) * k);
    ctx.restore();
  }
  if ($("show-lanes").checked && S.lanes) drawLanes();
  if ($("show-drives").checked) {
    ctx.lineWidth = 1.5 * devicePixelRatio;
    ctx.strokeStyle = "rgba(43,108,176,.9)";
    ctx.setLineDash([4 * devicePixelRatio, 4 * devicePixelRatio]);
    for (const d of S.drives) polyline(d.x, d.y);
    ctx.setLineDash([]);
  }
  if ($("show-roads").checked) {
    for (const j of S.junctions || []) {
      const [cx, cy] = toPx(j.x, j.y);
      const [ex, ey] = toPx(j.x + j.r, j.y);
      ctx.lineWidth = 2 * devicePixelRatio;
      ctx.strokeStyle = "rgba(214,120,0,.9)";
      ctx.setLineDash([6 * devicePixelRatio, 4 * devicePixelRatio]);
      ctx.beginPath();
      if (j.outline && j.outline.length > 2) {          // edge fitted to the curbs
        j.outline.forEach(([x, y], k) => { const [px, py] = toPx(x, y); k ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
        ctx.closePath();
      } else ctx.arc(cx, cy, Math.hypot(ex - cx, ey - cy), 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.font = `600 ${12 * devicePixelRatio}px system-ui`;
      ctx.lineWidth = 3 * devicePixelRatio; ctx.strokeStyle = "rgba(255,255,255,.9)";
      ctx.strokeText(`Junction ${j.id}`, cx + 6, cy);
      ctx.fillStyle = "#b35f00";
      ctx.fillText(`Junction ${j.id}`, cx + 6, cy);
    }
    const lanesShown = $("show-lanes").checked && S.lanes && S.lanes.lanes.length;
    for (const r of S.roads) {
      // with lanes on, the red driven-path line is drawn only for the selected road
      if (!lanesShown || r.id === S.road) {
        ctx.lineWidth = (r.id === S.road ? (lanesShown ? 2 : 4) : 2.5) * devicePixelRatio;
        ctx.strokeStyle = r.id === S.road ? "#b5121b" : "rgba(181,18,27,.6)";
        polyline(r.x, r.y);
      }
      const m = Math.floor(r.x.length / 2);
      const [px, py] = toPx(r.x[m], r.y[m]);
      ctx.font = `600 ${12 * devicePixelRatio}px system-ui`;
      ctx.lineWidth = 3 * devicePixelRatio; ctx.strokeStyle = "rgba(255,255,255,.9)";
      ctx.strokeText(roadName(r.id), px + 6, py - 6);
      ctx.fillStyle = "#b5121b";
      ctx.fillText(roadName(r.id), px + 6, py - 6);
    }
  }
  scaleBar();
  northArrow();
  mapLegend(p, layer);
}
// lanes from the exported .xodr: filled strips, edges, and the centre line
function drawLanes() {
  const dpr = devicePixelRatio;
  const col = (l) => l.type === "sidewalk" ? [150, 150, 150] : l.type !== "driving" ? null
    : l.junction ? [230, 130, 0] : l.lane < 0 ? [31, 119, 255] : [44, 160, 44];
  const path = (pts, move = true) => {
    pts.forEach(([x, y], i) => { const [px, py] = toPx(x, y); (i === 0 && move) ? ctx.moveTo(px, py) : ctx.lineTo(px, py); });
  };
  for (const pass of [false, true]) {               // roads first, junction lanes on top
    for (const l of S.lanes.lanes) {
      if (l.junction !== pass) continue;
      const c = col(l);
      if (!c) continue;
      ctx.beginPath();
      path(l.inner);
      path([...l.outer].reverse(), false);
      ctx.closePath();
      ctx.fillStyle = `rgba(${c},${l.junction ? 0.16 : l.type === "sidewalk" ? 0.25 : 0.33})`;
      ctx.fill();
      ctx.beginPath();
      path(l.outer);
      ctx.lineWidth = 1 * dpr;
      ctx.strokeStyle = `rgba(${c},.95)`;
      ctx.stroke();
    }
  }
  ctx.setLineDash([7 * dpr, 5 * dpr]);
  for (const c of S.lanes.centre) {
    ctx.beginPath();
    path(c.line);
    ctx.lineWidth = (c.junction ? 1 : 1.8) * dpr;
    ctx.strokeStyle = c.junction ? "rgba(230,130,0,.9)" : "#f2c200";
    ctx.stroke();
  }
  ctx.setLineDash([]);
  if (S.pick && S.pick.lane) {
    const l = S.pick.lane;
    ctx.beginPath(); path(l.inner); path([...l.outer].reverse(), false); ctx.closePath();
    ctx.lineWidth = 3 * dpr; ctx.strokeStyle = "#b5121b"; ctx.stroke();
  }
  // crosswalks (white bars) and stop lines (solid white) from the .xodr objects
  for (const m of S.lanes.markings || []) {
    const P = m.polygon.map(([x, y]) => toPx(x, y));
    ctx.beginPath(); P.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.closePath();
    if (m.type === "crosswalk") {
      ctx.fillStyle = "rgba(255,255,255,.35)"; ctx.fill();
      ctx.save(); ctx.clip();
      // bars along the crossing direction (the polygon's long sides are 0-1 and 2-3)
      const [a, b, c] = P, len = Math.hypot(c[0] - b[0], c[1] - b[1]), n = Math.max(2, Math.round(len / (1.2 * S.view.s * dpr)));
      ctx.strokeStyle = "rgba(255,255,255,.95)"; ctx.lineWidth = Math.max(1, 0.5 * S.view.s * dpr);
      for (let k = 0; k <= n; k++) {
        const f = k / n, p = [b[0] + (c[0] - b[0]) * f, b[1] + (c[1] - b[1]) * f], q = [a[0] + (P[3][0] - a[0]) * f, a[1] + (P[3][1] - a[1]) * f];
        ctx.beginPath(); ctx.moveTo(...p); ctx.lineTo(...q); ctx.stroke();
      }
      ctx.restore();
      ctx.beginPath(); P.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.closePath();
      ctx.strokeStyle = "rgba(60,60,60,.8)"; ctx.lineWidth = 1 * dpr; ctx.stroke();
    } else {
      ctx.fillStyle = "rgba(255,255,255,.95)"; ctx.fill();
      ctx.strokeStyle = "rgba(60,60,60,.9)"; ctx.lineWidth = 1 * dpr; ctx.stroke();
    }
  }
  drawSignals();
}
// signal span from the lidar: wires (purple, dashed where not seen), poles (black dots), heads (amber)
function drawSignals() {
  const dpr = devicePixelRatio;
  for (const sp of S.signals || []) {
    for (const w of sp.wires || []) {
      const [a, b] = [toPx(...w.a), toPx(...w.b)];
      ctx.setLineDash(w.seen_m > 0 ? [] : [6 * dpr, 5 * dpr]);
      ctx.strokeStyle = "rgba(120,40,160,.9)"; ctx.lineWidth = 2 * dpr;
      ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const h of sp.heads || []) {
      const [x, y] = toPx(h.x, h.y), r = Math.max(4 * dpr, 0.4 * S.view.s * dpr);
      ctx.fillStyle = "#f4b400"; ctx.strokeStyle = "#222"; ctx.lineWidth = 1.5 * dpr;
      ctx.fillRect(x - r, y - r, 2 * r, 2 * r); ctx.strokeRect(x - r, y - r, 2 * r, 2 * r);
    }
    for (const q of sp.poles || []) {
      const [x, y] = toPx(q.x, q.y), r = Math.max(5 * dpr, q.radius * S.view.s * dpr);
      ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI);
      ctx.fillStyle = "#111"; ctx.fill(); ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5 * dpr; ctx.stroke();
      ctx.font = `${11 * dpr}px system-ui, sans-serif`; ctx.fillStyle = "#111";
      ctx.fillText(`pole ${q.height.toFixed(1)} m`, x + r + 3 * dpr, y - r);
    }
  }
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
  relief: [[0, [62, 110, 70]], [0.18, [104, 150, 88]], [0.38, [168, 184, 118]], [0.55, [214, 196, 136]], [0.72, [196, 152, 104]], [0.88, [160, 128, 108]], [1, [238, 234, 228]]],
};
const LAYER_MAP = { relief: "relief", elevation: "terrain", slope: "heat", roughness: "heat", intensity: "gray" };
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
    lanesLegend(box);
    box.hidden = false;
    return;
  }
  const origin = S.status && S.status.origin;
  if (layer === "elevation" || layer === "relief") {
    legendBox(box, { title: "Elevation", ramp, lo: p.min, hi: p.max, unit: "m",
      digits: p.max - p.min < 5 ? 2 : 1, asl: origin ? origin[2] : undefined,
      note: "Relative = height compared with where the first drive starts." });
  } else if (layer === "slope") legendBox(box, { title: "Slope", ramp, lo: p.min, hi: p.max, unit: "°", digits: 0 });
  else if (layer === "roughness") legendBox(box, { title: "Roughness (RMS)", ramp, lo: 0, hi: 6, unit: "cm", digits: 1 });
  else legendBox(box, { title: "Lidar intensity", ramp, lo: p.min, hi: p.max, unit: "", digits: 0 });
  lanesLegend(box);
}
function lanesLegend(box) {
  if (!$("show-lanes").checked || !S.lanes || !S.lanes.lanes.length) return;
  box.append(el("div", { class: "t", style: "margin-top: 8px" }, "Lanes (OpenDRIVE)"));
  const row = (style, text) => box.append(el("div", { class: "st" }, el("span", { class: "sw", style }), el("span", { class: "v" }, text)));
  row("background: rgba(31,119,255,.55)", "driving, along the road");
  row("background: rgba(44,160,44,.55)", "driving, the other way");
  row("background: rgba(230,130,0,.45)", "connecting lane in a junction");
  row("background: transparent; border-top: 2px dashed #f2c200; height: 0; margin-top: 7px", "centre line");
  if ((S.lanes.markings || []).length) {
    row("background: repeating-linear-gradient(90deg, #fff 0 3px, #bbb 3px 6px); border: 1px solid #555", "crosswalk");
    row("background: #fff; border: 1px solid #555; height: 4px; margin-top: 5px", "stop line");
  }
  if ((S.signals || []).some((sp) => (sp.poles || []).length)) {
    row("background: #111; border-radius: 50%; border: 1px solid #fff", "signal pole (lidar)");
    row("background: transparent; border-top: 2px solid rgb(120,40,160); height: 0; margin-top: 7px", "span wire");
    row("background: #f4b400; border: 1px solid #222", "traffic light");
  }
  row("background: #b5121b; height: 3px; margin-top: 6px", "driven path of the selected road");
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
  if (drag && !drag.moved && !drag.rotate && e.target === cv && !pickLane(e.offsetX, e.offsetY)) pickRoad(e.offsetX, e.offsetY);
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
// --- lane editor -------------------------------------------------------------------------
function inPoly(x, y, P) {
  let ins = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const [xi, yi] = P[i], [xj, yj] = P[j];
    if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) ins = !ins;
  }
  return ins;
}
function laneAt(x, y) {
  if (!S.lanes || !$("show-lanes").checked) return null;
  const hits = S.lanes.lanes.filter((l) => l.type !== "sidewalk" && inPoly(x, y, [...l.inner, ...[...l.outer].reverse()]));
  if (!hits.length) return null;
  // junction lanes overlap: take the one whose middle is nearest
  const mid = (l) => {
    let bd = Infinity;
    for (let i = 0; i < l.inner.length; i++) {
      const mx = (l.inner[i][0] + l.outer[i][0]) / 2, my = (l.inner[i][1] + l.outer[i][1]) / 2;
      bd = Math.min(bd, Math.hypot(mx - x, my - y));
    }
    return bd;
  };
  const plain = hits.filter((l) => !l.junction);
  return (plain.length ? plain : hits).sort((a, b) => mid(a) - mid(b))[0];
}
function turnKey(l) {
  const m = /(?:turn|from) (\d+) lane (-?\d+) to (\d+) lane (-?\d+)/.exec(l.name || "");
  return m ? `${m[1]}:${m[2]}>${m[3]}:${m[4]}` : null;
}
function pickLane(px, py) {
  if (!S.ed) return false;
  const [x, y] = toW(px, py);
  const l = laneAt(x, y);
  if (!l) return false;
  S.pick = { lane: l };
  if (!l.junction) { S.road = Number(l.road); $("road-select").value = S.road; }
  drawMap(); editorPanel();
  return true;
}
function roadEdit(rid) {
  S.edits.roads[rid] = S.edits.roads[rid] || {};
  return S.edits.roads[rid];
}
function detailsOf(rid) {
  return Object.assign({}, S.ed.defaults, S.edits.roads[rid] || {});
}
function changed() { S.dirty = true; editorPanel(); }
function editorPanel() {
  const box = $("lane-editor");
  if (!S.ed || (!S.pick && !S.dirty)) { box.hidden = true; return; }
  box.innerHTML = "";
  box.append(el("button", { class: "x", title: "Close", onclick: () => { S.pick = null; drawMap(); editorPanel(); } }, "\u00d7"));
  const l = S.pick && S.pick.lane;
  const sel = (opts, val, on) => {
    const s = el("select", { onchange: (e) => on(e.target.value) });
    for (const [v, t] of opts) s.append(el("option", { value: v }, t));
    s.value = val;
    return s;
  };
  if (l && l.junction) {
    const key = turnKey(l);
    box.append(el("h3", {}, "Turn through the junction"), el("div", { class: "sub" }, l.name));
    const g = el("div", { class: "grp" });
    g.append(el("div", {}, "If this turn is not allowed here (a no-left-turn sign, a one-way street), remove it. Cars in CARLA will then never make it."));
    if (key) g.append(el("div", { class: "row" }, el("button", { onclick: () => {
      S.edits.removed_turns = [...new Set([...(S.edits.removed_turns || []), key])]; S.pick = null; changed(); } }, "Remove this turn")));
    box.append(g);
  } else if (l) {
    const rid = String(l.road), d = detailsOf(rid), info = (S.ed.roads || []).find((r) => String(r.id) === rid) || {};
    box.append(el("h3", {}, `${roadName(Number(rid))}, lane ${l.lane}`),
      el("div", { class: "sub" }, `${l.lane < 0 ? "along the road" : "the other way"}${info.stub ? " \u00b7 a street nobody drove (its shape is a guess from the curbs)" : ""}`));
    const g = el("div", { class: "grp" }, el("div", { class: "t" }, "Road"));
    g.append(el("label", {}, "Type", sel(S.ed.road_types.map((t) => [t, t]), d.type, (v) => { roadEdit(rid).type = v; changed(); })));
    const sp = el("input", { type: "number", min: "0", step: "5", value: d.speed ?? "", placeholder: "none",
      onchange: (e) => { roadEdit(rid).speed = e.target.value === "" ? null : Number(e.target.value); changed(); } });
    g.append(el("label", {}, "Speed limit", el("span", {}, sp, " ",
      sel([["mph", "mph"], ["km/h", "km/h"]], d.unit || "mph", (v) => { roadEdit(rid).unit = v; changed(); }))));
    g.append(el("label", {}, "Direction", sel([["no", "two-way"], ["forward", "one-way, blue direction"], ["backward", "one-way, green direction"]],
      d.one_way || "no", (v) => { roadEdit(rid).one_way = v; changed(); })));
    box.append(g);
    const le = ((S.edits.roads[rid] || {}).lanes || {})[String(l.lane)] || {};
    const g2 = el("div", { class: "grp" }, el("div", { class: "t" }, `Lane ${l.lane}`));
    const w = (() => { let a = []; for (let i = 0; i < l.inner.length; i++) a.push(Math.hypot(l.inner[i][0] - l.outer[i][0], l.inner[i][1] - l.outer[i][1])); a.sort((p, q) => p - q); return a[a.length >> 1] || 0; })();
    g2.append(el("label", {}, "Type", sel(S.ed.lane_types.map((t) => [t, t]), le.type || l.type, (v) => {
      const r = roadEdit(rid); r.lanes = r.lanes || {}; r.lanes[l.lane] = Object.assign(r.lanes[l.lane] || {}, { type: v }); changed(); })));
    g2.append(el("label", {}, "Width (m)", el("input", { type: "number", min: "0.5", max: "8", step: "0.05", value: le.width ?? "",
      placeholder: w.toFixed(2), onchange: (e) => {
        const r = roadEdit(rid); r.lanes = r.lanes || {};
        r.lanes[l.lane] = Object.assign(r.lanes[l.lane] || {}, { width: e.target.value === "" ? null : Number(e.target.value) }); changed(); } })));
    g2.append(el("div", { class: "sub" }, "Leave the width empty to keep the measured one. A width you type is used along the whole road."));
    box.append(g2);
    box.append(el("div", { class: "row" }, el("button", { onclick: () => {
      S.edits.removed_roads = [...new Set([...(S.edits.removed_roads || []), Number(rid)])]; S.pick = null; changed(); } }, "Delete this road")));
  } else box.append(el("h3", {}, "Lane editor"));
  const rm = el("div", { class: "grp" });
  if ((S.edits.removed_turns || []).length || (S.edits.removed_roads || []).length) {
    rm.append(el("div", { class: "t" }, "Removed"));
    const ul = el("ul");
    for (const k of S.edits.removed_turns || []) ul.append(el("li", {}, `turn ${k.replace(">", " to ").replace(/:/g, " lane ")} `,
      el("a", { href: "#", onclick: (e) => { e.preventDefault(); S.edits.removed_turns = S.edits.removed_turns.filter((v) => v !== k); changed(); } }, "put back")));
    for (const r of S.edits.removed_roads || []) ul.append(el("li", {}, `${roadName(r)} `,
      el("a", { href: "#", onclick: (e) => { e.preventDefault(); S.edits.removed_roads = S.edits.removed_roads.filter((v) => v !== r); changed(); } }, "put back")));
    rm.append(ul);
    box.append(rm);
  }
  const row = el("div", { class: "row" },
    el("button", { class: "primary", disabled: S.dirty ? null : "", onclick: guard(saveEdits) }, "Save and rewrite .xodr"),
    el("button", { disabled: S.dirty ? null : "", onclick: () => { S.edits = JSON.parse(JSON.stringify(S.ed.edits)); S.dirty = false; editorPanel(); } }, "Undo"));
  box.append(row);
  if (S.dirty) box.append(el("div", { class: "pending" }, "Changes not saved yet."));
  box.hidden = false;
}
async function saveEdits() {
  const res = await api(`/api/maps/${S.map}/edits`, { method: "PUT", body: { edits: S.edits } });
  S.ed = await api(`/api/maps/${S.map}/edits`);
  S.edits = JSON.parse(JSON.stringify(S.ed.edits)); S.dirty = false; S.pick = null;
  S.lanes = await api(`/api/maps/${S.map}/lanes`);
  S.acc = null;
  drawMap(); editorPanel();
  const c = res.check;
  toast(`Saved. The .xodr was rewritten; OpenDRIVE check: ${c.error} error(s), ${c.warning} warning(s).`, c.error > 0);
}
function markCopy() {
  const box = $("mark-copy");
  box.innerHTML = "";
  if (!S.ed || !(S.junctions || []).length) return;
  const others = (S.maps || []).filter((m) => m.slug !== S.map);
  if (!others.length) return;
  const srcs = S.ed.marking_sources || [];
  const s = el("select", {});
  for (const m of others) s.append(el("option", { value: m.slug }, m.name || m.slug));
  box.append(el("span", {}, "Crosswalks from "), s, " ",
    el("button", { class: "small", onclick: guard(async () => {
      const res = await api(`/api/maps/${S.map}/markings/import`, { method: "POST", body: { from: s.value } });
      await reloadMarkings();
      const mt = res.match || {};
      toast(`Copied ${res.added} of ${res.offered} stop lines and crosswalks from ${res.from}` +
        (mt.used ? ` (lined up on the paint, moved ${Math.hypot(mt.dx, mt.dy).toFixed(1)} m).` : " (placed by GPS only).") +
        ` OpenDRIVE check: ${res.check.error} error(s).`, res.check.error > 0);
    }) }, "Copy"));
  for (const n of srcs) box.append(" ", el("span", { class: "chip" }, `from ${n} `,
    el("a", { href: "#", title: "Take the copied ones off this map", onclick: guard(async (e) => {
      e.preventDefault();
      await api(`/api/maps/${S.map}/markings/import/${encodeURIComponent(n)}`, { method: "DELETE" });
      await reloadMarkings(); toast(`Removed the markings copied from ${n}.`);
    }) }, "\u00d7")));
}
async function reloadMarkings() {
  S.ed = await api(`/api/maps/${S.map}/edits`);
  S.lanes = await api(`/api/maps/${S.map}/lanes`);
  S.acc = null;
  drawMap(); markCopy();
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
  if (best !== null) { S.road = best; $("road-select").value = best; drawMap(); drawProfile(); toast(`${roadName(best)} selected; see the Road profile tab.`); }
}
["layer", "show-lanes", "show-roads", "show-drives", "show-legend"].forEach((id) => $(id).addEventListener("change", drawMap));
$("btn-fit").addEventListener("click", () => { S.view = null; drawMap(); });
window.addEventListener("resize", () => { drawMap(); drawProfile(); if (S.three) S.three.resize(); });

// --- road profile ---------------------------------------------------------------------
function fillRoadSelect() {
  const s = $("road-select");
  s.innerHTML = "";
  for (const r of S.roads) s.append(el("option", { value: r.id }, `${roadName(r.id)} (${r.length.toFixed(0)} m)`));
  if (S.road !== null) s.value = S.road;
  $("road-name").value = S.road !== null && S.status && S.status.road_names && S.status.road_names[S.road] || "";
  $("road-name").disabled = $("btn-road-name").disabled = S.road === null;
  const T = $("road-table");
  T.innerHTML = "";
  const list = (S.summary && S.summary.road_list) || [];
  if (!list.length) { T.append(el("div", { class: "muted" }, S.summary ? "No roads found in this map." : "Build a map to see its roads.")); return; }
  const tbl = el("table");
  tbl.append(el("tr", {}, ...["Road", "Length m", "Mean grade %", "Max |grade| %", "Mean bank °", "Max |bank| °", "Width m", "Sidewalk L %", "Sidewalk R %"].map((h) => el("th", {}, h))));
  for (const r of list) {
    const tr = el("tr", { class: r.id === S.road ? "sel" : "", onclick: () => { S.road = r.id; s.value = r.id; fillRoadSelect(); drawProfile(); drawMap(); } },
      ...[roadName(r.id), fmt(r.length_m, 0), fmt(r.mean_grade_pct, 2), fmt(r.max_abs_grade_pct, 1), fmt(r.mean_bank_deg, 2),
        fmt(r.max_abs_bank_deg, 1), fmt(r.median_width_m, 1), fmt(r.sidewalk_left_pct, 0), fmt(r.sidewalk_right_pct, 0)].map((v) => el("td", {}, v)));
    tbl.append(tr);
  }
  T.append(tbl);
}
async function saveRoadName() {
  if (S.road === null) return;
  const name = $("road-name").value.trim();
  const st = await api(`/api/maps/${S.map}`, { method: "PUT", body: { road_names: { [S.road]: name } } });
  S.status.road_names = st.road_names;
  if (S.summary) for (const r of S.summary.road_list || []) r.name = st.road_names[r.id] || null;
  fillRoadSelect(); drawMap();
  toast(name ? `Road ${S.road} is now "${name}". The .xodr has been updated.` : `Name removed from road ${S.road}.`);
}
$("btn-road-name").addEventListener("click", guard(saveRoadName));
$("road-name").addEventListener("keydown", (e) => { if (e.key === "Enter") guard(saveRoadName)(); });
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

// --- accuracy ---------------------------------------------------------------------------
const cm = (v, d = 1) => (v === null || v === undefined) ? "–" : (100 * v).toFixed(d) + " cm";
async function loadAccuracy() {
  const B = $("acc-body");
  B.innerHTML = "";
  if (!S.map || !S.summary) { B.append(el("p", { class: "muted" }, "Build a map first.")); return; }
  if (!S.acc) S.acc = await api(`/api/maps/${S.map}/accuracy`);
  const a = S.acc;
  if (!a) {
    $("acc-when").textContent = "Not checked yet. It takes about as long as reading the scans (seconds to a few minutes).";
  } else {
    $("acc-when").textContent = `Checked ${a.checked} on ${a.scans} scans, ${(a.points / 1e6).toFixed(1)} M ground points.`;
    B.append(el("h3", {}, "Findings"), el("ul", { class: "help" }, ...a.verdict.map((v) => el("li", {}, v))));
    // poses
    const tp = el("table");
    tp.append(el("tr", {}, ...["Recording", "Scans", "Lidar odometry", "Scans where lidar disagreed with GNSS/INS", "Final path vs GNSS (median / max)", "Motion corrected"].map((h) => el("th", {}, h))));
    for (const p of a.poses) {
      const g = p.gnss_vs_final_m || {};
      tp.append(el("tr", {}, ...[p.recording + (p.range ? " (part)" : ""), p.frames ?? "–", p.odometry || "–", p.odometry_resets ?? "–",
        g.median != null ? `${cm(g.median, 0)} / ${cm(g.max, 0)}` : "–", p.deskewed ? "yes" : "no"].map((v) => el("td", {}, v))));
    }
    B.append(el("h3", {}, "Drive path"), el("div", { class: "table-wrap" }, tp));
    // by range
    const tr = el("table");
    tr.append(el("tr", {}, ...["Distance from car", "Points", "Bias (+ = too high)", "Typical error", "90 % within"].map((h) => el("th", {}, h))));
    for (const r of a.by_range) {
      const bad = r.from_m >= 10 && (Math.abs(r.bias_m) > 0.02 || r.typical_m > 0.03);
      tr.append(el("tr", { class: bad ? "warn" : "" }, ...[`${r.from_m}–${r.to_m} m`, r.points.toLocaleString(), cm(r.bias_m), cm(r.typical_m), cm(r.p90_m)].map((v) => el("td", {}, v))));
    }
    B.append(el("h3", {}, "Ground height error by distance from the car"),
      el("p", { class: "muted help" }, "Each point's height compared with the same 20 cm cell seen from less than 12 m away, where the lidar is most precise."),
      el("div", { class: "table-wrap" }, tr));
    // halves
    const h = a.halves || {};
    B.append(el("h3", {}, "Repeatability"),
      el("p", { class: "help" }, `Two maps, one from the first half of the scans and one from the second, differ by ${cm(h.road_typical_m ?? h.typical_m)} on roads and sidewalks (90 % within ${cm(h.road_p90_m ?? h.p90_m)}), over ${(h.cells || 0).toLocaleString()} cells both halves see. Their overall offset is ${cm(h.offset_m)}.`));
    for (const r of a.roads || []) {
      const t = el("table");
      t.append(el("tr", {}, ...["Measurement", "Value (median)", "Found at", "Half vs half difference"].map((x) => el("th", {}, x))));
      for (const row of r.rows) {
        const f = (v) => v === null ? "–" : row.unit === "m" ? (row.key.startsWith("curb") ? cm(v) : v.toFixed(2) + " m") : v.toFixed(2) + (row.unit === "%" ? " %" : "°");
        t.append(el("tr", {}, ...[row.label, f(row.value), row.found_pct.toFixed(0) + " % of stations", row.repeat === null ? "–" : "± " + f(row.repeat)].map((v) => el("td", {}, v))));
      }
      B.append(el("h4", {}, `${roadName(r.id)} (${r.length_m.toFixed(0)} m)`), el("div", { class: "table-wrap" }, t));
    }
  }
  await loadXodrCheck();
  // cross-section controls
  const sel = $("sec-road");
  const keep = Number(sel.value);
  sel.innerHTML = "";
  for (const r of S.roads) sel.append(el("option", { value: r.id }, roadName(r.id)));
  if (S.roads.some((r) => r.id === keep)) sel.value = keep;
  setSectionRange();
  await loadSection();
}
// OpenDRIVE checks: is the .xodr valid, are its lanes steady, does the drive replay onto it?
async function loadXodrCheck() {
  const B = $("xodr-body");
  B.innerHTML = "";
  const x = await api(`/api/maps/${S.map}/xodr_check`);
  if (!x) return;
  const c = x.counts;
  B.append(el("h3", {}, "Road network (OpenDRIVE) checks"),
    el("p", { class: "muted help" }, "The exported .xodr is read back by the app's own OpenDRIVE reader and checked against the rules simulators rely on (it has not been loaded in CARLA itself yet). Errors can make a simulator reject or misread the file; warnings mean the file is valid but not yet a reliable, drivable lane map."),
    el("p", { class: "help" }, el("b", { class: c.error ? "bad" : "ok" }, `${c.error} error(s)`), `, ${c.warning} warning(s)`));
  const lvl = { error: "bad", warning: "warn-t", info: "muted" };
  B.append(el("ul", { class: "help" }, ...x.findings.map((f) =>
    el("li", {}, el("span", { class: lvl[f.level] || "" }, f.level.toUpperCase()), " ", f.road ? `${roadName(Number(f.road))}: ` : "", f.msg))));
  for (const r of x.replay || []) {
    const t = el("table");
    const row = (k, v) => t.append(el("tr", {}, el("td", {}, k), el("td", {}, v)));
    row("Poses", r.poses);
    row("On a road", fmt(r.on_road_pct, 0) + " %");
    row("Inside a driving lane", fmt(r.in_driving_lane_pct, 0) + " %");
    row("Lanes used", (r.lanes_used || []).map((s) => { const [rd, ln] = s.split(":"); return `${roadName(Number(rd))} lane ${ln}`; }).join(", ") || "–");
    row("Lane changes", r.lane_changes ?? "–");
    row("Moves between lanes the map does not connect", (r.unlinked_moves || []).length ? r.unlinked_moves.join("; ") : "none");
    row("Distance from lane centre (median / 95 %)", r.lane_centre_offset_median_m != null ? `${fmt(r.lane_centre_offset_median_m)} m / ${fmt(r.lane_centre_offset_p95_m)} m` : "–");
    row("Heading vs lane direction (median)", r.heading_error_median_deg != null ? fmt(r.heading_error_median_deg, 1) + "°" : "–");
    row("Driving against the lane direction", r.wrong_way_pct != null ? fmt(r.wrong_way_pct, 0) + " %" : "–");
    row("Vehicle height above the road surface (median / 10–90 % spread)", r.height_vs_road_median_m != null ? `${cm(r.height_vs_road_median_m)} / ${cm(r.height_vs_road_spread_m)}` : "–");
    B.append(el("h4", {}, `Replaying the car's own drive (${r.trajectory}) onto the map`), el("div", { class: "table-wrap" }, t));
  }
  for (const j of x.junctions || []) {
    const t = el("table");
    t.append(el("tr", {}, ...["From", "To", "Lanes (from → connecting lane)", "Connecting road"].map((h) => el("th", {}, h))));
    for (const c of j.connections) {
      t.append(el("tr", {}, ...[roadName(Number(c.incoming)), c.outgoing ? roadName(Number(c.outgoing)) : "–",
        c.lanes.map(([a, b]) => `${a} → ${b}`).join(", "), c.name || c.connecting].map((v) => el("td", {}, v))));
    }
    B.append(el("h4", {}, `Junction ${j.id}: which lane connects to which`),
      el("p", { class: "muted help" }, "Each row is one connecting lane through the intersection. Turns marked \"not driven\" were added so every movement between the roads exists; delete them where a turn is not allowed."),
      el("div", { class: "table-wrap" }, t));
  }
  if ((x.lane_widths || []).length) {
    const t = el("table");
    t.append(el("tr", {}, ...["Road", "Lane", "Typical width", "Narrowest", "Widest", "Change along road"].map((h) => el("th", {}, h))));
    for (const w of x.lane_widths) {
      const bad = w.min_m < 2.5 || w.max_m > 4.6 || w.spread_m > 0.3;
      t.append(el("tr", { class: bad ? "warn" : "" }, ...[roadName(Number(w.road)), w.lane, fmt(w.median_m) + " m", fmt(w.min_m) + " m", fmt(w.max_m) + " m", fmt(w.spread_m) + " m"].map((v) => el("td", {}, v))));
    }
    B.append(el("h4", {}, "Driving lane widths"), el("div", { class: "table-wrap" }, t));
  }
}
function setSectionRange() {
  const r = S.roads.find((x) => x.id === Number($("sec-road").value));
  const sl = $("sec-s");
  if (!r) { sl.disabled = true; return; }
  sl.disabled = false;
  sl.min = Math.floor(r.s[0]); sl.max = Math.floor(r.s[r.s.length - 1]);
  if (Number(sl.value) < Number(sl.min) || Number(sl.value) > Number(sl.max)) sl.value = Math.round((Number(sl.min) + Number(sl.max)) / 2);
}
async function loadSection() {
  const road = Number($("sec-road").value), s = Number($("sec-s").value);
  $("sec-s-text").textContent = `${s} m along the road`;
  if (!S.roads.length) { S.section = null; drawSection(); return; }
  const req = (loadSection._n = (loadSection._n || 0) + 1);
  $("sec-info").textContent = "Loading the scans ...";
  const d = await api(`/api/maps/${S.map}/section?road=${road}&s=${s}`);
  if (req !== loadSection._n) return;      // a newer request is on its way
  S.section = d;
  drawSection();
}
$("sec-road").addEventListener("change", guard(async () => { setSectionRange(); await loadSection(); }));
$("sec-s").addEventListener("input", () => { $("sec-s-text").textContent = `${$("sec-s").value} m along the road`; clearTimeout(loadSection._t); loadSection._t = setTimeout(guard(loadSection), 150); });
$("sec-zoom").addEventListener("change", () => drawSection());
$("btn-accuracy").addEventListener("click", guard(async () => {
  S.job = await api(`/api/maps/${S.map}/accuracy`, { method: "POST" });
  S.acc = null;
  pollJob();
}));
window.addEventListener("resize", () => { if ($("tab-accuracy").classList.contains("active")) drawSection(); });

function drawSection() {
  const c = $("sec-canvas");
  const w = c.clientWidth, h = c.clientHeight;
  if (!w) return;
  c.width = w * devicePixelRatio; c.height = h * devicePixelRatio;
  const g = c.getContext("2d");
  g.scale(devicePixelRatio, devicePixelRatio);
  g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.body);
  const muted = css.getPropertyValue("--muted"), line = css.getPropertyValue("--line");
  g.font = "11px system-ui";
  const d = S.section;
  if (!d || !d.lat.length) { g.fillStyle = muted; g.fillText(d ? "No lidar points here." : "Pick a road.", 10, 20); $("sec-info").textContent = ""; return; }
  const L = 52, R = 10, T = 22, Bm = 22;
  const hw = Math.max(...d.lat.map(Math.abs), 5);
  const pxm = (w - L - R) / (2 * hw);
  const zoom = Number($("sec-zoom").value);
  const pzm = pxm * zoom;
  const dzs = d.dz.slice().sort((a, b) => a - b);
  const mid = dzs[Math.floor(dzs.length / 2)];
  const X = (lat) => L + (hw - lat) * pxm;        // looking along the drive: left side on the left
  const Y = (dz) => T + (h - T - Bm) / 2 - (dz - mid) * pzm;
  // grid: height lines
  const span = (h - T - Bm) / pzm;
  const step = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5].find((s) => span / s <= 8) || 10;
  g.strokeStyle = line; g.fillStyle = muted; g.lineWidth = 1;
  for (let v = Math.ceil((mid - span / 2) / step) * step; v <= mid + span / 2; v += step) {
    const y = Y(v);
    g.beginPath(); g.moveTo(L, y); g.lineTo(w - R, y); g.stroke();
    g.fillText((step < 0.1 ? (100 * v).toFixed(0) + " cm" : v.toFixed(1) + " m"), 2, y + 4);
  }
  for (let v = -Math.floor(hw / 5) * 5; v <= hw; v += 5) {
    g.fillText(v === 0 ? "path" : `${Math.abs(v)} m ${v > 0 ? "L" : "R"}`, X(v) - 12, h - 6);
  }
  // points, coloured by scan
  const n = Math.max(d.scans - 1, 1);
  for (let i = 0; i < d.lat.length; i++) {
    g.fillStyle = `hsl(${(250 * d.scan[i] / n).toFixed(0)}, 80%, 50%)`;
    g.fillRect(X(d.lat[i]) - 1, Y(d.dz[i]) - 1, 2, 2);
  }
  // measured edges
  const ed = d.edges;
  g.lineWidth = 1.5;
  for (const [side, sg] of [["l", 1], ["r", -1]]) {
    const e = ed[side];
    if (e.road == null) continue;
    const xr = X(sg * e.road);
    g.strokeStyle = "#e53e3e"; g.setLineDash([4, 3]);
    g.beginPath(); g.moveTo(xr, T); g.lineTo(xr, h - Bm); g.stroke();
    g.setLineDash([]);
    g.fillStyle = "#e53e3e";
    const lab = (e.edge || "edge") + (e.curb_h ? ` ${(100 * e.curb_h).toFixed(0)} cm` : "");
    g.fillText(lab, xr + (sg > 0 ? 4 : -4 - g.measureText(lab).width), T - 8);
    if (e.walk) {
      const a = sg * (e.road + (e.verge || 0)), b = sg * (e.road + (e.verge || 0) + e.walk);
      g.fillStyle = "rgba(214, 158, 46, 0.35)";
      g.fillRect(Math.min(X(a), X(b)), T - 4, Math.abs(X(b) - X(a)), 6);
    }
  }
  // spread of the scans on the road: median distance of each point from its 20 cm column's median
  const cols = new Map();
  for (let i = 0; i < d.lat.length; i++) {
    const inRoad = (d.lat[i] >= 0 && d.lat[i] < (ed.l.road || 0)) || (d.lat[i] < 0 && -d.lat[i] < (ed.r.road || 0));
    if (!inRoad) continue;
    const k = Math.floor(d.lat[i] / 0.2);
    if (!cols.has(k)) cols.set(k, []);
    cols.get(k).push(d.dz[i]);
  }
  const dev = [];
  for (const v of cols.values()) {
    if (v.length < 5) continue;
    v.sort((a, b) => a - b);
    const m = v[Math.floor(v.length / 2)];
    for (const x of v) dev.push(Math.abs(x - m));
  }
  dev.sort((a, b) => a - b);
  const spread = dev.length ? dev[Math.floor(dev.length / 2)] : null;
  $("sec-info").textContent = `Colour = scan number (blue first, purple last); heights stretched ${zoom}×. ${d.lat.length.toLocaleString()} points from ${new Set(d.scan).size} scans at ${d.s.toFixed(0)} m. ` +
    (spread !== null ? `On the road the scans scatter by a typical ${(100 * spread).toFixed(1)} cm around each other. ` : "") +
    `Grade ${fmt(d.grade_pct)} %, banking ${fmt(d.bank_deg)}°.`;
}

// --- 3D -------------------------------------------------------------------------------
// three.js treats vertex colours as linear light; our colours are sRGB, so convert
// them or everything comes out pale and washed-out
const toLinear = (c) => (c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4));
const SURFACE_RGB = { 0: [120, 120, 120], 1: [90, 90, 97], 2: [230, 199, 89], 3: [230, 79, 61], 4: [110, 171, 89] };
let three3D = null;     // loaded three.js modules
async function open3D() {
  if (!S.summary) { $("three-msg").hidden = false; $("three-msg").textContent = "Build a map first."; return; }
  if (S.three) { S.three.resize(); return; }
  $("three-msg").hidden = false;
  $("three-msg").textContent = "Loading the 3D terrain ...";
  try {
    if (!three3D) {
      const THREE = await import("three");
      const { OrbitControls } = await import("three/addons/controls/OrbitControls.js");
      three3D = { THREE, OrbitControls };
    }
  } catch (e) {
    $("three-msg").textContent = "The 3D viewer (three.js) could not load: " + e.message;
    return;
  }
  const { THREE, OrbitControls } = three3D;
  const map = S.map;

  // --- data: height grid (solid surface) -------------------------------------------
  const buf = await api(`/api/maps/${map}/surface?max_px=900`);
  if (map !== S.map) return;
  const dv = new DataView(buf);
  const W = dv.getUint32(0, true), H = dv.getUint32(4, true);
  const [x0, y0, x1, y1] = [0, 1, 2, 3].map((k) => dv.getFloat32(8 + 4 * k, true));
  const Z = new Float32Array(buf.slice(24, 24 + 4 * W * H));
  const L = new Uint8Array(buf, 24 + 4 * W * H, W * H);
  const px = (x1 - x0) / W;
  const N = W * H;
  const valid = [];
  for (let i = 0; i < N; i++) if (Number.isFinite(Z[i])) valid.push(Z[i]);
  if (!valid.length) { $("three-msg").textContent = "This map has no ground cells."; return; }
  const zsort = Float32Array.from(valid).sort();
  const zmin = zsort[Math.floor(0.01 * (zsort.length - 1))], zmax = zsort[Math.floor(0.99 * (zsort.length - 1))];
  const zmid = zsort[Math.floor(0.5 * (zsort.length - 1))];
  // centre of the ground, so the view orbits around it
  let cxs = 0, cys = 0, nv = 0, cmin = W, cmax = 0, rmin = H, rmax = 0;
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) if (Number.isFinite(Z[r * W + c])) {
    cxs += c; cys += r; nv++;
    if (c < cmin) cmin = c; if (c > cmax) cmax = c; if (r < rmin) rmin = r; if (r > rmax) rmax = r;
  }
  const cc = cxs / nv, cr = cys / nv;
  // lightly smoothed heights for the solid surface: lidar speckle otherwise
  // shows as a crinkled, faceted skin (the raw heights stay in the points view)
  const ZS = new Float32Array(N);
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
    const i = r * W + c;
    if (!Number.isFinite(Z[i])) { ZS[i] = NaN; continue; }
    let sum = 0, n = 0;
    for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
      const rr = r + dr, cc2 = c + dc;
      if (rr < 0 || rr >= H || cc2 < 0 || cc2 >= W) continue;
      const v = Z[rr * W + cc2];
      if (Number.isFinite(v)) { const wgt = dr || dc ? 1 : 2; sum += wgt * v; n += wgt; }
    }
    ZS[i] = sum / n;
  }
  const cx = x0 + (cc + 0.5) * px, cy = y1 - (cr + 0.5) * px;

  const pos = new Float32Array(N * 3);
  const col = new Float32Array(N * 3);
  const uv = new Float32Array(N * 2);
  const rp = S.previews.relief;        // shaded-relief texture and where it sits
  if (rp) {
    const [tx0, ty0, tx1, ty1] = rp.extent;
    for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
      const i = r * W + c;
      uv[2 * i] = (x0 + (c + 0.5) * px - tx0) / (tx1 - tx0);
      uv[2 * i + 1] = (y1 - (r + 0.5) * px - ty0) / (ty1 - ty0);
    }
  }
  const idx = [];
  for (let r = 0; r + 1 < H; r++) {
    for (let c = 0; c + 1 < W; c++) {
      const a = r * W + c, b = a + 1, d = a + W, e = d + 1;
      const fa = Number.isFinite(Z[a]), fb = Number.isFinite(Z[b]), fd = Number.isFinite(Z[d]), fe = Number.isFinite(Z[e]);
      if (fa && fb && fd) idx.push(a, d, b);
      if (fb && fd && fe) idx.push(b, d, e);
      else if (fa && fd && fe && !(fa && fb && fd)) idx.push(a, d, e);
    }
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  geom.setAttribute("color", new THREE.BufferAttribute(col, 3));
  geom.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  geom.setIndex(new THREE.BufferAttribute(N > 65535 ? new Uint32Array(idx) : new Uint16Array(idx), 1));

  // --- scene ----------------------------------------------------------------------------
  const wrap = $("three-wrap");
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  wrap.append(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#cfdde6");      // pale sky, like a printed relief map
  const span = Math.max(cmax - cmin + 1, rmax - rmin + 1, 10) * px;   // size of the ground itself
  const cam = new THREE.PerspectiveCamera(50, 1, span * 0.002, span * 20);
  cam.up.set(0, 0, 1);
  const surfMat = new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide, roughness: 0.95, metalness: 0 });
  let reliefMat = null;
  if (rp && rp.texture) {
    const tex = await new THREE.TextureLoader().loadAsync(`/api/maps/${map}/preview/${rp.texture}?t=${Date.now()}`);
    if (map !== S.map) { tex.dispose(); return; }
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = renderer.capabilities.getMaxAnisotropy();
    reliefMat = new THREE.MeshStandardMaterial({ map: tex, side: THREE.DoubleSide, roughness: 0.95, metalness: 0, alphaTest: 0.5 });
  }
  if (!reliefMat) [...$("three-color").options].forEach((o) => { if (o.value === "relief") o.disabled = true; });
  if (!reliefMat && $("three-color").value === "relief") $("three-color").value = "label";
  const mesh = new THREE.Mesh(geom, reliefMat || surfMat);
  mesh.castShadow = true; mesh.receiveShadow = true;
  scene.add(mesh);
  const ptsMat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
  const points = new THREE.Points(geom, ptsMat);
  scene.add(points);
  // soft sky light plus a low, warm sun from the north-west (as on the 2D map),
  // casting shadows so curbs and banks stand out
  scene.add(new THREE.HemisphereLight(0xe4eef8, 0x4a4234, 0.8));
  const sun = new THREE.DirectionalLight(0xfff1dc, 3.2);
  sun.position.set(-0.6 * span, 0.8 * span, 0.55 * span);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  Object.assign(sun.shadow.camera, { left: -span, right: span, top: span, bottom: -span, near: 0.01 * span, far: 4 * span });
  sun.shadow.camera.updateProjectionMatrix();
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.02 * px;
  scene.add(sun);

  const controls = new OrbitControls(cam, renderer.domElement);
  // right button turns the view, left button moves it, wheel zooms
  controls.mouseButtons = { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };
  controls.screenSpacePanning = false;
  controls.enableDamping = true;
  renderer.domElement.addEventListener("contextmenu", (e) => e.preventDefault());
  const home = () => {
    cam.position.set(0, -span * 0.55, span * 0.45);
    controls.target.set(0, 0, 0);
    controls.update();
  };
  home();

  const update = () => {
    const ex = Number($("three-exag").value);
    const mode = $("three-color").value;
    const byZ = mode === "z" || mode === "relief";
    const asPoints = $("three-style").value === "points";
    for (let r = 0; r < H; r++) {
      for (let c = 0; c < W; c++) {
        const i = r * W + c, z = Z[i];
        pos[3 * i] = x0 + (c + 0.5) * px - cx;
        pos[3 * i + 1] = y1 - (r + 0.5) * px - cy;
        const zz = asPoints ? z : ZS[i];
        pos[3 * i + 2] = Number.isFinite(zz) ? (zz - zmid) * ex : 0;
        const rgb = byZ ? rampColor(mode === "relief" ? "relief" : "terrain", (z - zmin) / Math.max(zmax - zmin, 1e-6)) : (SURFACE_RGB[L[i]] || SURFACE_RGB[0]);
        col[3 * i] = toLinear(rgb[0] / 255); col[3 * i + 1] = toLinear(rgb[1] / 255); col[3 * i + 2] = toLinear(rgb[2] / 255);
      }
    }
    geom.attributes.position.needsUpdate = true;
    geom.attributes.color.needsUpdate = true;
    geom.computeVertexNormals();
    geom.computeBoundingSphere();
    mesh.material = mode === "relief" && reliefMat ? reliefMat : surfMat;
    mesh.visible = !asPoints;
    points.visible = asPoints;
    ptsMat.size = Number($("three-size").value);
    $("three-size-wrap").hidden = !asPoints;
    const origin = S.status && S.status.origin;
    if (byZ) legendBox($("three-legend"), { title: "Elevation", ramp: mode === "relief" ? "relief" : "terrain", lo: zmin, hi: zmax, unit: "m",
      digits: zmax - zmin < 5 ? 2 : 1, asl: origin ? origin[2] : undefined });
    else {
      const box = $("three-legend");
      box.innerHTML = "";
      box.append(el("div", { class: "t" }, "Surface"));
      [["road", 1], ["sidewalk", 2], ["curb", 3], ["other ground", 4]].forEach(([name, k]) =>
        box.append(el("div", { class: "st" }, el("span", { class: "sw", style: `background: rgb(${SURFACE_RGB[k].join(",")})` }), el("span", { class: "v" }, name))));
      box.hidden = false;
    }
  };
  // --- lidar layer: every lidar point, loaded the first time it is switched on --------
  const lidMat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
  let lid = null, lidLoading = false;
  const groundAt = (x, y) => {      // ground height under a point (nearest grid cell, NaN off the map)
    const c = Math.floor((x - x0) / px), r = Math.floor((y1 - y) / px);
    if (c < 0 || c >= W || r < 0 || r >= H) return NaN;
    return Z[r * W + c];
  };
  const lidarUpdate = async () => {
    const on = $("three-lidar").checked;
    $("three-lidar-opts").hidden = !on;
    if (!on) { if (lid) lid.points.visible = false; return; }
    if (!lid) {
      if (lidLoading) return;
      lidLoading = true;
      let b;
      try { b = await api(`/api/maps/${map}/lidar?max_points=2500000`); }
      catch (e) { lidLoading = false; $("three-lidar").checked = false; $("three-lidar-opts").hidden = true; toast(e.message, true); return; }
      if (map !== S.map || !alive) return;
      const n = new DataView(b).getUint32(0, true);
      const xyz = new Float32Array(b.slice(4, 4 + 12 * n));
      const it = new Uint8Array(b, 4 + 12 * n, n), gr = new Uint8Array(b, 4 + 13 * n, n), hits = new Uint8Array(b, 4 + 14 * n, n);
      const hag = new Float32Array(n);
      // ground under each point: the map's ground grid, else the nearest ground point's column is unknown -> use median
      for (let i = 0; i < n; i++) { const g = groundAt(xyz[3 * i], xyz[3 * i + 1]); hag[i] = Number.isFinite(g) ? xyz[3 * i + 2] - g : xyz[3 * i + 2] - zmid; }
      const g2 = new THREE.BufferGeometry();
      g2.setAttribute("position", new THREE.BufferAttribute(new Float32Array(3 * n), 3));
      g2.setAttribute("color", new THREE.BufferAttribute(new Float32Array(3 * n), 3));
      lid = { n, xyz, it, gr, hits, hag, geom: g2, points: new THREE.Points(g2, lidMat) };
      scene.add(lid.points);
      lidLoading = false;
    }
    const ex = Number($("three-exag").value), mode = $("three-lidar-color").value;
    const all = $("three-lidar-what").value === "all", steady = $("three-lidar-static").checked;
    const P = lid.geom.attributes.position.array, C = lid.geom.attributes.color.array;
    let m = 0;
    for (let i = 0; i < lid.n; i++) {
      if (!all && lid.gr[i]) continue;
      if (steady && lid.hits[i] < 4) continue;
      const z = lid.xyz[3 * i + 2];
      // keep the ground where the surface is drawn: lift by the same exaggeration as the ground,
      // and keep the true height of things above it
      const g = z - lid.hag[i];
      P[3 * m] = lid.xyz[3 * i] - cx; P[3 * m + 1] = lid.xyz[3 * i + 1] - cy;
      P[3 * m + 2] = (g - zmid) * ex + lid.hag[i];
      let rgb;
      if (mode === "intensity") { const v = lid.it[i] / 255; rgb = [40 + 215 * v, 40 + 215 * v, 40 + 215 * v]; }
      else if (mode === "height") rgb = rampColor("terrain", Math.min(Math.max(lid.hag[i] / 8, 0), 1));
      else rgb = lid.gr[i] ? [150, 140, 120] : [214, 60, 50];
      C[3 * m] = toLinear(rgb[0] / 255); C[3 * m + 1] = toLinear(rgb[1] / 255); C[3 * m + 2] = toLinear(rgb[2] / 255);
      m++;
    }
    lid.geom.setDrawRange(0, m);
    lid.geom.attributes.position.needsUpdate = true;
    lid.geom.attributes.color.needsUpdate = true;
    lid.geom.computeBoundingSphere();
    lidMat.size = Number($("three-lidar-size").value);
    lid.points.visible = true;
  };
  const lidIds = ["three-lidar", "three-lidar-color", "three-lidar-what", "three-lidar-static", "three-lidar-size", "three-exag"];
  const lidarOnInput = () => { lidarUpdate(); };
  lidIds.forEach((id) => $(id).addEventListener("input", lidarOnInput));

  // --- signal span: poles, wires and heads, at true height above the ground -----------
  const sigGroup = new THREE.Group();
  scene.add(sigGroup);
  const sigMats = [new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.5, metalness: 0.6 }),
    new THREE.LineBasicMaterial({ color: 0x7a28a0 }), new THREE.MeshStandardMaterial({ color: 0xf4b400, roughness: 0.6 })];
  const sigUpdate = () => {
    sigGroup.children.slice().forEach((c) => { sigGroup.remove(c); c.geometry.dispose(); });
    const ex = Number($("three-exag").value);
    const base = (x, y, gz) => { const g = groundAt(x, y); return ((Number.isFinite(g) ? g : gz) - zmid) * ex; };
    for (const sp of S.signals || []) {
      const top = {};
      for (const q of sp.poles || []) {
        const b = base(q.x, q.y, q.ground_z);
        const m = new THREE.Mesh(new THREE.CylinderGeometry(q.radius, q.radius, q.height, 12), sigMats[0]);
        m.rotation.x = Math.PI / 2; m.position.set(q.x - cx, q.y - cy, b + q.height / 2); m.castShadow = true;
        sigGroup.add(m); top[q.id] = b;
      }
      for (const w of sp.wires || []) {
        const [p, q] = w.poles.map((i) => (sp.poles || []).find((o) => o.id === i));
        if (!p || !q) continue;
        const hgt = w.height || 6.5;
        const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(p.x - cx, p.y - cy, top[p.id] + hgt), new THREE.Vector3(q.x - cx, q.y - cy, top[q.id] + hgt)]);
        sigGroup.add(new THREE.Line(g, sigMats[1]));
      }
      for (const h of sp.heads || []) {
        const b = base(h.x, h.y, h.ground_z), H_ = h.top - h.bottom;
        const m = new THREE.Mesh(new THREE.BoxGeometry(Math.max(h.width, 0.3), 0.35, H_), sigMats[2]);
        m.rotation.z = Math.atan2(h.wire_dir[1], h.wire_dir[0]);
        m.position.set(h.x - cx, h.y - cy, b + h.bottom + H_ / 2); m.castShadow = true;
        sigGroup.add(m);
      }
    }
  };
  sigUpdate();
  $("three-exag").addEventListener("input", sigUpdate);

  update();
  lidarUpdate();
  const ids = ["three-color", "three-exag", "three-size", "three-style"];
  ids.forEach((id) => $(id).addEventListener("input", update));
  $("three-home").onclick = home;
  const resize = () => {
    const w = wrap.clientWidth, h = wrap.clientHeight;
    renderer.setSize(w, h); cam.aspect = w / Math.max(h, 1); cam.updateProjectionMatrix();
  };
  let alive = true;
  const loop = () => { if (!alive) return; controls.update(); renderer.render(scene, cam); requestAnimationFrame(loop); };
  resize(); loop();
  $("three-msg").hidden = true;
  S.three = { resize, dispose: () => {
    alive = false;
    ids.forEach((id) => $(id).removeEventListener("input", update));
    lidIds.forEach((id) => $(id).removeEventListener("input", lidarOnInput));
    $("three-exag").removeEventListener("input", sigUpdate);
    sigGroup.children.forEach((c) => c.geometry.dispose()); sigMats.forEach((m_) => m_.dispose());
    if (lid) lid.geom.dispose();
    lidMat.dispose();
    $("three-legend").hidden = true;
    geom.dispose(); surfMat.dispose(); ptsMat.dispose(); controls.dispose();
    if (reliefMat) { reliefMat.map.dispose(); reliefMat.dispose(); }
    renderer.dispose(); renderer.domElement.remove();
  } };
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
    ["app.cpus", "CPU cores to use", ["all", "e-cores"], "e-cores: only the slower efficiency cores. Use it on an Intel 13th/14th-gen CPU that gives random errors on the fast P-cores. Applies to the next build."],
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
    ["lidar.deskew", "Correct scans for the car's motion", "bool", "Uses each point's time stamp so a scan taken while driving is not smeared."],
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
    ["xodr.road_defaults.type", "Road type", ["town", "rural", "motorway", "lowSpeed"], "For every road; change single roads in the lane editor (Map tab, click a lane)."],
    ["xodr.road_defaults.speed", "Speed limit", "number", "Written into the .xodr for every road unless changed in the lane editor."],
    ["xodr.road_defaults.unit", "Speed unit", ["mph", "km/h"]],
  ]],
  ["Junctions", [
    ["junctions.fit_to_curbs", "Fit the junction edge to the curbs", "bool", "Also adds a short road for each street at the junction that no drive went into."],
    ["junctions.movements", "Turns to connect", ["all", "driven"], "all: every turn between the streets (remove single ones in the lane editor); driven: only turns a drive made."],
    ["lanes.detect_markings", "Find stop lines and crosswalks", "bool"],
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

// --- stay on the running version ---------------------------------------------------------
// A tab left open from an earlier run keeps showing the old app even after the
// app is updated and restarted. Check the running version now and then and
// reload when it has changed.
const PAGE_VERSION = (document.querySelector(".ver") || { textContent: "" }).textContent.replace(/^v/, "").trim();
setInterval(async () => {
  if (S.job && S.job.state === "running") return;
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    const v = (await r.json()).version;
    if (v && PAGE_VERSION && v !== PAGE_VERSION) location.replace("/?v=" + encodeURIComponent(v));
  } catch (e) { /* app not running right now */ }
}, 5000);

// --- start -------------------------------------------------------------------------------
(guard(async () => {
  const st = await api("/api/state");
  if (!st.kiss_icp) $("log").textContent += "KISS-ICP is not installed: poses come from GNSS/INS only.\n";
  if (!st.patchworkpp) $("log").textContent += "Patchwork++ is not installed: using the built-in ground segmentation.\n";
  await loadRecordings();
  await loadMaps(false);
  if (st.job && st.job.state === "running") { S.job = st.job; pollJob(); } else showJob(st.job);
}))();
