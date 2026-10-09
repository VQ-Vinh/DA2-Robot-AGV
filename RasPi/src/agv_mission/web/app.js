// AGV Dashboard: ve ban do + xe, giao nhiem vu qua /api/command, nhan trang thai qua SSE /api/events.
'use strict';

const TASK = { pick: 'Lấy hàng', drop: 'Trả hàng', charge: 'Sạc pin', pass: 'Đi qua' };
const PHASE = {
  navigating: 'Đang di chuyển',
  waiting: 'Đang làm việc tại trạm',
  retry_wait: 'Chưa tới được, đang thử lại',
  idle: 'Đang chờ lệnh',
};
const KIND_VAR = { charge: '--st-charge', pick: '--st-pick', inbound: '--st-inbound', waypoint: '--st-waypoint' };
const DRAFT_KEY = 'agv-dashboard-draft';

const $ = (s) => document.querySelector(s);
const canvas = $('#map');
const ctx = canvas.getContext('2d');

let cfg = { stations: {}, missions: {} };
let map = null;            // {width, height, resolution, origin, version, image}
let live = { pose: null, path: [], state: null, link: false };
let view = null;           // {s, ox, oy}: pixel man hinh moi o ban do, vi tri goc anh
let gotoMode = false;
let selected = null;       // ten vi tri dang chon
let draft = loadDraft();
let orders = null;         // trang thai order_manager (/order/state)
let lastOrdersText = '';
let keepout = null;        // mat na vung cam (cac o ke mini), cung dang voi map
const loading = {};        // lop dang tai: {map: true, keepout: true}
let colors = {};

// ---------- tien ich ----------
function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function rgb(color) {
  const probe = document.createElement('span');
  probe.style.color = color;
  document.body.appendChild(probe);
  const m = getComputedStyle(probe).color.match(/\d+/g).map(Number);
  probe.remove();
  return m.slice(0, 3);
}

function readColors() {
  colors = {
    free: rgb(css('--map-free')), occ: rgb(css('--map-occ')), unknown: rgb(css('--map-unknown')),
    path: css('--path'), robot: css('--robot'), text: css('--text'), surface: css('--surface'),
    accent: css('--accent'), keepout: rgb(css('--keepout')), shelf: css('--shelf'),
  };
  for (const [k, v] of Object.entries(KIND_VAR)) colors[k] = css(v);
}

function stationColor(st) {
  return colors[st.kind] || colors.waypoint;
}

function label(name) {
  const st = cfg.stations[name];
  return st ? st.label || name : name;
}

let toastTimer = null;
function toast(text, isErr = false) {
  const el = $('#toast');
  el.textContent = text;
  el.classList.toggle('err', isErr);
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2500);
}

function el(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else e.setAttribute(k, v);
  }
  for (const c of children) e.append(c);
  return e;
}

// ---------- lenh ----------
function busy() {
  return live.state && live.state.phase !== 'idle';
}

async function send(cmd, confirmIfBusy = true) {
  if (confirmIfBusy && busy()) {
    const what = live.state.mission || label(live.state.step && live.state.step.station);
    if (!confirm(`Xe đang làm "${what}". Huỷ việc đó và làm lệnh mới?`)) return;
  }
  try {
    const r = await fetch('api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd }),
    });
    const res = await r.json();
    if (!res.ok) toast(res.message, true);
  } catch (e) {
    toast('Không gửi được lệnh: mất kết nối', true);
  }
}

// ---------- ban do ----------
async function loadGrid(layer) {
  if (loading[layer]) return;
  loading[layer] = true;
  try {
    const r = await fetch(`api/${layer}`);
    if (!r.ok) return;
    const g = await r.json();
    g.cells = Uint8Array.from(atob(g.data), (c) => c.charCodeAt(0));
    delete g.data;
    if (layer === 'map') {
      map = g;
      renderMapImage();
      if (!view) fit();
      $('#map-hint').hidden = true;
    } else {
      keepout = g;
      renderKeepoutImage();
      draw();
    }
  } catch (e) {
    // thu lai o lan cap nhat sau
  } finally {
    loading[layer] = false;
  }
}

// Ve OccupancyGrid thanh anh, moi o 1 pixel; color(v) tra [r, g, b, a] hoac null (trong suot)
function gridImage(grid, color) {
  const { width: w, height: h, cells } = grid;
  const off = document.createElement('canvas');
  off.width = w;
  off.height = h;
  const octx = off.getContext('2d');
  const img = octx.createImageData(w, h);
  for (let j = 0; j < h; j++) {
    const row = (h - 1 - j) * w;     // hang 0 cua OccupancyGrid o duoi cung
    for (let i = 0; i < w; i++) {
      const c = color(cells[j * w + i]);
      if (!c) continue;
      const p = (row + i) * 4;
      img.data[p] = c[0];
      img.data[p + 1] = c[1];
      img.data[p + 2] = c[2];
      img.data[p + 3] = c[3];
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

function renderMapImage() {
  if (!map) return;
  map.image = gridImage(map, (v) => [...(v === 255 ? colors.unknown : v >= 65 ? colors.occ : colors.free), 255]);
}

function renderKeepoutImage() {
  if (!keepout) return;
  // Mat na Nav2 (mode scale): o den = 100 = cam vao
  const c = [...colors.keepout, 150];
  keepout.image = gridImage(keepout, (v) => (v !== 255 && v >= 50 ? c : null));
}

function fit() {
  if (!map) return;
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  const pad = 24;
  if (W <= 2 * pad || H <= 2 * pad) {   // canvas chua co kich thuoc: ResizeObserver se goi lai
    view = null;
    return;
  }
  const s = Math.min((W - 2 * pad) / map.width, (H - 2 * pad) / map.height);
  view = { s, ox: (W - map.width * s) / 2, oy: (H - map.height * s) / 2 };
  draw();
}

function zoom(factor, cx, cy) {
  if (!view) return;
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  if (cx === undefined) { cx = W / 2; cy = H / 2; }
  const s = Math.min(Math.max(view.s * factor, 0.3), 80);
  const k = s / view.s;
  view = { s, ox: cx - (cx - view.ox) * k, oy: cy - (cy - view.oy) * k };
  draw();
}

// Toa do map (m) <-> man hinh (px CSS)
function toScreen(x, y) {
  const r = map.resolution;
  return [
    view.ox + ((x - map.origin[0]) / r) * view.s,
    view.oy + (map.height - (y - map.origin[1]) / r) * view.s,
  ];
}

function toWorld(sx, sy) {
  const r = map.resolution;
  return [
    map.origin[0] + ((sx - view.ox) / view.s) * r,
    map.origin[1] + (map.height - (sy - view.oy) / view.s) * r,
  ];
}

function metersToPx(m) {
  return (m / map.resolution) * view.s;
}

function targetPose() {
  const step = live.state && live.state.step;
  if (!step) return null;
  if (step.pose) return step.pose;
  return cfg.stations[step.station] || null;
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  if (!map || !map.image || !view) return;

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(map.image, view.ox, view.oy, map.width * view.s, map.height * view.s);

  // Vung cam: co the khac goc / kich thuoc voi /map nen dat theo toa do cua chinh no
  if (keepout && keepout.image) {
    const k = keepout;
    const [kx, ky] = toScreen(k.origin[0], k.origin[1] + k.height * k.resolution);
    const scale = (k.resolution / map.resolution) * view.s;
    ctx.drawImage(k.image, kx, ky, k.width * scale, k.height * scale);
  }

  // Duong di Nav2 (chi khi xe dang di)
  if (live.path.length > 1 && live.state && live.state.phase === 'navigating') {
    ctx.beginPath();
    live.path.forEach(([x, y], i) => {
      const [sx, sy] = toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.strokeStyle = colors.path;
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.setLineDash([]);
    ctx.stroke();
  }

  // Dich hien tai
  const tgt = targetPose();
  if (tgt) {
    const [sx, sy] = toScreen(tgt.x, tgt.y);
    const r = Math.max(14, metersToPx(0.35));
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.strokeStyle = colors.accent;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  drawShelves();

  // Cac vi tri
  const fontPx = Math.round(Math.min(Math.max(metersToPx(0.22), 10), 14));
  ctx.font = `600 ${fontPx}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'bottom';
  for (const [name, st] of Object.entries(cfg.stations)) {
    const [sx, sy] = toScreen(st.x, st.y);
    const r = Math.max(5, metersToPx(st.kind === 'waypoint' ? 0.12 : 0.25));
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fillStyle = stationColor(st);
    ctx.globalAlpha = 0.85;
    ctx.fill();
    ctx.globalAlpha = 1;
    if (name === selected) {
      ctx.lineWidth = 3;
      ctx.strokeStyle = colors.text;
      ctx.stroke();
    }
    const text = st.label || name;
    ctx.lineWidth = 3;
    ctx.strokeStyle = colors.surface;
    ctx.strokeText(text, sx, sy - r - 3);
    ctx.fillStyle = colors.text;
    ctx.fillText(text, sx, sy - r - 3);
  }

  // Xe: mui ten theo huong
  const p = live.pose;
  if (p) {
    const [sx, sy] = toScreen(p.x, p.y);
    const L = Math.max(12, metersToPx(0.3));
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-p.yaw);                 // y man hinh huong xuong
    ctx.beginPath();
    ctx.moveTo(L * 0.7, 0);
    ctx.lineTo(-L * 0.5, L * 0.45);
    ctx.lineTo(-L * 0.25, 0);
    ctx.lineTo(-L * 0.5, -L * 0.45);
    ctx.closePath();
    ctx.fillStyle = colors.robot;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = colors.surface;
    ctx.stroke();
    ctx.restore();
  }
}

// Ke mini: hinh vuong tai o dang dau; ke dang duoc cho thi ve theo xe
function drawShelves() {
  if (!orders || !orders.shelves) return;
  const cur = orders.orders && orders.orders.find((o) => o.id === orders.current);
  const size = 0.75;
  for (const [name, slot] of Object.entries(orders.shelves)) {
    const carried = cur && cur.shelf === name && orders.carrying && live.pose;
    const sl = orders.slots && orders.slots[slot];
    if (!sl && !carried) continue;
    const x = carried ? live.pose.x : sl.x;
    const y = carried ? live.pose.y : sl.y;
    const yaw = carried ? live.pose.yaw : sl.yaw;
    const [sx, sy] = toScreen(x, y);
    const h = metersToPx(size) / 2;
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-yaw);
    ctx.fillStyle = colors.shelf;
    ctx.globalAlpha = carried ? 0.55 : 0.35;
    ctx.fillRect(-h, -h, 2 * h, 2 * h);
    ctx.globalAlpha = 1;
    ctx.lineWidth = cur && cur.shelf === name ? 3 : 1.5;
    ctx.strokeStyle = colors.shelf;
    ctx.strokeRect(-h, -h, 2 * h, 2 * h);
    ctx.restore();
    ctx.font = `600 ${Math.round(Math.min(Math.max(metersToPx(0.18), 9), 12))}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = colors.text;
    ctx.fillText(name.replace('ke_', 'K'), sx, sy);
  }
}

// ---------- don hang ----------
const ORDER_STATUS = { pending: 'Chờ', running: 'Đang chạy', done: 'Xong', failed: 'Lỗi', cancelled: 'Đã huỷ' };

function fmtTime(t) {
  return t ? new Date(t * 1000).toLocaleTimeString('vi-VN', { hour: '2-digit', minute: '2-digit' }) : '';
}

function renderOrders() {
  const o = orders;
  if (!o) return;
  // Chon ke / tram: chi dung lai khi danh sach doi (giu lua chon cua nguoi dung)
  const shelfSel = $('#o-shelf');
  const names = Object.keys(o.shelves || {}).sort();
  if (shelfSel.options.length !== names.length) {
    shelfSel.replaceChildren(...names.map((n) => el('option', { value: n },
      `${n} (${o.shelves[n] ? 'ô ' + o.shelves[n] : 'chưa rõ vị trí'})`)));
  }
  const stSel = $('#o-station');
  const picks = Object.entries(cfg.stations).filter(([, st]) => st.kind === 'pick');
  if (stSel.options.length !== picks.length) {
    stSel.replaceChildren(...picks.map(([n, st]) => el('option', { value: n }, st.label || n)));
  }

  const b = o.battery;
  $('#bat-text').textContent = b ? `Pin ${Math.round(b.pct)} %${b.charging ? ' ⚡' : ''}` : 'Pin —';
  setPill('#pill-bat', !!b && !o.charge_hold, !!b && o.charge_hold);
  $('#o-pause').textContent = o.paused ? 'Tiếp tục' : 'Tạm dừng';
  $('#o-error').hidden = !o.error;
  $('#o-error-text').textContent = o.error ? `Xe dừng: ${o.error}` : '';
  $('#o-alarm').hidden = !o.alarm || !!o.error;
  $('#o-alarm').textContent = o.alarm ? `Cảnh báo: ${o.alarm}` : '';
  $('#o-estop').disabled = !!o.error;
  const cur = o.orders.find((x) => x.id === o.current);
  $('#o-current').replaceChildren(cur
    ? el('span', {}, el('b', {}, `#${cur.id} ${cur.shelf} → ${label(cur.station)}`), ` — ${o.step_label || ''}`)
    : (o.charge_hold ? `Pin yếu: đang sạc, chưa nhận đơn${o.step_label ? ' — ' + o.step_label : ''}`
      : o.paused ? 'Đang tạm dừng nhận đơn'
        : `Không có đơn đang chạy${o.step_label ? ' — ' + o.step_label : ''}`));
  $('#o-confirm').hidden = o.step !== 'wait_confirm';

  $('#o-queue').replaceChildren(...o.queue.map((q) => el('li', {},
    el('span', { class: 'o-id' }, `#${q.id}`),
    el('span', { class: 'o-text' }, `${q.shelf} → ${label(q.station)}`),
    ...(q.priority > 0 ? [el('span', { class: 'badge prio' }, 'Gấp')] : []),
    el('button', { title: 'Huỷ đơn', 'aria-label': 'Huỷ đơn', onclick: () => send(`order cancel ${q.id}`, false) }, '✕'),
  )));
  const hist = o.orders.filter((x) => x.status !== 'pending' && x.status !== 'running').reverse().slice(0, 8);
  $('#o-history').replaceChildren(...hist.map((h) => el('li', { title: h.error || '' },
    el('span', { class: 'o-id' }, `#${h.id}`),
    el('span', { class: 'o-text' }, `${h.shelf} → ${label(h.station)}`
      + (h.status === 'done' && h.started ? ` · ${Math.round(h.finished - h.started)} s` : '')
      + (h.error ? ` · ${h.error}` : '')),
    el('span', { class: `badge ${h.status}` }, ORDER_STATUS[h.status] || h.status),
    el('span', { class: 'o-id' }, fmtTime(h.finished)),
  )));
  const s = o.stats || {};
  $('#o-stats').textContent = s.done ? `Đã xong ${s.done} đơn, trung bình ${s.avg_s} s/đơn` : '';
}

$('#o-add').addEventListener('click', () =>
  send(`order add ${$('#o-shelf').value} ${$('#o-station').value} ${$('#o-prio').value}`, false));
$('#o-confirm').addEventListener('click', () => send('order confirm', false));
$('#o-ack').addEventListener('click', () => send('order ack', false));
$('#o-estop').addEventListener('click', () => send('order estop', false));
$('#o-pause').addEventListener('click', () => send(orders && orders.paused ? 'order resume' : 'order pause', false));

// ---------- tuong tac ban do ----------
function stationAt(sx, sy) {
  let best = null;
  let bestD = 18;
  for (const [name, st] of Object.entries(cfg.stations)) {
    const [x, y] = toScreen(st.x, st.y);
    const d = Math.hypot(x - sx, y - sy);
    if (d < bestD) { best = name; bestD = d; }
  }
  return best;
}

function pointerPos(e) {
  const r = canvas.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}

let drag = null;
canvas.addEventListener('pointerdown', (e) => {
  if (!view) return;
  canvas.setPointerCapture(e.pointerId);
  const [x, y] = pointerPos(e);
  drag = { x, y, ox: view.ox, oy: view.oy, moved: false };
});

canvas.addEventListener('pointermove', (e) => {
  if (!drag) return;
  const [x, y] = pointerPos(e);
  if (Math.hypot(x - drag.x, y - drag.y) > 4) {
    drag.moved = true;
    canvas.classList.add('dragging');
  }
  if (drag.moved) {
    view.ox = drag.ox + x - drag.x;
    view.oy = drag.oy + y - drag.y;
    draw();
  }
});

canvas.addEventListener('pointerup', (e) => {
  if (!drag) return;
  const wasDrag = drag.moved;
  drag = null;
  canvas.classList.remove('dragging');
  if (wasDrag || !map) return;
  const [sx, sy] = pointerPos(e);
  if (gotoMode) {
    const [x, y] = toWorld(sx, sy);
    const p = live.pose;
    const yaw = p ? Math.atan2(y - p.y, x - p.x) : 0;
    send(`goto_xy ${x.toFixed(2)} ${y.toFixed(2)} ${yaw.toFixed(3)}`);
    setGotoMode(false);
    return;
  }
  selectStation(stationAt(sx, sy));
});

canvas.addEventListener('pointercancel', () => {
  drag = null;
  canvas.classList.remove('dragging');
});

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [x, y] = pointerPos(e);
  zoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, x, y);
}, { passive: false });

function setGotoMode(on) {
  gotoMode = on;
  $('#btn-mode').classList.toggle('active', on);
  document.body.classList.toggle('goto-mode', on);
  if (on) {
    selectStation(null);
    toast('Bấm một điểm trên bản đồ để xe đi tới');
  }
}

function selectStation(name) {
  selected = name;
  const card = $('#station-card');
  card.hidden = !name;
  if (name) {
    const st = cfg.stations[name];
    $('#sc-name').textContent = st.label || name;
    $('#sc-sub').textContent = `${name} · x ${st.x}  y ${st.y}`;
    $('#b-station').value = name;
  }
  draw();
}

$('#btn-mode').addEventListener('click', () => setGotoMode(!gotoMode));
$('#btn-zoom-in').addEventListener('click', () => zoom(1.3));
$('#btn-zoom-out').addEventListener('click', () => zoom(1 / 1.3));
$('#btn-fit').addEventListener('click', fit);
$('#sc-close').addEventListener('click', () => selectStation(null));
$('#sc-goto').addEventListener('click', () => { if (selected) send(`goto ${selected}`); });
$('#sc-add').addEventListener('click', () => {
  if (!selected) return;
  const st = cfg.stations[selected];
  const task = { charge: 'charge', pick: 'drop', inbound: 'pick' }[st.kind] || 'pass';
  addStep(selected, task, task === 'charge' ? 0 : 3);
});

// ---------- tao nhiem vu ----------
function loadDraft() {
  try {
    const d = JSON.parse(localStorage.getItem(DRAFT_KEY) || '[]');
    return Array.isArray(d) ? d : [];
  } catch (e) {
    return [];
  }
}

function saveDraft() {
  try { localStorage.setItem(DRAFT_KEY, JSON.stringify(draft)); } catch (e) { /* khong co storage */ }
}

function addStep(station, task, wait) {
  draft.push({ station, task, wait: Math.max(0, Math.min(Number(wait) || 0, 600)) });
  renderDraft();
}

function renderDraft() {
  draft = draft.filter((s) => cfg.stations[s.station] && TASK[s.task]);
  const list = $('#b-list');
  list.replaceChildren(...draft.map((s, i) => el('li', {},
    el('span', { class: 'd-text' }, `${label(s.station)} — ${TASK[s.task]}${s.wait ? `, ${s.wait} s` : ''}`),
    el('button', { title: 'Lên', 'aria-label': 'Lên', onclick: () => moveStep(i, -1) }, '↑'),
    el('button', { title: 'Xuống', 'aria-label': 'Xuống', onclick: () => moveStep(i, 1) }, '↓'),
    el('button', { title: 'Xoá', 'aria-label': 'Xoá', onclick: () => { draft.splice(i, 1); renderDraft(); } }, '✕'),
  )));
  $('#b-run').disabled = draft.length === 0;
  saveDraft();
}

function moveStep(i, d) {
  const j = i + d;
  if (j < 0 || j >= draft.length) return;
  [draft[i], draft[j]] = [draft[j], draft[i]];
  renderDraft();
}

$('#b-task').addEventListener('change', () => {
  const t = $('#b-task').value;
  $('#b-wait').value = t === 'pick' || t === 'drop' ? 3 : 0;
});
$('#b-add').addEventListener('click', () => addStep($('#b-station').value, $('#b-task').value, $('#b-wait').value));
$('#b-clear').addEventListener('click', () => { draft = []; renderDraft(); });
$('#b-run').addEventListener('click', () => {
  if (!draft.length) return;
  send('seq ' + draft.map((s) => `${s.station}:${s.task}:${s.wait}`).join(' '));
});

// ---------- panel tinh (tu config) ----------
function renderConfig() {
  $('#missions').replaceChildren(...Object.entries(cfg.missions).map(([name, m]) => el('div', { class: 'mission' },
    el('div', { class: 'm-body' },
      el('div', { class: 'm-name' }, name),
      el('div', { class: 'm-desc' }, m.description || ''),
      el('div', { class: 'm-chain' }, m.steps.map((s) => label(s.station)).join(' → ')),
    ),
    el('button', { class: 'btn primary', onclick: () => send(`run ${name}`) }, 'Chạy'),
  )));

  $('#b-station').replaceChildren(...Object.entries(cfg.stations).map(([name, st]) =>
    el('option', { value: name }, st.label || name)));

  $('#quick').replaceChildren(...Object.entries(cfg.stations).map(([name, st]) => {
    const dot = el('span', { class: 'dot' });
    dot.style.background = stationColor(st);
    return el('button', { class: 'btn', title: name, onclick: () => send(`goto ${name}`) },
      dot, el('span', { class: 'q-label' }, st.label || name));
  }));
  renderDraft();
}

// ---------- trang thai song ----------
function renderStatus() {
  const s = live.state;
  const running = s && s.phase !== 'idle';
  $('#btn-stop').disabled = !running;
  if (!running) {
    $('#st-title').textContent = 'Rảnh';
    $('#st-sub').textContent = s && !s.ready ? 'Đang chờ Nav2 sẵn sàng…' : PHASE.idle;
    $('#st-bar').style.width = '0';
    $('#st-steps').replaceChildren();
    return;
  }
  const step = s.step;
  $('#st-title').textContent = s.mission || `Đi tới ${label(step.station)}`;
  $('#st-sub').textContent = `${PHASE[s.phase] || s.phase}: ${label(step.station)} (${TASK[step.task] || step.task})`
    + (s.runs_left ? ` · còn lặp ${s.runs_left} lần` : '');
  const done = s.done.length;
  $('#st-bar').style.width = `${(100 * done) / Math.max(s.total, 1)}%`;

  const items = [
    ...s.done.map((d) => el('li', { class: d.ok ? 'ok' : 'fail' },
      label(d.station), el('span', { class: 't' }, `${d.time_s}s`))),
    el('li', { class: 'now' }, `${label(step.station)} — ${TASK[step.task] || step.task}`),
    ...s.queue.map((q) => el('li', {}, `${label(q.station)} — ${TASK[q.task] || q.task}`)),
  ];
  $('#st-steps').replaceChildren(...items);
}

function renderPose() {
  const p = live.pose;
  $('#pose').textContent = p
    ? `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · θ ${Math.round((p.yaw * 180) / Math.PI) || 0}°`
    : 'x — · y — · θ —';
}

function appendLog(lines) {
  if (!lines.length) return;
  const box = $('#log');
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 30;
  for (const l of lines) {
    const cls = l.text.startsWith('TONG KET') ? 'sum'
      : /that bai|tu choi|Khong|sai|LOI/.test(l.text) ? 'err' : '';
    box.append(el('li', { class: cls }, el('time', {}, l.t), l.text));
  }
  while (box.children.length > 200) box.firstChild.remove();
  if (atBottom) box.scrollTop = box.scrollHeight;
}

function setPill(id, on, bad = false) {
  const p = $(id);
  p.classList.toggle('on', on);
  p.classList.toggle('bad', !on && bad);
}

let lastStateText = '';
let lastLogId = 0;
let boot = null;           // ma phien cua server; doi = node vua khoi dong lai
function connect() {
  // Tu ket noi lai (thay vi de EventSource tu lam) de gui kem id nhat ky cuoi, tranh in trung
  const es = new EventSource(`api/events?after=${lastLogId}&boot=${boot || ''}`);
  es.onopen = () => setPill('#pill-web', true);
  es.onerror = () => {
    es.close();
    setPill('#pill-web', false, true);
    setPill('#pill-link', false);
    setPill('#pill-nav', false);
    setTimeout(connect, 2000);
  };
  es.onmessage = (ev) => {
    const d = JSON.parse(ev.data);
    if (boot && d.boot !== boot) $('#log').replaceChildren();   // server moi gui lai nhat ky tu dau
    boot = d.boot;
    live.pose = d.pose;
    live.path = d.path;
    live.link = d.link;
    setPill('#pill-web', true);
    setPill('#pill-link', d.link, true);
    setPill('#pill-nav', !!(d.state && d.state.ready));
    const st = JSON.stringify(d.state);
    if (st !== lastStateText) {
      lastStateText = st;
      live.state = d.state;
      renderStatus();
    }
    const ot = JSON.stringify(d.orders);
    if (ot !== lastOrdersText) {
      lastOrdersText = ot;
      orders = d.orders;
      renderOrders();
    }
    const g = d.grids || {};
    if (g.map && (!map || map.version !== g.map)) loadGrid('map');
    if (g.keepout && (!keepout || keepout.version !== g.keepout)) loadGrid('keepout');
    appendLog(d.log);
    if (d.log.length) lastLogId = d.log[d.log.length - 1].id;
    renderPose();
    draw();
  };
}

$('#btn-stop').addEventListener('click', () => send('cancel', false));

// ---------- khoi dong ----------
async function init() {
  readColors();
  try {
    cfg = await (await fetch('api/config')).json();
  } catch (e) {
    toast('Không tải được cấu hình', true);
  }
  renderConfig();
  connect();
}

new ResizeObserver(() => { if (map && view) draw(); else if (map) fit(); }).observe(canvas);
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
  readColors();
  renderMapImage();
  renderKeepoutImage();
  renderConfig();
  draw();
});

init();
