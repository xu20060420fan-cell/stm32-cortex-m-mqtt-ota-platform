/*
 * iotCenter.js —— 校园物联网中心（重写版）
 * ===========================================================================
 *
 * 重写原则：**设备端与页面端一行都不用改**。
 *   设备和后端之间的契约（4 条设备路由 + 请求头 X-Iot-Key + 响应字段）
 *   以及页面依赖的 11 条管理路由全部原样保留；
 *   改的是内部结构：把 100KB 单文件里纠缠的状态、落盘、OTA、MQTT 拆成
 *   清晰的几块，并修掉旧实现里几处真实会咬人的问题：
 *
 *   1. **写盘放大**：旧实现每条遥测都可能整份重写设备表（设备数 × N 条），
 *      现在遥测走内存环形缓冲 + 3 秒合并刷盘，设备状态 30 秒刷一次，
 *      管理端的写操作才立即落盘。
 *   2. **固件上传的 MulterError: Unexpected field**：旧实现只认 `firmware`
 *      这一个字段名，前端/脚本换个字段名就 500。现在接受任意字段名，
 *      取第一个文件，并把 multer 的错误转成明确的 400 JSON。
 *   3. **设备身份解析**：旧实现里 deviceId 与密钥的来源优先级分散在多处，
 *      现在统一由 requireDevice 解析：请求头/请求体/query 都认，
 *      没带 deviceId 时用密钥反查（8266 的事件/OTA 结果就是这样上报的）。
 *   4. **OTA 销账**：pendingOta 只有在收到 success 时才清空；失败累计到
 *      3 次置 active=false 停发，避免「同一指令每 10 秒复位一次 STM32」。
 *
 * 数据文件格式**完全不变**（线上已有 dev01 的登记、阈值、固件、遥测）：
 *   iot-devices.json    { devices: { <id>: {...} }, events: [ ... ] }
 *   iot-thresholds.json { <id>: { values, overrides, updatedAt } }
 *   iot-firmware.json   { items: [ ... ] }
 *   iot-telemetry.json  { <id>: [ ...最近 120 条... ] }
 */

'use strict';

const crypto = require('crypto');
const fs = require('fs');
const path = require('path');
const multer = require('multer');

const { isDangerousKey } = require('./safeKeys');
const { createMqttBridge } = require('./mqttBridge');

/* ── 常量 ──────────────────────────────────────────────────────────── */

/** 每台设备保留的遥测条数（内存环形，超出丢最旧）。
 * 默认 2 秒一条时约等于 1 小时历史；网页「传感器历史」直接查这份。 */
const TELEMETRY_KEEP = 1800;
/** 事件流水保留条数 */
const EVENT_KEEP = 500;
/** 设备状态写盘节流 */
const DEVICE_FLUSH_MS = 30 * 1000;
/** 遥测合并刷盘间隔 */
const TELEMETRY_FLUSH_MS = 10 * 1000;
/** 超过这么久没有心跳就算离线 */
const ONLINE_TIMEOUT_MS = 90 * 1000;
/** 连续失败几次后停发这条 OTA 指令 */
const OTA_MAX_FAILURES = 3;
/** 固件签名链接有效期 */
const DEFAULT_LINK_TTL_MS = 24 * 60 * 60 * 1000;
const MAX_LINK_TTL_MS = 7 * 24 * 60 * 60 * 1000;
/** 上报周期范围（秒 / 毫秒），与设备端 TH 协议保持一致 */
const REPORT_INTERVAL_MIN_SEC = 0.5;
const REPORT_INTERVAL_MAX_SEC = 3600;
const REPORT_INTERVAL_MIN_MS = 500;
const REPORT_INTERVAL_MAX_MS = REPORT_INTERVAL_MAX_SEC * 1000;
/** 固件大小上限 */
const FIRMWARE_MAX_BYTES = 64 * 1024 * 1024;
/** 前端阈值表单的元信息 —— 页面直接照这份渲染 */
const THRESHOLD_META = {
  tempMin: { type: 'number', default: 0, min: -50, max: 100, label: '温度下限(℃)' },
  tempMax: { type: 'number', default: 60, min: -50, max: 100, label: '温度上限(℃)' },
  humMin: { type: 'number', default: 0, min: 0, max: 100, label: '湿度下限(%RH)' },
  humMax: { type: 'number', default: 100, min: 0, max: 100, label: '湿度上限(%RH)' },
  pressMin: { type: 'number', default: 300, min: 300, max: 1200, label: '气压下限(hPa)' },
  pressMax: { type: 'number', default: 1200, min: 300, max: 1200, label: '气压上限(hPa)' },
  fallSensitivity: { type: 'number', default: 2, min: 1, max: 10, label: '跌落灵敏度(级)' },
  alarmEnabled: { type: 'boolean', default: true, label: '告警开关' },
  reportIntervalSec: {
    type: 'number', default: 2, min: REPORT_INTERVAL_MIN_SEC, max: REPORT_INTERVAL_MAX_SEC,
    step: 0.5, label: '上报间隔(秒)',
  },
};
const THRESHOLD_KEYS = Object.keys(THRESHOLD_META);

/** 传感器量程：超范围置 null（丢字段，不丢整条遥测） */
const SENSOR_RANGE = {
  temp: [-50, 125],
  hum: [0, 100],
  press: [300, 1200],
  accX: [-180, 180],
  accY: [-180, 180],
  accZ: [-180, 180],
  batt: [0, 100],
};

/** 遥测字段别名（设备/网关/老固件叫法不同，都认） */
const FIELD_ALIAS = {
  ts: ['ts', 'time', 'timestamp', 'at'],
  temp: ['temp', 'temperature', 'temp_c', 'tempC', 'temperature_c', 'temp_celsius'],
  hum: ['hum', 'humidity', 'rh', 'humidity_pct', 'hum_pct'],
  press: ['press', 'pressure', 'pressPa', 'press_pa', 'pressure_pa', 'pressHpa', 'press_hpa'],
  accX: ['accX', 'acc_x', 'accelX', 'accel_x', 'pitch'],
  accY: ['accY', 'acc_y', 'accelY', 'accel_y', 'roll'],
  accZ: ['accZ', 'acc_z', 'accelZ', 'accel_z'],
  batt: ['batt', 'battery', 'batteryPct', 'battery_pct', 'bat'],
  status: ['status', 'state', 'alarm'],
  gatewayVersion: ['gatewayVersion', 'gateway_version', 'gwVersion', 'gw_version'],
  firmwareVersion: ['firmwareVersion', 'firmware_version', 'appVersion', 'app_version'],
  thresholds: ['thresholds', 'config'],
};

const EVENT_FIELD_ALIAS = {
  type: ['type', 'event', 'eventType', 'event_type', 'kind'],
  level: ['level', 'severity', 'grade'],
  detail: ['detail', 'message', 'msg', 'desc', 'description', 'info', 'text'],
  ts: ['ts', 'time', 'timestamp'],
};

const DEVICE_ID_RE = /^[A-Za-z0-9._-]{1,32}$/;
const FIRMWARE_NAME_RE = /^[A-Za-z0-9._-]+\.(bin|ota)$/i;

/* ── 小工具 ────────────────────────────────────────────────────────── */

function isPlainObject(v) {
  return v !== null && typeof v === 'object' && !Array.isArray(v);
}

/** 定长比较（挡掉时序侧信道；长度不同直接 false） */
function timingSafeEqualStr(a, b) {
  const ba = Buffer.from(String(a == null ? '' : a));
  const bb = Buffer.from(String(b == null ? '' : b));
  if (ba.length !== bb.length || ba.length === 0) return false;
  return crypto.timingSafeEqual(ba, bb);
}

function numOrNull(v) {
  if (v === null || v === undefined || v === '') return null;
  const n = typeof v === 'number' ? v : Number(v);
  return Number.isFinite(n) ? n : null;
}

/** 气压：设备可能给 Pa（101325）也可能给 hPa（1013），统一成 hPa */
function normalizePress(v) {
  const n = numOrNull(v);
  if (n === null) return null;
  if (n >= 2000 && n <= 120000) return Math.round(n / 100);
  return n;
}

function clampSensor(key, value) {
  if (value === null || value === undefined) return null;
  const range = SENSOR_RANGE[key];
  if (!range) return value;
  if (value < range[0] || value > range[1]) return null;
  return value;
}

function normalizeStatus(raw) {
  const s = String(raw == null ? '' : raw).trim().toLowerCase();
  if (!s) return 'normal';
  if (s === 'alarm' || s === 'error' || s === 'warn' || s === 'warning' || s === 'fault') return 'alarm';
  return 'normal';
}

function detailToText(v) {
  if (v === null || v === undefined) return '';
  if (typeof v === 'string') return v.slice(0, 240);
  try {
    return JSON.stringify(v).slice(0, 240);
  } catch {
    return String(v).slice(0, 240);
  }
}

/** 摊平常见包装（{data:{...}} / {sensors:{...}}） */
function flattenPayload(payload) {
  const out = {};
  if (!isPlainObject(payload)) return out;
  for (const k of Object.keys(payload)) {
    if (!isDangerousKey(k)) out[k] = payload[k];
  }
  for (const wrap of ['data', 'sensors', 'values']) {
    const inner = out[wrap];
    if (isPlainObject(inner)) {
      for (const k of Object.keys(inner)) {
        if (isDangerousKey(k)) continue;
        if (out[k] === undefined) out[k] = inner[k];
      }
    }
  }
  return out;
}

function pickField(p, aliases) {
  if (!p) return undefined;
  for (const name of aliases) {
    if (Object.prototype.hasOwnProperty.call(p, name) && p[name] !== undefined) return p[name];
  }
  return undefined;
}

/** CRC32（IEEE 802.3，与固件端的实现一致） */
const CRC32_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    let c = i;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    table[i] = c >>> 0;
  }
  return table;
})();

function crc32(buf) {
  let c = 0xFFFFFFFF;
  for (let i = 0; i < buf.length; i++) c = CRC32_TABLE[(c ^ buf[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
}

function crcHex(n) {
  return '0x' + (Number(n) >>> 0).toString(16).padStart(8, '0');
}

/** multipart 文本字段：busboy 默认按 latin1 解，中文会变乱码，这里转回 UTF-8 */
function decodeField(v) {
  const s = String(v == null ? '' : v);
  if (!s) return s;
  /* 已经含非 latin1 字符 ⇒ 本来就是按 UTF-8 解出来的，别动 */
  for (let i = 0; i < s.length; i++) {
    if (s.charCodeAt(i) > 0xFF) return s;
  }
  try {
    const t = Buffer.from(s, 'latin1').toString('utf8');
    return t.includes('\uFFFD') ? s : t;
  } catch {
    return s;
  }
}

function safeFileName(name) {
  const base = String(name || '').split(/[\\/]/).pop() || '';
  const cleaned = base.replace(/[^A-Za-z0-9._-]/g, '_').slice(0, 120);
  return cleaned || 'firmware.bin';
}

function randomKey() {
  return crypto.randomBytes(24).toString('hex');
}

function parseTime(v) {
  const t = v ? new Date(v).getTime() : NaN;
  return Number.isFinite(t) ? t : 0;
}

/* ── OTA 纯函数（可单测，handler 与 MQTT 共用） ─────────────────────── */

/**
 * 这条 pendingOta 现在该不该派发给设备？
 *   · active=false（失败太多次）→ drop，别再让设备反复复位
 *   · 设备当前版本已经是目标版本、且固件更新时间晚于指令下发时间 → 已经刷过了，销账
 *   · 其余 → 如实派发
 */
function otaPickForDevice(device) {
  const pend = device && device.pendingOta;
  if (!pend) return { pending: null, drop: false, autoRetire: false, reason: 'none' };
  /* 失败太多次：不再派发，但指令本身要留着让网页看得见「为什么没在推」 */
  if (pend.active === false) return { pending: null, drop: true, autoRetire: false, reason: 'inactive' };

  const target = String(pend.version || '');
  const current = String(device.firmwareVersion || '');
  if (target && current && target === current) {
    const issuedAt = parseTime(pend.issuedAt);
    const updatedAt = parseTime(device.firmwareUpdatedAt);
    /* 固件更新时间晚于指令下发时间 ⇒ 设备已经刷上去了，这条指令可以销账 */
    if (updatedAt && (!issuedAt || updatedAt >= issuedAt)) {
      return { pending: null, drop: true, autoRetire: true, reason: 'already-applied' };
    }
  }
  return { pending: pend, drop: false, autoRetire: false, reason: 'pending' };
}

/**
 * 收到一次 OTA 结果后，这条 pendingOta 的新状态。
 * 不带 id（老固件）时维持原语义：无条件按结果处理。
 */
function otaNextPending(pend, otaId, success, maxFailures = OTA_MAX_FAILURES, message = '') {
  if (!pend) return { pending: null, result: 'none' };
  const id = String(otaId || '');
  const matches = !id || !pend.id || String(pend.id) === id;
  if (!matches) return { pending: pend, result: 'ignored' };
  if (success) return { pending: null, result: 'success' };

  const failures = (Number(pend.failures) || 0) + 1;
  const next = Object.assign({}, pend, {
    failures,
    lastError: String(message || '').slice(0, 200),
    lastFailureAt: new Date().toISOString(),
  });
  if (failures >= maxFailures) {
    next.active = false;
    return { pending: next, result: 'dropped' };
  }
  return { pending: next, result: 'failed' };
}

/* ══════════════════════════════════════════════════════════════════════
 * 工厂
 * ══════════════════════════════════════════════════════════════════════ */

function createIotCenter(options) {
  const cfg = options || {};
  const files = cfg.files || {};
  const readJSON = cfg.readJSON;
  const writeJSON = cfg.writeJSON;
  const initFile = typeof cfg.initFile === 'function' ? cfg.initFile : () => {};
  const now = typeof cfg.now === 'function' ? cfg.now : () => new Date().toISOString();
  const addLog = typeof cfg.addLog === 'function' ? cfg.addLog : () => {};
  const requireAdmin = typeof cfg.requireAdmin === 'function' ? cfg.requireAdmin : (req, res, next) => next();
  const uploadDir = cfg.uploadDir;
  const getSecret = typeof cfg.getSecret === 'function' ? cfg.getSecret : () => '';
  const getClientIP = typeof cfg.getClientIP === 'function' ? cfg.getClientIP : () => '';
  const getPublicOrigin = typeof cfg.getPublicOrigin === 'function' ? cfg.getPublicOrigin : () => '';

  const FW_DIR = path.join(uploadDir, 'iot-firmware');
  try {
    if (!fs.existsSync(FW_DIR)) fs.mkdirSync(FW_DIR, { recursive: true });
  } catch (e) {
    console.error('[iot] 固件目录创建失败: ' + (e && e.message));
  }

  /* ── 内存状态 ────────────────────────────────────────────────────── */

  initFile(files.devices, { devices: {}, events: [] });
  initFile(files.thresholds, {});
  initFile(files.firmware, { items: [] });
  initFile(files.telemetry, {});

  const deviceDoc = readJSON(files.devices, { devices: {}, events: [] });
  const devices = isPlainObject(deviceDoc) && isPlainObject(deviceDoc.devices) ? deviceDoc.devices : {};
  const events = (isPlainObject(deviceDoc) && Array.isArray(deviceDoc.events)) ? deviceDoc.events : [];

  const thresholds = (() => {
    const raw = readJSON(files.thresholds, {});
    return isPlainObject(raw) ? raw : {};
  })();

  const telemetry = (() => {
    const raw = readJSON(files.telemetry, {});
    const out = {};
    if (!isPlainObject(raw)) return out;
    for (const key of Object.keys(raw)) {
      if (isDangerousKey(key)) continue;
      if (Array.isArray(raw[key])) out[key] = raw[key].slice(0, TELEMETRY_KEEP);
    }
    return out;
  })();

  const firmwareDoc = readJSON(files.firmware, { items: [] });
  const firmwareItems = (isPlainObject(firmwareDoc) && Array.isArray(firmwareDoc.items)) ? firmwareDoc.items : [];

  let deviceDirty = false;
  let telemetryDirty = false;

  function saveDeviceState() {
    writeJSON(files.devices, { devices, events });
    deviceDirty = false;
  }

  function flushTelemetry() {
    writeJSON(files.telemetry, telemetry);
    telemetryDirty = false;
  }

  const telemetryTimer = setInterval(() => {
    if (telemetryDirty) {
      try { flushTelemetry(); } catch (e) { console.warn('[iot] 遥测刷盘失败: ' + (e && e.message)); }
    }
  }, TELEMETRY_FLUSH_MS);
  if (telemetryTimer.unref) telemetryTimer.unref();

  const deviceTimer = setInterval(() => {
    if (deviceDirty) {
      try { saveDeviceState(); } catch (e) { console.warn('[iot] 设备表刷盘失败: ' + (e && e.message)); }
    }
  }, DEVICE_FLUSH_MS);
  if (deviceTimer.unref) deviceTimer.unref();

  /* ── 阈值 ────────────────────────────────────────────────────────── */

  function coerceThreshold(key, raw) {
    const meta = THRESHOLD_META[key];
    if (!meta) return undefined;
    if (meta.type === 'boolean') {
      if (typeof raw === 'boolean') return raw;
      const s = String(raw == null ? '' : raw).trim().toLowerCase();
      if (s === 'true' || s === '1' || s === 'on' || s === 'yes') return true;
      if (s === 'false' || s === '0' || s === 'off' || s === 'no') return false;
      return undefined;
    }
    const n = numOrNull(raw);
    if (n === null) return undefined;
    let v = n;
    if (meta.step) v = Math.round(v / meta.step) * meta.step;
    if (meta.min !== undefined) v = Math.max(meta.min, v);
    if (meta.max !== undefined) v = Math.min(meta.max, v);
    return Number(v.toFixed(3));
  }

  function defaultThresholds() {
    const out = {};
    for (const key of THRESHOLD_KEYS) out[key] = THRESHOLD_META[key].default;
    return out;
  }

  function thresholdRecord(deviceId, create) {
    let rec = thresholds[deviceId];
    if (!isPlainObject(rec)) {
      if (!create) return null;
      rec = { values: {}, overrides: {}, updatedAt: '' };
      thresholds[deviceId] = rec;
    }
    if (!isPlainObject(rec.values)) rec.values = {};
    if (!isPlainObject(rec.overrides)) rec.overrides = {};
    return rec;
  }

  /** 生效值 = 默认 ← 设备自报(values) ← 后台下发(overrides) */
  function resolveThresholds(deviceId) {
    const out = defaultThresholds();
    const rec = thresholdRecord(deviceId, false);
    if (!rec) return out;
    for (const key of THRESHOLD_KEYS) {
      const v = coerceThreshold(key, rec.values[key]);
      if (v !== undefined) out[key] = v;
    }
    for (const key of THRESHOLD_KEYS) {
      const v = coerceThreshold(key, rec.overrides[key]);
      if (v !== undefined) out[key] = v;
    }
    return out;
  }

  function lockedThresholdKeys(deviceId) {
    const rec = thresholdRecord(deviceId, false);
    if (!rec) return [];
    return THRESHOLD_KEYS.filter((key) => coerceThreshold(key, rec.overrides[key]) !== undefined);
  }

  /** 后台配置的秒数 → 实际下发的毫秒值。
   *  【原先这里写死 Math.min(2000, ms)，所以把「上报间隔」改成 10 秒，
   *    发给设备的仍然是 2000ms —— 这就是「怎么改都是 2 秒」的后端根因。
   *    现在只夹在合法范围内（500ms..1 小时）。 */
  function reportIntervalMs(sec) {
    const ms = Math.round((Number(sec) || 0) * 1000);
    return Math.max(REPORT_INTERVAL_MIN_MS, Math.min(REPORT_INTERVAL_MAX_MS, ms));
  }

  function thresholdsEqual(a, b) {
    for (const key of THRESHOLD_KEYS) {
      if (a[key] !== b[key]) return false;
    }
    return true;
  }

  /* ── 事件 ────────────────────────────────────────────────────────── */

  function recordEvent(deviceId, input) {
    const event = {
      id: crypto.randomUUID ? crypto.randomUUID() : String(Date.now()) + Math.random().toString(16).slice(2),
      deviceId: String(deviceId || '').slice(0, 32),
      type: String((input && input.type) || 'device_event').slice(0, 32),
      level: String((input && input.level) || 'info').slice(0, 16),
      detail: detailToText(input && input.detail),
      ts: (input && input.ts) ? String(input.ts).slice(0, 40) : now(),
    };
    events.unshift(event);
    if (events.length > EVENT_KEEP) events.length = EVENT_KEEP;
    deviceDirty = true;
    return event;
  }

  /* ── 设备 ────────────────────────────────────────────────────────── */

  function isDeviceOnline(device) {
    const last = parseTime(device && device.lastSeenAt);
    return !!last && (Date.now() - last) <= ONLINE_TIMEOUT_MS;
  }

  function upsertDevice(input) {
    const deviceId = String((input && input.deviceId) || '').trim();
    if (!DEVICE_ID_RE.test(deviceId)) throw new Error('设备 ID 只能包含字母、数字、点、下划线、短横线（1-32 位）');

    const ts = now();
    let device = devices[deviceId];
    const created = !device;
    if (!device) {
      device = {
        registeredAt: ts,
        deviceId,
        deviceKey: '',
        name: deviceId,
        group: 'default',
        sensors: {},
        status: 'normal',
        lastSeenAt: '',
        lastTelemetryAt: '',
        lastConfigAt: '',
        lastConfigValues: {},
        firmwareVersion: '',
        gatewayVersion: '',
        pendingOta: null,
      };
      devices[deviceId] = device;
    }

    const key = String((input && input.deviceKey) || '').trim();
    if (key) device.deviceKey = key;
    else if (input && input.rotateKey) device.deviceKey = randomKey();
    else if (!device.deviceKey) device.deviceKey = randomKey();

    if (input && typeof input.name === 'string' && input.name.trim()) device.name = input.name.trim().slice(0, 40);
    if (input && typeof input.group === 'string') device.group = input.group.trim().slice(0, 32);
    device.updatedAt = ts;
    deviceDirty = true;
    addLog({ type: 'iot_device_upsert', deviceId, detail: created ? '登记新设备' : '更新设备信息' });
    return { device, created };
  }

  function findDeviceByKey(key) {
    if (!key) return null;
    for (const id of Object.keys(devices)) {
      const device = devices[id];
      if (device && timingSafeEqualStr(key, device.deviceKey)) return device;
    }
    return null;
  }

  function touchDevice(device) {
    device.lastSeenAt = now();
    deviceDirty = true;
  }

  function thresholdState(deviceId) {
    const device = devices[deviceId] || null;
    const effective = resolveThresholds(deviceId);
    const lastValues = (device && isPlainObject(device.lastConfigValues)) ? device.lastConfigValues : null;
    const applied = !!lastValues && thresholdsEqual(lastValues, effective);
    return { effective, applied, appliedAt: applied ? (device.lastConfigAt || '') : '' };
  }

  /* ── 遥测 ────────────────────────────────────────────────────────── */

  function pushTelemetrySample(deviceId, sample) {
    let list = telemetry[deviceId];
    if (!Array.isArray(list)) list = telemetry[deviceId] = [];
    list.unshift(sample);
    if (list.length > TELEMETRY_KEEP) list.length = TELEMETRY_KEEP;
    telemetryDirty = true;
  }

  function evaluateAlarm(cfgTh, sample) {
    if (!cfgTh.alarmEnabled) return { alarm: false, detail: '' };
    const parts = [];
    if (sample.temp !== null && (sample.temp < cfgTh.tempMin || sample.temp > cfgTh.tempMax)) {
      parts.push(`温度 ${sample.temp}℃ 越界[${cfgTh.tempMin},${cfgTh.tempMax}]`);
    }
    if (sample.hum !== null && (sample.hum < cfgTh.humMin || sample.hum > cfgTh.humMax)) {
      parts.push(`湿度 ${sample.hum}% 越界[${cfgTh.humMin},${cfgTh.humMax}]`);
    }
    if (sample.press !== null && (sample.press < cfgTh.pressMin || sample.press > cfgTh.pressMax)) {
      parts.push(`气压 ${sample.press}hPa 越界[${cfgTh.pressMin},${cfgTh.pressMax}]`);
    }
    return { alarm: parts.length > 0, detail: parts.join('；') };
  }

  function ingestTelemetry(deviceId, payload) {
    const device = devices[deviceId];
    if (!device) return null;

    const p = flattenPayload(payload);
    const sample = {
      ts: now(),
      temp: clampSensor('temp', numOrNull(pickField(p, FIELD_ALIAS.temp))),
      hum: clampSensor('hum', numOrNull(pickField(p, FIELD_ALIAS.hum))),
      press: clampSensor('press', normalizePress(pickField(p, FIELD_ALIAS.press))),
      accX: clampSensor('accX', numOrNull(pickField(p, FIELD_ALIAS.accX))),
      accY: clampSensor('accY', numOrNull(pickField(p, FIELD_ALIAS.accY))),
      accZ: clampSensor('accZ', numOrNull(pickField(p, FIELD_ALIAS.accZ))),
      status: normalizeStatus(pickField(p, FIELD_ALIAS.status)),
      batt: clampSensor('batt', numOrNull(pickField(p, FIELD_ALIAS.batt))),
    };
    pushTelemetrySample(deviceId, sample);

    const cfgTh = resolveThresholds(deviceId);
    const { alarm, detail } = evaluateAlarm(cfgTh, sample);
    const status = alarm ? 'alarm' : normalizeStatus(sample.status);
    const previous = String(device.status || '');
    sample.status = status;

    device.sensors = {
      temp: sample.temp, hum: sample.hum, press: sample.press,
      accX: sample.accX, accY: sample.accY, accZ: sample.accZ,
      batt: sample.batt, status,
    };
    device.status = status;
    device.lastTelemetryAt = sample.ts;
    device.lastSeenAt = sample.ts;

    const gw = pickField(p, FIELD_ALIAS.gatewayVersion);
    if (gw !== undefined) {
      const v = String(gw).trim().slice(0, 32);
      if (v) device.gatewayVersion = v;
    }
    const fw = pickField(p, FIELD_ALIAS.firmwareVersion);
    if (fw !== undefined && gw === undefined) {
      const v = String(fw).trim().slice(0, 32);
      if (v) device.gatewayVersion = v;
    }

    const reported = pickField(p, FIELD_ALIAS.thresholds);
    if (isPlainObject(reported)) applyReportedThresholds(deviceId, reported);

    /* 只在状态翻转时记事件，避免 2 秒一条把事件表刷满 */
    if (status === 'alarm' && previous !== 'alarm') {
      recordEvent(deviceId, { type: 'alarm', level: 'error', detail: detail || '读数越界' });
    } else if (status === 'normal' && previous === 'alarm') {
      recordEvent(deviceId, { type: 'alarm_clear', level: 'info', detail: '读数恢复正常' });
    }

    deviceDirty = true;
    return { sample, status, alarm, detail, config: cfgTh };
  }

  /** 设备自报阈值只写进 values，后台 overrides 仍然优先 */
  function applyReportedThresholds(deviceId, reported) {
    const rec = thresholdRecord(deviceId, true);
    let touched = false;
    for (const key of THRESHOLD_KEYS) {
      if (coerceThreshold(key, rec.overrides[key]) !== undefined) continue;
      const v = coerceThreshold(key, reported[key]);
      if (v === undefined) continue;
      if (rec.values[key] !== v) {
        rec.values[key] = v;
        touched = true;
      }
    }
    if (touched) rec.updatedAt = now();
    return touched;
  }

  function ingestEvent(deviceId, payload) {
    const p = flattenPayload(payload);
    const event = recordEvent(deviceId, {
      type: String(pickField(p, EVENT_FIELD_ALIAS.type) || 'device_event').slice(0, 32),
      level: String(pickField(p, EVENT_FIELD_ALIAS.level) || 'info').slice(0, 16),
      detail: pickField(p, EVENT_FIELD_ALIAS.detail),
      ts: pickField(p, EVENT_FIELD_ALIAS.ts),
    });
    const device = devices[deviceId];
    if (device) touchDevice(device);
    return event;
  }

  /* ── 设备鉴权 ────────────────────────────────────────────────────── */

  function deviceKeyFrom(req) {
    const h = req.headers || {};
    const fromHeader = h['x-iot-key'];
    if (fromHeader) return String(fromHeader).trim();
    const body = isPlainObject(req.body) ? req.body : {};
    const fromBody = body.deviceKey || body.device_key || body.token;
    if (fromBody) return String(fromBody).trim();
    const q = req.query || {};
    if (q.key) return String(q.key).trim();
    return '';
  }

  function deviceIdFrom(req) {
    const body = isPlainObject(req.body) ? req.body : {};
    if (body.deviceId) return String(body.deviceId).trim().slice(0, 32);
    if (body.device_id) return String(body.device_id).trim().slice(0, 32);
    const q = req.query || {};
    if (q.deviceId) return String(q.deviceId).trim().slice(0, 32);
    if (q.device_id) return String(q.device_id).trim().slice(0, 32);
    const h = req.headers || {};
    if (h['x-iot-device-id']) return String(h['x-iot-device-id']).trim().slice(0, 32);
    if (h['x-device-id']) return String(h['x-device-id']).trim().slice(0, 32);
    return '';
  }

  /* 设备侧限流：按设备 240/min，按 IP 1200/min（全校设备共用一个出口 NAT） */
  const rateBuckets = new Map();
  function rateAllowed(bucketKey, limitPerMin) {
    const t = Date.now();
    let b = rateBuckets.get(bucketKey);
    if (!b || t - b.start >= 60000) {
      b = { start: t, count: 0 };
      rateBuckets.set(bucketKey, b);
    }
    b.count += 1;
    return b.count <= limitPerMin;
  }

  function requireDevice(req, res, next) {
    try {
      const key = deviceKeyFrom(req);
      if (!key) {
        return res.status(401).json({ success: false, code: 'IOT_DEVICE_UNAUTHORIZED', message: '设备未授权：缺少 X-Iot-Key' });
      }

      const wantId = deviceIdFrom(req);
      let device = wantId ? devices[wantId] : null;

      if (device) {
        if (!timingSafeEqualStr(key, device.deviceKey)) {
          return res.status(401).json({ success: false, code: 'IOT_DEVICE_UNAUTHORIZED', message: '设备未授权：设备密钥不匹配' });
        }
      } else {
        const byKey = findDeviceByKey(key);
        if (wantId && byKey) {
          return res.status(401).json({ success: false, code: 'IOT_DEVICE_UNAUTHORIZED', message: '设备未授权：设备 ID 与密钥不匹配' });
        }
        if (byKey) {
          device = byKey;
        } else if (wantId && DEVICE_ID_RE.test(wantId)) {
          /* 装机自注册：显式 IOT_ENROLL_KEY 优先；否则默认允许自动登记（IOT_AUTO_ENROLL=0 关闭） */
          const enrollKey = String(process.env.IOT_ENROLL_KEY || '').trim();
          const autoEnroll = String(process.env.IOT_AUTO_ENROLL || '1').trim() !== '0';
          if (enrollKey && timingSafeEqualStr(key, enrollKey)) {
            device = upsertDevice({ deviceId: wantId, deviceKey: key, name: wantId, group: 'default' }).device;
          } else if (autoEnroll) {
            device = upsertDevice({ deviceId: wantId, deviceKey: key, name: wantId, group: 'default' }).device;
            addLog({ type: 'iot_device_enrolled', deviceId: wantId, detail: '自动登记（未配置登记密钥）' });
          }
        }
        if (!device) {
          return res.status(401).json({ success: false, code: 'IOT_DEVICE_UNAUTHORIZED', message: '设备未授权：设备未登记或密钥错误' });
        }
      }

      const perDevice = Number(process.env.IOT_RATE_PER_MIN) || 240;
      const perIp = Number(process.env.IOT_IP_RATE_PER_MIN) || 1200;
      if (!rateAllowed('dev:' + device.deviceId, perDevice) || !rateAllowed('ip:' + getClientIP(req), perIp)) {
        return res.status(429).json({ success: false, message: '上报过于频繁，请稍后再试' });
      }

      req.iotDeviceId = device.deviceId;
      req.iotDevice = device;
      touchDevice(device);
      return next();
    } catch (e) {
      return next(e);
    }
  }

  /** 管理端：默认免登录（测试阶段）；IOT_REQUIRE_ADMIN=1 切回主后台会话鉴权 */
  function adminOpen(req, res, next) {
    if (String(process.env.IOT_REQUIRE_ADMIN || '0').trim() === '1') return requireAdmin(req, res, next);
    return next();
  }

  /* ── 固件签名与下载 ──────────────────────────────────────────────── */

  function linkSecret() {
    return String(getSecret() || process.env.IOT_LINK_SECRET || 'iot-link-fallback');
  }

  function signFirmware(filename, exp) {
    return crypto.createHmac('sha256', linkSecret())
      .update(String(filename) + '\n' + String(exp))
      .digest('hex');
  }

  function deviceOrigin(req) {
    const explicit = String(process.env.IOT_DEVICE_ORIGIN || '').trim();
    if (explicit) return explicit.replace(/\/+$/, '');

    const host = String((req && req.get && req.get('host')) || '').trim();
    const bare = host.replace(/^https?:\/\//i, '');
    /* 设备端只有明文 HTTP（8266 没有 TLS 栈），这里一律给 http://。
     * 本机/环回地址对设备毫无意义（设备在校园网另一头），要退回公网地址。 */
    const loopback = /^(127\.|localhost|\[::1\]|0\.0\.0\.0|::1)/i.test(bare);
    if (!bare || loopback) {
      return 'http://' + String(process.env.IOT_DEVICE_HOST || 'YOUR_SERVER_HOST').trim();
    }
    return 'http://' + bare;
  }

  function absoluteDeviceOrigin() {
    const explicit = String(process.env.IOT_DEVICE_ORIGIN || '').trim();
    if (explicit) return explicit.replace(/\/+$/, '');
    return 'http://' + String(process.env.IOT_DEVICE_HOST || 'YOUR_SERVER_HOST').trim();
  }

  function firmwareUrl(origin, item, ttlMs) {
    const ttl = Math.max(60000, Math.min(MAX_LINK_TTL_MS, Number(ttlMs) || DEFAULT_LINK_TTL_MS));
    const exp = Date.now() + ttl;
    const sig = signFirmware(item.filename, exp);
    return {
      url: origin + '/admin/api/iot/firmware/download?f=' + encodeURIComponent(item.filename) +
        '&exp=' + exp + '&t=' + sig,
      expiresAt: new Date(exp).toISOString(),
      expiresInMs: ttl,
    };
  }

  function findFirmware(id) {
    const key = String(id || '').trim();
    return firmwareItems.find((item) => String(item.id) === key) || null;
  }

  function otaPayloadFor(origin, pend) {
    if (!pend) return null;
    const item = findFirmware(pend.firmwareId);
    if (!item) return null;
    const link = firmwareUrl(origin, item, DEFAULT_LINK_TTL_MS);
    return {
      id: pend.id,
      firmwareId: item.id,
      filename: item.filename,
      version: pend.version || item.version || '',
      size: Number(item.size) || 0,
      crc32: crcHex(item.crc32),
      sha256: item.sha256 || '',
      url: link.url,
      expiresAt: link.expiresAt,
      note: pend.note || '',
    };
  }

  /* ── 设备侧路由 ──────────────────────────────────────────────────── */

  function registerDeviceRoutes(app) {
    app.post('/api/iot/telemetry', requireDevice, (req, res) => {
      try {
        const result = ingestTelemetry(req.iotDeviceId, req.body || {});
        if (!result) return res.status(400).json({ success: false, message: '遥测写入失败' });
        return res.json({
          success: true,
          ts: result.sample.ts,
          alarm: result.alarm,
          event: null,
          config: result.config,
        });
      } catch (e) {
        console.error('[iot] telemetry 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '遥测写入失败' });
      }
    });

    app.post('/api/iot/event', requireDevice, (req, res) => {
      try {
        const event = ingestEvent(req.iotDeviceId, req.body || {});
        return res.json({ success: true, event });
      } catch (e) {
        console.error('[iot] event 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '事件写入失败' });
      }
    });

    /* 设备每 10 秒拉一次：阈值 + 待升级指令一次带回。
     * 8266 与 S3 都从 doc["config"] / doc["ota"] 取值，字段必须保持原样。 */
    app.get('/api/iot/config', requireDevice, (req, res) => {
      try {
        const device = req.iotDevice;
        const deviceId = req.iotDeviceId;
        const fw = String((req.query && req.query.fw) || '').trim().slice(0, 32);
        if (fw) device.gatewayVersion = fw;

        const pick = otaPickForDevice(device);
        /* 只有「已经刷成目标版本」才销账；失败停发的指令要留着给网页看 */
        if (pick.autoRetire && device.pendingOta) {
          device.pendingOta = null;
          deviceDirty = true;
        }

        const info = thresholdState(deviceId);
        device.lastConfigAt = now();
        device.lastConfigValues = Object.assign({}, info.effective);

        const ota = pick.pending ? otaPayloadFor(deviceOrigin(req), pick.pending) : null;
        deviceDirty = true;

        const body = {
          success: true,
          deviceId,
          config: info.effective,
          thresholds: info.effective,
          reportIntervalMs: reportIntervalMs(info.effective.reportIntervalSec),
          locked: lockedThresholdKeys(deviceId),
          thresholdsUpdatedAt: (thresholdRecord(deviceId, false) || {}).updatedAt || '',
          serverTime: now(),
          ts: now(),
        };
        if (ota) body.ota = ota;
        return res.json(body);
      } catch (e) {
        console.error('[iot] config 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取配置失败' });
      }
    });

    app.post('/api/iot/ota/result', requireDevice, (req, res) => {
      try {
        const device = req.iotDevice;
        const body = req.body || {};
        const success = body.success === true || body.success === 'true' || body.success === 1 || body.success === '1';
        const version = String(body.version || '').trim().slice(0, 40);
        const message = String(body.message || '').slice(0, 200);

        const next = otaNextPending(device.pendingOta, body.id, success, OTA_MAX_FAILURES, message);
        if (next.result === 'ignored') {
          return res.json({ success: true });
        }
        device.pendingOta = next.pending;
        if (success) {
          if (version) device.firmwareVersion = version;
          device.firmwareUpdatedAt = now();
          recordEvent(req.iotDeviceId, {
            type: 'ota', level: 'info',
            detail: '升级成功 ' + (version || ''),
          });
        } else if (next.result === 'dropped') {
          recordEvent(req.iotDeviceId, {
            type: 'ota', level: 'error',
            detail: '连续失败，已停止下发：' + message,
          });
        } else {
          recordEvent(req.iotDeviceId, {
            type: 'ota', level: 'error',
            detail: '升级失败：' + message,
          });
        }
        deviceDirty = true;
        return res.json({ success: true });
      } catch (e) {
        console.error('[iot] ota/result 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '升级结果处理失败' });
      }
    });
  }

  /* ── 管理侧路由 ──────────────────────────────────────────────────── */

  function serializeDevice(device) {
    const id = device.deviceId;
    const online = isDeviceOnline(device);
    const history = telemetry[id];
    const latest = Array.isArray(history) && history.length ? history[0] : null;
    const info = thresholdState(id);
    return {
      deviceId: id,
      name: device.name || '',
      group: device.group || '',
      deviceKey: device.deviceKey || '',
      status: online ? (device.status || 'normal') : 'offline',
      online,
      sensors: device.sensors || {},
      latest,
      registeredAt: device.registeredAt || '',
      updatedAt: device.updatedAt || '',
      lastSeenAt: device.lastSeenAt || '',
      lastTelemetryAt: device.lastTelemetryAt || '',
      lastConfigAt: device.lastConfigAt || '',
      lastConfigValues: device.lastConfigValues || {},
      firmwareVersion: device.firmwareVersion || '',
      firmwareUpdatedAt: device.firmwareUpdatedAt || '',
      gatewayVersion: device.gatewayVersion || '',
      pendingOta: device.pendingOta || null,
      thresholdsApplied: info.applied,
      thresholdsAppliedAt: info.appliedAt,
    };
  }

  function listDevices() {
    return Object.keys(devices)
      .map((id) => devices[id])
      .filter((d) => d && typeof d === 'object')
      .map(serializeDevice)
      .sort((a, b) => (a.deviceId < b.deviceId ? -1 : a.deviceId > b.deviceId ? 1 : 0));
  }

  /* 上传：旧实现只认 `firmware` 一个字段名，换个名字就 MulterError。
   * 这里用 any() + 取第一个文件，前端原样工作，脚本也不会再 500。 */
  const upload = multer({
    storage: multer.diskStorage({
      destination: (req, file, cb) => cb(null, FW_DIR),
      filename: (req, file, cb) => {
        const safe = safeFileName(file.originalname);
        const stem = safe.replace(/\.(bin|ota)$/i, '');
        const ext = (safe.match(/\.(bin|ota)$/i) || ['.bin'])[0].toLowerCase();
        const base = 'iot-' + stem.slice(0, 80);
        let name = base + ext;
        let n = 1;
        while (fs.existsSync(path.join(FW_DIR, name))) {
          name = base + '-' + n + ext;
          n += 1;
        }
        cb(null, name);
      },
    }),
    limits: { fileSize: FIRMWARE_MAX_BYTES, files: 1 },
    fileFilter: (req, file, cb) => {
      if (!FIRMWARE_NAME_RE.test(safeFileName(file.originalname))) {
        return cb(new Error('只接受 .bin / .ota 固件文件'));
      }
      return cb(null, true);
    },
  });

  function uploadMiddleware(req, res, next) {
    upload.any()(req, res, (err) => {
      if (err) {
        const msg = err.code === 'LIMIT_FILE_SIZE' ? '固件超过 64MB 上限' : String(err.message || '上传失败');
        return res.status(400).json({ success: false, message: msg });
      }
      return next();
    });
  }

  function needAdminLog(req, detail) {
    addLog({ type: 'iot_admin_action', ip: getClientIP(req), detail });
  }

  function registerAdminRoutes(app) {
    app.get('/admin/api/iot/devices', adminOpen, (req, res) => {
      try {
        const list = listDevices();
        return res.json({ success: true, total: list.length, devices: list, onlineTimeoutMs: ONLINE_TIMEOUT_MS });
      } catch (e) {
        console.error('[iot] devices 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取设备列表失败' });
      }
    });

    app.post('/admin/api/iot/devices', adminOpen, (req, res) => {
      try {
        const body = req.body || {};
        const result = upsertDevice({
          deviceId: body.deviceId,
          name: body.name,
          group: body.group,
          deviceKey: body.deviceKey,
          rotateKey: body.rotateKey === true || body.rotateKey === 'true',
        });
        saveDeviceState();
        needAdminLog(req, (result.created ? '登记设备 ' : '更新设备 ') + result.device.deviceId);
        return res.json({ success: true, device: serializeDevice(result.device) });
      } catch (e) {
        return res.status(400).json({ success: false, message: String(e.message || '设备登记失败') });
      }
    });

    app.get('/admin/api/iot/thresholds/:deviceId', adminOpen, (req, res) => {
      try {
        const deviceId = String(req.params.deviceId || '');
        if (!DEVICE_ID_RE.test(deviceId)) {
          return res.status(400).json({ success: false, message: '设备 ID 不合法' });
        }
        const rec = thresholdRecord(deviceId, false);
        const info = thresholdState(deviceId);
        const device = devices[deviceId];
        return res.json({
          success: true,
          deviceId,
          thresholds: info.effective,
          reportIntervalMs: reportIntervalMs(info.effective.reportIntervalSec),
          values: (rec && rec.values) || {},
          overrides: (rec && rec.overrides) || {},
          locked: lockedThresholdKeys(deviceId),
          updatedAt: (rec && rec.updatedAt) || '',
          applied: info.applied,
          appliedAt: info.appliedAt || (device && device.lastConfigAt) || '',
          deviceOnline: !!device && isDeviceOnline(device),
          meta: THRESHOLD_KEYS.map((key) => Object.assign({ key }, THRESHOLD_META[key])),
        });
      } catch (e) {
        console.error('[iot] thresholds 读取失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取阈值失败' });
      }
    });

    app.post('/admin/api/iot/thresholds/:deviceId', adminOpen, (req, res) => {
      try {
        const deviceId = String(req.params.deviceId || '');
        if (!DEVICE_ID_RE.test(deviceId)) {
          return res.status(400).json({ success: false, message: '设备 ID 不合法' });
        }
        const body = req.body || {};
        const payload = isPlainObject(body.thresholds) ? body.thresholds : body;

        const rec = thresholdRecord(deviceId, true);
        const before = resolveThresholds(deviceId);
        let changed = 0;

        for (const key of Object.keys(payload)) {
          if (!THRESHOLD_KEYS.includes(key)) continue;
          const raw = payload[key];
          if (raw === null || raw === undefined || raw === '') {
            /* 清空 = 解除后台锁定，回到设备自报值 */
            if (rec.overrides[key] !== undefined) {
              delete rec.overrides[key];
              changed += 1;
            }
            continue;
          }
          const value = coerceThreshold(key, raw);
          if (value === undefined) continue;
          if (rec.overrides[key] !== value) {
            rec.overrides[key] = value;
            changed += 1;
          }
        }
        rec.updatedAt = now();

        const effective = resolveThresholds(deviceId);
        if (!changed) {
          /* 值没变也要刷新 updatedAt，让设备端下一次拉取拿到同一份配置 */
          changed = THRESHOLD_KEYS.filter((k) => before[k] !== effective[k]).length;
        }

        writeJSON(files.thresholds, thresholds);
        saveDeviceState();
        needAdminLog(req, '下发阈值 ' + deviceId + '（改动 ' + changed + ' 项）');

        /* MQTT 秒级下发（HTTP 轮询仍然是主通道，10 秒内也会取到） */
        mqtt.publish(deviceId, 'config', {
          deviceId,
          config: effective,
          reportIntervalMs: reportIntervalMs(effective.reportIntervalSec),
          serverTime: now(),
        });

        const info = thresholdState(deviceId);
        const device = devices[deviceId];
        return res.json({
          success: true,
          changed,
          deviceId,
          thresholds: effective,
          reportIntervalMs: reportIntervalMs(effective.reportIntervalSec),
          overrides: rec.overrides,
          values: rec.values,
          locked: lockedThresholdKeys(deviceId),
          updatedAt: rec.updatedAt,
          applied: info.applied,
          appliedAt: info.appliedAt || (device && device.lastConfigAt) || '',
          deviceOnline: !!device && isDeviceOnline(device),
          effectiveAfterSec: 10,
        });
      } catch (e) {
        console.error('[iot] thresholds 写入失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '保存阈值失败' });
      }
    });

    app.get('/admin/api/iot/firmware', adminOpen, (req, res) => {
      try {
        const items = firmwareItems.slice().sort((a, b) => (parseTime(b.uploadedAt) - parseTime(a.uploadedAt)));
        return res.json({
          success: true, total: items.length, items,
          defaultTtlMs: DEFAULT_LINK_TTL_MS, maxTtlMs: MAX_LINK_TTL_MS,
        });
      } catch (e) {
        console.error('[iot] firmware 列表失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取固件清单失败' });
      }
    });

    app.post('/admin/api/iot/firmware', adminOpen, uploadMiddleware, (req, res) => {
      try {
        const list = Array.isArray(req.files) ? req.files : [];
        const file = list[0];
        if (!file) return res.status(400).json({ success: false, message: '没有收到固件文件（字段名需为 firmware 或任意文件字段）' });

        const buf = fs.readFileSync(file.path);
        const version = decodeField((req.body && req.body.version) || '').trim().slice(0, 32) ||
          file.filename.replace(/^iot-/, '').replace(/\.(bin|ota)$/i, '');
        const groups = decodeField((req.body && req.body.groups) || '')
          .split(',').map((s) => s.trim()).filter(Boolean).slice(0, 20);
        const note = decodeField((req.body && req.body.note) || '').slice(0, 120);

        const item = {
          id: crypto.randomUUID ? crypto.randomUUID() : String(Date.now()),
          filename: file.filename,
          version,
          sha256: crypto.createHash('sha256').update(buf).digest('hex'),
          crc32: crc32(buf),
          size: buf.length,
          groups,
          note,
          uploadedAt: now(),
        };
        firmwareItems.unshift(item);
        writeJSON(files.firmware, { items: firmwareItems });
        needAdminLog(req, '上传固件 ' + item.filename + ' v' + item.version);
        return res.json({ success: true, firmware: item });
      } catch (e) {
        console.error('[iot] firmware 上传失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '固件入库失败' });
      }
    });

    app.get('/admin/api/iot/firmware/link', adminOpen, (req, res) => {
      try {
        const item = findFirmware(req.query && req.query.id);
        if (!item) return res.status(404).json({ success: false, message: '固件不存在' });
        const link = firmwareUrl(getPublicOrigin(req) || absoluteDeviceOrigin(), item, req.query && req.query.ttl);
        return res.json({
          success: true,
          message: '下载链接已生成',
          url: link.url,
          expiresAt: link.expiresAt,
          expiresInMs: link.expiresInMs,
          file: { filename: item.filename, version: item.version, size: item.size, crc32: item.crc32, sha256: item.sha256 },
        });
      } catch (e) {
        console.error('[iot] firmware/link 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '生成下载链接失败' });
      }
    });

    /* 签名下载：不挂 adminOpen（设备无权登录），靠 HMAC 签名。
     * 必须支持 Range —— 8266 断点续传靠它（206 + Content-Range）。 */
    app.get('/admin/api/iot/firmware/download', (req, res) => {
      try {
        const filename = String((req.query && req.query.f) || '');
        const exp = String((req.query && req.query.exp) || '');
        const sig = String((req.query && req.query.t) || '');
        if (!filename || !exp || !sig) {
          return res.status(403).json({ success: false, message: '链接参数不完整' });
        }
        if (!Number(exp) || Number(exp) < Date.now()) {
          return res.status(403).json({ success: false, message: '链接已过期' });
        }
        const expect = signFirmware(filename, exp);
        if (!timingSafeEqualStr(sig, expect)) {
          return res.status(403).json({ success: false, message: '链接签名无效' });
        }

        const item = firmwareItems.find((f) => String(f.filename) === filename);
        if (!item) return res.status(404).json({ success: false, message: '固件不存在' });
        const full = path.join(FW_DIR, safeFileName(item.filename));
        if (!fs.existsSync(full)) return res.status(404).json({ success: false, message: '固件文件已丢失' });

        const stat = fs.statSync(full);
        const size = stat.size;
        res.setHeader('Content-Type', 'application/octet-stream');
        res.setHeader('Content-Disposition', 'attachment; filename="' + safeFileName(item.filename) + '"');
        res.setHeader('Accept-Ranges', 'bytes');
        res.setHeader('Cache-Control', 'no-store');

        const range = String((req.headers && req.headers.range) || '');
        const m = range.match(/bytes=(\d*)-(\d*)/);
        if (m && (m[1] || m[2])) {
          let start = m[1] ? Number(m[1]) : 0;
          let end = m[2] ? Number(m[2]) : size - 1;
          start = Math.max(0, Math.min(size - 1, start));
          end = Math.max(start, Math.min(size - 1, end));
          res.status(206);
          res.setHeader('Content-Range', 'bytes ' + start + '-' + end + '/' + size);
          res.setHeader('Content-Length', String(end - start + 1));
          return fs.createReadStream(full, { start, end }).pipe(res);
        }

        res.setHeader('Content-Length', String(size));
        return fs.createReadStream(full).pipe(res);
      } catch (e) {
        console.error('[iot] firmware/download 失败:', e && e.stack);
        if (!res.headersSent) return res.status(500).json({ success: false, message: '下载失败' });
        return undefined;
      }
    });

    app.post('/admin/api/iot/ota/push', adminOpen, (req, res) => {
      try {
        const body = req.body || {};
        const item = findFirmware(body.firmwareId);
        if (!item) return res.status(404).json({ success: false, message: '固件不存在' });

        const wanted = [];
        if (Array.isArray(body.deviceIds)) wanted.push(...body.deviceIds.map(String));
        if (Array.isArray(body.groups)) {
          const groups = body.groups.map(String);
          for (const id of Object.keys(devices)) {
            if (groups.includes(String(devices[id].group || ''))) wanted.push(id);
          }
        }
        const targets = [...new Set(wanted.map((s) => s.trim()).filter((id) => DEVICE_ID_RE.test(id) && devices[id]))];
        if (!targets.length) {
          return res.status(400).json({ success: false, message: '没有匹配的目标设备（填设备 ID 或分组）' });
        }

        const otaId = crypto.randomUUID ? crypto.randomUUID() : String(Date.now());
        const issuedAt = now();
        const ota = {
          id: otaId,
          firmwareId: item.id,
          filename: item.filename,
          version: item.version,
          sha256: item.sha256,
          crc32: crcHex(item.crc32),
          size: item.size,
          issuedAt,
          note: String(body.note || '').slice(0, 120),
          failures: 0,
          active: true,
        };

        for (const id of targets) {
          devices[id].pendingOta = Object.assign({}, ota);
          mqtt.publish(id, 'ota', otaPayloadFor(absoluteDeviceOrigin(), ota) || ota);
        }
        saveDeviceState();
        needAdminLog(req, '下发 OTA ' + item.filename + ' → ' + targets.join(','));
        return res.json({ success: true, otaId, targets, pushed: targets.length, ota });
      } catch (e) {
        console.error('[iot] ota/push 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '下发升级失败' });
      }
    });

    app.get('/admin/api/iot/events', adminOpen, (req, res) => {
      try {
        const limit = Math.max(1, Math.min(EVENT_KEEP, Number(req.query && req.query.limit) || 100));
        return res.json({ success: true, total: events.length, events: events.slice(0, limit) });
      } catch (e) {
        return res.status(500).json({ success: false, message: '读取事件失败' });
      }
    });

    /* 传感器历史：给网页「传感器历史」视图用。
     * 查询参数：deviceId（必填）、limit（默认 300，最大 TELEMETRY_KEEP）、
     *          from / to（可选，ISO 时间或毫秒时间戳）。
     * 返回 samples 按时间**从新到旧**，网页画图时自己反转即可。 */
    app.get('/admin/api/iot/telemetry', adminOpen, (req, res) => {
      try {
        const q = req.query || {};
        const deviceId = String(q.deviceId || '').trim();
        if (!DEVICE_ID_RE.test(deviceId)) {
          return res.status(400).json({ success: false, message: '设备 ID 不合法' });
        }
        if (!devices[deviceId]) {
          return res.status(404).json({ success: false, message: '设备未登记：' + deviceId });
        }

        const list = Array.isArray(telemetry[deviceId]) ? telemetry[deviceId] : [];
        const from = parseTime(q.from);
        const to = parseTime(q.to);
        let filtered = list;
        if (from || to) {
          filtered = list.filter((sample) => {
            const t = parseTime(sample && sample.ts);
            if (!t) return false;
            if (from && t < from) return false;
            if (to && t > to) return false;
            return true;
          });
        }

        const limit = Math.max(1, Math.min(TELEMETRY_KEEP, Number(q.limit) || 300));
        const sec = resolveThresholds(deviceId).reportIntervalSec;
        return res.json({
          success: true,
          deviceId,
          total: filtered.length,
          samples: filtered.slice(0, limit),
          intervalMs: reportIntervalMs(sec),
          intervalSec: sec,
          keep: TELEMETRY_KEEP,
        });
      } catch (e) {
        console.error('[iot] telemetry 历史查询失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取遥测历史失败' });
      }
    });

    app.get('/admin/api/iot/stats', adminOpen, (req, res) => {
      try {
        const list = listDevices();
        const today = new Date().toDateString();
        const todayEvents = events.filter((e) => {
          const t = parseTime(e && e.ts);
          return t && new Date(t).toDateString() === today;
        }).length;

        const distribution = new Map();
        for (const d of list) {
          const v = String(d.firmwareVersion || '').trim();
          if (!v) continue;
          distribution.set(v, (distribution.get(v) || 0) + 1);
        }
        const firmwareDistribution = [...distribution.entries()]
          .map(([version, count]) => ({ version, count }))
          .sort((a, b) => b.count - a.count);

        const stats = {
          total: list.length,
          online: list.filter((d) => d.online).length,
          offline: list.filter((d) => !d.online).length,
          normal: list.filter((d) => d.online && d.status === 'normal').length,
          alarm: list.filter((d) => d.status === 'alarm').length,
          todayEvents,
          totalEvents: events.length,
          firmwareCount: firmwareItems.length,
          firmwareDistribution,
          mqtt: mqtt.status(),
        };
        return res.json({ success: true, stats });
      } catch (e) {
        console.error('[iot] stats 失败:', e && e.stack);
        return res.status(500).json({ success: false, message: '读取统计失败' });
      }
    });
  }

  /* ── MQTT ────────────────────────────────────────────────────────── */

  const mqtt = createMqttBridge({
    url: process.env.MQTT_URL || process.env.IOT_MQTT_URL || '',
    prefix: process.env.MQTT_PREFIX || 'campus/iot',
    clientId: process.env.MQTT_CLIENT_ID || 'campus-iot',
    handlers: {
      onTelemetry: (deviceId, payload) => {
        if (!deviceId || !devices[deviceId]) return;
        ingestTelemetry(deviceId, payload);
      },
      onEvent: (deviceId, payload) => {
        if (!deviceId) return;
        ingestEvent(deviceId, payload);
      },
      onStatus: (deviceId, payload) => {
        const device = devices[deviceId];
        if (!device) return;
        touchDevice(device);
        const status = payload && (payload.status || payload.state);
        if (status) device.status = normalizeStatus(status);
        deviceDirty = true;
      },
    },
    log: (msg) => console.log(msg),
  });

  function startMqtt() {
    try {
      return mqtt.start();
    } catch (e) {
      console.warn('[iot] MQTT 启动失败: ' + (e && e.message));
      return false;
    }
  }

  function stopMqtt() {
    try { mqtt.stop(); } catch { /* 停止失败不影响退出 */ }
  }

  return {
    registerDeviceRoutes,
    registerAdminRoutes,
    startMqtt,
    stopMqtt,
    flushTelemetry,
    saveDeviceState,
    mqttStatus: () => mqtt.status(),
    listDevices,
  };
}

module.exports = {
  createIotCenter,
  THRESHOLD_META,
  THRESHOLD_KEYS,
  TELEMETRY_KEEP,
  EVENT_KEEP,
  otaPickForDevice,
  otaNextPending,
};
