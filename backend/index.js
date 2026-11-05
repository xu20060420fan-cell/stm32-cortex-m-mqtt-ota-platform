/*
 * 校园物联网中心 · 独立服务
 * ===========================================================================
 *
 * 为什么要从 8080 主后端里拆出来（2026-10-25）：
 *   用户原话：「我这个后端里面好像还有个 STM32 的后端吧？把 STM32 的后端单独抽出来，
 *   STM32 的后端是单独出一个管理界面，不要把 STM32 的后端搞跟别的后端搞一块儿去了。」
 *
 * 拆之前的真实状态（不是猜，是读代码读出来的）：
 *   · 管理界面**早就是独立的** —— `iot.html`（568 行，黑白极简）挂在 `/iot`，
 *     与 5000+ 行的 `admin.html` **零耦合**（admin.html 里 "iot" 出现 0 次）；
 *   · 后端逻辑也**早就是独立文件** —— `iotCenter.js`（1363 行）；
 *   · **但进程没拆** —— 它寄生在 8080 主进程里，靠主后端注入
 *     `requireAdmin` / `jwtSecret` / `getPublicOrigin` / `getClientIP`，
 *     设备侧路由还得**注册在全局限流器之前**（因为全校设备共用同一个出口 NAT IP，
 *     按 IP 限流会把全校设备一起限死），MQTT 也挂在主进程的启动流程里。
 *   ⇒ 这就是「搞一块儿」的实际含义：**一个进程、一套生命周期、一份故障域**。
 *
 * 拆完之后：
 *   · 本服务 = 独立进程（默认 8093）、独立 pm2 条目、独立数据目录引用；
 *   · 管理界面仍是 `https://YOUR_DOMAIN/iot`（nginx 内部转发，地址不变、仍是 HTTPS）；
 *   · 登录**沿用主后台那一个账号、那一套登录态**（读同一份 admin-sessions.json），
 *     不新增第二套口令（见 adminAuth.js）；
 *   · 设备侧限流天然独立 —— 不再需要「注册在全局限流器之前」这种脆弱约定。
 *
 * ⚠ 数据目录与主后端**共用同一份**（DATA_DIR / UPLOAD_DIR 由环境变量指过去）：
 *   iot-*.json 是设备登记册、阈值、固件清单、遥测；
 *   uploads/iot-firmware/ 是固件文件；
 *   admin-sessions.json 是后台会话表。
 *   两个进程**不同时写同一个文件**：本服务只写 iot-*.json 与自己的日志，
 *   会话表**只读**（写由主后端负责），config.json **只读**。
 */

'use strict';

const crypto = require('crypto');
const fs = require('fs');
const http = require('http');
const path = require('path');
const express = require('express');
const helmet = require('helmet');

const { createIotCenter } = require('./iotCenter');
const { createRequireAdmin } = require('./adminAuth');
const { getPublicOrigin, getClientIP } = require('./netContext');

/* ── 路径与配置 ────────────────────────────────────────────────────── */

const BASE_DIR = __dirname;
const PORT = Number(process.env.IOT_PORT || 8093);
/* ⚠ 必须与主后端指到**同一个**目录；服务器上由 ecosystem 显式给绝对路径 */
const DATA_DIR = process.env.DATA_DIR
  ? path.resolve(process.env.DATA_DIR)
  : path.resolve(BASE_DIR, '..', 'data', 'iot_backend');
const UPLOAD_DIR = process.env.UPLOAD_DIR
  ? path.resolve(process.env.UPLOAD_DIR)
  : path.resolve(BASE_DIR, '..', 'uploads');
const SESSIONS_FILE = path.join(DATA_DIR, 'admin-sessions.json');
const CONFIG_FILE = path.join(DATA_DIR, 'config.json');
const LOG_FILE = path.join(DATA_DIR, 'iot-service-logs.json');

[DATA_DIR, UPLOAD_DIR].forEach((dir) => {
  try {
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
  } catch (e) {
    console.error('[iot-service] 目录创建失败 ' + dir + '：' + (e && e.message));
  }
});

/* ── 小工具 ────────────────────────────────────────────────────────── */

function now() {
  return new Date().toISOString();
}

function stripBom(text) {
  return String(text || '').replace(/^\uFEFF/, '');
}

function readJSONFile(file, defaultValue = {}) {
  try {
    if (!fs.existsSync(file)) return defaultValue;
    const parsed = JSON.parse(stripBom(fs.readFileSync(file, 'utf8')));
    return parsed === null || parsed === undefined ? defaultValue : parsed;
  } catch {
    return defaultValue;
  }
}

/**
 * 原子写：先写临时文件再改名。
 * ⚠ 只用于**本服务独占**的文件（iot-*.json 与自己的日志）——
 *   绝不能碰主后端也在写的文件（会话表、config.json）。
 */
function writeJSONFile(file, data) {
  const tmp = file + '.tmp';
  fs.writeFileSync(tmp, JSON.stringify(data, null, 2), 'utf8');
  fs.renameSync(tmp, file);
}

/**
 * 本服务自己的操作留痕（**不写**主后端的日志文件，避免两个进程抢同一个 JSON）。
 */
function addLog(entry) {
  try {
    const logs = readJSONFile(LOG_FILE, []);
    const list = Array.isArray(logs) ? logs : [];
    list.unshift({
      id: crypto.randomUUID ? crypto.randomUUID() : String(Date.now()),
      createdAt: now(),
      ...entry,
    });
    if (list.length > 2000) list.length = 2000;
    writeJSONFile(LOG_FILE, list);
  } catch (e) {
    console.warn('[iot-service] 留痕失败:', e && e.message);
  }
}

/** 读主后端的 config.json（**只读**：口令哈希与 jwtSecret 都在里面） */
function getConfig() {
  return readJSONFile(CONFIG_FILE, {});
}

/* ── 应用 ──────────────────────────────────────────────────────────── */

const app = express();

app.disable('x-powered-by');
app.use(helmet({
  contentSecurityPolicy: false,
  crossOriginEmbedderPolicy: false,
}));
app.use(express.json({ limit: '2mb' }));
app.use(express.urlencoded({ limit: '50mb', extended: true }));

/* 安全响应头（与主后端一致：API 不缓存、禁 iframe 嵌套） */
app.use((req, res, next) => {
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('X-Frame-Options', 'SAMEORIGIN');
  res.setHeader('Referrer-Policy', 'no-referrer');
  res.setHeader('Cache-Control', 'no-store');
  next();
});

/* ── 后台鉴权（沿用主后台账号与登录态） ───────────────────────────── */

const requireAdmin = createRequireAdmin({
  sessionsFile: SESSIONS_FILE,
  onDenied: ({ req, status, message }) => {
    addLog({
      type: 'iot_admin_denied',
      ip: getClientIP(req),
      detail: `后台访问被拒（${status}）：${message}`,
    });
  },
});

/* ── 物联网中心 ────────────────────────────────────────────────────── */

const iotCenter = createIotCenter({
  files: {
    devices: path.join(DATA_DIR, 'iot-devices.json'),
    thresholds: path.join(DATA_DIR, 'iot-thresholds.json'),
    firmware: path.join(DATA_DIR, 'iot-firmware.json'),
    telemetry: path.join(DATA_DIR, 'iot-telemetry.json'),
  },
  readJSON: readJSONFile,
  writeJSON: writeJSONFile,
  initFile: (file, def) => {
    try {
      if (!fs.existsSync(file)) fs.writeFileSync(file, JSON.stringify(def, null, 2));
    } catch (e) {
      console.warn('[iot-service] initFile failed: ' + (e && e.message));
    }
  },
  now,
  addLog,
  requireAdmin,
  uploadDir: UPLOAD_DIR,
  // 签名密钥复用 config.json 里的 jwtSecret（与主后端同一套，不新增人工管理的凭据）
  getSecret: () => (getConfig() || {}).jwtSecret || '',
  getPublicOrigin,
  getClientIP,
});

/*
 * 设备侧路由（/api/iot/*）
 * ⚠ 这里**不需要**「注册在全局限流器之前」那种脆弱约定了 ——
 *   本服务是独立进程，没有主后端那个 120/min/IP 的全局限流器，
 *   而设备侧按**设备**限流（iotCenter 内部，粒度比 IP 细得多）。
 *   全校设备共用同一个出口 NAT IP 这件事，在独立进程里不再是问题。
 */
iotCenter.registerDeviceRoutes(app);

/* 管理侧路由（/admin/api/iot/*） */
iotCenter.registerAdminRoutes(app);

/* ── 管理界面 ──────────────────────────────────────────────────────── */

const IOT_HTML = path.join(BASE_DIR, 'iot.html');

function sendIotPage(req, res) {
  if (!fs.existsSync(IOT_HTML)) {
    return res.status(404).type('text/plain; charset=utf-8').send('物联网后台页面不存在');
  }
  res.setHeader('Content-Type', 'text/html; charset=utf-8');
  res.setHeader('Cache-Control', 'no-store');
  return res.sendFile(IOT_HTML);
}

app.get('/iot', sendIotPage);
app.get('/iot/', sendIotPage);
app.get('/', sendIotPage);

/*
 * 后台登录接口的**转发**（2026-10-25）
 * ------------------------------------------------------------------
 * 为什么需要：管理页面里的登录是相对路径 \`fetch('/admin/api/login')\`。
 *   · 走 nginx 同源访问（https://YOUR_DOMAIN/iot）时，这个请求由 nginx
 *     交给 8080 的主后台 ⇒ 正常；
 *   · **直连本服务端口**（http://<ip>:8093/）时，这个请求会打到**本服务**
 *     ⇒ 没有这条转发就是 404 ⇒ 页面能打开但**登录永远失败**。
 *
 * 为什么不自己实现口令校验：主后端那边有 IP 限流、失败递增退避、600ms 延迟、
 *   IP 白名单与审计留痕。转发过去 ⇒ 行为与网页版后台**完全一致**，
 *   也不存在「两套口令校验实现漂移」的风险。
 *
 * ⚠ 转发时必须带上真实客户端 IP：
 *   主后端的登录限流与失败锁定都按 IP 算，而本服务与它同机、直连方恒为 127.0.0.1
 *   ⇒ 不传 X-Forwarded-For 的话，所有直连用户会被算成同一个 IP 一起锁死。
 *   主后端的 TRUSTED_PROXY_IPS 含 127.0.0.1 ⇒ 它会采信这个头。
 *
 * ⚠⚠ 2026-10-25 踩坑：转发时**漏了 User-Agent**，主后台立刻返回 403 且**空 body** ——
 *   原因是主后台有个「屏蔽爬虫/扫描器对 admin 路径的嗅探」中间件：
 *     if (req.path.startsWith('/admin')) { const ua = ...; if (!ua || /sqlmap|nikto|.../i.test(ua)) return 403 }
 *   而 http.request **默认不发送 User-Agent** ⇒ 主后台看到的是「无 UA 的请求」⇒ 直接拒。
 *   ⇒ 转发必须把 UA 带上（转发真实浏览器 UA，或给一个明确的 UA）。
 */
const MAIN_BACKEND_PORT = Number(process.env.MAIN_BACKEND_PORT || 8080);

app.post('/admin/api/login', (req, res) => {
  const body = JSON.stringify(req.body || {});
  const upstream = http.request({
    hostname: '127.0.0.1',
    port: MAIN_BACKEND_PORT,
    path: '/admin/api/login',
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Content-Length': Buffer.byteLength(body),
      'X-Forwarded-For': getClientIP(req),
      // ⚠ 必须带：主后台会拒「无 User-Agent 的 /admin/* 请求」
      'User-Agent': String((req.headers && req.headers['user-agent']) || 'campus-iot-service/1.0'),
    },
  }, (up) => {
    res.status(up.statusCode || 502);
    up.pipe(res);
  });
  upstream.on('error', (e) => {
    console.warn('[iot-service] 转发后台登录失败:', e && e.message);
    if (!res.headersSent) {
      res.status(502).json({ success: false, message: '主后台暂时不可达，请稍后再试' });
    }
  });
  upstream.setTimeout(8000, () => {
    upstream.destroy(new Error('UPSTREAM_TIMEOUT'));
  });
  upstream.end(body);
});

/* 健康检查（给 pm2 / nginx / 运维看） */
app.get('/api/iot/health', (req, res) => {
  res.json({
    success: true,
    service: 'iot-backend',
    port: PORT,
    dataDir: DATA_DIR,
    uptimeSec: Math.round(process.uptime()),
  });
});

/* ── 404 / 错误处理 ────────────────────────────────────────────────── */

app.use((req, res) => {
  res.status(404).json({ success: false, message: '接口不存在' });
});

app.use((err, req, res, next) => {
  const status = err.status || err.statusCode || 500;
  if (!res.headersSent) {
    res.status(status).json({
      success: false,
      message: status >= 500 ? '服务器内部错误' : (err.message || '请求失败'),
    });
  }
  if (status >= 500) {
    console.error('[iot-service] 未捕获异常:', err && err.stack);
  }
});

/* ── 启动 ──────────────────────────────────────────────────────────── */

const server = app.listen(PORT, '0.0.0.0', () => {
  console.log('[iot-service] 校园物联网中心已启动');
  console.log('  端口     : ' + PORT);
  console.log('  数据目录 : ' + DATA_DIR);
  console.log('  上传目录 : ' + UPLOAD_DIR);
  console.log('  管理界面 : https://YOUR_DOMAIN/iot');
  console.log('  会话表   : ' + SESSIONS_FILE + '（只读，与主后台共用）');
});

/*
 * MQTT 是**可选增强**（HTTP 直连的设备完全不依赖它）：
 *   没配 MQTT_URL → 不订阅、不连接、不报错；
 *   配了但 broker 连不上 → iotCenter 内部指数退避重连，不抛到主进程。
 * ⚠ 独立之后这条更关键了：MQTT 挂掉**只**影响本服务，不会牵连主后端。
 */
iotCenter.startMqtt();

/* ── 优雅退出：先把内存里的遥测与设备状态刷盘 ──────────────────────── */

let shuttingDown = false;
function shutdown(signal) {
  if (shuttingDown) return;
  shuttingDown = true;
  console.log('[iot-service] 收到 ' + signal + '，开始刷盘并退出…');
  try {
    iotCenter.flushTelemetry();
    iotCenter.saveDeviceState();
  } catch (e) {
    console.warn('[iot-service] 退出前刷盘失败:', e && e.message);
  }
  try {
    iotCenter.stopMqtt();
  } catch (e) { /* MQTT 停了就算了 */ }
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(0), 5000).unref();
}

process.on('SIGTERM', () => shutdown('SIGTERM'));
process.on('SIGINT', () => shutdown('SIGINT'));
process.on('unhandledRejection', (reason) => {
  console.error('[iot-service] 未处理的 Promise 拒绝:', reason);
});

module.exports = { app, server, iotCenter };
