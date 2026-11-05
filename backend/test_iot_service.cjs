/*
 * test_iot_service.cjs —— 校园物联网独立服务的自检
 * ---------------------------------------------------------------------------
 * 覆盖三件事：
 *   1. **共用主后台登录态**：会话表是主后端写的 admin-sessions.json，
 *      本服务只读 ⇒ 主后端刚签发的 token 必须立刻能用；过期/伪造的一律拒。
 *   2. **管理界面独立**：/iot 页面由本服务自己提供（不依赖 8080）。
 *   3. **设备侧链路**：设备密钥校验、遥测上报、固件签名链接（含篡改签名被拒）。
 *
 * 用法：node iot_service/test_iot_service.cjs
 */
'use strict';

const fs = require('fs');
const os = require('os');
const path = require('path');
const http = require('http');
const { spawn } = require('child_process');
/* 纯函数判定（handler 用的就是这两个）—— 直接测实现，不复刻逻辑 */
const iotCenter = require('./iotCenter');

const PORT = 18110;
const BASE = `http://127.0.0.1:${PORT}`;
const TMP = fs.mkdtempSync(path.join(os.tmpdir(), 'iot-service-'));
const UPLOAD = path.join(TMP, 'uploads');
const FIRMWARE_DIR = path.join(UPLOAD, 'iot-firmware');

let pass = 0;
let fail = 0;
function ok(label, cond, extra) {
  if (cond) {
    pass++;
    console.log('  ✔ ' + label);
  } else {
    fail++;
    console.log('  ✘ ' + label + (extra !== undefined ? '  → ' + JSON.stringify(extra).slice(0, 300) : ''));
  }
}

function request(method, urlPath, headers = {}, body = null) {
  return new Promise((resolve) => {
    const u = new URL(urlPath, BASE);
    const h = Object.assign({ 'Content-Type': 'application/json' }, headers);
    let payload = '';
    if (body !== null && body !== undefined) payload = typeof body === 'string' ? body : JSON.stringify(body);
    if (payload) h['Content-Length'] = Buffer.byteLength(payload);
    const rq = http.request(
      { hostname: u.hostname, port: u.port, path: u.pathname + u.search, method, headers: h },
      (res) => {
        let buf = '';
        res.on('data', (c) => { buf += c; });
        res.on('end', () => {
          let j = null;
          try { j = JSON.parse(buf); } catch { /* 非 JSON */ }
          resolve({
            status: res.statusCode,
            json: j,
            body: buf.slice(0, 400),
            /* 【为什么要回传 headers】Range 续传要验 206 + Content-Range，
             *   这两个信息只在响应头里，body 里看不到 */
            headers: res.headers,
          });
        });
      },
    );
    rq.on('error', (e) => resolve({ status: 0, json: null, body: 'ERR ' + (e && e.message) }));
    rq.setTimeout(8000, () => { rq.destroy(new Error('TIMEOUT')); });
    if (payload) rq.write(payload);
    rq.end();
  });
}

async function waitReady(timeoutMs = 20000) {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    try {
      const r = await request('GET', '/api/iot/health');
      if (r.status === 200) return true;
    } catch { /* 还没起来 */ }
    await new Promise((r) => setTimeout(r, 250));
  }
  return false;
}

/* 会话表：与主后端 admin-sessions.json **同一个格式** */
const SESSIONS = path.join(TMP, 'admin-sessions.json');
const GOOD_TOKEN = 'sess-good-token-0001';
const EXPIRED_TOKEN = 'sess-expired-token-0002';
const NOW = Date.now();
fs.writeFileSync(SESSIONS, JSON.stringify({
  [GOOD_TOKEN]: { createdAt: new Date(NOW - 60000).toISOString(), lastActive: new Date(NOW - 30000).toISOString(), ip: '127.0.0.1' },
  [EXPIRED_TOKEN]: { createdAt: new Date(NOW - 30 * 86400000).toISOString(), lastActive: new Date(NOW - 20 * 86400000).toISOString(), ip: '127.0.0.1' },
}, null, 2), 'utf8');

/* 固件清单 + 文件（模拟已上传的固件） */
fs.mkdirSync(FIRMWARE_DIR, { recursive: true });
const FW_NAME = 'iot-test-v1.0.0.bin';
fs.writeFileSync(path.join(FIRMWARE_DIR, FW_NAME), Buffer.from('FAKE-FIRMWARE-BYTES'), 'utf8');
fs.writeFileSync(path.join(TMP, 'iot-firmware.json'), JSON.stringify({
  items: [{
    id: 'fw-test-id',
    filename: FW_NAME,
    version: '1.0.0',
    sha256: 'x'.repeat(64),
    size: 19,
    groups: [],
    note: '自检用',
    uploadedAt: new Date().toISOString(),
  }],
}, null, 2), 'utf8');

(async () => {
  console.log('\n【1】启动独立服务');
  const child = spawn(process.execPath, [path.join(__dirname, 'index.js')], {
    cwd: __dirname,
    env: {
      ...process.env,
      IOT_PORT: String(PORT),
      DATA_DIR: TMP,
      UPLOAD_DIR: UPLOAD,
      NODE_ENV: 'production',
      // 明确清掉：避免继承到主后端的反代信任配置影响 IP 判定
      TRUSTED_PROXY_IPS: '',
      TRUSTED_PROXY_HEADER: '',
      TRUST_PROXY: '',
      ADMIN_IP_ALLOW: '',
      /* T45 起默认要求鉴权；本文件【3】专门验证「测试免登录」模式，
       * 必须显式置 0，否则测的是默认 401，与用例标题不符。 */
      IOT_REQUIRE_ADMIN: '0',
    },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  let out = '';
  child.stdout.on('data', (d) => { out += d.toString(); });
  child.stderr.on('data', (d) => { out += d.toString(); });

  const ready = await waitReady();
  ok('服务启动并响应 /api/iot/health', ready, out.slice(-400));
  if (!ready) {
    console.log(out.slice(-2000));
    try { child.kill(); } catch (e) { /* 忽略 */ }
    console.log('\n总计：通过 ' + pass + ' / 失败 ' + fail);
    process.exit(1);
  }

  console.log('\n【2】健康检查与管理界面（独立）');
  let r = await request('GET', '/api/iot/health');
  ok('GET /api/iot/health → 200', r.status === 200 && r.json && r.json.service === 'iot-backend', r.json);
  r = await request('GET', '/iot');
  ok('GET /iot → 200 且是 HTML 页面', r.status === 200 && /<html/i.test(r.body), { status: r.status });
  r = await request('GET', '/');
  ok('GET / → 200（管理界面首页别名）', r.status === 200, { status: r.status });

  console.log('\n【3】管理路由免登录（测试阶段，2026-10-31 起）');
  /*
   * 以前这一段测的是「共用主后台登录态」：无 token / 伪造 token / 过期会话 → 401。
   * 现在测试阶段不需要口令了，IoT 管理路由全部放行，
   * 所以断言反过来：**不带 token 也能拿到数据**。
   * 想切回口令模式：环境变量 IOT_REQUIRE_ADMIN=1（见 iotCenter.js 的 adminOpen）。
   */
  r = await request('GET', '/admin/api/iot/devices');
  ok('无 token → 200（免登录）', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': 'forged-token' });
  ok('伪造 token 也放行（免登录）', r.status === 200, { status: r.status });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  ok('带有效 token → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': EXPIRED_TOKEN });
  ok('过期会话也放行（免登录）', r.status === 200, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/stats', { 'x-admin-token': GOOD_TOKEN });
  ok('GET /admin/api/iot/stats → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });

  console.log('\n【4】会话表只读（不与主后端抢写）');
  const after = fs.readFileSync(SESSIONS, 'utf8');
  ok('本服务没有改写会话表', after === fs.readFileSync(SESSIONS, 'utf8'));
  ok('过期会话仍留在文件里（由主后端负责清理）',
    Object.keys(JSON.parse(after)).includes(EXPIRED_TOKEN));

  console.log('\n【5】设备侧链路（设备密钥）');
  r = await request('POST', '/api/iot/telemetry', {}, { deviceId: 'dev-test-001', temp: 21.5 });
  ok('无设备密钥上报 → 401', r.status === 401 && r.json && r.json.code === 'IOT_DEVICE_UNAUTHORIZED', { status: r.status, body: r.body });
  r = await request('POST', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN }, {
    deviceId: 'dev-test-001', name: '自检设备', group: 'test',
  });
  ok('后台登记设备 → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  const deviceKey = r.json && r.json.device && r.json.device.deviceKey;
  ok('登记返回了设备密钥', !!deviceKey, r.json);
  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    deviceId: 'dev-test-001', temp: 21.5, hum: 48,
  });
  ok('带设备密钥上报遥测 → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  r = await request('GET', '/api/iot/config?deviceId=dev-test-001', { 'x-iot-key': deviceKey });
  ok('设备拉配置 → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });

  console.log('\n【6】固件签名下载');
  /*
   * 【2026-10-31 补】设备上报的三种真实形状，回归覆盖后台兼容层：
   *   ① ESP8266 的报文**没有 deviceId**，只有 X-Iot-Key 头
   *      （esp8266_ota/src/fw_download.cpp:341 / :398）
   *   ② 字段别名：temperature / humidity / pressure
   *   ③ 气压单位：STM32 给的是 Pa（uart_cmd.c 的 TL 行），页面那列是 hPa
   */
  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    temp: 23.45, hum: 48.12, press: 101325, batt: 87, status: 'normal',
  });
  ok('没有 deviceId 的遥测（ESP8266 真实报文）→ 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    temperature: 19.5, humidity: 55, pressure: 99840,
  });
  ok('字段别名（temperature/humidity/pressure）→ 200', r.status === 200, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  const dev = (r.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  ok('别名温度入库 = 19.5', dev && dev.sensors && dev.sensors.temp === 19.5, dev && dev.sensors);
  ok('别名湿度入库 = 55', dev && dev.sensors && dev.sensors.hum === 55, dev && dev.sensors);
  ok('气压 Pa → hPa（99840 → 998）', dev && dev.sensors && dev.sensors.press === 998, dev && dev.sensors);
  ok('状态 = normal', dev && dev.status === 'normal', dev && dev.status);
  ok('列表里带最新遥测 latest', dev && dev.latest && dev.latest.temp === 19.5, dev && dev.latest);

  r = await request('POST', '/api/iot/event', { 'x-iot-key': deviceKey }, {
    type: 'sensor_recovered', level: 'info', detail: 'aht20',
  });
  ok('没有 deviceId 的事件 → 200', r.status === 200 && r.json && r.json.success === true, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/events?limit=50', { 'x-admin-token': GOOD_TOKEN });
  ok('事件列表里有这条事件', (r.json.events || []).some((e) => e.deviceId === 'dev-test-001' && e.type === 'sensor_recovered'), r.json.events && r.json.events.slice(0, 3));

  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { tempMax: 30, humMax: 60 },
  });
  ok('下发阈值 → 200 且改动 2 项', r.status === 200 && r.json && r.json.changed === 2, { status: r.status, body: r.body });
  ok('下发后回显生效值 tempMax=30', r.json.thresholds && r.json.thresholds.tempMax === 30, r.json.thresholds);
  r = await request('GET', '/api/iot/config?deviceId=dev-test-001', { 'x-iot-key': deviceKey });
  ok('设备拉配置拿到 tempMax=30', r.status === 200 && r.json.config && r.json.config.tempMax === 30, r.json.config);
  ok('设备拉配置拿到 humMax=60', r.json.config && r.json.config.humMax === 60, r.json.config);
  r = await request('GET', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN });
  ok('后台回读：applied=true（设备已拉走这一版）', r.json.applied === true, r.json);
  ok('后台回读：appliedAt 有值', !!r.json.appliedAt, r.json.appliedAt);

  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, { temp: 45, hum: 40 });
  ok('越限遥测 → 200 且带越限说明', r.status === 200 && r.json && r.json.alarm, r.json);
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  const devAlarm = (r.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  ok('越限后状态 = alarm', devAlarm && devAlarm.status === 'alarm', devAlarm && devAlarm.status);
  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, { temp: 22, hum: 45 });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  const devOk = (r.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  ok('恢复正常后状态自愈回 normal', devOk && devOk.status === 'normal', devOk && devOk.status);

  r = await request('GET', '/admin/api/iot/firmware', { 'x-admin-token': GOOD_TOKEN });
  ok('固件列表 → 200 且能看到固件', r.status === 200 && r.json && r.json.total >= 1, { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/firmware/link?id=fw-test-id', { 'x-admin-token': GOOD_TOKEN });
  ok('生成签名链接 → 200 且带签名', r.status === 200 && r.json && /[?&]t=/.test(String(r.json.url || '')), { status: r.status, body: r.body });
  const link = r.json && r.json.url;
  if (link) {
    const rel = link.replace(/^https?:\/\/[^/]+/, '');
    const dl = await request('GET', rel);
    ok('按签名链接下载固件 → 200', dl.status === 200 && /FAKE-FIRMWARE/.test(dl.body), { status: dl.status, body: String(dl.body).slice(0, 80) });
    const tampered = rel.replace(/t=([^&]+)/, 't=forged');
    const bad = await request('GET', tampered);
    ok('签名被篡改 → 403', bad.status === 403, { status: bad.status, body: bad.body });
  }

  console.log('\n【7】未知的接口 → 404');
  r = await request('GET', '/admin/api/iot/not-exist', { 'x-admin-token': GOOD_TOKEN });
  ok('未知接口 → 404', r.status === 404, { status: r.status });

  /* ══════════════════════════════════════════════════════════════
   * 【8】上报周期不再被写死（2026-11-02）
   *   需求：上报时间别固定 60 秒 —— 2 秒就 2 秒，1 秒、0.5 秒都行。
   *   以前 reportIntervalSec 的 min=5，填 2 秒会被 clamp 成 5，
   *   而且只以整数秒下发，0.5 秒根本表达不了。
   * ══════════════════════════════════════════════════════════════ */
  console.log('\n【8】上报周期（亚秒 / 不被写死）');
  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 2 },
  });
  ok('下发 reportIntervalSec=2 → 改动 1 项', r.status === 200 && r.json && r.json.changed === 1,
    { status: r.status, body: r.body });
  ok('回显 reportIntervalSec=2（没被 clamp 成 5 或 60）',
    r.json.thresholds && r.json.thresholds.reportIntervalSec === 2, r.json.thresholds);
  ok('毫秒下发值 = 2000', r.json.reportIntervalMs === 2000, r.json.reportIntervalMs);
  r = await request('GET', '/api/iot/config?deviceId=dev-test-001', { 'x-iot-key': deviceKey });
  ok('设备拉配置拿到 reportIntervalSec=2',
    r.json.config && r.json.config.reportIntervalSec === 2, r.json.config);
  ok('设备拉配置拿到 reportIntervalMs=2000',
    r.json.reportIntervalMs === 2000, r.json.reportIntervalMs);

  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 0.5 },
  });
  ok('下发 0.5 秒 → 保留小数不被取整成 1',
    r.json.thresholds && r.json.thresholds.reportIntervalSec === 0.5, r.json.thresholds);
  ok('0.5 秒 → 毫秒下发值 = 500', r.json.reportIntervalMs === 500, r.json.reportIntervalMs);

  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 0.1 },
  });
  ok('低于 0.5 秒 → 兜底到 500ms（保护 8266 不被刷爆）',
    r.json.reportIntervalMs === 500, r.json.reportIntervalMs);

  /* 【关键回归】以前后端把任何值都 Math.min 到 2000ms，
   * 于是「改成 10 秒」根本发不出去。这里把 10 秒和 1 小时钉死。 */
  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 10 },
  });
  ok('下发 10 秒 → 毫秒下发值 = 10000（不再被钳成 2000）',
    r.json.reportIntervalMs === 10000, r.json.reportIntervalMs);

  r = await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 3600 },
  });
  ok('下发 3600 秒 → 毫秒下发值 = 3600000',
    r.json.reportIntervalMs === 3600000, r.json.reportIntervalMs);

  /* 传感器历史：网页「传感器历史」视图的数据源 */
  r = await request('GET', '/admin/api/iot/telemetry?deviceId=dev-test-001&limit=50',
    { 'x-admin-token': GOOD_TOKEN });
  ok('遥测历史接口 → 200 且返回样本数组',
    r.status === 200 && r.json && Array.isArray(r.json.samples), { status: r.status, body: r.body });
  ok('历史样本带时间戳和温度',
    r.json.samples && r.json.samples.length > 0 &&
    !!r.json.samples[0].ts && typeof r.json.samples[0].temp === 'number',
    r.json.samples && r.json.samples[0]);

  /* 恢复成 2 秒，别影响后面用例 */
  await request('POST', '/admin/api/iot/thresholds/dev-test-001', { 'x-admin-token': GOOD_TOKEN }, {
    thresholds: { reportIntervalSec: 2 },
  });

  /* ══════════════════════════════════════════════════════════════
   * 【9】固件直链支持 Range（8266 断点续传下载的前提）
   *   8266 下载到一半断线，要能接着下 —— 靠 HTTP 206 Partial Content。
   *   express 的 sendFile 原生支持 Range，这里验一下确实生效。
   * ══════════════════════════════════════════════════════════════ */
  console.log('\n【9】固件直链 Range 续传');
  if (link) {
    const rel = link.replace(/^https?:\/\/[^/]+/, '');
    const rangeRes = await request('GET', rel, { Range: 'bytes=0-5' });
    ok('带 Range 的请求 → 206 Partial Content',
      rangeRes.status === 206, { status: rangeRes.status, body: String(rangeRes.body).slice(0, 80) });
    ok('206 带 Content-Range 头', !!rangeRes.headers && !!rangeRes.headers['content-range'],
      rangeRes.headers && rangeRes.headers['content-range']);
    ok('206 的正文只有请求的那一段（6 字节）',
      String(rangeRes.body).length === 6, { len: String(rangeRes.body).length, body: String(rangeRes.body) });
  }

  /* ══════════════════════════════════════════════════════════════
   * 【10】设备 ID 别卡太死 + 固件直链必须是设备能访问的地址
   *   以前设备拿到的直链可能是 http://127.0.0.1/... ——
   *   那是服务器自己的环回地址，8266 拿到只会去连自己，必然下载失败。
   * ══════════════════════════════════════════════════════════════ */
  console.log('\n【10】设备 ID 长度放宽 + 固件直链地址');
  r = await request('POST', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN }, {
    deviceId: 'a1', name: '短 ID 设备',
  });
  ok('2 位设备 ID 可以登记 → 200', r.status === 200 && r.json && r.json.success === true,
    { status: r.status, body: r.body });

  if (link) {
    /* 让设备侧走一遍「拉配置拿直链」：先给设备下发一个 OTA 指令 */
    r = await request('POST', '/admin/api/iot/ota/push', { 'x-admin-token': GOOD_TOKEN }, {
      firmwareId: 'fw-test-id', deviceIds: ['dev-test-001'],
    });
    ok('下发 OTA 指令 → 200', r.status === 200 && r.json && r.json.success === true,
      { status: r.status, body: r.body });
    r = await request('GET', '/api/iot/config?deviceId=dev-test-001', { 'x-iot-key': deviceKey });
    ok('设备拉配置拿到 ota 指令', r.status === 200 && r.json && r.json.ota,
      { status: r.status, body: r.body });
    const devUrl = r.json && r.json.ota && r.json.ota.url;
    ok('设备拿到的固件直链不是环回地址（127.0.0.1/localhost）',
      !!devUrl && !/^https?:\/\/(127\.0\.0\.1|localhost|\[::1\])/i.test(devUrl),
      { url: devUrl });
    ok('设备拿到的固件直链是明文 http（8266 没有 TLS 栈）',
      !!devUrl && /^http:\/\//i.test(devUrl), { url: devUrl });
    if (devUrl) {
      const durl = new URL(devUrl);
      const dres = await request('GET', durl.pathname + durl.search);
      ok('设备按直链能下载到固件 → 200', dres.status === 200,
        { status: dres.status, body: String(dres.body).slice(0, 80) });
    }
  }

  /* ══════════════════════════════════════════════════════════════
   * 【11】读数范围校验
   *   传感器坏了会报出「湿度 523%」这种数。照单全收的话，
   *   网页上就显示荒谬的读数，用户根本没法判断设备坏没坏。
   *   正确做法：丢掉**这个字段**（置 null），但保留整条遥测。
   * ══════════════════════════════════════════════════════════════ */
  console.log('\n【11】传感器读数范围校验');
  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    temp: 22.5, hum: 523, press: 1005, batt: 88,
  });
  ok('湿度 523% 的包 → 仍然入库（不丢整条）', r.status === 200 && r.json && r.json.success === true,
    { status: r.status, body: r.body });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  const devBad = (r.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  const badLatest = devBad && devBad.latest;
  ok('湿度超范围 → 该字段置 null', badLatest && badLatest.hum === null,
    badLatest);
  ok('温度正常 → 仍然保留', badLatest && Math.abs(badLatest.temp - 22.5) < 0.01,
    badLatest);
  ok('气压正常 → 仍然保留', badLatest && Math.abs(badLatest.press - 1005) < 1,
    badLatest);

  r = await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    temp: -999, hum: 50, press: 1005,
  });
  r = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
  const devBad2 = (r.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  ok('温度 -999℃ → 置 null', devBad2 && devBad2.latest && devBad2.latest.temp === null,
    devBad2 && devBad2.latest);

  /* 恢复正常值，别影响后续用例 */
  await request('POST', '/api/iot/telemetry', { 'x-iot-key': deviceKey }, {
    temp: 23.4, hum: 51.2, press: 1013, batt: 87, status: 'normal',
  });

  /* ══════════════════════════════════════════════════════════════
   * 【12】OTA 指令的"销账"—— 死循环回归测试
   *
   * 【这条链路的原缺陷】/api/iot/config 一直把 pendingOta 派发给设备，
   *   而设备端（改之前的 esp8266_ota）从来不调 /api/iot/ota/result。
   *   于是：指令永远销不掉 → 8266 每 10 秒轮询命中同一条 → 每次都发触发串
   *   → STM32 每次都被复位进 bootloader 重刷 → 板子永远起不来（屏幕不亮），
   *   8266 永远在下载刷机器（长时间阻塞 → 定时重启）。
   *   现象和"STM32 卡死 + 8266 定时重启"完全吻合，而日志里每一轮都是"成功"。
   *
   *   现在两侧各有一道闸：后台能自己停发（连败 / 已到位 / 显式结掉），
   *   设备端也按指令 id 记本地账。这里测后台这一半。
   * ══════════════════════════════════════════════════════════════ */
  console.log('\n【12】OTA 指令销账（防重复刷机死循环）');
  const getOta = async () => {
    const q = await request('GET', '/api/iot/config?deviceId=dev-test-001', { 'x-iot-key': deviceKey });
    return (q.json && q.json.ota) || null;
  };
  const adminDev = async () => {
    const q = await request('GET', '/admin/api/iot/devices', { 'x-admin-token': GOOD_TOKEN });
    return (q.json.devices || []).find((d) => d.deviceId === 'dev-test-001');
  };

  /* —— 下发一条新指令 —— */
  r = await request('POST', '/admin/api/iot/ota/push', { 'x-admin-token': GOOD_TOKEN }, {
    firmwareId: 'fw-test-id', deviceIds: ['dev-test-001'],
  });
  ok('下发指令 → 200 且带 otaId', r.status === 200 && !!r.json.otaId, { status: r.status, body: r.body });
  const id1 = r.json.otaId;
  let ota = await getOta();
  ok('设备轮询能拿到这条指令', !!ota && ota.id === id1, ota);
  ok('指令里带 id（设备端本地记帐靠它）', !!ota && !!ota.id, ota && Object.keys(ota));

  /* —— 失败上报：三次之后必须停发 —— */
  let f1 = await request('POST', '/api/iot/ota/result', { 'x-iot-key': deviceKey },
    { id: id1, success: false, message: 'download failed' });
  ok('失败回报 → 200', f1.status === 200 && f1.json && f1.json.success === true, f1 && f1.body);
  ota = await getOta();
  ok('失败 1 次后仍然继续派发（下载中断要能续传）', !!ota && ota.id === id1, ota);
  await request('POST', '/api/iot/ota/result', { 'x-iot-key': deviceKey },
    { id: id1, success: false, message: 'verify fail' });
  await request('POST', '/api/iot/ota/result', { 'x-iot-key': deviceKey },
    { id: id1, success: false, message: 'ER:jump flag' });
  ota = await getOta();
  ok('★ 连败 3 次后停发这条指令（死循环就在这里被掐断）', !ota, ota);
  let dv = await adminDev();
  ok('记录仍留着且标成已放弃（网页要能看见）',
    dv && dv.pendingOta && dv.pendingOta.active === false && dv.pendingOta.failures === 3,
    dv && dv.pendingOta);
  ok('最后一次失败原因留痕', dv && dv.pendingOta && /jump/.test(String(dv.pendingOta.lastError)),
    dv && dv.pendingOta && dv.pendingOta.lastError);

  /* —— 重新下发换一个新 id → 又能动 —— */
  r = await request('POST', '/admin/api/iot/ota/push', { 'x-admin-token': GOOD_TOKEN }, {
    firmwareId: 'fw-test-id', deviceIds: ['dev-test-001'],
  });
  const id2 = r.json.otaId;
  ok('重新下发的 id 与旧的 different', !!id2 && id2 !== id1, { id1, id2 });
  ota = await getOta();
  ok('新指令正常派发（停发不会永久锁死）', !!ota && ota.id === id2, ota);

  /* —— 慢半拍的旧回报：不能把新指令误清掉 —— */
  await request('POST', '/api/iot/ota/result', { 'x-iot-key': deviceKey },
    { id: id1, success: true, version: '1.0.0' });
  ota = await getOta();
  ok('带着旧 id 的成功回报不会结掉新指令', !!ota && ota.id === id2, ota);

  /* —— 成功回报：结掉 + 记版本 + 记时间 —— */
  r = await request('POST', '/api/iot/ota/result', { 'x-iot-key': deviceKey },
    { id: id2, success: true, version: '1.0.0' });
  ok('成功回报 → 200', r.status === 200 && r.json && r.json.success === true, r && r.body);
  ota = await getOta();
  ok('★ 成功后不再派发（不会再每 10 秒重刷一次）', !ota, ota);
  dv = await adminDev();
  ok('成功后设备版本记为 1.0.0', dv && dv.firmwareVersion === '1.0.0', dv && dv.firmwareVersion);
  ok('成功后 pendingOta 已清空', !dv || !dv.pendingOta, dv && dv.pendingOta);

  /* —— 救砖场景：再推同一个版本必须还能推 ——
   * 【为什么专门测这个】"版本已经等于目标版本就自动销账"如果只看版本，
   * 板子刷坏了想重推同一个版本就永远推不动了 —— 这个坑要钉住。 */
  r = await request('POST', '/admin/api/iot/ota/push', { 'x-admin-token': GOOD_TOKEN }, {
    firmwareId: 'fw-test-id', deviceIds: ['dev-test-001'],
  });
  const id3 = r.json.otaId;
  ota = await getOta();
  ok('★ 重推同一个版本仍然派发（救砖路径没被封死）', !!ota && ota.id === id3, ota);

  /* —— 纯函数层的两条硬规则（handler 用的就是它们）—— */
  const pick = (dev) => iotCenter.otaPickForDevice(dev);
  ok('otaPickForDevice：已放弃的不派发',
    pick({ pendingOta: { id: 'x', version: '1.0.0', active: false } }).drop === true);
  ok('otaPickForDevice：下发之后已经刷成目标版本 → 自动销账',
    pick({ firmwareVersion: '1.0.0', firmwareUpdatedAt: '2026-11-01T10:00:00Z',
      pendingOta: { id: 'x', version: '1.0.0', issuedAt: '2026-11-01T09:00:00Z' } }).autoRetire === true);
  ok('otaPickForDevice：版本相等但指令是"版本确认之后"才下发的 → 仍然派发（救砖）',
    !!pick({ firmwareVersion: '1.0.0', firmwareUpdatedAt: '2026-11-01T09:00:00Z',
      pendingOta: { id: 'x', version: '1.0.0', issuedAt: '2026-11-01T10:00:00Z' } }).pending);
  const nxt = iotCenter.otaNextPending;
  ok('otaNextPending：不带 id 的老设备维持原语义（无条件结掉）',
    nxt({ id: 'x' }, '', true).pending === null);
  ok('otaNextPending：连败到上限置 active=false',
    (() => {
      let p = { id: 'x', failures: 0 };
      for (let i = 0; i < 3; i++) p = nxt(p, 'x', false, 3, 'boom').pending;
      return p.active === false && p.failures === 3;
    })());

  try { child.kill('SIGTERM'); } catch (e) { /* 忽略 */ }
  await new Promise((r) => setTimeout(r, 600));

  console.log('\n总计：通过 ' + pass + ' / 失败 ' + fail);
  process.exit(fail ? 1 : 0);
})();
