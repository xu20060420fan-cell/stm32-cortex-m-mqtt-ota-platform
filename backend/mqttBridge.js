/*
 * mqttBridge.js —— 可选的 MQTT 通道
 * ===========================================================================
 *
 * 定位：**增强，不是依赖**。
 *   · HTTP 是设备的主通道（8266 与 S3 现在都走 HTTP）；
 *   · MQTT 只做「阈值秒级下发」和「OTA 指令秒级下发」。
 *   · MQTT_URL 没配 / 配成 off → 本模块完全不启动，不报错、不占资源。
 *   · broker 挂了 → 自己指数退避重连，只影响本服务，不影响 HTTP。
 *
 * 主题约定（与旧后端保持一致，设备端固件不用改）：
 *   上行（本服务订阅）：campus/iot/<deviceId>/telemetry
 *                       campus/iot/<deviceId>/event
 *                       campus/iot/<deviceId>/status
 *   下行（本服务发布）：campus/iot/<deviceId>/config
 *                       campus/iot/<deviceId>/ota
 *                       campus/iot/broadcast
 */

'use strict';

function createMqttBridge(options) {
  const cfg = options || {};
  const prefix = String(cfg.prefix || 'campus/iot').replace(/\/+$/, '');
  const handlers = cfg.handlers || {};
  const log = typeof cfg.log === 'function' ? cfg.log : () => {};

  let client = null;
  let enabled = false;
  let connected = false;
  let lastError = '';
  let retryMs = 2000;
  let retryTimer = null;
  let stopped = false;

  function url() {
    const raw = String(cfg.url || process.env.MQTT_URL || process.env.IOT_MQTT_URL || '').trim();
    const low = raw.toLowerCase();
    if (!raw || low === 'off' || low === 'none' || low === 'false' || low === '0') return '';
    return raw;
  }

  function parseJson(buf) {
    try {
      const obj = JSON.parse(buf.toString('utf8'));
      return (obj && typeof obj === 'object' && !Array.isArray(obj)) ? obj : null;
    } catch {
      return null;
    }
  }

  function deviceFromTopic(topic) {
    const parts = String(topic || '').split('/');
    return parts.length >= 3 ? parts[2] : '';
  }

  function scheduleRetry() {
    if (stopped || retryTimer) return;
    const wait = retryMs;
    retryMs = Math.min(retryMs * 2, 30000);
    retryTimer = setTimeout(() => {
      retryTimer = null;
      start();
    }, wait);
    if (retryTimer.unref) retryTimer.unref();
  }

  function start() {
    stopped = false;
    const target = url();
    if (!target) {
      enabled = false;
      return false;
    }
    enabled = true;

    let mqtt;
    try {
      /* 可选依赖：没装 mqtt 包时只打一行警告，服务照常跑 HTTP */
      mqtt = require('mqtt');
    } catch (e) {
      lastError = 'mqtt 包未安装';
      log('[mqtt] ' + lastError + '，MQTT 通道停用');
      return false;
    }

    try {
      client = mqtt.connect(target, {
        clientId: cfg.clientId || process.env.MQTT_CLIENT_ID || 'campus-iot',
        username: cfg.username || process.env.MQTT_USERNAME || undefined,
        password: cfg.password || process.env.MQTT_PASSWORD || undefined,
        clean: true,
        connectTimeout: 8000,
        /* 关掉库自带的重连：它退避太凶，日志会刷屏，这里自己控制节奏 */
        reconnectPeriod: 0,
        resubscribe: false,
      });
    } catch (e) {
      lastError = String((e && e.message) || e);
      log('[mqtt] 创建客户端失败: ' + lastError);
      scheduleRetry();
      return false;
    }

    client.on('connect', () => {
      connected = true;
      lastError = '';
      retryMs = 2000;
      const topics = [
        prefix + '/+/telemetry',
        prefix + '/+/event',
        prefix + '/+/status',
      ];
      client.subscribe(topics, { qos: 0 }, (err) => {
        if (err) {
          lastError = String(err.message || err);
          log('[mqtt] 订阅失败: ' + lastError);
        }
      });
      log('[mqtt] 已连接: ' + target.replace(/\/\/[^@]*@/, '//***@'));
    });

    client.on('message', (topic, buf) => {
      const payload = parseJson(buf);
      if (!payload) return;
      const deviceId = String(payload.deviceId || payload.device_id || deviceFromTopic(topic) || '');
      const kind = String(topic).split('/').pop();
      try {
        if (kind === 'telemetry' && typeof handlers.onTelemetry === 'function') {
          handlers.onTelemetry(deviceId, payload);
        } else if (kind === 'event' && typeof handlers.onEvent === 'function') {
          handlers.onEvent(deviceId, payload);
        } else if (kind === 'status' && typeof handlers.onStatus === 'function') {
          handlers.onStatus(deviceId, payload);
        }
      } catch (e) {
        log('[mqtt] 处理消息失败: ' + ((e && e.message) || e));
      }
    });

    client.on('error', (err) => {
      lastError = String((err && err.message) || err);
      log('[mqtt] 错误: ' + lastError);
    });

    client.on('close', () => {
      if (connected) log('[mqtt] 连接断开，准备重连');
      connected = false;
      if (!stopped) scheduleRetry();
    });

    client.on('offline', () => {
      connected = false;
      if (!stopped) scheduleRetry();
    });

    return true;
  }

  /** 发布一条消息；未连接时静默丢弃（HTTP 才是主通道） */
  function publish(deviceId, suffix, obj) {
    if (!client || !connected) return false;
    try {
      client.publish(prefix + '/' + String(deviceId) + '/' + suffix, JSON.stringify(obj), { qos: 0 });
      return true;
    } catch (e) {
      lastError = String((e && e.message) || e);
      return false;
    }
  }

  function stop() {
    stopped = true;
    if (retryTimer) {
      clearTimeout(retryTimer);
      retryTimer = null;
    }
    if (client) {
      try { client.end(true); } catch { /* 已经断了 */ }
      client = null;
    }
    connected = false;
  }

  function status() {
    return { enabled, connected, lastError };
  }

  return { start, stop, publish, status, url };
}

module.exports = { createMqttBridge };
