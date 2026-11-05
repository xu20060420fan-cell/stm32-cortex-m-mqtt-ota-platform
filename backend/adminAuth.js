/*
 * adminAuth.js —— 校园物联网独立服务的**后台鉴权**
 * ---------------------------------------------------------------------------
 * 目标：**沿用主后台（8080）那一个账号、那一套登录态**，不新增第二套口令。
 *
 * 怎么做到的：
 *   · 主后台登录接口 POST /admin/api/login 签发的 sessionId，
 *     存在 data/iot_backend/admin-sessions.json 里；
 *   · 后台网页 / 物联网后台页都把 sessionId 放在 `x-admin-token` 头里；
 *   ⇒ 本模块只要**读同一份会话表**并按同一套规则判定，就等于共用登录态。
 *
 * ⚠ 为什么读**文件**而不是查 PostgreSQL：
 *   线上 STORAGE_DRIVER=pg，但 campus_server/db/store.cjs 的 readJSON 是
 *   「**先读文件**、缓存命中且 mtime 未变才复用；PG 只是异步写回的投影层」。
 *   ⇒ 权威存储始终是文件，本模块直读文件 = 与主后端看到的**完全一致**，
 *     而且独立服务**不需要连数据库**（少一个依赖、少一个故障点）。
 *
 * ⚠ 为什么**不写**这个文件：
 *   主后端与本服务是两个进程，同时写同一个 JSON 会互相覆盖（丢会话）。
 *   ⇒ 本模块**只读**。过期会话只在本进程内判为无效，不落盘清理
 *     （清理交给主后端 —— 它是会话的唯一写入方）。
 *
 * 判定顺序与主后端 requireAdmin 完全一致：
 *   ① IP 白名单（ADMIN_IP_ALLOW）：不通过 → 403
 *   ② 有没有 x-admin-token、会话表里有没有这条 → 否则 401
 *   ③ 会话是否过期（距上次活跃超过 SESSION_TTL_MS）→ 401
 */

'use strict';

const fs = require('fs');
const path = require('path');

const { SECURITY, getClientIP, adminIpAllowed } = require('./netContext');

/* ── 会话表读取 ───────────────────────────────────────────────────── */

/** 去掉 UTF-8 BOM */
function stripBom(text) {
  return String(text || '').replace(/^\uFEFF/, '');
}

/**
 * 读会话表（每次调用都真读盘，不缓存）。
 * 损坏时从 .bak 恢复（**只读恢复，不回写** —— 回写会和主后端抢同一个文件）。
 */
function readSessions(file) {
  try {
    if (!fs.existsSync(file)) return {};
    const raw = stripBom(fs.readFileSync(file, 'utf8'));
    const parsed = JSON.parse(raw);
    return (parsed && typeof parsed === 'object' && !Array.isArray(parsed)) ? parsed : {};
  } catch (e) {
    try {
      const bak = file + '.bak';
      if (fs.existsSync(bak)) {
        const parsed = JSON.parse(stripBom(fs.readFileSync(bak, 'utf8')));
        return (parsed && typeof parsed === 'object' && !Array.isArray(parsed)) ? parsed : {};
      }
    } catch (e2) { /* 备份也坏了 → 当作没有会话 */ }
    return {};
  }
}

/**
 * 校验后台会话。
 * @param {object} req  Express 请求
 * @param {string} sessionsFile  会话表路径
 * @returns {{ok:true}|{ok:false, status:number, message:string}}
 */
function verifyAdminSession(req, sessionsFile) {
  const clientIp = getClientIP(req);
  if (!adminIpAllowed(clientIp)) {
    return { ok: false, status: 403, message: '当前 IP 不在后台白名单内' };
  }
  const token = String((req.headers && req.headers['x-admin-token']) || '').trim();
  if (!token) {
    return { ok: false, status: 401, message: '未登录或会话已失效' };
  }
  const sessions = readSessions(sessionsFile);
  const session = sessions[token];
  if (!session || typeof session !== 'object') {
    return { ok: false, status: 401, message: '未登录或会话已失效' };
  }
  const nowMs = Date.now();
  const lastActive = session.lastActive
    ? new Date(session.lastActive).getTime()
    : new Date(session.createdAt || 0).getTime();
  if (nowMs - lastActive > SECURITY.SESSION_TTL_MS) {
    // ⚠ 只拒绝、**不删**：删除要写文件，会和主后端抢同一个 JSON。
    return { ok: false, status: 401, message: '会话已过期，请重新登录后台' };
  }
  return { ok: true, token, session };
}

/**
 * 生成 Express 中间件形式的 requireAdmin。
 * @param {object} options
 * @param {string} options.sessionsFile  会话表路径
 * @param {Function} [options.onDenied]  被拒绝时的回调（用于留痕）
 */
function createRequireAdmin(options) {
  const sessionsFile = options.sessionsFile;
  return function requireAdmin(req, res, next) {
    const result = verifyAdminSession(req, sessionsFile);
    if (result.ok) {
      req.adminSession = result.session;
      req.adminToken = result.token;
      return next();
    }
    if (typeof options.onDenied === 'function') {
      try {
        options.onDenied({ req, status: result.status, message: result.message });
      } catch (e) { /* 留痕失败不阻塞 */ }
    }
    return res.status(result.status).json({ success: false, message: result.message });
  };
}

module.exports = {
  stripBom,
  readSessions,
  verifyAdminSession,
  createRequireAdmin,
  SESSIONS_FILE_NAME: 'admin-sessions.json',
  resolveSessionsFile: (dataDir) => path.join(dataDir, 'admin-sessions.json'),
};
