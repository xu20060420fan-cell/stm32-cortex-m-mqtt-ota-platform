/*
 * netContext.js —— 独立服务（校园物联网中心）的网络上下文工具
 * ---------------------------------------------------------------------------
 * 为什么这里有一份「和主后端一样的」实现：
 *   本模块从 iot_service_server/index.js 原样搬过来，逻辑**逐字一致**。
 *   搬而不是 import，是因为主后端 index.js 有 9200 行、是一整块单体，
 *   把它抽成公共模块要动几十处调用点 ⇒ 拆 IoT 服务这种「本来就独立的活」会被拖成大重构。
 *   ⇒ 代价是「以后改了主后端这里要跟着改」，
 *     对冲手段：tools/check-iot-net-context.cjs 会对两份实现做**行为一致性比对**，
 *     不一致就报错（见该脚本）。
 *
 * 这里的行为为什么重要：
 *   · getClientIP —— 决定限流、留痕、日志里记的是谁的 IP。
 *     ⚠ 绝不能采信客户端能自己伪造的头（X-Forwarded-For），否则限流与锁定全部形同虚设。
 *   · getPublicOrigin —— 决定固件签名下载链接里的域名。
 *     ⚠ 被污染 = 攻击者把自己的域名写进下载链接（用户会去他的站上下固件）。
 *   · adminIpAllowed —— 后台的 IP 白名单（后台口令是公开的，这一层是主要防线）。
 */

'use strict';

const SECURITY = {
  // session 过期（与主后端 index.js 的 SECURITY.SESSION_TTL_MS 一致：7 天）
  SESSION_TTL_MS: 7 * 24 * 60 * 60 * 1000,
};

/*
 * 是否处于「内网/本机」地址 —— 用来判断前面是不是有反向代理。
 * 10.x / 172.16-31.x / 192.168.x / 127.x / ::1 / fc00:: 都算。
 */
function isPrivateAddress(ip) {
  const v = String(ip || '').replace(/^::ffff:/i, '').trim();
  if (!v) return true;
  if (v === '::1' || v === 'localhost') return true;
  if (v.startsWith('fc') || v.startsWith('fd') || v.startsWith('fe80')) return true;
  const m = v.match(/^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/);
  if (!m) return false;
  const [a, b] = [Number(m[1]), Number(m[2])];
  if (a === 10 || a === 127) return true;
  if (a === 172 && b >= 16 && b <= 31) return true;
  if (a === 192 && b === 168) return true;
  return false;
}

/*
 * 反代信任配置
 *   · trustProxy    —— TRUST_PROXY=1/true：老开关，显式声明「我就在反代后面」。
 *   · trustedHeader —— TRUSTED_PROXY_HEADER：我指定了「我的反代会覆盖哪个头」。
 *   · trustedIps    —— TRUSTED_PROXY_IPS：我指定了「我的反代是从哪个 IP 连过来的」。
 */
function proxyTrustConfig() {
  return {
    trustProxy: process.env.TRUST_PROXY === '1' || process.env.TRUST_PROXY === 'true',
    trustedHeader: String(process.env.TRUSTED_PROXY_HEADER || '').trim().toLowerCase(),
    trustedIps: String(process.env.TRUSTED_PROXY_IPS || '')
      .split(',')
      .map((s) => s.trim())
      .filter(Boolean),
  };
}

/*
 * 直连方 IP 是否落在受信任反代白名单里。
 * ⚠ TRUSTED_PROXY_IPS 没配 = 恒 false ——「没配白名单」不能当成「谁都可信」。
 */
function isTrustedProxyIp(direct, trustedIps) {
  return trustedIps.length > 0 && trustedIps.includes(direct);
}

/*
 * 转发头（X-Forwarded-Host / X-Forwarded-Proto）是否可信。
 * 判定顺序：直连方在白名单里 → 老开关 TRUST_PROXY=1 → 兜底不采信。
 */
function isForwardedHeaderTrusted(req) {
  const direct = String((req.socket && req.socket.remoteAddress) || '').trim();
  const { trustProxy, trustedIps } = proxyTrustConfig();
  if (isTrustedProxyIp(direct, trustedIps)) return true;
  if (trustProxy) return true;
  return false;
}

/*
 * 取客户端 IP（用于限流、留痕、日志）
 * 判定顺序（先命中先用）：
 *   ① TRUSTED_PROXY_HEADER=<头名>：采信我自己反代设置的那个头；
 *   ② TRUSTED_PROXY_IPS=<列表>：直连方在列表里 → 采信 X-Forwarded-For / X-Real-IP；
 *   ③ TRUST_PROXY=1：显式声明「我就在反代后面」；
 *   ④ 兜底：socket.remoteAddress（客户端伪造不了）。
 */
function getClientIP(req) {
  const direct = String((req.socket && req.socket.remoteAddress) || '').trim();
  const xff = String((req.headers && req.headers['x-forwarded-for']) || '').split(',')[0].trim();
  const xri = String((req.headers && req.headers['x-real-ip']) || '').trim();
  const { trustProxy, trustedHeader, trustedIps } = proxyTrustConfig();

  if (trustedHeader) {
    const v = String((req.headers && req.headers[trustedHeader]) || '').split(',')[0].trim();
    if (v && (!trustedIps.length || trustedIps.includes(direct))) return v;
  }
  if (isTrustedProxyIp(direct, trustedIps)) {
    return xff || xri || direct || 'unknown';
  }
  if (trustProxy) {
    return xff || xri || direct || 'unknown';
  }
  return direct || 'unknown';
}

/*
 * 请求的公网 origin（拼绝对下载链接用）。
 * ⚠ 只在「转发头可信」时才采信 X-Forwarded-Host / -Proto，否则用 req.get('host')。
 */
function getPublicOrigin(req) {
  const trusted = isForwardedHeaderTrusted(req);
  const proto = String(trusted ? ((req.headers && req.headers['x-forwarded-proto']) || '') : '')
    .split(',')[0].trim() || req.protocol || 'http';
  const host = String(trusted ? ((req.headers && req.headers['x-forwarded-host']) || '') : '')
    .split(',')[0].trim() || (req.get && req.get('host')) || '';
  return host ? `${proto}://${host}` : '';
}

/*
 * 后台面板的 IP 白名单。
 * 语义：未设置 / 空 → 不启用（全部放行）；配了 → 只有列表里的 IP 放行；
 * 私网段永远放行（自家局域网 / SSH 隧道不会被挡住）。
 */
function adminIpAllowed(ip) {
  const raw = String(process.env.ADMIN_IP_ALLOW || '').trim();
  if (!raw) return true;
  const list = raw.split(',').map((s) => s.trim()).filter(Boolean);
  if (list.includes('*')) return true;
  if (isPrivateAddress(String(ip || ''))) return true;
  return list.includes(String(ip || '').trim());
}

/*
 * 把相对地址补成绝对地址（绝对地址原样返回）。
 */
function toAbsoluteUrl(req, rawUrl = '') {
  const value = String(rawUrl || '').trim();
  if (!value) return '';
  if (/^https?:\/\//i.test(value)) return value;
  if (value.startsWith('//')) {
    const proto = String(isForwardedHeaderTrusted(req) ? ((req.headers && req.headers['x-forwarded-proto']) || '') : '')
      .split(',')[0].trim() || req.protocol || 'http';
    return `${proto}:${value}`;
  }
  const origin = getPublicOrigin(req);
  if (!origin) return value;
  return value.startsWith('/') ? `${origin}${value}` : `${origin}/${value}`;
}

module.exports = {
  SECURITY,
  isPrivateAddress,
  proxyTrustConfig,
  isTrustedProxyIp,
  isForwardedHeaderTrusted,
  getClientIP,
  getPublicOrigin,
  adminIpAllowed,
  toAbsoluteUrl,
};
