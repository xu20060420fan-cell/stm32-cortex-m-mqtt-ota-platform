/*
 * safeKeys.js —— 原型链污染防护（**本地兜底实现**）
 * ============================================================================
 *
 * 【为什么要有这个文件】
 *   iotCenter.js 原来写的是 require('../safeKeys') —— 那是主后台工程
 *   （iot_service_server）里的公共模块。本服务被单独拆成
 *   backend_iot_service 之后，**上一级目录里可能根本没有 safeKeys.js**，
 *   一 require 就整个服务起不来（MODULE_NOT_FOUND）。
 *   所以 iotCenter.js 改成：先试 '../safeKeys'（主后台有就用主后台那份，
 *   行为完全不变），取不到再用本文件的本地实现兜底。
 *
 * 【本文件只做三件事】
 *   1. isDangerousKey：判断是不是 '__proto__' / 'constructor' / 'prototype'
 *   2. safeGet：只读**自有属性**，原型链上的东西一律读不到
 *   3. safeSet：用 defineProperty 写，挡掉原型链污染
 *
 * 设备 ID、事件类型这些全是外部来的键名，必须走这一套。
 */

'use strict';

const DANGEROUS_KEYS = new Set(['__proto__', 'constructor', 'prototype']);

/** 是不是原型链上的危险键名 */
function isDangerousKey(key) {
  return DANGEROUS_KEYS.has(String(key));
}

/** 安全读取：只认自有属性 */
function safeGet(obj, key) {
  if (!obj || typeof obj !== 'object') return undefined;
  if (isDangerousKey(key)) return undefined;
  if (!Object.prototype.hasOwnProperty.call(obj, key)) return undefined;
  return obj[key];
}

/** 安全写入：绕开 setter，定义成普通的自有可枚举属性 */
function safeSet(obj, key, value) {
  if (!obj || typeof obj !== 'object') return value;
  if (isDangerousKey(key)) return value;
  Object.defineProperty(obj, key, {
    value: value,
    writable: true,
    enumerable: true,
    configurable: true,
  });
  return value;
}

module.exports = {
  isDangerousKey,
  safeGet,
  safeSet,
};
