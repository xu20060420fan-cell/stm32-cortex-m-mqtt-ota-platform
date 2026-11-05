/*
 * ota_proto.cpp —— OTA 协议引擎
 *
 *   只依赖 <stdio.h>/<string.h>，不碰 Arduino / WiFi / 串口。
 *   所有跟"外面打交道"的东西都走 OtaSink / FwSource 两个抽象接口。
 *   于是 test/native 里编译的就是这份代码 —— 协议对不对，PC 上就能验。
 *
 * 【协议时序（抄自 STM32 侧 users/appbl.c）】
 *   上位机  O <len> 0x<crc32> <chunk>\r\n
 *   MCU     擦 W25Q → 回 "READY\r\n"
 *   上位机  发 chunk 字节（裸二进制）
 *   MCU     收够 chunk 字节 → 回 "OK\r\n"
 *   MCU     从 W25Q 读回来算 CRC32 → 比对
 *            失败 → "ER:crc fail ..."（内部 Flash 未动，安全）
 *            通过   → "verify ok\r\n"
 */
#include "ota_proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 单包缓冲：文件级静态，放 .bss，不占 ota_task 的栈。 */
static uint8_t s_chunkBuf[OTA_CHUNK_MAX];

OtaEngine::OtaEngine(OtaSink& sink)
    : sink_(sink),
      tReadyMs_(30000),
      tChunkMs_(10000),
      tVerifyMs_(30000),
      chunk_(OTA_CHUNK_MAX),
      targetSlot_(0),
      targetSlotSet_(false),
      lineCb_(0),
      lineCtx_(0),
      bytesSent_(0),
      chunksSent_(0)
{
    lastError_[0] = '\0';
}

void OtaEngine::setTimeouts(uint32_t readyMs, uint32_t chunkMs, uint32_t verifyMs)
{
    tReadyMs_ = readyMs;
    tChunkMs_ = chunkMs;
    tVerifyMs_ = verifyMs;
}

void OtaEngine::setChunkSize(uint32_t chunk)
{
    /* MCU 侧 clamp_chunk() 的规则是：0 或超过 RX_CHUNK_MAX 一律退回 512。
     * 我们这边必须跟它一模一样地收敛，否则双方分帧大小不同，全部错位。 */
    if (chunk == 0 || chunk > OTA_CHUNK_MAX) {
        chunk = OTA_CHUNK_MAX;
    }
    chunk_ = chunk;
}

void OtaEngine::setTargetSlot(uint32_t slot)
{
    targetSlot_    = slot;
    targetSlotSet_ = true;
}

void OtaEngine::clearTargetSlot()
{
    targetSlot_    = 0;
    targetSlotSet_ = false;
}

void OtaEngine::setLineCallback(void (*cb)(const char*, void*), void* ctx)
{
    lineCb_ = cb;
    lineCtx_ = ctx;
}

bool OtaEngine::sendLine(const char* text)
{
    char buf[64];
    size_t n = strlen(text);
    size_t total;

    if (n + 2 > sizeof(buf)) {
        return false;
    }
    memcpy(buf, text, n);
    buf[n]     = '\r';
    buf[n + 1] = '\n';
    total = n + 2;

    /* 【行尾用 \r\n】跟 STM32 侧上位机 tools/firmware_tool.py 的 _send() 一致：
     *   self.link.write((text + '\r\n').encode('ascii'))
     * STM32 的 wait_command() 认 \r 或 \n 任意一个做行尾，两者都行，
     * 但保持一致最保险。 */
    return sink_.write((const uint8_t*)buf, total) == total;
}

/* 去掉行首尾的空白和行尾符。
 * 【为什么连 '\n' 也要去掉】
 *   MCU 的输出层把 '\n' 补成 "\r\n"（appbl.c 的 bl_puts）。
 *   ESP8266 那边的 uart_task 收行时已经把行尾吃掉了，
 *   但协议引擎不该依赖这一点 —— 拿到什么行都得能正确比对。
 */
static void trimLine(char* s)
{
    size_t n;
    size_t start = 0;

    while (*s == ' ' || *s == '\t') {
        s++;
        start++;
    }
    if (start > 0) {
        memmove(s - start, s, strlen(s) + 1);
    }

    n = strlen(s);
    while (n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n' ||
                     s[n - 1] == ' '  || s[n - 1] == '\t')) {
        s[--n] = '\0';
    }
}

/* 一直读到某一行等于 expect 为止。
 *   - 读到 "ER..."   → 立刻失败，把原文存进 lastError_
 *   - 读到等于 expect → 成功
 *   - 其它行         → 当噪声跳过（横幅、"erasing internal flash..." 之类）
 * 一行都没等到就超时 → 返回对应的 *_TIMEOUT。
 */
OtaStatus OtaEngine::waitForLine(const char* expect, uint32_t timeoutMs,
                                 char* lineOut, size_t cap)
{
    char line[OTA_LINE_MAX];
    uint32_t waited = 0;

    if (cap == 0) {
        cap = sizeof(line);
    }
    lineOut[0] = '\0';

    /* 每轮等 200ms 就重新看一眼 elapsed，
     * 这样即使对端一个字都不发，我们也能准时放弃而不是卡死。 */
    while (waited < timeoutMs) {
        uint32_t slice = timeoutMs - waited;

        if (slice > 200) {
            slice = 200;
        }

        if (!sink_.readLine(line, sizeof(line), slice)) {
            waited += slice;
            continue;               /* 这一段没读到行，继续等 */
        }

        trimLine(line);

        if (line[0] == '\0') {
            continue;               /* 空行，忽略 */
        }

        if (lineCb_ != 0) {
            lineCb_(line, lineCtx_);
        }

        /* MCU 的所有失败路径都以 "ER" 开头（appbl.c 里统一用 ER:<原因>） */
        if (strncmp(line, "ER", 2) == 0) {
            size_t n = strlen(line);
            if (n >= sizeof(lastError_)) {
                n = sizeof(lastError_) - 1;
            }
            memcpy(lastError_, line, n);
            lastError_[n] = '\0';
            strncpy(lineOut, line, cap - 1);
            lineOut[cap - 1] = '\0';
            return OTA_ERR_MCU;
        }

        if (strcmp(line, expect) == 0) {
            strncpy(lineOut, line, cap - 1);
            lineOut[cap - 1] = '\0';
            return OTA_OK;
        }

        /* 噪声行：记到 lineOut 让调用方知道最后看到了什么，但不中断流程 */
        strncpy(lineOut, line, cap - 1);
        lineOut[cap - 1] = '\0';
    }

    if (strcmp(expect, "READY") == 0) {
        return OTA_ERR_READY_TIMEOUT;
    }
    if (strcmp(expect, "OK") == 0) {
        return OTA_ERR_CHUNK_TIMEOUT;
    }
    return OTA_ERR_VERIFY_TIMEOUT;
}

OtaStatus OtaEngine::run(FwSource& src, uint32_t totalLen, uint32_t crc32,
                         const char* ver)
{
    char cmd[96];
    char line[OTA_LINE_MAX];
    uint32_t sent = 0;
    OtaStatus st;

    if (totalLen == 0) {
        return OTA_ERR_PARAM;
    }
    if (chunk_ == 0 || chunk_ > OTA_CHUNK_MAX) {
        chunk_ = OTA_CHUNK_MAX;
    }

    lastError_[0] = '\0';

    /* 发新命令之前先把残留输入丢掉：
     * 上一次命令的尾巴（比如 ? 的最后几行）如果留在流里，
     * 会被当成这一次命令的应答，协议就错位了。 */
    sink_.flushInput();

    /* ---- 第一步：发 O 命令 ----
     * ver 非空 → 新格式 O <slot> <len> <crc32> <ver> <chunk>（bootloader v2.0）。
     * 先发 ? 查 ACTIVE，把固件推到"非激活槽"—— 不动正在运行的版本，
     * 版本号随 O 登记进槽位元数据（网页上 APPVER 就有值了）。
     * ver 为空 → 旧格式 O <len> <crc32> <chunk>（MCU 自动选槽，版本记 0.0.0）。 */
    if (ver != NULL && ver[0] != '\0') {
        char info[192];
        uint32_t slot = 0;
        if (targetSlotSet_) {
            /* 续传：槽必须跟断点记录一致，bootloader 才认 RESUME */
            slot = targetSlot_;
        } else if (queryDeviceInfo(info, sizeof(info), 5000UL)) {
            const char* p = strstr(info, "ACTIVE=");
            if (p != NULL) {
                if (p[7] == '0') slot = 1;      /* ACTIVE=0 → 推到槽 1 */
                else if (p[7] == '1') slot = 0; /* ACTIVE=1 → 推到槽 0 */
                else slot = 0;                  /* 无记录/解析不到 → 槽 0 */
            }
        }
        int n = snprintf(cmd, sizeof(cmd), "O %lu %lu 0x%08lX %s %lu",
                         (unsigned long)slot,
                         (unsigned long)totalLen,
                         (unsigned long)crc32,
                         ver,
                         (unsigned long)chunk_);
        if (n <= 0 || (size_t)n >= sizeof(cmd)) {
            return OTA_ERR_PARAM;
        }
    } else {
        int n = snprintf(cmd, sizeof(cmd), "O %lu 0x%08lX %lu",
                         (unsigned long)totalLen,
                         (unsigned long)crc32,
                         (unsigned long)chunk_);
        if (n <= 0 || (size_t)n >= sizeof(cmd)) {
            return OTA_ERR_PARAM;
        }
    }
    if (!sendLine(cmd)) {
        return OTA_ERR_LINK;
    }
    bytesSent_ += (uint32_t)strlen(cmd) + 2;

    /* ---- 第二步：等 READY（全新传输）或 RESUME（断点续传） ----
     * 全新传输：MCU 擦槽（16 个 4K 扇区，要好几秒）后回 READY，从 0 开始发。
     * 断点续传：bootloader v2.0 比对 O 里的指纹（槽/大小/CRC），与断点一致时
     *   回 "RESUME <written> <chunk>"——不擦，从 written 接着发，
     *   并且必须沿用 MCU 记录的 chunk（它记的是上次断线时的分帧）。
     * 这里的循环不能直接用 waitForLine：两种合法应答都要接住。 */
    {
        uint32_t waited = 0;
        bool settled = false;
        while (waited < tReadyMs_) {
            uint32_t slice = tReadyMs_ - waited;
            if (slice > 200) slice = 200;

            if (!sink_.readLine(line, sizeof(line), slice)) {
                waited += slice;
                continue;
            }
            trimLine(line);
            if (line[0] == '\0') continue;
            if (lineCb_ != 0) lineCb_(line, lineCtx_);

            if (strncmp(line, "ER", 2) == 0) {
                size_t n = strlen(line);
                if (n >= sizeof(lastError_)) n = sizeof(lastError_) - 1;
                memcpy(lastError_, line, n);
                lastError_[n] = '\0';
                return OTA_ERR_MCU;
            }
            if (strcmp(line, "READY") == 0) {
                settled = true;                 /* 全新传输，sent 保持 0 */
                break;
            }
            if (strncmp(line, "RESUME", 6) == 0) {
                /* RESUME <written> <chunk>
                 * 【written 是字节数，不是包号】
                 *   断点位置直接拿来 seek，前面的字节一个都不重发。 */
                long written = strtol(line + 6, NULL, 10);
                const char* sp = strchr(line + 6, ' ');
                if (sp != NULL) {
                    long mc = strtol(sp + 1, NULL, 10);
                    if (mc > 0 && mc <= OTA_CHUNK_MAX) {
                        chunk_ = (uint32_t)mc;  /* 必须沿用 MCU 的分帧 */
                    }
                }
                if (written < 0 || (uint32_t)written > totalLen) {
                    /* 断点比整包还大：本地文件被换过/截断了，
                     * 接着发只会把错的字节喂进去 —— 放弃续传。 */
                    snprintf(lastError_, sizeof(lastError_), "ER:resume out of range");
                    return OTA_ERR_MCU;
                }
                if (written > 0 && written < (long)totalLen) {
                    if (!src.seek((uint32_t)written)) {
                        /* 来源不能定位（不该发生：文件来源都支持 seek） */
                        snprintf(lastError_, sizeof(lastError_), "ER:source no seek");
                        return OTA_ERR_MCU;
                    }
                    sent = (uint32_t)written;
                } else if ((uint32_t)written == totalLen) {
                    sent = totalLen;            /* 收完了还没校验 → 直接去等 verify */
                }
                settled = true;
                break;
            }
            /* 其它行（横幅、日志）：当噪声跳过 */
        }
        if (!settled) {
            return OTA_ERR_READY_TIMEOUT;
        }
    }

    /* ---- 第三步：逐包发数据，每包等一个 OK ----
     * 【铁律】一包 = chunk 字节，发完必须等一个 OK 才能发下一包。
     * 绝对不能靠延时来"对齐"——分帧靠的是长度，不是时间。 */
    while (sent < totalLen) {
        uint32_t want = totalLen - sent;
        size_t got;

        if (want > chunk_) {
            want = chunk_;
        }

        got = src.read(s_chunkBuf, want);
        if (got == 0) {
            /* 来源提前读完了：实际拿到的字节数比声明的少，
             * MCU 那边会一直等剩下的字节，最后超时。 */
            return OTA_ERR_SHORT;
        }
        if (got > want) {
            got = want;
        }

        if (sink_.write(s_chunkBuf, got) != got) {
            return OTA_ERR_LINK;
        }
        bytesSent_ += (uint32_t)got;

        st = waitForLine("OK", tChunkMs_, line, sizeof(line));
        if (st != OTA_OK) {
            return st;
        }
        chunksSent_++;
        sent += (uint32_t)got;
    }

    /* ---- 第四步：等 verify ok ----
     * MCU 从 W25Q 把整个镜像读回来算 CRC32，跟 O 命令里声明的比对。
     * 通过才打 "verify ok"（appbl.c 里那行注释写着"上位机就在等这一行，不能改"）。
     * 不通过会打 "ER:crc fail ..."，上面 waitForLine 已经把它变成 OTA_ERR_MCU 了。 */
    return waitForLine("verify ok", tVerifyMs_, line, sizeof(line));
}

bool OtaEngine::queryDeviceInfo(char* out, size_t cap, uint32_t timeoutMs)
{
    char line[OTA_LINE_MAX];
    size_t used = 0;
    uint32_t waited = 0;

    if (cap == 0) {
        return false;
    }
    out[0] = '\0';

    sink_.flushInput();

    if (!sendLine("?")) {
        return false;
    }

    /* ? 的应答是：OK + 若干 KV 行 + END
     * 【必须一直读到 END】读到一半就返回，剩下的行会留在流里，
     * 把后面 O 命令的应答搞乱。 */
    while (waited < timeoutMs) {
        uint32_t slice = timeoutMs - waited;

        if (slice > 200) {
            slice = 200;
        }

        if (!sink_.readLine(line, sizeof(line), slice)) {
            waited += slice;
            continue;
        }

        trimLine(line);

        if (line[0] == '\0') {
            continue;
        }
        if (lineCb_ != 0) {
            lineCb_(line, lineCtx_);
        }
        if (strncmp(line, "ER", 2) == 0) {
            return false;
        }
        if (strcmp(line, "END") == 0) {
            return true;
        }
        if (strcmp(line, "OK") == 0) {
            continue;               /* 多行应答的第一行，不收 */
        }

        /* 把 KV 行拼进 out，用 '\n' 分隔 */
        {
            size_t n = strlen(line);
            if (used + n + 2 >= cap) {
                break;              /* 装不下了，截断返回已经拿到的 */
            }
            memcpy(out + used, line, n);
            used += n;
            out[used++] = '\n';
            out[used]   = '\0';
        }
    }
    return false;
}

/* ================================================================
 * 断点续传信息（Q 命令）
 *
 * bootloader 的 cmd_prog() 回三行：
 *   OK
 *   PROG=SLOT=0,SIZE=1234,CRC=0x89ABCDEF,WRITTEN=512,NEXTSEQ=1,CHUNK=512
 *   END
 * 没有断点时中间那行是 "PROG=none"。
 * ================================================================ */

/* 从 "KEY=VALUE,KEY=VALUE,..." 里取某个键的值。
 * base=10 取十进制（SIZE/WRITTEN/NEXTSEQ/CHUNK/SLOT），
 * base=16 取十六进制（CRC 是 0x%08X）。
 * 找不到返回 false。 */
static bool progGetU32(const char* s, const char* key, uint32_t* out, int base)
{
    size_t klen = strlen(key);
    const char* p = s;

    if (s == NULL || out == NULL) {
        return false;
    }

    while (*p != '\0') {
        const char* segEnd = strchr(p, ',');
        size_t segLen = segEnd ? (size_t)(segEnd - p) : strlen(p);

        if (segLen > klen && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            char tmp[24];
            size_t vlen = segLen - klen - 1;
            if (vlen >= sizeof(tmp)) {
                vlen = sizeof(tmp) - 1;
            }
            memcpy(tmp, p + klen + 1, vlen);
            tmp[vlen] = '\0';
            *out = (uint32_t)strtoul(tmp, NULL, base);
            return true;
        }
        if (segEnd == NULL) {
            break;
        }
        p = segEnd + 1;
    }
    return false;
}

bool OtaEngine::queryResumeInfo(OtaResumeInfo* out, uint32_t timeoutMs)
{
    char line[OTA_LINE_MAX];
    uint32_t waited = 0;
    bool got = false;

    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->slot = 0xFFFFFFFFUL;      /* 跟 bootloader 的"无槽"表示一致 */

    sink_.flushInput();
    if (!sendLine("Q")) {
        return false;
    }

    while (waited < timeoutMs) {
        uint32_t slice = timeoutMs - waited;

        if (slice > 200) {
            slice = 200;
        }
        if (!sink_.readLine(line, sizeof(line), slice)) {
            waited += slice;
            continue;
        }

        trimLine(line);
        if (line[0] == '\0') {
            continue;
        }
        if (lineCb_ != 0) {
            lineCb_(line, lineCtx_);
        }
        if (strncmp(line, "ER", 2) == 0) {
            return false;
        }
        if (strcmp(line, "END") == 0) {
            break;                  /* Q 的应答到 END 为止 */
        }
        if (strncmp(line, "PROG=", 5) == 0) {
            const char* kv = line + 5;
            if (strcmp(kv, "none") == 0) {
                out->valid = false; /* 没有断点 —— 这不是错误 */
                got = true;
                break;
            }
            {
                uint32_t v = 0;
                if (progGetU32(kv, "SLOT", &v, 10))    out->slot    = v;
                if (progGetU32(kv, "SIZE", &v, 10))    out->size    = v;
                if (progGetU32(kv, "CRC", &v, 16))     out->crc     = v;
                if (progGetU32(kv, "WRITTEN", &v, 10)) out->written = v;
                if (progGetU32(kv, "NEXTSEQ", &v, 10)) out->nextSeq = v;
                if (progGetU32(kv, "CHUNK", &v, 10))   out->chunk   = v;

                /* 指纹字段（SIZE/CRC）齐了才算一条能用的断点记录 */
                if (progGetU32(kv, "SIZE", &v, 10) && progGetU32(kv, "CRC", &v, 16)) {
                    out->valid = true;
                    got = true;
                }
            }
            break;
        }
        /* 其它行（OK 之类）当噪声跳过 */
    }
    return got;
}
