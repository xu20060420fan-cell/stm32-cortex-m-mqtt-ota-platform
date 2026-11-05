/*
 * ota_proto.h —— STM32 bootloader 的 ASCII OTA 协议引擎
 *
 * ================================================================
 * 一、这一层为什么单独抽出来
 * ================================================================
 * 本文件不依赖 Arduino / WiFi / 任何串口库，只依赖两个抽象接口：
 *
 *   OtaSink   —— "把字节发给 MCU" / "从 MCU 读一行"
 *   FwSource  —— "从固件来源取下一块数据"
 *
 * 于是同一份协议代码可以跑在两个地方：
 *   1) ESP8266 上：OtaSink = SoftwareSerial，FwSource = LittleFS 文件
 *   2) PC 自测  ：OtaSink = TCP socket，FwSource = 内存
 *
 * 第 2 种方式让我们在**没有硬件**的情况下也能证明协议对得上：
 * test/native 里编译的就是这份 ota_proto.cpp，一个字节都不用改。
 *
 * ================================================================
 * 二、协议原文（抄自 STM32 侧 users/appbl.c，不是猜的）
 * ================================================================
 *   上位机  O <len> 0x<crc32> <chunk>\r\n
 *   MCU     擦 W25Q → 回 "READY\r\n"
 *   上位机  发 chunk 字节（裸二进制，不带任何包头）
 *   MCU     收够 chunk 字节 → 回 "OK\r\n"     ← 一包一个 OK，绝不会多
 *   MCU     从 W25Q 读回来算 CRC32 → 比对
 *            不通过 → "ER:crc fail calc=0x.. recv=0x..\r\n"
 *                     （此时内部 Flash 一个字节都没动，可以安全重来）
 *            通过   → "verify ok\r\n"
 *
 * MCU 后续还会打 "backing up app..." / "backup ok" / "erasing internal flash..."，
 * 成功搬完内部 Flash 之后直接跳 app，不再有输出。
 *
 * ================================================================
 * 三、铁律：分帧靠"每包字节数"，绝对不靠时间
 * ================================================================
 * 上位机必须在 O 命令里声明 chunk，MCU 才会按 chunk 攒够才回一个 OK。
 * 绝对不能"发固定 512 字节然后 sleep 一下等应答"——
 * USB 串口芯片会把一次 write 切成好几段，老实现就是死在这里。
 * 对应的 MCU 侧说明在 users/bootloader.h 的 RX_RING_SIZE 注释里。
 */
#ifndef OTA_PROTO_H
#define OTA_PROTO_H

#include <stddef.h>
#include <stdint.h>

/* 单行缓冲最大长度（含结尾 '\0'） */
#ifndef OTA_LINE_MAX
#define OTA_LINE_MAX 72
#endif

/* MCU 单包上限。bootloader.h: RX_CHUNK_MAX = uart_rec_len = 512 */
#ifndef OTA_CHUNK_MAX
#define OTA_CHUNK_MAX 512
#endif

/* 与 STM32 对话的抽象。
 * ESP8266 上是 SoftwareSerial；PC 自测上是 TCP socket。 */
class OtaSink {
public:
    virtual ~OtaSink() {}

    /* 发字节。返回实际发出的字节数；短写视为失败。 */
    virtual size_t write(const uint8_t* data, size_t len) = 0;

    /* 读一行（去掉行尾的 \r\n）。超时返回 false。
     * out 一定以 '\0' 结尾；超长的行会被截断（宁可截断也不能越界）。 */
    virtual bool readLine(char* out, size_t cap, uint32_t timeoutMs) = 0;

    /* 丢掉还没处理的输入。发新命令前调用，
     * 免得上一条命令的残留应答把这一条的对上号。 */
    virtual void flushInput() = 0;
};

/* 固件来源抽象：最多取 max 字节，返回实际取到的字节数；返回 0 表示读完。
 * seek 用于断点续传：MCU 回 RESUME <written> 时从 written 处接着读。
 * 来源不支持 seek（返回 false）时续传不可用，引擎会当一次失败重来。 */
class FwSource {
public:
    virtual ~FwSource() {}
    virtual size_t read(uint8_t* buf, size_t max) = 0;
    virtual bool seek(uint32_t pos) { (void)pos; return false; }
};

/* 断点续传信息（bootloader 的 Q 命令）。
 * 【bootloader 的原文】
 *   PROG=SLOT=0,SIZE=1234,CRC=0x89ABCDEF,WRITTEN=512,NEXTSEQ=1,CHUNK=512
 *   没有断点时回 "PROG=none"。
 * 【SIZE/CRC 是"指纹"】
 *   拿它跟手里要传的固件比：一致 = 同一份文件，可以接着传；
 *   不一致 = 换了固件，bootloader 会自己擦槽重传。 */
struct OtaResumeInfo {
    bool     valid;     /* true = 解析出了一条有效的断点记录 */
    uint32_t slot;      /* 断点所在槽号 */
    uint32_t size;      /* 那次传输的总长度（字节） */
    uint32_t crc;       /* 那次传输的 CRC32 */
    uint32_t written;   /* 已经写进去的字节数（字节，不是包号） */
    uint32_t nextSeq;   /* 下一包的序号 = written / chunk */
    uint32_t chunk;     /* 那次传输用的分包大小 */
};

/* 协议执行结果 */
enum OtaStatus {
    OTA_OK = 0,
    OTA_ERR_PARAM,          /* 参数非法（长度 0、分包超上限等） */
    OTA_ERR_LINK,           /* 写串口失败 */
    OTA_ERR_READY_TIMEOUT,  /* 等 READY 超时 */
    OTA_ERR_CHUNK_TIMEOUT,  /* 等每包一个 OK 超时 */
    OTA_ERR_VERIFY_TIMEOUT, /* 等 verify ok 超时 */
    OTA_ERR_MCU,            /* MCU 回 ER:xxx，原因在 lastError() */
    OTA_ERR_SHORT           /* 来源数据比声明的长度少 */
};

class OtaEngine {
public:
    explicit OtaEngine(OtaSink& sink);

    void setTimeouts(uint32_t readyMs, uint32_t chunkMs, uint32_t verifyMs);

    /* 每包字节数。大于 OTA_CHUNK_MAX 会被夹到 OTA_CHUNK_MAX ——
     * MCU 那边 clamp_chunk() 会把超限的值退回 512，双方一旦不一致就全错位。 */
    void setChunkSize(uint32_t chunk);
    uint32_t chunkSize() const { return chunk_; }

    /* 指定本次要推到哪个槽。
     * 【为什么需要它】断点续传时 bootloader 只在"O 命令里的槽"和
     *   断点记录的槽**一致**时才回 RESUME（appbl.c 的 ota_begin）。
     *   默认槽是"非激活槽"（1 - ACTIVE），跟断点记录的槽可能不是一回事，
     *   所以续传前要用 queryResumeInfo() 查到断点的槽并 setTargetSlot()。 */
    void setTargetSlot(uint32_t slot);
    void clearTargetSlot();

    /* 每读到一行都回调（横幅、进度提示、错误行都在里面）。只用于打日志。 */
    void setLineCallback(void (*cb)(const char* line, void* ctx), void* ctx);

    /* 主流程：
     *   发 O → 等 READY（或 RESUME 续传）→ 逐包发数据（每包等一个 OK）→ 等 verify ok
     * 返回 OTA_OK 表示整条链路走通。
     *
     * ver 非空时走 bootloader v2.0 的新格式 "O <slot> <len> <crc32> <ver> <chunk>"：
     *   引擎先发 ? 查 ACTIVE，把固件推到"非激活槽"，版本号会登记进槽位元数据。
     * ver 为空时走旧格式 "O <len> <crc32> <chunk>"（MCU 自动选非激活槽，版本记 0.0.0）。
     *
     * 【断点续传】MCU 发现与断点记录同指纹（槽/大小/CRC 一致）时回
     *   "RESUME <written> <chunk>"：不擦槽，src.seek(written) 后接着发，
     *   且必须沿用 MCU 记录的 chunk。
     *   written 是**字节数**（不是包号）：seek 到 written 处接着发，
     *   前面的字节一个都不重发。 */
    OtaStatus run(FwSource& src, uint32_t totalLen, uint32_t crc32,
                  const char* ver = nullptr);

    /* 读设备信息（? 命令）：
     *   发 "?"，一直读到 "END"，把中间的 KV 行用 '\n' 拼进 out。
     *   用途：拿 MAXCHUNK（真实分包上限），顺带确认对面确实是 bootloader。 */
    bool queryDeviceInfo(char* out, size_t cap, uint32_t timeoutMs);

    /* 查断点续传进度（Q 命令）：
     *   发 "Q"，读到 "END" 为止，把 "PROG=..." 那一行解析进 out。
     *   没有断点（PROG=none）时 out->valid = false，返回 true。
     *   MCU 不认这个命令 / 超时 → 返回 false。 */
    bool queryResumeInfo(OtaResumeInfo* out, uint32_t timeoutMs);

    /* MCU 回的最后一条错误行原文（"ER:..."），没有则为空串。 */
    const char* lastError() const { return lastError_; }

    uint32_t bytesSent() const { return bytesSent_; }
    uint32_t chunksSent() const { return chunksSent_; }

private:
    /* 一直读到某一行等于 expect 为止。
     *   读到 "ER..." → 立刻返回 OTA_ERR_MCU，并把原文存进 lastError_
     *   其它行      → 当噪声跳过（横幅、"erasing internal flash..." 之类）
     * 一次行都没等到就超时 → 返回对应的 *_TIMEOUT。 */
    OtaStatus waitForLine(const char* expect, uint32_t timeoutMs, char* lineOut, size_t cap);

    /* 发一行 ASCII 命令（自动补 \r\n）。 */
    bool sendLine(const char* text);

    OtaSink&  sink_;
    uint32_t  tReadyMs_;
    uint32_t  tChunkMs_;
    uint32_t  tVerifyMs_;
    uint32_t  chunk_;
    uint32_t  targetSlot_;
    bool      targetSlotSet_;
    void    (*lineCb_)(const char*, void*);
    void*     lineCtx_;
    char      lastError_[OTA_LINE_MAX];
    uint32_t  bytesSent_;
    uint32_t  chunksSent_;
};

#endif /* OTA_PROTO_H */
