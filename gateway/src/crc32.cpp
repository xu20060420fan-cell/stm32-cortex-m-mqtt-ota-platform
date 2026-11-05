/*
 * crc32.cpp —— 标准 zlib CRC32，逐位算
 *
 * 【为什么逐位】和 STM32 侧一样：256 项查找表要占 1KB RAM，ESP8266 的 RAM 同样金贵。
 * 44KB 的固件约 36 万次内层迭代，80MHz 下不到 30ms，可以接受。
 *
 * 这个文件刻意不依赖任何 Arduino 头文件 —— 它同时被
 *   - ESP8266 固件（src/）
 *   - PC 离线自测（test/native/）
 * 编译，两边算出来的数必须一模一样。
 */
#include "crc32.h"

#define CRC32_POLY 0xEDB88320UL   /* 0x04C11DB7 的位反转形式 */

uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++) {
        uint8_t bit;

        crc ^= data[i];

        for (bit = 0; bit < 8; bit++) {
            /* (crc & 1) 是 1 时 mask = 0xFFFFFFFF，是 0 时 mask = 0。
             * 这样写没有分支，编译器会生成条件传送指令。 */
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));

            crc = (crc >> 1) ^ (CRC32_POLY & mask);
        }
    }
    return crc;
}

uint32_t crc32Calc(const uint8_t* data, size_t len)
{
    return crc32Update(0xFFFFFFFFU, data, len) ^ 0xFFFFFFFFU;
}
