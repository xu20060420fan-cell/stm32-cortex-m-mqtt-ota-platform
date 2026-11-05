/*
 * crc32.h —— 标准 zlib CRC32（和 STM32 侧 users/crc32.c 完全一致）
 *
 * 【为什么自己写而不是用 zlib 的 crc32()】
 *   两端必须算出**同一个数**。STM32 那边是按位算的（为了省 1KB flash 的查找表），
 *   这里照抄同样的算法：初值 0xFFFFFFFF、逐位、多项式 0xEDB88320、最后异或 0xFFFFFFFF。
 *   按位法和查表法的结果完全一致，所以两边对得上。
 */
#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

/* 续算：crc 传进传出的就是中间态（0xFFFFFFFF 起算、还没做最后异或的那个值） */
uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len);

/* 一次算完：返回最终 CRC32（已做最后异或），可以直接填进 O 命令 */
uint32_t crc32Calc(const uint8_t* data, size_t len);

#endif /* CRC32_H */
