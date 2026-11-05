/*
 * crc32.h
 *
 * 标准 CRC32（zlib 同款）：多项式 0xEDB88320，初值 0xFFFFFFFF，结果取反。
 * 和 Python 的 zlib.crc32() 完全一致，上位机可以直接对比。
 */

#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

/* 【流式接口】分块计算时用这个。
 * 第一块传初值 0xFFFFFFFF，后续把上一次的返回值传回来，
 * 最后一块算完再 ^ 0xFFFFFFFF 就是最终结果。 */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);

/* 一次性算完（= crc32_update(0xFFFFFFFF,...) ^ 0xFFFFFFFF） */
uint32_t crc32_calc(const uint8_t *data, uint32_t len);

#endif
