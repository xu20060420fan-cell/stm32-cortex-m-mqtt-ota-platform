/*
 * crc32.c —— 标准 zlib CRC32
 *
 * 【为什么不用 256 项查找表】
 * 那张表要占 1024 字节的 .data —— 既占 flash（要存初值）又占 RAM
 * （启动时从 flash 拷过去），20K 分区里这 1KB 很贵。
 * 改成逐位算：
 *   44KB 的 app 约 36 万次内层迭代，72MHz 下不到 30ms，
 *   对 bootloader 来说完全可接受，换来 1KB flash + 1KB RAM。
 *
 * 逐位算法就是查表法的展开，结果完全一致：
 *   crc ^= 字节;
 *   重复 8 次：crc 最低位是 1 → 右移后异或多项式；是 0 → 只右移
 */

#include "crc32.h"

#define CRC32_POLY 0xEDB88320UL     /* 0x04C11DB7 的位反转形式 */

uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		crc ^= data[i];

		for (uint8_t bit = 0; bit < 8; bit++) {
			/* (crc & 1) 是 1 时 mask = 0xFFFFFFFF，是 0 时 mask = 0。
			 * 这样写没有分支，编译器会生成条件传送指令 */
			uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));

			crc = (crc >> 1) ^ (CRC32_POLY & mask);
		}
	}
	return crc;
}

uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
	return crc32_update(0xFFFFFFFFU, data, len) ^ 0xFFFFFFFFU;
}
