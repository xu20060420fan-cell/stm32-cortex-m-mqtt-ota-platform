/*
 * app_version.c
 *
 * APP 版本号实现 + 给 bootloader 读的固定地址版本标签。
 *
 * 版本标签 .app_tag 落在链接脚本里指定的位置（FLASH ORIGIN + 固定偏移），
 * 里面是 2 个 32 位字：
 *     [0] = APP_TAG_MAGIC  ('APPT' 小端)
 *     [1] = APP_VERSION_PACKED
 * bootloader 只要读那 8 个字节就知道当前 app 是哪个版本。
 */
#include "app_version.h"

/* 版本字符串常量放 Flash（.rodata），不用 sprintf */
static const char s_app_version_str[] = APP_VERSION_STRING;

const char *app_version_str(void)
{
    return s_app_version_str;
}

/* 固定位置的版本标签：链接脚本把它放到 FLASH 里的固定偏移处。
 * used：防止被 --gc-sections 当成没用的东西丢掉。 */
__attribute__((section(".app_tag"), used))
const uint32_t app_tag[2] = { APP_TAG_MAGIC, APP_VERSION_PACKED };
