# STM32 Cortex-M + MQTT + OTA 物联网环境监测平台

一套从设备端到云端的完整链路：STM32F103 采集温湿度、气压、姿态并驱动 OLED 显示，ESP8266 作为联网网关，通过 HTTP / MQTT 与云端交互；云端网页支持阈值下发、遥测历史曲线、固件管理与远程升级。

## 系统架构

```
 AHT20 / BMP280 / MPU6050 / 按键 / 蜂鸣器
        │  I2C / SPI / GPIO
 ┌──────┴───────────────────────────────┐
 │  STM32F103C8T6  (FreeRTOS, Cortex-M3)│
 │  · 采样任务 / 跌倒检测 / 低功耗      │
 │  · OLED 首页动画 + 状态行            │
 │  · USART1 ASCII 行协议               │
 └──────┬───────────────────────────────┘
        │  TL / TH / PB / OA / AV ...
 ┌──────┴───────────────────────────────┐
 │  ESP8266 网关（WiFi + 协作式调度）   │
 │  · 遥测上报 / 阈值转发 / MQTT        │
 │  · 固件预下载（LittleFS，断点续传）  │
 └──────┬───────────────────────────────┘
        │  HTTP / MQTT over WebSocket
 ┌──────┴───────────────────────────────┐
 │  云端服务（Node.js / Express）        │
 │  · 设备管理 / 遥测历史 / 事件流      │
 │  · 固件上传、灰度下发、OTA 结果      │
 └──────────────────────────────────────┘
```

## 核心特性

- **双区 OTA**：bootloader（`0x08000000`）与 APP（`0x08006000`）分区独立；APP 上限 40KB。
- **非阻塞固件下载**：ESP8266 每次只搬运 1KB 就返回调度器，下载 40KB 固件期间遥测上报、串口转发不中断（实测最大间隔 < 0.7s）。
- **两级升级语义**：固件下载完成后只写入本地 flash 并置位备份寄存器标记，**真正刷写发生在用户下一次 RESET / 上电**，后台推送不会打断正在运行的业务。
- **升级状态可视化**：`下载中 x%` → `已写入 W25Q` → `下载完成`；复位进入 bootloader 后显示 `升级中` 与进度条。
- **协议容错**：USART 接收改为中断 + 环形缓冲，容忍行首坏字节与半包；CRC32 / 大小双重校验，失败自动续传。
- **云端能力**：阈值与上报周期热配置（500ms~1h）、遥测历史查询与图表、设备事件流（复位原因、链路通断、传感器停更）。

## 仓库结构

```
app/         STM32 APP 工程（CubeIDE）
  user/      业务任务：传感器、OLED、串口协议、OTA 上报
bootloader/  STM32 引导程序（OTA 状态机 + 升级界面）
gateway/     ESP8266 网关（PlatformIO，Arduino 框架）
backend/     云端服务（Express API + 管理页面 iot.html）
```

## 构建与运行

1. **STM32**
   - CubeIDE 分别导入 `app/` 与 `bootloader/`，编译烧录。
   - bootloader 起始地址 `0x08000000`，APP 起始地址 `0x08006000`。
2. **ESP8266 网关**
   - `cd gateway && pio run -e esp8266_ota`
   - 烧录：`pio run -e esp8266_ota -t upload`
3. **云端服务**
   - `cd backend && npm install && node index.js`

## 配置说明

- 网关参数集中在 [`gateway/include/config.h`](gateway/include/config.h)：把 `YOUR_WIFI_SSID`、`YOUR_WIFI_PASSWORD`、`YOUR_SERVER_HOST`、`YOUR_MQTT_PASSWORD`、`YOUR_DEVICE_KEY` 换成自己的值。
- 后端通过环境变量配置：`IOT_PORT`、`DATA_DIR`、`UPLOAD_DIR`、`IOT_DEVICE_HOST`、`MQTT_PREFIX` 等。

## 硬件

STM32F103C8T6 · ESP8266 (4MB Flash) · SSD1306 OLED (I2C) · AHT20 · BMP280 · MPU6050 · nRF24L01（可选）。
