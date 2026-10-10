#pragma once

// TinyUSB config for the espressif/tinyusb component (CONFIG_FUGU_WITH_USB_MSC): CDC console + MSC drive.

#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_OS_INC_PATH freertos/
#define CFG_TUSB_DEBUG 0

#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_DWC2_SLAVE_ENABLE 1
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 1

#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 1024
#define CFG_TUD_CDC_EP_BUFSIZE 64
#define CFG_TUD_MSC_EP_BUFSIZE 512
