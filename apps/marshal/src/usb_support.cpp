/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <marshal/usb_support.h>

#include <zephyr/device.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
#endif
#if defined(CONFIG_USBD_MSC_CLASS)
#include <zephyr/usb/class/usbd_msc.h>
#endif

namespace marshal {

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_DEVICE_DEFINE(marshal_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x0483, 0x5740);
USBD_DESC_LANG_DEFINE(marshal_lang);
USBD_DESC_MANUFACTURER_DEFINE(marshal_mfr, "Wild West Rocketry");
USBD_DESC_PRODUCT_DEFINE(marshal_product, "Marshal Flight Computer");
USBD_DESC_SERIAL_NUMBER_DEFINE(marshal_sn);
USBD_DESC_CONFIG_DEFINE(marshal_fs_cfg, "FS Config");
USBD_CONFIGURATION_DEFINE(marshal_fs_config, USB_SCD_SELF_POWERED, 125, &marshal_fs_cfg);

static void setUsbCodeTriple(enum usbd_speed speed) {
    if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) || IS_ENABLED(CONFIG_USBD_MSC_CLASS)) {
        usbd_device_set_code_triple(&marshal_usbd, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
        return;
    }

    usbd_device_set_code_triple(&marshal_usbd, speed, 0, 0, 0);
}
#endif

#if defined(CONFIG_USBD_MSC_CLASS)
USBD_DEFINE_MSC_LUN(marshal, "marshal", "WWR", "Marshal", "1.00");
#endif

int initUsb() {
#if !defined(CONFIG_USB_DEVICE_STACK_NEXT)
    return 0;
#else
    int ret = usbd_add_descriptor(&marshal_usbd, &marshal_lang);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_mfr);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_product);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_descriptor(&marshal_usbd, &marshal_sn);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_add_configuration(&marshal_usbd, USBD_SPEED_FS, &marshal_fs_config);
    if (ret != 0) {
        return ret;
    }

    ret = usbd_register_all_classes(&marshal_usbd, USBD_SPEED_FS, 1, NULL);
    if (ret != 0) {
        return ret;
    }

    setUsbCodeTriple(USBD_SPEED_FS);

    ret = usbd_init(&marshal_usbd);
    if (ret != 0) {
        return ret;
    }

    return usbd_enable(&marshal_usbd);
#endif
}

} // namespace marshal
