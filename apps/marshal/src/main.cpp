#include <core/io/Led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(marshal, LOG_LEVEL_INF);

static gpio_dt_spec statusLedSpec = GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios);
static Led statusLed(&statusLedSpec);

int main() {
    int rc = statusLed.init();

    LOG_INF("Hello World!");

    while (true) {
        statusLed.toggle();
        k_msleep(1000);
    }

    return 0;
}