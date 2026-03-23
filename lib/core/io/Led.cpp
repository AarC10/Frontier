#include <core/io/Led.h>

#include <zephyr/kernel.h>

Led::Led(const gpio_dt_spec *spec) : spec(spec) {}

int Led::init() const {
    if (spec == nullptr) {
        return -EINVAL;
    }

    if (!gpio_is_ready_dt(spec)) {
        return -ENODEV;
    }

    return gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
}

void Led::on() const {
    if (spec != nullptr) {
        gpio_pin_set_dt(spec, 1);
    }
}

void Led::off() const {
    if (spec != nullptr) {
        gpio_pin_set_dt(spec, 0);
    }
}

void Led::toggle() const {
    if (spec != nullptr) {
        gpio_pin_toggle_dt(spec);
    }
}
