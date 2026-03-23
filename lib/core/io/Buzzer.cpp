#include <core/io/Buzzer.h>

#include <zephyr/kernel.h>

Buzzer::Buzzer(const gpio_dt_spec *spec) : spec(spec) {}

int Buzzer::init() const {
    if (spec == nullptr) {
        return -EINVAL;
    }

    if (!gpio_is_ready_dt(spec)) {
        return -ENODEV;
    }

    return gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
}

void Buzzer::on() const {
    if (spec != nullptr) {
        gpio_pin_set_dt(spec, 1);
    }
}

void Buzzer::off() const {
    if (spec != nullptr) {
        gpio_pin_set_dt(spec, 0);
    }
}

void Buzzer::beep(uint32_t durationMs) const {
    on();
    k_sleep(K_MSEC(durationMs));
    off();
}
