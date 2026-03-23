#pragma once

#include <zephyr/drivers/gpio.h>

class Buzzer {
  public:
    explicit Buzzer(const gpio_dt_spec *spec);

    int init() const;
    void on() const;
    void off() const;
    void beep(uint32_t durationMs) const;

  private:
    const gpio_dt_spec *spec;
};
