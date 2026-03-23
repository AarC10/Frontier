#pragma once

#include <zephyr/drivers/gpio.h>

class Led {
  public:
    explicit Led(const gpio_dt_spec *spec);

    int init() const;
    void on() const;
    void off() const;
    void toggle() const;

  private:
    const gpio_dt_spec *spec;
};
