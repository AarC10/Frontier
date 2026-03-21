#pragma once

#include <atomic>
#include <zephyr/drivers/gnss.h>

void setGnssReciever(class GnssReceiver* rec);

void gnssCallback(const device* dev, const gnss_data* data);

class GnssReceiver {
  public:
    GnssReceiver();

    void callback(const gnss_data& data);

    bool isFixAcquired() const { return fixAcquired; }

    const gnss_data& getLatestData() const { return latestData; }

  private:
    gnss_data latestData;
    std::atomic<bool> fixAcquired{false};
};
