#include "core/GnssReceiver.h"

#include "zephyr/logging/log.h"

#include <atomic>
#include <cstring>

LOG_MODULE_REGISTER(GnssReciever);

static GnssReceiver* receiver = nullptr;

void setGnssReciever(GnssReceiver* rec) { receiver = rec; }

void gnssCallback(const device* dev, const gnss_data* data) {
    if (!data) {
        return;
    }

    if (!receiver) {
        LOG_WRN("GNSS receiver not initialized");
        return;
    }

    receiver->callback(*data);
}
GnssReceiver::GnssReceiver() {}

void GnssReceiver::callback(const gnss_data& data) {
    std::memcpy(&latestData, &data, sizeof(gnss_data));
    const bool has_fix = data.info.fix_status != GNSS_FIX_STATUS_NO_FIX;
    fixAcquired.store(has_fix, std::memory_order_relaxed);
}
