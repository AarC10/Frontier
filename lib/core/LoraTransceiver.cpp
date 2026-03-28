#include "core/LoraTransceiver.h"

#include "core/defs.h"
#include "zephyr/drivers/gnss.h"
#include "zephyr/logging/log.h"

#include <array>
#include <cstring>

LOG_MODULE_REGISTER(LoraTransceiver);

static void loraReceiveCallback(const device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr,
                                void *userData) {
    if (auto *transceiver = static_cast<LoraTransceiver *>(userData)) {
        transceiver->receiveCallback(data, size, rssi, snr);
    }
}

static int32_t nanoToMicro(const int64_t nano) { return static_cast<int32_t>(nano / 1'000); }

static double microToDeg(const int32_t micro) { return static_cast<double>(micro) / 1'000'000.0; }

LoraTransceiver::LoraTransceiver(const uint8_t nodeId, const float frequencyMHz) : nodeId(nodeId) {
    config.frequency = static_cast<uint32_t>(frequencyMHz * 1'000'000);
    init();
}

bool LoraTransceiver::txNoFixPayload() {
    NoFixFrame packet{};
#ifdef CONFIG_LICENSED_FREQUENCY
    memcpy(&packet.callsign, callsign, std::min(strlen(callsign), CALLSIGN_CHAR_COUNT));
#endif
    packet.node_id = nodeId;

    return tx(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

bool LoraTransceiver::txGnssPayload(const gnss_data &gnssData) {
    LoraFrame packet{};

#ifdef CONFIG_LICENSED_FREQUENCY
    memcpy(&packet.callsign, callsign, std::min(strlen(callsign), CALLSIGN_CHAR_COUNT));
#endif
    packet.node_id = nodeId;
    packet.gnssInfo.latitude = nanoToMicro(gnssData.nav_data.latitude);
    packet.gnssInfo.longitude = nanoToMicro(gnssData.nav_data.longitude);
    packet.gnssInfo.satellites_cnt = static_cast<uint8_t>(gnssData.info.satellites_cnt);
    packet.gnssInfo.fix_status = gnssData.info.fix_status;

    return tx(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

int LoraTransceiver::awaitRxPacket() {
    if (config.tx) {
        LOG_WRN("LoRa is in TX mode, cannot receive");
        return -1;
    }

    if (lora_recv_async(dev, loraReceiveCallback, this) != 0) {
        LOG_ERR("LoRa async receive setup failed");
        return -1;
    }

    return 0;
}

int LoraTransceiver::awaitCancel() {
    if (lora_recv_async(dev, nullptr, nullptr) != 0) {
        LOG_ERR("LoRa async cancel setup failed");
        return -1;
    }

    return 0;
}

void LoraTransceiver::receiveCallback(uint8_t *data, uint16_t size, int16_t rssi, int8_t snr) {
    if (config.tx || !data || size == 0) return;

    switch (size) {
        case sizeof(LoraFrame): {
            const auto frame = reinterpret_cast<LoraFrame *>(data);
            parseLoraFrame(*frame, size, rssi, snr);
            break;
        }
        case NOFIX_PACKET_SIZE: {
            const auto frame = reinterpret_cast<NoFixFrame *>(data);
#ifdef CONFIG_LICENSED_FREQUENCY
            LOG_INF("%.6s-%d: (%d bytes | %d dBm | %d dB):", frame->callsign, frame->node_id, size, rssi, snr);
#else
            LOG_INF("Node %d: (%d bytes | %d dBm | %d dB):", frame->node_id, size, rssi, snr);
#endif
            LOG_INF("\tNo fix acquired!");
            break;
        }
        default:
            LOG_INF("(%d bytes | %d dBm | %d dB):", size, rssi, snr);
#ifdef CONFIG_LICENSED_FREQUENCY
            LOG_INF("\tCallsign: %.*s", CALLSIGN_CHAR_COUNT, data);
#endif
            LOG_INF("\tReceived data: %s", data);
            break;
    }
}

bool LoraTransceiver::setTx() {
    config.tx = true;
    const int ret = lora_config(dev, &config);
    if (ret != 0) {
        LOG_ERR("LoRa configuration failed %d", ret);
        return false;
    }
    return true;
};

bool LoraTransceiver::setRx() {
    config.tx = false;
    const int ret = lora_config(dev, &config);
    if (ret != 0) {
        LOG_ERR("LoRa configuration failed %d", ret);
        return false;
    }
    return true;
};

bool LoraTransceiver::setFrequency(uint32_t frequency) {
    config.frequency = frequency;
    const int ret = lora_config(dev, &config);
    if (ret != 0) {
        LOG_ERR("LoRa configuration failed %d", ret);
        return false;
    }
    return true;
};

bool LoraTransceiver::init() {
    if (!device_is_ready(dev)) {
        LOG_ERR("LoRa device not ready (dev ptr %p)", dev);
        return false;
    }

    LOG_INF("LoRa device name: %s, addr: %p", dev->name ? dev->name : "UNKNOWN", dev);

    const int ret = lora_config(dev, &config);
    if (ret != 0) {
        LOG_ERR("LoRa configuration failed, rc=%d", ret);
        return false;
    }

    LOG_INF("LoRa initialized successfully (freq=%u, sf=%d, bw=%d, cr=%d, "
            "txp=%d, public=%d)",
            config.frequency, config.datarate, config.bandwidth, config.coding_rate, config.tx_power,
            config.public_network);

    return true;
}

bool LoraTransceiver::tx(uint8_t *data, uint32_t data_len) {
    if (!data || data_len == 0) {
        LOG_ERR("LoRa send called with empty payload");
        return false;
    }

#ifdef CONFIG_LICENSED_FREQUENCY
    if (strnlen(callsign, CALLSIGN_CHAR_COUNT) == 0) {
        LOG_ERR("Callsign not set, cannot transmit on licensed frequency");
        return false;
    }
#endif

    if (lora_send_async(dev, data, data_len, nullptr) != 0) {
        LOG_ERR("LoRa send failed");
        return false;
    }
    LOG_DBG("Transmitted %u bytes over LoRa", data_len);
    return true;
}

#ifdef CONFIG_LICENSED_FREQUENCY
void LoraTransceiver::setCallsign(const char *callsign) { this->callsign = callsign; }
#endif

void LoraTransceiver::setNodeId(uint8_t id) { nodeId = id; }

void LoraTransceiver::parseLoraFrame(const LoraFrame &frame, const size_t size, const int16_t rssi,
                                     const int8_t snr) const {
    LOG_INF("Node %d: (%d bytes | %d dBm | %d dB):", frame.node_id, size, rssi, snr);

#ifdef CONFIG_LICENSED_FREQUENCY
    LOG_INF("\tCallsign: %.*s", CALLSIGN_CHAR_COUNT, frame.callsign);
#endif
    LOG_INF("\tLatitude: %f", microToDeg(frame.gnssInfo.latitude));
    LOG_INF("\tLongitude: %f", microToDeg(frame.gnssInfo.longitude));
    LOG_INF("\tSatellites count: %u", frame.gnssInfo.satellites_cnt);
    switch (frame.gnssInfo.fix_status) {
        case GNSS_FIX_STATUS_NO_FIX:
            LOG_INF("\tFix status: NO FIX");
            break;
        case GNSS_FIX_STATUS_GNSS_FIX:
            LOG_INF("\tFix status: FIX");
            break;
        case GNSS_FIX_STATUS_DGNSS_FIX:
            LOG_INF("\tFix status: DIFF FIX");
            break;
        case GNSS_FIX_STATUS_ESTIMATED_FIX:
            LOG_INF("\tFix status: EST FIX");
            break;
        default:
            LOG_INF("\tFIX status: UNKNOWN");
            break;
    }
}
