#pragma once

#include "zephyr/drivers/gnss.h"

#include <array>
#include <stdint.h>
#include <zephyr/drivers/lora.h>

struct LoraFrame;

class LoraTransceiver {
  public:
    LoraTransceiver(const uint8_t nodeId, const float frequencyMHz);

    /**
     * Transmit payload with no GNSS fix
     * @return Whether transmission was successful
     */
    bool txNoFixPayload();

    /**
     * Transmit GNSS payload
     * @param gnssData GNSS data to transmit
     * @return Whether transmission was successful
     */
    bool txGnssPayload(const gnss_data& gnssData);

    /**
     * Setup asynchronous reception
     * @return Zephyr error code indicating if setup was successful
     */
    int awaitRxPacket();

    /**
     * Cancel the awaitRxPacket wait
     * @return Zephyr error code indicating if the wait was cancelled
     */
    int awaitCancel();

    /**
     * Receive callback
     * @param data Data received
     * @param size Size of the data received
     * @param rssi Received Signal Strength Indicator
     * @param snr Signal to Noise Ratio
     */
    void receiveCallback(uint8_t* data, uint16_t size, int16_t rssi, int8_t snr);

    /**
     * Check if the LoRa modem is in TX mode
     * @return Whether the LoRa modem is in TX mode
     */
    bool isTx() const { return config.tx; };

    /**
     * Set the LoRa modem to TX mode
     * @return Whether setting to TX mode was successful
     */
    bool setTx();

    /**
     * Set the LoRa modem to RX mode
     * @return Whether setting to RX mode was successful
     */
    bool setRx();

    /**
     * Set the LoRa frequency and reconfigure the modem
     * @param frequency Frequency in Hz
     * @return Whether reconfiguration was successful
     */
    bool setFrequency(uint32_t frequency);

#ifdef CONFIG_LICENSED_FREQUENCY
    /**
     * Set the callsign for transmission to be used for licensed bands
     * @param callsign Set the callsign for transmission if on licensed band
     */
    void setCallsign(const char* callsign);

#endif

    /**
     * Set the node ID for transmission
     * @param id Node ID to set for transmission
     */
    void setNodeId(uint8_t id);

  private:
    lora_modem_config config{
#ifdef CONFIG_LICENSED_FREQUENCY
        .frequency = 435000000,
#else
        .frequency = 903000000,
#endif
        .bandwidth = BW_125_KHZ,
        .datarate = SF_10,
        .coding_rate = CR_4_5,
        .preamble_len = 8,
        .tx_power = 22,
        .tx = false,
        .iq_inverted = false,
        .public_network = false,
    };

#ifdef CONFIG_LICENSED_FREQUENCY
    const char* callsign;
#endif

    const device* dev = DEVICE_DT_GET(DT_ALIAS(lora));
    uint8_t nodeId;

    /**
     * Initialize the LoRa modem
     * @return Initialization success
     */
    bool init();

    /**
     * Transmit data
     * @param data Data to transmit
     * @param data_len Length of data to transmit
     * @return Whether transmission was successful
     */
    bool tx(uint8_t* data, uint32_t data_len);

    /**
     * Check if frequency is in 433MHz band
     * @return True if frequency is in 433MHz band
     */
    bool is433MHzBand() const { return (config.frequency >= 410'000'000 && config.frequency <= 450'000'000); }

    /**
     * Prints the contents of a LoRa frame
     * @param frame LoRa frame containing the data to print
     */
    void parseLoraFrame(const LoraFrame& frame, const size_t size, const int16_t rssi, const int8_t snr) const;
};
