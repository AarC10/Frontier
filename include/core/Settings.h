#pragma once

#if defined(CONFIG_SHELL_FREQUENCY) || defined(CONFIG_LICENSED_FREQUENCY) || defined(CONFIG_SHELL_NODE_ID)

#include <stdint.h>

namespace Settings {

#ifdef CONFIG_LICENSED_FREQUENCY
constexpr uint32_t DEFAULT_FREQUENCY = 435000000;
#else
constexpr uint32_t DEFAULT_FREQUENCY = 903.000000;
#endif
constexpr int CALLSIGN_LEN = 6;
constexpr uint8_t DEFAULT_NODE_ID = 1;

/**
 * Initialize the settings subsystem and load persisted values from NVS.
 * Call once at startup before reading any settings.
 * @return 0 on success, negative errno on failure
 */
int load();

#ifdef CONFIG_SHELL_FREQUENCY
/**
 * Get the persisted LoRa frequency, or DEFAULT_FREQUENCY if not yet saved.
 */
uint32_t getFrequency();

int saveFrequency(uint32_t frequency);
#endif

#ifdef CONFIG_LICENSED_FREQUENCY
/**
 * Copy the persisted callsign into out (exactly CALLSIGN_LEN bytes, zero-padded).
 */
void getCallsign(char out[CALLSIGN_LEN]);

int saveCallsign(const char callsign[CALLSIGN_LEN]);
#endif

#ifdef CONFIG_SHELL_NODE_ID
int saveNodeId(uint8_t nodeId);
uint8_t getNodeId();
#endif

} // namespace Settings

#endif