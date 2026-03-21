#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef CONFIG_LICENSED_FREQUENCY
inline constexpr size_t CALLSIGN_CHAR_COUNT = 6;
inline constexpr size_t NODE_ID_START_INDEX = CALLSIGN_CHAR_COUNT;
#else
inline constexpr size_t CALLSIGN_CHAR_COUNT = 0;
inline constexpr size_t NODE_ID_START_INDEX = 0;
#endif
inline constexpr uint8_t NOFIX[] = "NOFIX";

#pragma pack(push, 1)
struct GnssInfo {
    int32_t latitude{0};
    int32_t longitude{0};
    uint8_t satellites_cnt{0};
    uint8_t fix_status{0};
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LoraFrame {
#ifdef CONFIG_LICENSED_FREQUENCY
    char callsign[CALLSIGN_CHAR_COUNT]{0};
#endif
    uint8_t version{0x01};
    uint8_t node_id{0};
    GnssInfo gnssInfo{};
};
#pragma pack(pop)

#pragma pack(push, 1)
struct NoFixFrame {
#ifdef CONFIG_LICENSED_FREQUENCY
    char callsign[CALLSIGN_CHAR_COUNT]{0};
#endif
    uint8_t node_id{0};
    const uint8_t nofix[sizeof(NOFIX)]{'N', 'O', 'F', 'I', 'X'};
};

inline constexpr size_t GNSS_INFO_SIZE = sizeof(GnssInfo);
inline constexpr size_t NODE_ID_SIZE = 1;
inline constexpr size_t NOFIX_PACKET_SIZE = NODE_ID_SIZE + sizeof(NOFIX) + CALLSIGN_CHAR_COUNT;
inline constexpr size_t MAX_PAYLOAD_SIZE = NODE_ID_SIZE + GNSS_INFO_SIZE + CALLSIGN_CHAR_COUNT;
