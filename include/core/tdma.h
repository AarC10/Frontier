#pragma once

#include <cstdint>

constexpr std::uint32_t TDMA_FRAME_LEN_MS = 10000;
constexpr std::uint32_t TDMA_SLOT_LEN_MS = 1100;
constexpr std::uint32_t TDMA_MAX_SLOTS = TDMA_FRAME_LEN_MS / TDMA_SLOT_LEN_MS;
constexpr std::uint32_t TDMA_MAX_NODES = 5;

void tdma_init(std::uint8_t node_id);
