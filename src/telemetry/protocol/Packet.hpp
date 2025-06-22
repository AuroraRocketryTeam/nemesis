#pragma once

#include <cstdint>
#include <cstddef>

constexpr size_t MAX_PACKET_SIZE = 200;
/* 4 bytes are reserved for ADDH, ADDL, CHAN, and RSSI in the library */
constexpr size_t RESERVED_BYTES = 4;


/* 2 bytes for CRC */
constexpr size_t CRC_SIZE = sizeof(uint16_t);
/* HEADER size + PAYLOAD size + crc size */
constexpr size_t MAX_TX_PACKET_SIZE = MAX_PACKET_SIZE - RESERVED_BYTES;


#pragma pack(push, 1) // Evita padding nelle strutture

/**
 * @brief The packet header.
 *
 * @param packetNumber The number of the packet.
 * @param totalChunks The total number of chunks.
 * @param chunkNumber The number of the chunk (message-id).
 * @param chunkSize The size of the chunk.
 * @param payloadSize The size of the payload.
 * @param timestamp The timestamp of the packet.
 * @param crc The CRC of the packet.
 */
struct PacketHeader
{
    uint16_t packetNumber = 1;
    uint8_t totalChunks;
    uint8_t chunkNumber;
    uint8_t chunkSize;   // In bytes
    uint8_t payloadSize; // In bytes
    uint32_t timestamp;  // Unix timestamp
    const uint8_t protocolVersion = 1; // Versione del protocollo
};

constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
constexpr size_t MAX_PAYLOAD_SIZE = MAX_TX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE;

/**
 * @brief The payload of a packet.
 * @param data The data of the payload (a byte array).
 */
struct PacketPayload
{
    uint8_t data[MAX_PAYLOAD_SIZE];
};

/**
 * @brief A transmission packet with a header and a payload.
 *
 */
struct Packet
{
    PacketHeader header;
    PacketPayload payload;
    uint16_t crc;

    /**
     * @brief Calculate the CRC16 of the packet.
     */
    void calculateCRC();
};

#pragma pack(pop) // Ripristina l'allineamento predefinito
