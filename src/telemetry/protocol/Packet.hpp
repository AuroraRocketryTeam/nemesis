#pragma once

#include <cstdint>
/**
 * @brief The packet header.
 *
 */
struct PacketHeader
{
    uint8_t packetNumber;
    uint8_t packetSize;
    uint8_t chunkNumber;
    uint8_t chunkSize;
    uint16_t length;
    uint32_t timestamp;
};

/**
 * @brief The payload of a packet.
 *
 */
struct PacketPayload
{
    uint8_t data[114]; // Max packet size (200  bytes) - header size (80 bytes) - crc size (16 bytes)
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
};

/**
 * @brief Calculate the CRC16 of a data buffer.
 *
 * @param data the data buffer
 * @param length the length of the data buffer
 * @return uint16_t the CRC16 of the data buffer
 */
uint16_t calculateCRC(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

#pragma pack(pop)