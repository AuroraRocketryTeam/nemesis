#pragma once

#include <cstddef>
#include <cstdint>

// Fixed packet size for transmission (optimized for 64-byte TelemetryPacket)
// 70 bytes = Header(7) + Payload(57) + CRC(2) + Padding(4) = 3x faster than 250 bytes
constexpr size_t FIXED_PACKET_SIZE = 70;
// Maximum raw packet size we assume for transmit buffers (including header and CRC)
constexpr size_t MAX_PACKET_SIZE = FIXED_PACKET_SIZE;
constexpr size_t RESERVED_BYTES = 0;

/* 2 bytes for CRC */
constexpr size_t CRC_SIZE = sizeof(uint16_t);
/* HEADER size + PAYLOAD size + crc size */
constexpr size_t MAX_TX_PACKET_SIZE = MAX_PACKET_SIZE - RESERVED_BYTES;

// NOTE: All multi-byte integer fields are encoded in little-endian order on the
// wire. This matches the common endianness of most microcontrollers used with
// PlatformIO/Arduino. If you change the target architecture, document any
// endianness differences here.

// The CRC used by Packet::calculateCRC() below uses a 16-bit polynomial with
// initial value 0xFFFF and polynomial 0xA001 (the reflected representation of
// the CRC-16/ARC / CRC-16-IBM polynomial). The implementation deliberately
// computes the CRC over the packed Packet structure up to (but excluding) the
// `crc` field using offsetof(Packet, crc). If you change fields or their
// ordering, update HEADER_SIZE and be aware this changes CRC coverage.

#pragma pack(push, 1) // Avoid padding inside the packet structures

/**
 * @brief The packet header.
 *
 * Fields are chosen to be compact and sufficient for reassembly and control.
 * - messageId: 16-bit identifier for the logical message/transfer. Wraps on overflow.
 * - flags: 8-bit bitfield (bit0 = Start of Message, bit1 = End of Message, bit2 = ACK request, bits 3-7 reserved)
 * - totalChunks: total number of chunks in the message (0..255)
 * - chunkIndex: zero-based index of this chunk within the message (0..totalChunks-1)
 * - payloadSize: number of valid payload bytes in this packet (0..LORA_MAX_PAYLOAD_SIZE)
 * - protocolVersion: fixed protocol format version. Increment to indicate incompatible layout changes.
 */
struct PacketHeader
{
    uint16_t messageId = 1;      // 2 bytes - keep at start for natural alignment
    uint8_t totalChunks = 0;     // 1 byte - message-level info
    uint8_t chunkIndex = 0;      // 1 byte - chunk-level info (pairs logically with totalChunks)
    uint8_t payloadSize = 0;     // 1 byte - this packet's payload size
    uint8_t flags = 0;           // 1 byte - control bits
    uint8_t protocolVersion = 1; // 1 byte - least likely to change during processing
};

constexpr size_t HEADER_SIZE = sizeof(PacketHeader);
constexpr size_t LORA_MAX_PAYLOAD_SIZE =
    MAX_TX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE - 4;

/**
 * @brief The payload of a packet. This is a fixed-size buffer equal to the
 * maximum payload allowed by the header and constants. When a chunk is smaller
 * than this size it should be padded deterministically (the code currently
 * uses 0x01) so CRCs remain deterministic across sender/receiver.
 */
struct PacketPayload
{
    uint8_t data[LORA_MAX_PAYLOAD_SIZE];
};

/**
 * @brief A transmission packet with a header and a payload.
 *
 * The CRC field is placed after the payload. Packet::calculateCRC() computes
 * the CRC over the bytes from the start of the struct up to (but excluding)
 * this crc field.
 */
struct Packet
{
    PacketHeader header;
    PacketPayload payload;
    uint16_t crc;

    /**
     * @brief Calculate the CRC16 of the packet. This writes into `crc`.
     */
    void calculateCRC();
    /**
     * @brief Print a human-readable representation of the packet to Serial.
     *
     * The output is intended for debugging. Ensure `Serial.begin()` has been
     * called by the sketch before calling this.
     */
    void printPacket();
};
#pragma pack(pop) // Restore default alignment
