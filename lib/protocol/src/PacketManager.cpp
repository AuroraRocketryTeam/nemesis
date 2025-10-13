#include "PacketManager.hpp"
#include "Packet.hpp"
#include <cstring>
#include <algorithm>
#include <vector>
#include <Arduino.h>

std::vector<uint8_t> PacketManager::serialize(const Packet &packet)
{
    // Make a local copy so we can compute CRC safely
    Packet pkt = packet;
    
    // Ensure payload buffer is padded with 0x01 for deterministic CRC
    const size_t validPayloadSize = std::min<size_t>(pkt.header.payloadSize, LORA_MAX_PAYLOAD_SIZE);
    if (validPayloadSize < LORA_MAX_PAYLOAD_SIZE)
    {
        std::memset(pkt.payload.data + validPayloadSize, 0x01, LORA_MAX_PAYLOAD_SIZE - validPayloadSize);
    }
    
    pkt.calculateCRC();

    // ALWAYS create a fixed-size packet of FIXED_PACKET_SIZE bytes
    std::vector<uint8_t> out(FIXED_PACKET_SIZE, 0x01); // Initialize all bytes to 0x01 (padding)

    size_t offset = 0;
    
    // Copy header bytes
    const uint8_t *hdrPtr = reinterpret_cast<const uint8_t *>(&pkt.header);
    std::memcpy(out.data() + offset, hdrPtr, HEADER_SIZE);
    offset += HEADER_SIZE;

    // Copy full payload buffer (all LORA_MAX_PAYLOAD_SIZE bytes, including padding)
    std::memcpy(out.data() + offset, pkt.payload.data, LORA_MAX_PAYLOAD_SIZE);
    offset += LORA_MAX_PAYLOAD_SIZE;

    // Append CRC (little-endian)
    out[offset] = static_cast<uint8_t>(pkt.crc & 0xFF);
    out[offset + 1] = static_cast<uint8_t>((pkt.crc >> 8) & 0xFF);
    offset += CRC_SIZE;
    
    // Remaining bytes (if any) are already filled with 0x01 padding
    // offset should now be HEADER_SIZE + LORA_MAX_PAYLOAD_SIZE + CRC_SIZE
    // Any remaining bytes up to FIXED_PACKET_SIZE stay as 0x01

    return out;
}

Packet PacketManager::deserialize(const uint8_t *data, size_t length)
{
    Packet pkt;
    // Basic size check: must contain at least header + CRC
    if (data == nullptr || length < HEADER_SIZE + CRC_SIZE)
    {
        return pkt; // empty/default packet
    }

    // Copy header
    std::memcpy(&pkt.header, data, HEADER_SIZE);

    // Sanitize payloadSize
    size_t payloadSize = pkt.header.payloadSize;
    if (payloadSize > LORA_MAX_PAYLOAD_SIZE)
    {
        payloadSize = LORA_MAX_PAYLOAD_SIZE;
    }

    // Ensure buffer contains stated payload + CRC
    if (length < HEADER_SIZE + payloadSize + CRC_SIZE)
    {
        // Adjust payloadSize down to available bytes (if any)
        if (length >= HEADER_SIZE + CRC_SIZE)
        {
            payloadSize = length - HEADER_SIZE - CRC_SIZE;
        }
        else
        {
            payloadSize = 0;
        }
    }

    // Fill payload buffer with deterministic padding (0x01) as document describes
    std::memset(pkt.payload.data, 0x01, LORA_MAX_PAYLOAD_SIZE);
    if (payloadSize > 0)
    {
        std::memcpy(pkt.payload.data, data + HEADER_SIZE, payloadSize);
    }

    // Read CRC from wire (little-endian)
    size_t crcOffset = HEADER_SIZE + payloadSize;
    if (length >= crcOffset + CRC_SIZE)
    {
        uint16_t receivedCrc = static_cast<uint16_t>(data[crcOffset]) |
                               (static_cast<uint16_t>(data[crcOffset + 1]) << 8);
        // Compute CRC over the packet contents
        Packet tmp = pkt;
        tmp.crc = 0;
        tmp.calculateCRC();
        uint16_t computedCrc = tmp.crc;

        pkt.crc = receivedCrc; // keep received value in packet

        if (computedCrc != receivedCrc)
        {
            Serial.printf("PacketManager::deserialize - CRC mismatch: recv=0x%04X calc=0x%04X\n",
                          receivedCrc, computedCrc);
            // leave pkt as-is; callers can inspect header/flags or crc to detect errors
        }
    }
    else
    {
        pkt.crc = 0;
    }

    return pkt;
}

std::vector<Packet> PacketManager::divideMessage(const uint8_t *message, size_t length)
{
    std::vector<Packet> packets;
    if (message == nullptr || length == 0)
        return packets;

    const size_t chunkSize = LORA_MAX_PAYLOAD_SIZE;
    // Calculate number of chunks, cap at 255 because header.totalChunks is a uint8_t
    size_t totalChunks64 = (length + chunkSize - 1) / chunkSize;
    uint8_t totalChunks = static_cast<uint8_t>(std::min<size_t>(totalChunks64, 255));

    static uint16_t nextMessageId = 1;
    const uint16_t messageId = nextMessageId++;

    size_t offset = 0;
    for (uint8_t idx = 0; idx < totalChunks; ++idx)
    {
        Packet pkt{};
        pkt.header.messageId = messageId;
        pkt.header.totalChunks = totalChunks;
        pkt.header.chunkIndex = idx;

        size_t remaining = length - offset;
        size_t thisSize = std::min<size_t>(remaining, chunkSize);
        pkt.header.payloadSize = static_cast<uint8_t>(thisSize);

        // Flags: bit0 = Start, bit1 = End
        pkt.header.flags = 0;
        if (idx == 0)
            pkt.header.flags |= 0x01;  // Start of Message
        if (idx == totalChunks - 1)
            pkt.header.flags |= 0x02;  // End of Message
        
        pkt.header.protocolVersion = 1;

        // Fill payload deterministically with 0x01 then copy data
        std::memset(pkt.payload.data, 0x01, LORA_MAX_PAYLOAD_SIZE);
        if (thisSize > 0)
        {
            std::memcpy(pkt.payload.data, message + offset, thisSize);
        }

        // Calculate CRC for this packet
        pkt.calculateCRC();

        packets.push_back(pkt);
        offset += thisSize;
    }

    return packets;
}
