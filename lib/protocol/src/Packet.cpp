#include "Packet.hpp"

#include <Arduino.h>

void Packet::calculateCRC()
{
    // CRC-16 algorithm (reflected) with initial value 0xFFFF and polynomial
    // 0xA001 (this is the reflected form of 0x8005 used by CRC-16/ARC).
    uint16_t crc = 0xFFFF;
    const uint8_t *data = reinterpret_cast<const uint8_t *>(this);

    // Compute length up to (but not including) the crc field.
    size_t length = offsetof(Packet, crc);

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    this->crc = crc;
}

void Packet::printPacket()
{
    Serial.println("######## HEADER ########");
    Serial.println("Message ID: " + String(this->header.messageId));

    // Decode flags
    bool som = (this->header.flags & 0x01) != 0;
    bool eom = (this->header.flags & 0x02) != 0;
    bool ackReq = (this->header.flags & 0x04) != 0;
    Serial.println(String("Flags: 0x") + String(this->header.flags, HEX) + String(" (SOM=") +
                   String(som ? "1" : "0") + String(", EOM=") + String(eom ? "1" : "0") +
                   String(", ACKReq=") + String(ackReq ? "1" : "0") + String(")"));

    Serial.println("Total Chunks: " + String(this->header.totalChunks));
    // chunkIndex is zero-based; present it to humans as 0-based and also show 1-based for convenience
    Serial.println("Chunk Index (0-based): " + String(this->header.chunkIndex) +
                   " (1-based: " + String(this->header.chunkIndex + 1) + ")");
    Serial.println("Payload Size: " + String(this->header.payloadSize));
    Serial.println("Protocol Version: " + String(this->header.protocolVersion));
    Serial.println("######## PAYLOAD ########");

    // Print only the declared payloadSize bytes (rest may be padding)
    int toPrint = this->header.payloadSize;
    if (toPrint > (int)LORA_MAX_PAYLOAD_SIZE)
        toPrint = LORA_MAX_PAYLOAD_SIZE;
    for (int i = 0; i < toPrint; i++)
    {
        if (this->payload.data[i] < 0x10)
            Serial.print("0");
        Serial.print(this->payload.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("CRC: 0x" + String(this->crc, HEX));
}