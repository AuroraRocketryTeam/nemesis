#include "Packet.hpp"
#include <Arduino.h>

void Packet::calculateCRC()
{
    uint16_t crc = 0xFFFF; // Valore iniziale del CRC
    const uint8_t *data = reinterpret_cast<const uint8_t *>(this);

    // Calcola la lunghezza corretta senza includere il campo CRC
    size_t length = offsetof(Packet, crc);

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // XOR con il byte corrente
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; // Polinomio standard CRC-16-CCITT
            else
                crc = crc >> 1;
        }
    }
    this->crc = crc;
}


void Packet::printPacket()
{
    Serial.println("######## HEADER ########");
    Serial.println("Packet Number: " + String(this->header.packetNumber));
    Serial.println("Total Chunks: " + String(this->header.totalChunks));
    Serial.println("Chunk Number: " + String(this->header.chunkNumber));
    Serial.println("Chunk Size: " + String(this->header.chunkSize));
    Serial.println("Payload Size: " + String(this->header.payloadSize));
    Serial.println("Timestamp: " + String(this->header.timestamp));
    Serial.println("Protocol Version: " + String(this->header.protocolVersion));
    Serial.println("######## PAYLOAD ########");
    for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
    {
        Serial.print(String(this->payload.data[i], HEX) + " ");
    }
    Serial.println();
    Serial.println("CRC: " + String(this->crc, HEX));
}