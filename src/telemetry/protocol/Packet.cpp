#include "Packet.hpp"

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