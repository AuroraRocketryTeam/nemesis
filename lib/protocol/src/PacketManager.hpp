#pragma once

#include <vector>
#include <cstdint>
#include <Packet.hpp>

class PacketManager
{
public:
    /**
     * @brief Serializes a Packet object into a byte vector.
     *
     * @param packet The Packet object to serialize.
     * @return std::vector<uint8_t> The serialized byte vector.
     */
    static std::vector<uint8_t> serialize(const Packet &packet);

    /**
     * @brief Deserializes a byte array into a Packet object.
     * @param data Pointer to the byte array.
     * @param length Length of the byte array.
     * @return Packet The deserialized Packet object.
     */
    static Packet deserialize(const uint8_t *data, size_t length);

    /**
     * @brief Divides a large message into multiple Packet objects.
     *
     * @param message Pointer to the message byte array.
     * @param length Length of the message byte array.
     * @return std::vector<Packet> A vector of Packet objects representing the divided message.
     */
    static std::vector<Packet> divideMessage(const uint8_t *message, size_t length);
    /**
     * @brief Reassembles a complete message from multiple Packet objects.
     *
     * @param packets A vector of Packet objects to reassemble.
     * @return std::vector<uint8_t> The reassembled message as a byte vector.
     */
    static std::vector<uint8_t> reassembleMessage(const std::vector<Packet> &packets);
};