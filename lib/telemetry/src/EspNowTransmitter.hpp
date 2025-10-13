#pragma once

#include <ITransmitter.hpp>
#include <Packet.hpp>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief ESP-NOW transmitter for sending Packet structures to a peer receiver.
 * 
 * This class implements the ITransmitter interface for Packet objects using ESP-NOW protocol.
 * It handles peer registration, packet transmission, and delivery acknowledgment.
 * 
 * Thread-safe for concurrent transmissions.
 */
class EspNowTransmitter : public ITransmitter<Packet>
{
public:
    /**
     * @brief Construct a new ESP-NOW Transmitter.
     * 
     * @param peerMacAddress The MAC address of the peer receiver (6 bytes).
     * @param wifiChannel The WiFi channel to use (1-13, default 1).
     */
    EspNowTransmitter(const uint8_t peerMacAddress[6], uint8_t wifiChannel = 1);
    
    /**
     * @brief Destroy the ESP-NOW Transmitter and cleanup resources.
     */
    ~EspNowTransmitter();

    /**
     * @brief Initialize the ESP-NOW transmitter.
     * 
     * Sets up WiFi in STA mode, initializes ESP-NOW, and registers the peer.
     * 
     * @return ResponseStatusContainer Success (code 0) or error with description.
     */
    ResponseStatusContainer init() override;

    /**
     * @brief Transmit a Packet via ESP-NOW.
     * 
     * Serializes the packet and sends it to the registered peer.
     * Blocks until send completes or times out (default 1000ms).
     * 
     * @param packet The Packet to transmit.
     * @return ResponseStatusContainer Success (code 0) or error with description.
     */
    ResponseStatusContainer transmit(Packet packet) override;

    /**
     * @brief Get the last recorded RSSI (signal strength).
     * 
     * @return int8_t RSSI in dBm, or 0 if no data received.
     */
    int8_t getLastRSSI() const { return lastRSSI; }

    /**
     * @brief Get transmission statistics.
     * 
     * @param sent Output: number of successfully sent packets.
     * @param failed Output: number of failed transmissions.
     */
    void getStats(uint32_t &sent, uint32_t &failed) const;

private:
    uint8_t peerMac[6];
    uint8_t channel;
    bool initialized;
    
    // Statistics
    uint32_t packetsSent;
    uint32_t packetsFailed;
    int8_t lastRSSI;
    
    // Synchronization
    SemaphoreHandle_t sendMutex;
    SemaphoreHandle_t sendCompleteSemaphore;
    volatile bool sendSuccess;
    
    // ESP-NOW callbacks (must be static)
    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    
    // Instance pointer for callback access
    static EspNowTransmitter* instance;
    
    /**
     * @brief Add peer to ESP-NOW peer list.
     * 
     * @return true if peer added successfully.
     */
    bool addPeer();
};
