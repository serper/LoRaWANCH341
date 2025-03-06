/**
 * @file LoRaWAN.cpp
 * @brief Implementation of the LoRaWAN protocol for communication with LoRaWAN networks.
 * 
 * This file contains the implementation of the LoRaWAN class, which provides methods for
 * initializing the LoRaWAN stack, joining a network, sending and receiving messages, and
 * handling various LoRaWAN protocol features such as ADR (Adaptive Data Rate) and MAC commands.
 * 
 * The implementation uses the RFM95 radio module and supports both CH341SPI and Linux SPI interfaces.
 * 
 * @note This implementation is designed for use with the RFM95 radio module and may require
 * modifications to work with other radio modules.
 * 
 * @dependencies
 * - OpenSSL (for AES encryption)
 * - Standard C++ libraries (iostream, thread, chrono, queue, mutex, array, deque, bitset)
 * - Project-specific headers (LoRaWAN.hpp, RFM95.hpp, AES-CMAC.hpp, SessionManager.hpp)
 * 
 * @section Usage
 * 
 * To use this implementation, create an instance of the LoRaWAN class and call its methods
 * to initialize the radio, join a network, and send/receive messages. The class provides
 * methods for setting device parameters (DevEUI, AppEUI, AppKey, etc.), configuring the
 * radio, and handling protocol-specific tasks such as calculating MICs and processing
 * MAC commands.
 * 
 * Example:
 * @code
 * LoRaWAN lorawan;
 * lorawan.setDevEUI("0004A30B001C0530");
 * lorawan.setAppEUI("70B3D57ED00201A6");
 * lorawan.setAppKey("8D7F3B4C5A6B7C8D9E0F1A2B3C4D5E6F");
 * if (lorawan.join(LoRaWAN::JoinMode::OTAA)) {
 *     std::vector<uint8_t> data = {0x01, 0x02, 0x03};
 *     lorawan.send(data, 1, false);
 * }
 * @endcode
 * 
 * @section Classes
 * - LoRaWAN: Main class for LoRaWAN protocol handling.
 * - LoRaWAN::Impl: Internal implementation details for the LoRaWAN class.
 * 
 * @section Methods
 * - LoRaWAN::LoRaWAN(): Constructor to initialize the LoRaWAN instance.
 * - LoRaWAN::~LoRaWAN(): Destructor to clean up resources.
 * - bool LoRaWAN::init(int deviceIndex): Initialize the radio module.
 * - void LoRaWAN::setDeviceClass(DeviceClass deviceClass): Set the device class (A, B, or C).
 * - void LoRaWAN::setDevEUI(const std::string& devEUI): Set the device EUI.
 * - void LoRaWAN::setAppEUI(const std::string& appEUI): Set the application EUI.
 * - void LoRaWAN::setAppKey(const std::string& appKey): Set the application key.
 * - void LoRaWAN::setDevAddr(const std::string& devAddr): Set the device address.
 * - void LoRaWAN::setNwkSKey(const std::string& nwkSKey): Set the network session key.
 * - void LoRaWAN::setAppSKey(const std::string& appSKey): Set the application session key.
 * - bool LoRaWAN::join(JoinMode mode, unsigned long timeout): Join a LoRaWAN network.
 * - std::vector<uint8_t> LoRaWAN::encryptPayload(const std::vector<uint8_t>& payload, uint8_t port): Encrypt the payload.
 * - std::vector<uint8_t> LoRaWAN::decryptPayload(const std::vector<uint8_t>& payload, uint8_t port): Decrypt the payload.
 * - float LoRaWAN::calculateTimeOnAir(size_t payload_size): Calculate the time on air for a given payload size.
 * - bool LoRaWAN::checkDutyCycle(float frequency, size_t payload_size): Check if the duty cycle allows transmission.
 * - float LoRaWAN::getDutyCycleUsage(int channel): Get the duty cycle usage for a specific channel.
 * - void LoRaWAN::resetDutyCycle(): Reset the duty cycle usage.
 * - bool LoRaWAN::send(const std::vector<uint8_t>& data, uint8_t port, bool confirmed, bool force_duty_cycle): Send a message.
 * - void LoRaWAN::update(): Update the LoRaWAN state and handle received messages.
 * - bool LoRaWAN::receive(Message& message, unsigned long timeout): Receive a message.
 * - void LoRaWAN::onReceive(std::function<void(const Message&)> callback): Set a callback for received messages.
 * - void LoRaWAN::onJoin(std::function<void(bool)> callback): Set a callback for join events.
 * - void LoRaWAN::setRegion(int region): Set the LoRaWAN region.
 * - int LoRaWAN::getRegion() const: Get the current LoRaWAN region.
 * - float LoRaWAN::getFrequency() const: Get the current frequency.
 * - void LoRaWAN::setFrequency(float freq_mhz): Set the frequency.
 * - int LoRaWAN::getChannelFromFrequency(float freq_mhz) const: Get the channel index from a frequency.
 * - float LoRaWAN::getFrequencyFromChannel(int channel) const: Get the frequency from a channel index.
 * - void LoRaWAN::setChannel(uint8_t channel): Set the current channel.
 * - uint8_t LoRaWAN::getChannel() const: Get the current channel.
 * - void LoRaWAN::setSingleChannel(bool enable, float freq_mhz, int sf, int bw, int cr, int power, int preamble): Enable or disable single channel mode.
 * - bool LoRaWAN::getSingleChannel() const: Check if single channel mode is enabled.
 * - float LoRaWAN::getSingleChannelFrequency() const: Get the frequency for single channel mode.
 * - void LoRaWAN::setTxPower(int8_t power): Set the transmission power.
 * - int LoRaWAN::getRSSI() const: Get the RSSI (Received Signal Strength Indicator).
 * - int LoRaWAN::getSNR() const: Get the SNR (Signal-to-Noise Ratio).
 * - uint32_t LoRaWAN::getFrameCounter() const: Get the frame counter.
 * - void LoRaWAN::setFrameCounter(uint32_t counter): Set the frame counter.
 * - void LoRaWAN::wake(): Wake up the radio module.
 * - void LoRaWAN::sleep(): Put the radio module to sleep.
 * - bool LoRaWAN::validateKeys() const: Validate the session keys.
 * - void LoRaWAN::enableADR(bool enable): Enable or disable ADR (Adaptive Data Rate).
 * - bool LoRaWAN::isADREnabled() const: Check if ADR is enabled.
 * - void LoRaWAN::resetSession(): Reset the LoRaWAN session.
 * - void LoRaWAN::applyADRSettings(uint8_t dataRate, uint8_t txPower, const std::vector<uint8_t>& channelMask): Apply ADR settings.
 * - void LoRaWAN::processMACCommands(const std::vector<uint8_t>& commands, std::vector<uint8_t>& response): Process MAC commands.
 * - void LoRaWAN::processLinkADRReq(const std::vector<uint8_t>& cmd, size_t index, std::vector<uint8_t>& response): Process LinkADRReq command.
 * 
 * @section Debugging
 * The implementation includes several debug macros for conditional printing:
 * - DEBUG_PRINT(x): Print a message if verbose mode is enabled.
 * - DEBUG_PRINTLN(x): Print a message with a newline if verbose mode is enabled.
 * - DEBUG_HEX(x): Print a value in hexadecimal format if verbose mode is enabled.
 * 
 * @section Internal Structures
 * - struct LoRaWAN::Impl: Internal implementation details, including radio configuration, session keys, counters, and state management.
 * 
 * @section Constants
 * - BASE_FREQ: Base frequencies for different regions.
 * - CHANNEL_STEP: Channel step sizes for different regions.
 * - MAX_POWER: Maximum transmission power for different regions.
 * - RX2_FREQ: RX2 frequencies for different regions.
 * - RX2_SF: RX2 spreading factors for different regions.
 * - RX2_BW: RX2 bandwidths for different regions.
 * - RX2_CR: RX2 coding rates for different regions.
 * - RX2_PREAMBLE: RX2 preamble lengths for different regions.
 * - ADR_ACK_LIMIT: ADR acknowledgment limit.
 * - ADR_ACK_DELAY: ADR acknowledgment delay.
 * - MAX_CHANNELS: Maximum number of channels.
 * - REGIONS: Number of supported regions.
 * 
 * @section Enums
 * - enum DeviceClass: Device classes (A, B, C).
 * - enum JoinMode: Join modes (OTAA, ABP).
 * - enum ConfirmationState: Confirmation states (NONE, WAITING_ACK, ACK_PENDING).
 * - enum RxWindowState: RX window states (RX_IDLE, RX_WINDOW_1, RX_WINDOW_2, RX_CONTINUOUS).
 * 
 * @section Typedefs
 * - typedef std::function<void(const Message&)> ReceiveCallback: Callback type for received messages.
 * - typedef std::function<void(bool)> JoinCallback: Callback type for join events.
 * 
 * @section Structs
 * - struct Message: Structure representing a LoRaWAN message.
 * 
 * @section Authors
 * - Sergio Pérez (Original Author)
 * 
 * @section License
 * This code is provided under the MIT License.
 */
#include <openssl/evp.h>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include "LoRaWAN.hpp"
#include "RFM95.hpp"
#include "AES-CMAC.hpp"
#include "SessionManager.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <array>
#include <deque>
#include <bitset>

#if (defined(RFM_USE_CH341) || !defined(RFM_USE_LINUX_SPI))
// With CH341SPI
auto ch341_spi = SPIFactory::createCH341SPI(0, true);
auto rfm = RFM95(std::move(ch341_spi));
#else
#if defined(RFM_USE_LINUX_SPI)
// With Linux SPI
auto linux_spi = SPIFactory::createLinuxSPI("/dev/spidev0.0", 1000000);
auto rfm = RFM95(std::move(linux_spi));
#endif // defined(RFM_USE_LINUX_SPI)
#endif // defined(RFM_USE_CH341)

bool LoRaWAN::isVerbose = false;

// Debug helper for conditional output
#define DEBUG_PRINT(x) do { if(LoRaWAN::getVerbose()) { std::cout << x; } } while(0)
#define DEBUG_PRINTLN(x) do { if(LoRaWAN::getVerbose()) { std::cout << x << std::endl; } } while(0)
#define DEBUG_HEX(x) do { if(LoRaWAN::getVerbose()) { std::cout << std::hex << (x) << std::dec; } } while(0)

struct LoRaWAN::Impl {
    std::unique_ptr<RFM95> rfm;
    std::queue<Message> rxQueue;
    std::mutex queueMutex;
    
    // Keys & Addresses
    std::array<uint8_t, 8> devEUI;
    std::array<uint8_t, 8> appEUI;
    std::array<uint8_t, 16> appKey;
    std::array<uint8_t, 4> devAddr;
    std::array<uint8_t, 16> nwkSKey;
    std::array<uint8_t, 16> appSKey;
    
    // Counters
    uint32_t uplinkCounter;
    uint32_t downlinkCounter;
    
    // Configuration
    uint8_t dataRate;
    int8_t txPower;
    uint8_t channel;

    // RX Window Management
    RxWindowState rxState = RX_IDLE;
    std::chrono::steady_clock::time_point rxWindowStart;
    std::chrono::steady_clock::time_point txEndTime;

    std::vector<uint16_t> usedNonces;

    std::string sessionFile = "lorawan_session.json";
    
    bool saveSessionData() {
        SessionManager::SessionData data;
        data.devAddr = devAddr;
        data.nwkSKey = nwkSKey;
        data.appSKey = appSKey;
        data.uplinkCounter = uplinkCounter;
        data.downlinkCounter = downlinkCounter;
        data.lastDevNonce = lastDevNonce;
        data.usedNonces = usedNonces;
        data.joined = true;
        
        return SessionManager::saveSession(sessionFile, data);
    }

    bool loadSessionData() {
        SessionManager::SessionData data;
        if (SessionManager::loadSession(sessionFile, data)) {
            devAddr = data.devAddr;
            nwkSKey = data.nwkSKey;
            appSKey = data.appSKey;
            uplinkCounter = data.uplinkCounter;
            downlinkCounter = data.downlinkCounter;
            lastDevNonce = data.lastDevNonce;
            usedNonces = data.usedNonces;
            return data.joined;
        }
        return false;
    }

    Impl(std::unique_ptr<SPIInterface> spi_interface) {
        rfm = std::make_unique<RFM95>(std::move(spi_interface));
        uplinkCounter = 0;
        downlinkCounter = 0;
        dataRate = 0;
        txPower = 14;
        channel = 0;
        devEUI.fill(0);
        appEUI.fill(0);
        appKey.fill(0);
        devAddr.fill(0);
        nwkSKey.fill(0);
        appSKey.fill(0);
    }

    // Function to build Join Request packet
    std::vector<uint8_t> buildJoinRequest()
    {
        std::vector<uint8_t> packet;
        packet.reserve(23);

        // MHDR (Join-request = 0x00)
        packet.push_back(0x00);

        // AppEUI - send in little-endian
        for (int i = 7; i >= 0; i--)
        {
            packet.push_back(appEUI[i]);
        }

        // DevEUI - send in little-endian
        for (int i = 7; i >= 0; i--)
        {
            packet.push_back(devEUI[i]);
        }

        // DevNonce (random, 2 bytes)
        uint16_t nonce = generateDevNonce();
        lastDevNonce = nonce;
        DEBUG_PRINTLN("Generated DevNonce: 0x" << std::hex << nonce << std::dec);

        packet.push_back(nonce & 0xFF);        // LSB
        packet.push_back((nonce >> 8) & 0xFF); // MSB

        // Calculate and add MIC
        calculateMIC(packet);

        // Detailed debug of the packet
        DEBUG_PRINTLN("Join Request details (all in LE):");
        DEBUG_PRINT("MHDR: 0x" << std::hex << std::setw(2) << std::setfill('0')
                               << static_cast<int>(packet[0]) << "\n");

        DEBUG_PRINT("AppEUI (LE): ");
        for (int i = 1; i <= 8; i++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT("\n");

        DEBUG_PRINT("DevEUI (LE): ");
        for (int i = 9; i <= 16; i++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT("\n");

        DEBUG_PRINT("DevNonce: ");
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(packet[17]) << " "
                             << std::setw(2) << static_cast<int>(packet[18]) << "\n");

        // Print the full packet for verification
        DEBUG_PRINT("Join Request packet: ");
        for (const auto &byte : packet)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINTLN(std::dec);

        return packet;
    }

    // Function to calculate MIC (Message Integrity Code)
    void calculateMIC(std::vector<uint8_t>& packet) {
        // Debug the data before calculating the MIC
        DEBUG_PRINT("Calculating MIC for data: ");
        for(size_t i = 0; i < packet.size(); i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        DEBUG_PRINT("Using Key: ");
        // For Join Request we use AppKey, for data we use NwkSKey
        bool isJoinRequest = (packet[0] & 0xE0) == 0x00;
        const auto& key = isJoinRequest ? appKey : nwkSKey;
        
        for(size_t i = 0; i < 16; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(key[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // Do not check size for data packets
        if (isJoinRequest && packet.size() != 19)
        {
            DEBUG_PRINTLN("Error: Join Request must have exactly 19 bytes before the MIC");
            return;
        }

        // If it's a data packet, use the specific MIC algorithm for data
        std::array<uint8_t, 16> cmac;
        if (isJoinRequest)
        {
            // For Join Request, the MIC is CMAC(AppKey, MHDR | AppEUI | DevEUI | DevNonce)
            cmac = AESCMAC::calculate(packet, key);
        }
        else
        {
            // For data packets, the MIC is calculated over the following data:
            // B0 | MHDR | FHDR | FPort | FRMPayload
            std::vector<uint8_t> micData;
            micData.reserve(1 + 16 + packet.size()); // B0 + message

            // Block B0
            micData.push_back(0x49); // Block B0
            micData.insert(micData.end(), 4, 0x00); // 0x00^4
            micData.push_back(0x00); // Dir = 0 for uplink
            micData.insert(micData.end(), packet.begin() + 1, packet.begin() + 5);
            micData.insert(micData.end(), packet.begin() + 6, packet.begin() + 8);
            micData.push_back(0x00); // 0x00
            micData.push_back(static_cast<uint8_t>(packet.size())); // len(Data)

            // Add the full message after Block B0
            micData.insert(micData.end(), packet.begin(), packet.end());

            // Debug the data block for MIC
            DEBUG_PRINT("MIC calculation data block: ");
            for(const auto& byte : micData) {
                DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(byte) << " ");
            }
            DEBUG_PRINT(std::dec << std::endl);

            cmac = AESCMAC::calculate(micData, key);
        }

        // Debug the calculated CMAC
        DEBUG_PRINT("Full CMAC: ");
        for (const auto &byte : cmac)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINTLN(std::dec);

        // The first 4 bytes of CMAC are the MIC
        packet.insert(packet.end(), cmac.begin(), cmac.begin() + 4);
    }

    bool processJoinAccept(std::vector<uint8_t>& response) {
        if (response.size() < 17) { // MHDR(1) + AppNonce(3) + NetID(3) + DevAddr(4) + DLSettings(1) + RxDelay(1) + MIC(4)
            DEBUG_PRINTLN("Join Accept: Invalid packet size");
            return false;
        }

        // 1. Decrypt the Join Accept using AppKey
        std::vector<uint8_t> decrypted(response.size());
        decrypted[0] = response[0]; // MHDR is not encrypted
        
        // Decrypt the rest in 16-byte blocks
        for (size_t i = 1; i < response.size(); i += 16) {
            size_t block_size = std::min(size_t(16), response.size() - i);
            std::array<uint8_t, 16> block, out;
            std::copy(response.begin() + i, response.begin() + i + block_size, block.begin());
            AESCMAC::aes_encrypt(block.data(), appKey.data(), out.data());
            std::copy(out.begin(), out.begin() + block_size, decrypted.begin() + i);
        }

        // 2. Verify MIC
        std::vector<uint8_t> mic_data(decrypted.begin(), decrypted.end() - 4);
        std::array<uint8_t, 16> calculated_mic = AESCMAC::calculate(mic_data, appKey);
        
        for (int i = 0; i < 4; i++) {
            if (calculated_mic[i] != decrypted[decrypted.size() - 4 + i]) {
                DEBUG_PRINTLN("Join Accept: Invalid MIC");
                return false;
            }
        }

        // 3. Extract parameters in correct order
        [[maybe_unused]] uint32_t appNonce = (decrypted[3] << 16) | (decrypted[2] << 8) | decrypted[1];
        [[maybe_unused]] uint32_t netId = (decrypted[6] << 16) | (decrypted[5] << 8) | decrypted[4];
        
        // DevAddr (little-endian as it comes in the message)
        devAddr[0] = decrypted[7];
        devAddr[1] = decrypted[8];
        devAddr[2] = decrypted[9];
        devAddr[3] = decrypted[10];

        uint8_t dlSettings = decrypted[11];
        [[maybe_unused]] uint8_t rxDelay = decrypted[12];

        // 4. Derive session keys
        std::array<uint8_t, 16> keyInput;
        keyInput.fill(0x00);

        // NwkSKey = aes128_encrypt(AppKey, 0x01|AppNonce|NetID|DevNonce|pad16)
        keyInput[0] = 0x01;
        // AppNonce (3 bytes, little-endian)
        keyInput[1] = decrypted[1];
        keyInput[2] = decrypted[2];
        keyInput[3] = decrypted[3];
        // NetID (3 bytes, little-endian)
        keyInput[4] = decrypted[4];
        keyInput[5] = decrypted[5];
        keyInput[6] = decrypted[6];
        // DevNonce (2 bytes, little-endian)
        keyInput[7] = lastDevNonce & 0xFF;
        keyInput[8] = (lastDevNonce >> 8) & 0xFF;

        // Debug the input for NwkSKey
        DEBUG_PRINT("NwkSKey Input: ");
        for(const auto& byte : keyInput) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // Generate NwkSKey
        AESCMAC::aes_encrypt(keyInput.data(), appKey.data(), nwkSKey.data());

        // Debug NwkSKey result
        DEBUG_PRINT("NwkSKey: ");
        for(const auto& byte : nwkSKey) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // AppSKey = aes128_encrypt(AppKey, 0x02|AppNonce|NetID|DevNonce|pad16)
        keyInput[0] = 0x02; // Only change the first byte
        AESCMAC::aes_encrypt(keyInput.data(), appKey.data(), appSKey.data());

        // Debug AppSKey result
        DEBUG_PRINT("AppSKey: ");
        for(const auto& byte : appSKey) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // 5. Configure parameters
        dataRate = (dlSettings >> 4) & 0x0F;
        txPower = dlSettings & 0x0F;
        
        // Reset counters
        uplinkCounter = 0;
        downlinkCounter = 0;

        DEBUG_PRINTLN("Join Accept processed successfully");
        DEBUG_PRINT("DevAddr: ");
        for(int i = 0; i < 4; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(devAddr[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        return true;
    }

    uint16_t lastDevNonce;

    // Generar nuevo DevNonce único
    // uint16_t generateDevNonce() {
    //     // Si no hay nonces previos, empezar desde un valor base
    //     uint16_t newNonce;
    //     if (usedNonces.empty()) {
    //         newNonce = 0x0001;
    //     } else {
    //         newNonce = lastDevNonce + 1;
            
    //         // Si llegamos al máximo, empezar desde 0x0001 de nuevo
    //         if (newNonce == 0) {
    //             newNonce = 0x0001;
    //         }
    //     }
        
    //     lastDevNonce = newNonce;
    //     usedNonces.push_back(newNonce);
        
    //     // Debug
    //     DEBUG_PRINTLN("Generated DevNonce: 0x" << std::hex << newNonce << std::dec);
        
    //     return newNonce;
    // }
    uint16_t generateDevNonce()
    {
        uint16_t nonce;
        bool isUnique = false;

        while (!isUnique)
        {
            // Generate a random nonce between 1 and 0xFFFF
            nonce = (std::rand() % 0xFFFF) + 1;

            // Check that it hasn't been used before
            if (std::find(usedNonces.begin(), usedNonces.end(), nonce) == usedNonces.end())
            {
                isUnique = true;
                usedNonces.push_back(nonce);

                // Keep the size of the nonce list under control
                if (usedNonces.size() > 100)
                {
                    usedNonces.erase(usedNonces.begin());
                }
            }
        }

        return nonce;
    }

    void resetDevNonces()
    {
        usedNonces.clear();
        lastDevNonce = 0;
    }

    // Add ADR statistics
    std::deque<float> snrHistory;
    std::deque<int> rssiHistory;
    
    float getAverageSnr() const {
        if (snrHistory.empty()) return 0;
        float sum = 0;
        for (auto snr : snrHistory) sum += snr;
        return sum / snrHistory.size();
    }
    
    int getAverageRssi() const {
        if (rssiHistory.empty()) return -120;
        int sum = 0;
        for (auto rssi : rssiHistory) sum += rssi;
        return sum / rssiHistory.size();
    }
    
    void addSnrSample(float snr) {
        snrHistory.push_back(snr);
        if (snrHistory.size() > 10) snrHistory.pop_front();
    }
    
    void addRssiSample(int rssi) {
        rssiHistory.push_back(rssi);
        if (rssiHistory.size() > 10) rssiHistory.pop_front();
    }
};

LoRaWAN::LoRaWAN() : 
    pimpl(new Impl(SPIFactory::createCH341SPI(0))), // Uses CH341SPI by default
    joined(false),
    currentClass(DeviceClass::CLASS_A),
    joinMode(JoinMode::OTAA),
    adrEnabled(false), // Initialize ADR disabled
    adrAckCounter(0)   // Initialize ADR counter
{
    // Initialize duty cycle records
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Start as 24 hours ago
        channelAirTime[i] = 0.0f;
    }
    
    // Configure channel frequencies for the selected region
    for (int i = 0; i < MAX_CHANNELS; i++) {
        channelFrequencies[i] = (i < 8) ? BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region] : 0.0f;
    }
}

LoRaWAN::LoRaWAN(std::unique_ptr<SPIInterface> spi_interface) : 
    pimpl(new Impl(std::move(spi_interface))), // Uses the provided interface
    joined(false),
    currentClass(DeviceClass::CLASS_A),
    joinMode(JoinMode::OTAA),
    adrEnabled(false), // Initialize ADR disabled
    adrAckCounter(0)   // Initialize ADR counter
{
    // Initialize duty cycle records
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Start as 24 hours ago
        channelAirTime[i] = 0.0f;
    }
    
    // Configure channel frequencies for the selected region
    for (int i = 0; i < MAX_CHANNELS; i++) {
        channelFrequencies[i] = (i < 8) ? BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region] : 0.0f;
    }
}

LoRaWAN::~LoRaWAN() = default;

bool LoRaWAN::init(int deviceIndex) {
    if (!pimpl->rfm->begin()) {
        DEBUG_PRINTLN("Failed to initialize RFM95");
        return false;
    }
    // Realizar prueba de comunicación
    if (!pimpl->rfm->testCommunication()) {
        DEBUG_PRINTLN("RFM95 communication failed");
        return false;
    }
    // Configurar el módulo para LoRaWAN
    pimpl->rfm->setFrequency(BASE_FREQ[lora_region]);
    current_channel = 0;
    pimpl->rfm->setTxPower(14, true);
    current_power = 14;
    pimpl->rfm->setSpreadingFactor(9);
    current_sf = 9;
    pimpl->rfm->setBandwidth(125.0);
    current_bw = 125;
    pimpl->rfm->setCodingRate(5);
    current_cr = 5;
    pimpl->rfm->setPreambleLength(8);
    current_preamble = 8;
    pimpl->rfm->setSyncWord(0x34); // LoRaWAN sync word
    current_sync_word = 0x34;
    pimpl->rfm->setLNA(0x23, true);
    current_lna = 0x23;
    pimpl->rfm->setInvertIQ(false);
    updateDataRateFromSF();
    return true;
}

void LoRaWAN::setDeviceClass(DeviceClass deviceClass) {
    currentClass = deviceClass;
    
    // If we change to Class C, configure immediately for continuous reception on RX2
    if (deviceClass == DeviceClass::CLASS_C && joined) {
        DEBUG_PRINTLN("Configuring Class C mode (continuous reception at 869.525 MHz)");
        
        // Configure radio for RX2 window
        pimpl->rfm->standbyMode();
        pimpl->rfm->setFrequency(869.525); // RX2 frequency for EU868
        
        pimpl->rfm->setSpreadingFactor(9);
        pimpl->rfm->setBandwidth(125.0);
        pimpl->rfm->setInvertIQ(true);  // Invert IQ for downlink
        pimpl->rfm->setLNA(1, true);
        
        // Start continuous reception
        pimpl->rfm->setContinuousReceive();
    }
}

void LoRaWAN::setDevEUI(const std::string& devEUI) {
    DEBUG_PRINTLN("Setting DevEUI: " << devEUI);
    
    // Convert hex string to bytes in natural order
    for(size_t i = 0; i < 8 && (i*2+1) < devEUI.length(); i++) {
        std::string byteStr = devEUI.substr(i*2, 2);
        pimpl->devEUI[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: show DevEUI as stored
    if (LoRaWAN::getVerbose()) {
        std::cout << "DevEUI stored: ";
        for(int i = 0; i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(pimpl->devEUI[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
}

void LoRaWAN::setAppEUI(const std::string& appEUI) {
    DEBUG_PRINTLN("Setting AppEUI: " << appEUI);
    
    // Convert hex string to bytes in natural order
    for(size_t i = 0; i < 8 && (i*2+1) < appEUI.length(); i++) {
        std::string byteStr = appEUI.substr(i*2, 2);
        pimpl->appEUI[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: show AppEUI as stored
    if (LoRaWAN::getVerbose()) {
        std::cout << "AppEUI stored: ";
        for(int i = 0; i < 8; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(pimpl->appEUI[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
}

void LoRaWAN::setAppKey(const std::string& appKey) {
    DEBUG_PRINTLN("Setting AppKey: " << appKey);
    
    // Convert hex string to bytes without changing the order
    for(size_t i = 0; i < 16 && i*2+1 < appKey.length(); i++) {
        std::string byteStr = appKey.substr(i*2, 2);
        pimpl->appKey[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: show AppKey as stored
    if (LoRaWAN::getVerbose()) {
        std::cout << "AppKey stored: ";
        for(int i = 0; i < 16; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(pimpl->appKey[i]) << " ";
        }
        std::cout << std::dec << std::endl;
    }
}

void LoRaWAN::setDevAddr(const std::string& devAddr) {
    std::copy(devAddr.begin(), devAddr.end(), pimpl->devAddr.begin());
}

void LoRaWAN::setNwkSKey(const std::string& nwkSKey) {
    std::copy(nwkSKey.begin(), nwkSKey.end(), pimpl->nwkSKey.begin());
}

void LoRaWAN::setAppSKey(const std::string& appSKey) {
    std::copy(appSKey.begin(), appSKey.end(), pimpl->appSKey.begin());
}

bool LoRaWAN::join(JoinMode mode, unsigned long timeout) {
    // Try to load existing session first ONLY if a reset wasn't forced
    if (!joined && pimpl->loadSessionData()) {
        joined = true;
        DEBUG_PRINTLN("Restored previous session");
        return true;
    }
    
    // If no valid session, do normal join
    DEBUG_PRINTLN("Performing new OTAA join...");
    joinMode = mode;
    
    if (mode == JoinMode::OTAA) {
        // Configure radio for Join Request
        pimpl->rfm->standbyMode();  // Standby mode before configuring
        pimpl->rfm->setFrequency(one_channel_gateway ? channelFrequencies[0] : channelFrequencies[rand() % 8]);
        current_channel = pimpl->rfm->getFrequency();
        pimpl->rfm->setTxPower(MAX_POWER[lora_region], true);
        current_power = MAX_POWER[lora_region];
        pimpl->rfm->setSpreadingFactor(9);
        current_sf = 9;
        pimpl->rfm->setBandwidth(125.0);
        current_bw = 125;
        pimpl->rfm->setCodingRate(5);
        current_cr = 5;
        pimpl->rfm->setPreambleLength(8);
        current_preamble = 8;
        pimpl->rfm->setInvertIQ(false);
        pimpl->rfm->setSyncWord(0x34);
        current_sync_word = 0x34;
        pimpl->rfm->setLNA(0x23, true);
        current_lna = 0x23;
        updateDataRateFromSF();

        // Clear interrupt flags
        pimpl->rfm->clearIRQFlags();

        // Prepare and send Join Request
        auto joinRequest = pimpl->buildJoinRequest();
        
        if (!pimpl->rfm->send(joinRequest)) {
            DEBUG_PRINTLN("Failed to send Join Request");
            return false;
        }

        // Configure RX1
        pimpl->rfm->standbyMode();
        pimpl->rfm->setFrequency(channelFrequencies[current_channel]);
        pimpl->rfm->setSpreadingFactor(current_sf);
        pimpl->rfm->setBandwidth(current_bw);
        pimpl->rfm->setInvertIQ(true);
        pimpl->rfm->setLNA(current_lna, true);
        
        // Start continuous receive
        pimpl->rfm->setContinuousReceive();
        
        // Configure RX1
        DEBUG_PRINTLN("Opening RX1 window...");
        // Wait for the packet for 1 second
        auto start = std::chrono::steady_clock::now();
        bool received = false;
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count() < (RECEIVE_DELAY1 + WINDOW_DURATION)) {

            uint8_t flags = pimpl->rfm->getIRQFlags();
            if (flags & RFM95::IRQ_RX_DONE_MASK) {
                if (flags & RFM95::IRQ_PAYLOAD_CRC_ERROR_MASK) {
                    DEBUG_PRINTLN("CRC error in RX1 window");
                } else {
                    auto response = pimpl->rfm->readPayload();
                    if (!response.empty()) {
                        received = true;
                        if (pimpl->processJoinAccept(response)) {
                            joined = true;
                            if (joinCallback) {
                                joinCallback(true);
                            }
                            // If the join was successful, save session
                            if (joined) {
                                pimpl->saveSessionData();
                            }
                            return true;
                        }
                    }
                }
                pimpl->rfm->clearIRQFlagRxDone();
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!received) {
            // Second RX window (RX2)
            DEBUG_PRINTLN("Opening RX2 window...");
            
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(869.525);
            pimpl->rfm->setSpreadingFactor(12);
            pimpl->rfm->setBandwidth(125.0);
            pimpl->rfm->setInvertIQ(true);
            pimpl->rfm->setLNA(1, true);
            
            // Clear flags before RX2
            pimpl->rfm->clearIRQFlags();
            pimpl->rfm->setContinuousReceive();
            
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count() < RECEIVE_DELAY1 + RECEIVE_DELAY2 + WINDOW_DURATION) {
                
                uint8_t flags = pimpl->rfm->getIRQFlags();
                if (flags & RFM95::IRQ_RX_DONE_MASK) {
                    if (flags & RFM95::IRQ_PAYLOAD_CRC_ERROR_MASK) {
                        DEBUG_PRINTLN("CRC error in RX2 window");
                    } else {
                        auto response = pimpl->rfm->readPayload();
                        if (!response.empty()) {
                            if (pimpl->processJoinAccept(response)) {
                                joined = true;
                                if (joinCallback) {
                                    joinCallback(true);
                                }
                                // If the join was successful, save session
                                if (joined) {
                                    pimpl->saveSessionData();
                                }
                                return true;
                            }
                        }
                    }
                    pimpl->rfm->clearIRQFlagRxDone();
                    break;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        DEBUG_PRINTLN("No Join Accept received");
        return false;
    }
    
    // For ABP, only validate that we have the necessary keys
    joined = validateKeys();
    return joined;
}

std::vector<uint8_t> LoRaWAN::encryptPayload(const std::vector<uint8_t> &payload, uint8_t port)
{
    // If there is no payload, return an empty vector
    if (payload.empty())
    {
        return std::vector<uint8_t>();
    }

    // Select key based on port (0 = NwkSKey, others = AppSKey)
    const auto &key = (port == 0) ? pimpl->nwkSKey : pimpl->appSKey;

    // For debugging
    if (getVerbose())
    {
        DEBUG_PRINTLN("Encryption parameters:");
        DEBUG_PRINTLN("  Direction: Uplink (0)");
        DEBUG_PRINT("  DevAddr: ");
        for (int i = 0; i < 4; i++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pimpl->devAddr[i]) << " ");
        }
        DEBUG_PRINTLN("");
        DEBUG_PRINTLN("  FCnt: " << pimpl->uplinkCounter << " (0x" << std::hex << pimpl->uplinkCounter << std::dec << ")");
        DEBUG_PRINT("  Key: ");
        for (const auto &b : key)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ");
        }
        DEBUG_PRINTLN(std::dec);
    }

    // Prepare block A for encryption
    std::array<uint8_t, 16> block_a;
    block_a.fill(0); // Initialize everything to 0

    block_a[0] = 0x00; // 0 for uplinks according to LoRaWAN specification

    // DevAddr in network order (big-endian)
    block_a[1] = pimpl->devAddr[0]; // MSB
    block_a[2] = pimpl->devAddr[1];
    block_a[3] = pimpl->devAddr[2];
    block_a[4] = pimpl->devAddr[3]; // LSB

    // FCnt in little-endian
    block_a[5] = pimpl->uplinkCounter & 0xFF;        // LSB
    block_a[6] = (pimpl->uplinkCounter >> 8) & 0xFF; // MSB
    block_a[7] = 0;                                  // MSB always 0 for 16-bit FCnt
    block_a[8] = 0;                                  // MSB always 0 for 16-bit FCnt

    // Fill and block counter
    // bytes 9-14 are already 0 due to fill(0)
    block_a[15] = 0x01; // Block counter 1

    // For debug, show the original block A
    if (getVerbose())
    {
        DEBUG_PRINT("Original Block A: ");
        for (const auto &b : block_a)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ");
        }
        DEBUG_PRINTLN(std::dec);
    }

    // Encrypt block A with the key to obtain the pad
    std::array<uint8_t, 16> a_encrypted;
    AESCMAC::aes_encrypt(block_a.data(), key.data(), a_encrypted.data());

    // XOR between the payload and the first bytes of A
    std::vector<uint8_t> encrypted(payload.size());
    for (size_t i = 0; i < payload.size(); i++)
    {
        encrypted[i] = payload[i] ^ a_encrypted[i];
    }

    // For debug, show the encrypted payload
    if (getVerbose())
    {
        DEBUG_PRINT("Encrypted payload: ");
        for (const auto &b : encrypted)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ");
        }
        DEBUG_PRINTLN(std::dec);
    }

    return encrypted;
}

std::vector<uint8_t> LoRaWAN::decryptPayload(const std::vector<uint8_t> &payload, uint8_t port)
{
    if (payload.empty())
        return payload;

    // Select the correct key based on the port
    const auto &key = (port == 0) ? pimpl->nwkSKey : pimpl->appSKey;

    // Create block A for AES-CTR decryption
    std::array<uint8_t, 16> block_a;
    block_a[0] = 0x01; // Block type
    block_a[1] = 0x00; // Padding
    block_a[2] = 0x00; // Padding
    block_a[3] = 0x00; // Padding
    block_a[4] = 0x00; // Padding
    block_a[5] = 0x01; // Dirección = 0x01 para downlink

    // Copy DevAddr (little endian)
    std::copy(pimpl->devAddr.begin(), pimpl->devAddr.end(), block_a.begin() + 6);

    // Frame counter (of the downlink)
    uint16_t fcnt = pimpl->downlinkCounter;
    block_a[10] = fcnt & 0xFF;
    block_a[11] = (fcnt >> 8) & 0xFF;
    block_a[12] = 0x00; // FCnt MSB (32 bits)
    block_a[13] = 0x00; // FCnt MSB (32 bits)
    block_a[14] = 0x00; // Padding
    block_a[15] = 0x01; // Block counter

    // Debug to see the decryption parameters
    DEBUG_PRINTLN("Decryption parameters:");
    DEBUG_PRINTLN("  Direction: Downlink (1)");
    DEBUG_PRINT("  DevAddr: ");
    for (int i = 0; i < 4; i++)
    {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(pimpl->devAddr[i]) << " ");
    }
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("  FCnt: " << std::dec << fcnt << " (0x"
                             << std::hex << fcnt << std::dec << ")");
    DEBUG_PRINT("  Key: ");
    for (const auto &byte : key)
    {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINTLN("");

    // Vector to store the decrypted data
    std::vector<uint8_t> decrypted;
    decrypted.reserve(payload.size());

    // Process the payload in blocks of 16 bytes
    for (size_t i = 0; i < payload.size(); i += 16)
    {
        // Generate the S block using AES-128
        std::array<uint8_t, 16> s;
        AESCMAC::aes_encrypt(block_a.data(), key.data(), s.data());

        // Debug of the S block before XOR
        DEBUG_PRINT("S block for XOR: ");
        for (int j = 0; j < 16; j++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(s[j]) << " ");
        }
        DEBUG_PRINTLN("");

        // Debug of the encrypted payload for XOR
        DEBUG_PRINT("Encrypted payload for XOR: ");
        for (size_t j = 0; j < std::min(size_t(16), payload.size() - i); j++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(payload[i + j]) << " ");
        }
        DEBUG_PRINTLN("");

        // XOR with the encrypted payload
        size_t block_size = std::min(size_t(16), payload.size() - i);
        for (size_t j = 0; j < block_size; j++)
        {
            decrypted.push_back(payload[i + j] ^ s[j]);
        }

        // Increment the block counter
        block_a[15]++;
    }

    // Debug: show the result of the decryption
    DEBUG_PRINT("Decrypted payload: ");
    for (const auto &byte : decrypted)
    {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINTLN("");

    return decrypted;
}

float LoRaWAN::calculateTimeOnAir(size_t payload_size) {
    // Extract current parameters
    int sf = pimpl->rfm->getSpreadingFactor();  // Changed . to ->
    float bw = pimpl->rfm->getBandwidth() * 1000; // Convert from kHz to Hz
    int cr = pimpl->rfm->getCodingRate();
    
    // Calculate the number of symbols in preamble
    int preambleSymbols = pimpl->rfm->getPreambleLength() + 4.25;
    
    // Calculate symbol duration (ms)
    double symbolDuration = std::pow(2.0, static_cast<double>(sf)) / bw;
    
    // Calculate size in bits with overhead and encoding
    size_t packet_size = payload_size + 13; // Data + LoRaWAN overhead
    double payloadSymbols = 8 + std::max(std::ceil((8 * packet_size - 4 * sf + 28 + 16) / (4 * sf)) * (cr + 4), 0.0);
    
    // Total time in ms = (preamble + payload) * symbol duration
    float timeOnAir = (preambleSymbols + payloadSymbols) * symbolDuration * 1000;
    
    DEBUG_PRINTLN("Calculated time on air: " << timeOnAir << " ms");
    DEBUG_PRINTLN("Parameters: SF=" << sf << ", BW=" << (bw/1000) << "kHz, CR=4/" << cr);
    DEBUG_PRINTLN("Symbols: preamble=" << preambleSymbols << ", payload=" << payloadSymbols);
    DEBUG_PRINTLN("Symbol duration: " << (symbolDuration*1000) << " ms");
    
    return timeOnAir;
}

bool LoRaWAN::checkDutyCycle(float frequency, size_t payload_size) {
    // Identify the channel based on frequency
    int channel = -1;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (std::abs(frequency - channelFrequencies[i]) < 0.01) {
            channel = i;
            break;
        }   
    }
    
    if (channel == -1) {
        // Channel not found, use channel 0 as default
        channel = 0;
    }
    
    // Get current time
    auto now = std::chrono::steady_clock::now();
    
    // Calculate elapsed time since last use
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastChannelUse[channel]).count();
    
    // Calculate required air time
    float airTime = calculateTimeOnAir(payload_size);
    
    // Calculate minimum wait time based on duty cycle (1%)
    // t_wait = (t_air / duty_cycle) - t_air
    float minWaitTime = (airTime / 0.01) - airTime;
    
    // Check if enough time has passed
    if (elapsed < minWaitTime) {
        DEBUG_PRINTLN("Duty cycle restriction: need to wait " 
                  << (minWaitTime - elapsed) << " ms more on channel " 
                  << channel << " (" << frequency << " MHz)");
        return false;
    }
    
    // Register channel usage
    lastChannelUse[channel] = now;
    channelAirTime[channel] += airTime;
    
    return true;
}

float LoRaWAN::getDutyCycleUsage(int channel) {
    if(channel < 0 || channel >= MAX_CHANNELS) {
        return 0.0f; // Invalid channel
    }
    
    // Calculate elapsed time since last use
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastChannelUse[channel]).count();
    
    // If more than 1 hour has passed, consider duty cycle as 0
    if(elapsed_ms > 3600000) {
        return 0.0f;
    }
    
    // Calculate current duty cycle usage as a percentage
    return (channelAirTime[channel] / 36000.0f) * 100.0f; // Percentage usage in the last hour
}

void LoRaWAN::resetDutyCycle() {
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Start as if it were 24 hours ago
        channelAirTime[i] = 0.0f;
    }
}

bool LoRaWAN::send(const std::vector<uint8_t>& data, uint8_t port, bool confirmed, bool force_duty_cycle) {
    if (!joined) return false;

    // If there's already a confirmation pending, don't allow another confirmed message
    if (confirmed && confirmState == ConfirmationState::WAITING_ACK) {
        DEBUG_PRINTLN("Error: There is already a confirmed message waiting for ACK");
        return false;
    }

    // If we need to send an ACK, add it to the message
    bool ackbit = (confirmState == ConfirmationState::ACK_PENDING);

    // Debug the original payload
    DEBUG_PRINTLN("Preparing uplink packet:");
    DEBUG_PRINT("Data to send: ");
    for(const auto& byte : data) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);

    // Configure radio for uplink
    pimpl->rfm->standbyMode();
    
    // If a single-channel gateway, use that frequency
    if (one_channel_gateway) {
        pimpl->rfm->setFrequency(one_channel_freq);
        pimpl->rfm->setSpreadingFactor(one_channel_sf);
        pimpl->rfm->setBandwidth(one_channel_bw);
        pimpl->rfm->setCodingRate(one_channel_cr);
        pimpl->rfm->setPreambleLength(one_channel_preamble);
        pimpl->rfm->setInvertIQ(false);
        pimpl->rfm->setSyncWord(0x34);
        updateDataRateFromSF();
    } else {
        // Select channel based on best duty cycle availability
        float lowestUsage = 100.0f;
        int bestChannel = 0;
        // Find the channel with the lowest duty cycle usage
        for (int i = 0; i < 8; i++) {  // Only the first 8 channels
            float usage = getDutyCycleUsage(i);
            if (usage < lowestUsage && channelFrequencies[i] > 0) {
                lowestUsage = usage;
                bestChannel = i;
            }
        }
        pimpl->rfm->setFrequency(channelFrequencies[bestChannel]);
        pimpl->rfm->setSpreadingFactor(current_sf);
        pimpl->rfm->setBandwidth(current_bw);
        pimpl->rfm->setCodingRate(current_cr);
        pimpl->rfm->setPreambleLength(current_preamble);
        pimpl->rfm->setInvertIQ(false);
        pimpl->rfm->setSyncWord(0x34);

        DEBUG_PRINTLN("Selected channel " << bestChannel << " with frequency " 
                   << channelFrequencies[bestChannel] << " MHz (usage: " << lowestUsage << "%)");
    }

    // Store current parameters for use in RX1 window
    current_channel = getChannelFromFrequency(pimpl->rfm->getFrequency());
    current_sf = pimpl->rfm->getSpreadingFactor();
    current_bw = pimpl->rfm->getBandwidth();
    current_cr = pimpl->rfm->getCodingRate();
    current_preamble = pimpl->rfm->getPreambleLength();

    // Debug session keys
    DEBUG_PRINT("Using NwkSKey: ");
    for(const auto& byte : pimpl->nwkSKey) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);
    
    DEBUG_PRINT("Using AppSKey: ");
    for(const auto& byte : pimpl->appSKey) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);

    // Build LoRaWAN packet strictly according to specification 1.0.4
    std::vector<uint8_t> packet;

    // If there are pending MAC responses, we should include them
    bool hasPendingMAC = !pendingMACResponses.empty();

    // MHDR: Unconfirmed (0x40) o Confirmed (0x80) Data Up
    uint8_t mhdr = confirmed ? 0x80 : 0x40;

    // If we need to add an ACK, activate the ACK bit (0x20)
    if (ackbit)
    {
        mhdr |= 0x20;
        DEBUG_PRINTLN("Adding ACK bit to outgoing message");
    }

    packet.push_back(mhdr);

    //  FHDR in the correct order according to specification section 4.3.1:
    // 1. DevAddr (4 bytes)
    packet.insert(packet.end(), pimpl->devAddr.begin(), pimpl->devAddr.end());
    
    // 2. FCtrl (1 byte) - BEFORE FCnt
    // Configure FCtrl for ADR
    uint8_t fctrl = 0x00;

    // If there are pending MAC commands, include them in FOpts
    // Include length of FOpts (maximum 15 bytes)
    if (hasPendingMAC)
    {
        uint8_t fopts_len = std::min(static_cast<size_t>(15), pendingMACResponses.size());
        fctrl |= fopts_len & 0x0F;
        DEBUG_PRINTLN("Including " << static_cast<int>(fopts_len) << " bytes of MAC commands in FOptsLen");
    }

    // Configure FCtrl for ADR
    if (adrEnabled) {
        fctrl |= 0x80; // bit 7 = ADR
        
    }

    // ADR ACK request if necessary
    if (adrAckCounter >= ADR_ACK_LIMIT) {
        fctrl |= 0x40; // bit 6 = ADR ACK REQ
        DEBUG_PRINTLN("Sending ADR ACK request (counter: " << adrAckCounter << ")");
    }

    // Bit ACK if necessary
    if (needsAck || ackbit)
    {
        fctrl |= 0x20; // Bit 5 = ACK
        DEBUG_PRINTLN("Adding ACK bit in FCtrl (0x" << std::hex << (int)fctrl << std::dec << ")");
    }

    packet.push_back(fctrl);
    
    // 3. FCnt (2 bytes, little-endian) - AFTER FCtrl
    packet.push_back(pimpl->uplinkCounter & 0xFF);        // FCnt LSB
    packet.push_back((pimpl->uplinkCounter >> 8) & 0xFF); // FCnt MSB

    // FOpts: MAC commands if there are any pending (up to 15 bytes)
    if (hasPendingMAC)
    {
        size_t mac_size = std::min(static_cast<size_t>(15), pendingMACResponses.size());
        DEBUG_PRINTLN("Adding " << mac_size << " bytes of MAC commands in FOpts");

        for (size_t i = 0; i < mac_size; i++)
        {
            packet.push_back(pendingMACResponses[i]);
        }

        // Debug: show what MAC commands are being sent
        DEBUG_PRINT("MAC commands sent: ");
        for (size_t i = 0; i < mac_size; i++)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << static_cast<int>(pendingMACResponses[i]) << " ");
        }
        DEBUG_PRINTLN(std::dec);

        // Clear sent MAC commands
        pendingMACResponses.erase(pendingMACResponses.begin(), pendingMACResponses.begin() + mac_size);
    }

    // 4. FPort (1 byte)
    packet.push_back(port);
    
    // 5. FRMPayload (encrypted)
    auto encrypted = encryptPayload(data, port);
    packet.insert(packet.end(), encrypted.begin(), encrypted.end());

    // Detailed debug of the packet before the MIC
    DEBUG_PRINTLN("Packet structure:");
    DEBUG_PRINTLN("  MHDR: " << std::hex << static_cast<int>(packet[0]));
    DEBUG_PRINT("  DevAddr: ");
    for(int i = 1; i <= 4; i++) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                << static_cast<int>(packet[i]) << " ");
    }
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("  FCtrl: " << std::hex << static_cast<int>(packet[5]));
    DEBUG_PRINTLN("  FCnt: " << std::hex << static_cast<int>(packet[6]) << " " 
             << static_cast<int>(packet[7]));
    DEBUG_PRINTLN("  FPort: " << std::hex << static_cast<int>(packet[8]));
    DEBUG_PRINT("  Encrypted Payload: ");
    for(size_t i = 9; i < packet.size(); i++) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                << static_cast<int>(packet[i]) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);
    
    // CCalculation of MIC according to spec 4.4
    // B0 block exactly as defined by the specification
    std::array<uint8_t, 16> b0;
    std::fill(b0.begin(), b0.end(), 0);
    b0[0] = 0x49;  // Block code for MIC
    // bytes 1-4 are 0x00 (reserved)
    b0[5] = 0x00;  // Dir = 0 for uplink
    
    // DevAddr (4 bytes, little-endian)
    std::copy(pimpl->devAddr.begin(), pimpl->devAddr.end(), b0.begin() + 6);
    
    // FCnt (4 bytes, little-endian, the upper 2 bytes are 0)
    b0[10] = pimpl->uplinkCounter & 0xFF;
    b0[11] = (pimpl->uplinkCounter >> 8) & 0xFF;
    // bytes 12-13 are 0x00 (FCntMSB = 0)
    
    // byte 14 is 0x00 (always 0 for small messages)
    b0[15] = static_cast<uint8_t>(packet.size());  // Length of the message

    // Debug of the B0 block
    DEBUG_PRINT("B0 block for MIC: ");
    for(const auto& byte : b0) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);

    // Build the block for CMAC
    std::vector<uint8_t> cmacData;
    cmacData.insert(cmacData.end(), b0.begin(), b0.end());
    cmacData.insert(cmacData.end(), packet.begin(), packet.end());
    
    // Calculate CMAC with NwkSKey
    auto cmac = AESCMAC::calculate(cmacData, pimpl->nwkSKey);
    
    // Add the first 4 bytes as MIC
    packet.insert(packet.end(), cmac.begin(), cmac.begin() + 4);
    
    // Calculate the size of the packet to estimate air time
    size_t packetSize = data.size() + 13; // Data + overhead LoRaWAN
    
    // Verify duty cycle if not forced
    float frequency = 868.1; // For one-channel gateway, we always use this frequency
    
    if (!force_duty_cycle && !checkDutyCycle(frequency, packetSize)) {
        DEBUG_PRINTLN("Duty cycle restriction active, delaying transmission");
        // Wait the necessary time
        int channel = 0; // Single channel gateway always uses channel 0
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastChannelUse[channel]).count();
        
        float airTime = calculateTimeOnAir(packetSize);
        float minWaitTime = (airTime / 0.01) - airTime; // 1% duty cycle
        
        if (elapsed < minWaitTime) {
            unsigned long wait_ms = static_cast<unsigned long>(minWaitTime - elapsed);
            DEBUG_PRINTLN("Waiting " << wait_ms << " ms for duty cycle...");
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        }
    }
    
    // Debug radio parameters for verification
    DEBUG_PRINTLN("Radio parameters:");
    DEBUG_PRINTLN("  Frequency: " << pimpl->rfm->getFrequency() << " MHz");
    DEBUG_PRINTLN("  SF: " << pimpl->rfm->getSpreadingFactor());
    DEBUG_PRINTLN("  BW: " << pimpl->rfm->getBandwidth() << " kHz");
    DEBUG_PRINTLN("  CR: 4/" << pimpl->rfm->getCodingRate());
    DEBUG_PRINTLN("  Power: " << pimpl->rfm->getTxPower() << " dBm");
    DEBUG_PRINTLN("Sending packet...");
    
    // Ensure the packet is sent correctly by checking radio status
    pimpl->rfm->clearIRQFlags();
    uint8_t opMode = pimpl->rfm->readRegister(RFM95::REG_OP_MODE);
    DEBUG_PRINTLN("Mode before TX: 0x" << std::hex << (int)opMode << std::dec);
    
    // Transmit the packet
    bool result = pimpl->rfm->send(packet);
    
    // Check result even if the flag isn't updated
    if (result) {
        DEBUG_PRINTLN("Packet sending completed");
        pimpl->rfm->standbyMode();
        // Increment counter and save session
        pimpl->uplinkCounter++;

        // Save the timestamp of the last uplink
        pimpl->txEndTime = std::chrono::steady_clock::now();
        setupRxWindows(); // Configure RX1 and RX2 windows

        // Increment ADR counter if enabled
        if (adrEnabled) {
            adrAckCounter++;
            
            // Reduce DR (increase SF) if we haven't received a response in a while
            if (adrAckCounter > ADR_ACK_LIMIT + ADR_ACK_DELAY) {
                updateTxParamsForADR();
            }
        }
        
        pimpl->saveSessionData();
        // // Wait 6 seconds to comply with duty cycle (1% on 868.1 MHz)
        // DEBUG_PRINTLN("Waiting 6 seconds for duty cycle...");
        // std::this_thread::sleep_for(std::chrono::seconds(6));
        
        // Return to continuous reception mode with appropriate configuration based on class
        if (currentClass == DeviceClass::CLASS_C) {
            DEBUG_PRINTLN("Configuring continuous reception at RX2 (869.525 MHz, Class C)");
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);
            pimpl->rfm->setSpreadingFactor(RX2_SF[lora_region]);
            pimpl->rfm->setBandwidth(RX2_BW[lora_region]);
            pimpl->rfm->setCodingRate(RX2_CR[lora_region]);
            pimpl->rfm->setPreambleLength(RX2_PREAMBLE[lora_region]);
            pimpl->rfm->setInvertIQ(true);      // Inverted IQ for downlink
            pimpl->rfm->setContinuousReceive();
        } else {
            // For Class A, configure RX1 normally
            pimpl->rfm->setFrequency(channelFrequencies[current_channel]);
            pimpl->rfm->setSpreadingFactor(current_sf);
            pimpl->rfm->setBandwidth(current_bw);
            pimpl->rfm->setCodingRate(current_cr);
            pimpl->rfm->setPreambleLength(current_preamble);
            pimpl->rfm->setInvertIQ(false);
            pimpl->rfm->setContinuousReceive();
            DEBUG_PRINTLN("Returning to standby mode (Class A)");
            pimpl->rfm->standbyMode();
        }
    } else {
        DEBUG_PRINTLN("Error sending packet");

        // In case of error, don't configure RX windows
        pimpl->rxState = RX_IDLE;

        // For Class C, return to continuous listening on RX2
        if (currentClass == DeviceClass::CLASS_C)
        {
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);
            pimpl->rfm->setSpreadingFactor(RX2_SF[lora_region]);
            pimpl->rfm->setBandwidth(RX2_BW[lora_region]);
            pimpl->rfm->setCodingRate(RX2_CR[lora_region]);
            pimpl->rfm->setPreambleLength(RX2_PREAMBLE[lora_region]);
            pimpl->rfm->setInvertIQ(true);
            pimpl->rfm->setContinuousReceive();
        }
    }

    // If it's a confirmed message and was sent successfully, update the state
    if (confirmed && result)
    {
        confirmState = ConfirmationState::WAITING_ACK;
        confirmRetries++;
        lastConfirmAttempt = std::chrono::steady_clock::now();
        pendingAck = data;
        ackPort = port;
        DEBUG_PRINTLN("Confirmed message sent, waiting for ACK. Attempt: " << confirmRetries);
    }

    // If we had to ACK a message and have sent it, reset
    if (ackbit && result)
    {
        resetConfirmationState();
    }

    return result;
}

void LoRaWAN::update() {
    if (!joined) return;

    // Manage reception windows
    updateRxWindows();

    // Manage pending confirmations
    handleConfirmation();

    // Check if the class has changed or we need to restart continuous listening
    uint8_t opMode = pimpl->rfm->readRegister(RFM95::REG_OP_MODE);

    // Only reconfigure if we're not already in continuous RX mode
    if ((opMode & 0x07) != RFM95::MODE_RX_CONTINUOUS &&
        pimpl->rxState != RX_WINDOW_1 && pimpl->rxState != RX_WINDOW_2) {
        // If we're in Class C, always listen on RX2
        if (currentClass == DeviceClass::CLASS_C) {
            // Configure for RX2
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);
            pimpl->rfm->setSpreadingFactor(RX2_SF[lora_region]);
            pimpl->rfm->setBandwidth(RX2_BW[lora_region]);
            pimpl->rfm->setCodingRate(RX2_CR[lora_region]);
            pimpl->rfm->setPreambleLength(RX2_PREAMBLE[lora_region]);
            pimpl->rfm->setInvertIQ(true);  // Invert IQ for downlink
            pimpl->rfm->setContinuousReceive();
            pimpl->rxState = RX_CONTINUOUS;
            DEBUG_PRINTLN("Radio reconfigured for continuous RX2 at " << RX2_FREQ[lora_region] << " MHz (SF" << RX2_SF[lora_region] << ")");
        } else {
            // For Class A, configure at the main frequency
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(channelFrequencies[current_channel]);
            pimpl->rfm->setSpreadingFactor(current_sf);
            pimpl->rfm->setBandwidth(current_bw);
            pimpl->rfm->setCodingRate(current_cr);
            pimpl->rfm->setPreambleLength(current_preamble);
            pimpl->rfm->setInvertIQ(true);
            pimpl->rfm->setContinuousReceive();
            DEBUG_PRINTLN("Returning to standby mode (Class A)");
        }
    }

    // Check if there is received data by checking IRQ flags
    uint8_t flags = pimpl->rfm->getIRQFlags();
    
    if (flags & RFM95::IRQ_RX_DONE_MASK) {
        DEBUG_PRINTLN("Packet reception detected!");
        
        // Check if there's a CRC error
        if (flags & RFM95::IRQ_PAYLOAD_CRC_ERROR_MASK) {
            DEBUG_PRINTLN("CRC error in received packet");
        } else {
            // Show detailed information about the packet
            int rssi = pimpl->rfm->getRSSI();
            float snr = pimpl->rfm->getSNR();
            
            auto payload = pimpl->rfm->readPayload();
            if (!payload.empty()) {
                DEBUG_PRINTLN("Packet received: " << payload.size() << " bytes, RSSI: " 
                          << rssi << " dBm, SNR: " << snr << " dB");
                DEBUG_PRINT("Hex: ");
                for (const auto& b : payload) {
                    DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << (int)b);
                }
                DEBUG_PRINTLN(std::dec);
                
                // Process the packet if it's a valid LoRaWAN format
                if (payload.size() >= 13) {
                    Message msg;
                    uint8_t mhdr = payload[0];
                    
                    // Check if it's a downlink (0x60 = unconfirmed, 0xA0 = confirmed)
                    if ((mhdr & 0xE0) == 0x60 || (mhdr & 0xE0) == 0xA0) {
                        // Extract DevAddr for verification
                        std::array<uint8_t, 4> recvDevAddr;
                        std::copy(payload.begin() + 1, payload.begin() + 5, recvDevAddr.begin());
                        
                        // Verify that the DevAddr matches ours
                        bool addressMatch = std::equal(recvDevAddr.begin(), recvDevAddr.end(), pimpl->devAddr.begin());
                        
                        if (addressMatch) {
                            // Collect statistics for ADR
                            pimpl->addSnrSample(snr);
                            pimpl->addRssiSample(rssi);
                            
                            // Use handleReceivedMessage to process the message
                            handleReceivedMessage(payload, msg);
                            
                            // Notify via callback
                            if (receiveCallback) {
                                receiveCallback(msg);
                            } else {
                                // Save in the queue
                                std::lock_guard<std::mutex> lock(pimpl->queueMutex);
                                pimpl->rxQueue.push(msg);
                            }

                            // Check if there are MAC commands in FPort 0 or in FOpts
                            if (payload.size() > 8)
                            {
                                uint8_t fctrl = payload[5];
                                uint8_t fopts_len = fctrl & 0x0F;

                                if (fopts_len > 0)
                                {
                                    // There are MAC commands in FOpts
                                    DEBUG_PRINTLN("Detected " << static_cast<int>(fopts_len) << " bytes of MAC commands in FOpts");

                                    // Extract MAC commands from FOpts (after FCnt)
                                    std::vector<uint8_t> macCommands(payload.begin() + 8, payload.begin() + 8 + fopts_len);

                                    // Process commands and generate response
                                    std::vector<uint8_t> macResponse;
                                    processMACCommands(macCommands, macResponse);

                                    // Store response to include in next uplink
                                    if (!macResponse.empty())
                                    {
                                        pendingMACResponses = macResponse;
                                        DEBUG_PRINTLN("MAC response saved for next uplink: " << macResponse.size() << " bytes");
                                    }
                                }

                                // Also check if there are commands in FPort 0
                                if (payload.size() > 8 && payload[8] == 0 && payload.size() > 9)
                                {
                                    // FPort 0 indicates that the payload contains MAC commands
                                    std::vector<uint8_t> encrypted(payload.begin() + 9, payload.end() - 4);
                                    std::vector<uint8_t> macCommands = decryptPayload(encrypted, 0);

                                    DEBUG_PRINT("Received MAC commands in FPort 0: ");
                                    for (const auto &b : macCommands)
                                    {
                                        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ");
                                    }
                                    DEBUG_PRINTLN(std::dec);

                                    // Process commands and generate response
                                    std::vector<uint8_t> macResponse;
                                    processMACCommands(macCommands, macResponse);

                                    // Store response to include in next uplink
                                    if (!macResponse.empty())
                                    {
                                        pendingMACResponses = macResponse;
                                        DEBUG_PRINTLN("MAC response (FPort 0) saved for next uplink: " << macResponse.size() << " bytes");
                                    }
                                }
                            }
                        } else {
                            DEBUG_PRINTLN("DevAddr doesn't match, ignoring packet");
                        }
                    }
                }
            }
        }
        
        // Clear flag and reconfigure to continue receiving
        pimpl->rfm->clearIRQFlagRxDone();
        pimpl->rfm->setContinuousReceive();
    }
}

bool LoRaWAN::receive(Message& message, unsigned long timeout) {
    if (!joined) return false;

    auto data = pimpl->rfm->receive(timeout / 1000.0);
    if (!data.empty()) {
        // Extraer información del encabezado
        message.port = data[8];
        message.confirmed = (data[0] & 0x20) != 0;
        
        // Extraer y decriptar payload
        std::vector<uint8_t> encrypted_payload(data.begin() + 9, data.end() - 4);
        message.payload = decryptPayload(encrypted_payload, message.port);
        
        return true;
    }
    
    return false;
}

void LoRaWAN::onReceive(std::function<void(const Message&)> callback) {
    receiveCallback = callback;
}

void LoRaWAN::onJoin(std::function<void(bool)> callback) {
    joinCallback = callback;
}

void LoRaWAN::setRegion(int region) {
    if (region >= 0 && region < REGIONS) {
        lora_region = region;
        pimpl->rfm->setFrequency(BASE_FREQ[region]);
        current_channel = 0; // Canal 0 por defecto
        
        // Actualizar canales según la región
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channelFrequencies[i] = (i < 8) ? BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region] : 0.0f;
        }
    }
}

int LoRaWAN::getRegion() const {
    return lora_region;
}

float LoRaWAN::getFrequency() const {
    return pimpl->rfm->getFrequency();
}

void LoRaWAN::setFrequency(float freq_mhz) {
    // Verificar si la frecuencia está en un canal permitido
    int channel = getChannelFromFrequency(freq_mhz);
    if (channel >= 0) {
        pimpl->rfm->setFrequency(freq_mhz);
    } else {
        DEBUG_PRINTLN("Invalid frequency or not allowed on any channel");
    }
}

int LoRaWAN::getChannelFromFrequency(float freq_mhz) const {
    if (lora_region < 0 || lora_region >= REGIONS) {
        DEBUG_PRINTLN("Invalid LoRa region");
        return -1;
    }
    
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (std::abs(channelFrequencies[i] - freq_mhz) < 0.01) {
            return i;
        }
    }
    return -1;
}

float LoRaWAN::getFrequencyFromChannel(int channel) const {
    if (channel >= 0 && channel < MAX_CHANNELS) {
        return channelFrequencies[channel];
    }
    return 0.0f;
}

void LoRaWAN::setChannel(uint8_t channel) {
    if (channel < MAX_CHANNELS && channelFrequencies[channel] > 0) {
        pimpl->channel = channel;
        // Set the corresponding frequency in the radio
        pimpl->rfm->setFrequency(channelFrequencies[channel]);
    } else {
        DEBUG_PRINTLN("Invalid or disabled channel: " << channel);
    }
}

uint8_t LoRaWAN::getChannel() const {
    return pimpl->channel; // Return directly the stored channel
}

void LoRaWAN::setSingleChannel(bool enable, float freq_mhz, int sf, int bw, int cr, int power, int preamble)
{
    one_channel_gateway = enable;
    one_channel_freq = freq_mhz;
    one_channel_sf = sf;
    one_channel_bw = bw;
    one_channel_cr = cr;
    one_channel_power = power;
    one_channel_preamble = preamble;
}

bool LoRaWAN::getSingleChannel() const
{
    return one_channel_gateway;
}

float LoRaWAN::getSingleChannelFrequency() const {
    return one_channel_freq;
}

void LoRaWAN::setTxPower(int8_t power) {
    // Limit between 2 dBm and the maximum allowed by the region
    if (power < 2) power = 2; // Most LoRa modules do not go below 2 dBm
    if (power > MAX_POWER[lora_region]) power = MAX_POWER[lora_region];
    
    pimpl->txPower = power;
    pimpl->rfm->setTxPower(power, true); // true = PA_BOOST
}

int LoRaWAN::getRSSI() const {
    return pimpl->rfm->getRSSI();
}

int LoRaWAN::getSNR() const {
    return pimpl->rfm->getSNR();
}

uint32_t LoRaWAN::getFrameCounter() const {
    return pimpl->uplinkCounter;
}

void LoRaWAN::setFrameCounter(uint32_t counter) {
    pimpl->uplinkCounter = counter;
}

void LoRaWAN::wake() {
    pimpl->rfm->standbyMode();
}

void LoRaWAN::sleep() {
    pimpl->rfm->sleepMode();
}

bool LoRaWAN::validateKeys() const {
    // Validate DevAddr - shouldn't be all zeros
    bool validDevAddr = false;
    for (const auto& byte : pimpl->devAddr) {
        if (byte != 0) {
            validDevAddr = true;
            break;
        }
    }
    if (!validDevAddr) {
        DEBUG_PRINTLN("ABP validation failed: DevAddr is all zeros");
        std::cerr << "Error: Invalid DevAddr for ABP" << std::endl;
        return false;
    }
    
    // Validate NwkSKey - shouldn't be all zeros
    bool validNwkSKey = false;
    for (const auto& byte : pimpl->nwkSKey) {
        if (byte != 0) {
            validNwkSKey = true;
            break;
        }
    }
    if (!validNwkSKey) {
        DEBUG_PRINTLN("ABP validation failed: NwkSKey is all zeros");
        std::cerr << "Error: Invalid NwkSKey for ABP" << std::endl;
        return false;
    }
    
    // Validate AppSKey - shouldn't be all zeros
    bool validAppSKey = false;
    for (const auto& byte : pimpl->appSKey) {
        if (byte != 0) {
            validAppSKey = true;
            break;
        }
    }
    if (!validAppSKey) {
        DEBUG_PRINTLN("ABP validation failed: AppSKey is all zeros");
        std::cerr << "Error: Invalid AppSKey for ABP" << std::endl;
        return false;
    }
    
    DEBUG_PRINTLN("ABP validation successful");
    DEBUG_PRINT("  DevAddr: ");
    for (const auto& byte : pimpl->devAddr) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);
    
    return true;
}

void LoRaWAN::enableADR(bool enable) {
    adrEnabled = enable;
    DEBUG_PRINTLN("ADR " << (enable ? "enabled" : "disabled"));
}

bool LoRaWAN::isADREnabled() const {
    return adrEnabled;
}

void LoRaWAN::resetSession()
{
    // Clear session keys
    pimpl->devAddr.fill(0);
    pimpl->nwkSKey.fill(0);
    pimpl->appSKey.fill(0);

    // Reset flags and counters
    pimpl->uplinkCounter = 0;
    pimpl->downlinkCounter = 0;
    joined = false;

    // Delete session file if it exists
    SessionManager::clearSession(pimpl->sessionFile);

    // Reset DevNonces
    pimpl->resetDevNonces();

    // Also reset ADR statistics
    pimpl->snrHistory.clear();
    pimpl->rssiHistory.clear();
    adrAckCounter = 0;

    DEBUG_PRINTLN("LoRaWAN session reset successfully");
}

void LoRaWAN::applyADRSettings(uint8_t dataRate, uint8_t txPower, const std::vector<uint8_t>& channelMask) {

    // Map DR to SF/BW based on the region
    int sf = 9; // Default value
    float bw = 125.0f;

    // Determine SF and BW based on the region and DR
    switch (lora_region) {
        case REGION_EU868:
            if (dataRate < 6) {
                sf = 12 - dataRate;
                bw = 125.0f;
            }
            else if (dataRate == 6) {
                sf = 7;
                bw = 250.0f;
            }
            else { // DR7
                sf = 7;
                bw = 125.0f;
            }
            break;
            
        // Add other regions as necessary
        default:
            // Generic mapping for other regions
            sf = dataRate <= 6 ? (12 - dataRate) : 7;
            bw = dataRate == 6 ? 250.0f : 125.0f;
    }

    // Determine power based on the region
    int power = 14; // Default value
    switch (lora_region) {
        case REGION_EU868:
            power = MAX_POWER[lora_region] - (txPower * 2);
            break;
        // Add other regions as necessary
        default:
            power = MAX_POWER[lora_region] - (txPower * 2);
    }
    
    // Limit to safe values
    if (sf < 7) sf = 7;
    if (sf > 12) sf = 12;
    if (power < 2) power = 2;
    if (power > MAX_POWER[lora_region]) power = MAX_POWER[lora_region];
    
    // Apply settings to the radio using the pimpl (this is done in LoRaWAN.cpp)
    pimpl->rfm->setSpreadingFactor(sf);
    current_sf = sf;
    pimpl->rfm->setBandwidth(bw);
    current_bw = bw;
    pimpl->rfm->setTxPower(power, true);
    pimpl->txPower = power;
    updateDataRateFromSF();
    DEBUG_PRINTLN("ADR settings applied: DataRate=" << static_cast<int>(dataRate) << ", TxPower=" << power);
}

void LoRaWAN::processMACCommands(const std::vector<uint8_t> &commands, std::vector<uint8_t> &response)
{
    DEBUG_PRINTLN("Processing MAC commands:");
    for (size_t i = 0; i < commands.size(); i++)
    {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(commands[i]) << " ");
    }
    DEBUG_PRINTLN(std::dec);

    size_t index = 0;

    while (index < commands.size())
    {
        uint8_t cmd = commands[index++];

        switch (cmd)
        {
        case MAC_LINK_ADR_REQ:
            DEBUG_PRINTLN("Received LinkADR command:");
            DEBUG_PRINTLN("  DataRate_TXPower: 0x" << std::hex
                                                   << static_cast<int>(commands[index + 1]));
            DEBUG_PRINTLN("  ChMask: 0x" << std::hex
                                         << static_cast<int>(commands[index + 2])
                                         << static_cast<int>(commands[index + 3]));
            DEBUG_PRINTLN("  Redundancy: 0x" << std::hex
                                             << static_cast<int>(commands[index + 4]) << std::dec);

            if (index + 4 <= commands.size())
            {
                processLinkADRReq(commands, index - 1, response);
                index += 4; // Skip the parameters
            }
            break;

        case MAC_DUTY_CYCLE_REQ:
            if (index < commands.size())
            {
                uint8_t maxDutyCycle = commands[index++];
                float dutyCycle = maxDutyCycle == 0 ? 1.0 : 1.0 / (1 << maxDutyCycle);

                DEBUG_PRINTLN("DutyCycle: MaxDutyCycle=" << static_cast<int>(maxDutyCycle)
                                                         << ", duty cycle=" << (dutyCycle * 100) << "%");

                // Always accept duty cycle
                response.push_back(MAC_DUTY_CYCLE_ANS);
            }
            break;

        case MAC_DEV_STATUS_REQ:
            // Respond with device status (battery and signal margin)
            {
                DEBUG_PRINTLN("Received DEV_STATUS_REQ command");
                // Add response command
                response.push_back(MAC_DEV_STATUS_ANS);

                // Add battery level (0-254, 0=external power, 1=min, 254=max, 255=not measured)
                uint8_t battery = 254; // Use a real value if you have a battery sensor
                response.push_back(battery);

                // Add signal margin (-32...31 dB)
                float snr = pimpl->rfm->getSNR();
                int8_t margin = static_cast<int8_t>(std::max(-32.0f, std::min(31.0f, snr)));
                response.push_back(static_cast<uint8_t>(margin));

                DEBUG_PRINTLN("Responding DEV_STATUS: Battery=" << static_cast<int>(battery)
                                                                << ", Margin=" << static_cast<int>(margin) << " dB");
            }
            break;

        case MAC_LINK_CHECK_ANS:
            if (index + 1 < commands.size())
            {
                uint8_t margin = commands[index++];
                uint8_t gwCount = commands[index++];

                DEBUG_PRINTLN("Received LINK_CHECK_ANS: Margin=" << static_cast<int>(margin)
                                                                 << " dB, GW Count=" << static_cast<int>(gwCount));

                // Here you could store these values or notify via a callback
            }
            break;

        case MAC_RX_PARAM_SETUP_REQ:
            DEBUG_PRINTLN("Received RX_PARAM_SETUP_REQ");

            if (index + 3 <= commands.size())
            {
                uint8_t dlSettings = commands[index++];
                uint8_t frequency_msb = commands[index++];
                uint8_t frequency_mid = commands[index++];
                uint8_t frequency_lsb = commands[index++];

                // Extract parameters
                rx1DrOffset = (dlSettings >> 4) & 0x07; // Upper 3 bits of DLSettings
                rx2DataRate = dlSettings & 0x0F;        // Lower 4 bits of DLSettings

                // Calculate RX2 frequency (24 bits, in multiples of 100 Hz)
                uint32_t freq_value = (frequency_msb << 16) | (frequency_mid << 8) | frequency_lsb;
                float rx2_freq = static_cast<float>(freq_value) / 10000.0f; // Convert to MHz

                DEBUG_PRINTLN("  RX1DrOffset=" << static_cast<int>(rx1DrOffset)
                                               << ", RX2DataRate=" << static_cast<int>(rx2DataRate)
                                               << ", RX2 Freq=" << rx2_freq << " MHz");

                // Response: status bits
                uint8_t status = 0x07; // Default all bits to 1 (success)

                // Validate parameters
                bool rx1DrOffsetOK = true;
                bool rx2DataRateOK = true;
                bool channelOK = true;

                // Validate RX1 DR Offset (0-7)
                if (rx1DrOffset > 7)
                {
                    rx1DrOffsetOK = false;
                    status &= ~0x04; // Error in RX1DrOffset
                }

                // Validate RX2 DataRate based on region
                int maxDR = 7; // Default for EU868
                switch (lora_region)
                {
                case REGION_EU868:
                    maxDR = 7;
                    break;
                case REGION_US915:
                    maxDR = 4;
                    break;
                    // Add other regions as needed
                }

                if (rx2DataRate > maxDR)
                {
                    rx2DataRateOK = false;
                    status &= ~0x02; // Error in RX2 DataRate
                }

                // Validate frequency channel
                if (rx2_freq < 100.0f || rx2_freq > 1000.0f)
                {
                    // Arbitrary values for example, adjust according to region
                    channelOK = false;
                    status &= ~0x01; // Error in channel
                }

                // If all OK, apply changes
                if (status == 0x07)
                {
                    // Store the old RX2 frequency
                    float old_rx2_freq = RX2_FREQ[lora_region];

                    // Update internal RX2 configuration
                    // TODO: You might add attributes to store custom RX2 frequency

                    DEBUG_PRINTLN("RX parameters updated: RX1DrOffset=" << static_cast<int>(rx1DrOffset)
                                                                        << ", RX2DataRate=" << static_cast<int>(rx2DataRate)
                                                                        << ", changing RX2 from " << old_rx2_freq << " to " << rx2_freq << " MHz");
                }
                else
                {
                    DEBUG_PRINTLN("Invalid RX parameters, status=" << std::bitset<3>(status));
                }

                // Add response
                response.push_back(MAC_RX_PARAM_SETUP_ANS);
                response.push_back(status);
            }
            break;

        default:
            // Skip unrecognized commands
            DEBUG_PRINTLN("Unrecognized MAC command: 0x" << std::hex << static_cast<int>(cmd));
            break;
        }
    }
}

void LoRaWAN::processLinkADRReq(const std::vector<uint8_t> &cmd, size_t index, std::vector<uint8_t> &response)
{
    if (index + 4 >= cmd.size())
        return;

    uint8_t datarate_txpower = cmd[index + 1];
    uint8_t dr = (datarate_txpower >> 4) & 0x0F; // 4 bits most significant
    uint8_t txpower = datarate_txpower & 0x0F;   // 4 bits least significant
    uint16_t chmask = (cmd[index + 3] << 8) | cmd[index + 2];
    uint8_t redundancy = cmd[index + 4];
    uint8_t chmaskcntl = (redundancy >> 4) & 0x07;
    uint8_t nbRep = redundancy & 0x0F;

    if (nbRep < 1)
        nbRep = 1;
    if (nbRep > 15)
        nbRep = 15;

    DEBUG_PRINTLN("LinkADRReq received:");
    DEBUG_PRINT("  Raw DataRate_TXPower byte: 0x" << std::hex << (int)datarate_txpower << std::dec << "\n");
    DEBUG_PRINT("  Binary: " << std::bitset<8>(datarate_txpower) << "\n");
    DEBUG_PRINTLN("  DR=" << (int)dr << " (SF" << (dr < 6 ? 12 - dr : 7) << ")");
    DEBUG_PRINTLN("  BW=" << (dr == 6 ? 250 : 125) << "kHz");
    DEBUG_PRINTLN("  TXPower=" << (int)txpower << " (" << (14 - 2 * txpower) << "dBm)");
    DEBUG_PRINTLN("  ChMask=0x" << std::hex << chmask << std::dec);
    DEBUG_PRINTLN("  ChMaskCntl=" << (int)chmaskcntl);
    DEBUG_PRINTLN("  NbRep=" << (int)nbRep);

    //  Construct response
    uint8_t status = 0b111; // Bits: Channel mask OK, Data rate OK, Power OK

    // Check validity of data rate according to region
    int maxDR = 5; // Default for most regions
    switch (lora_region)
    {
    case REGION_US915:
    case REGION_AU915:
        maxDR = 4;
        break;
    case REGION_EU868:
    case REGION_EU433:
        maxDR = 7; // Default for EU868 (SF12-SF7 + modulation variants)
        break;
        // Other regions as needed
    }

    if (dr > maxDR)
    {
        status &= ~0x04; // DR not accepted
        DEBUG_PRINTLN("DR " << static_cast<int>(dr) << " not valid for this region, max=" << maxDR);
    }

    // Check validity of power according to region
    int maxPower = 7; // Default value for EU868
    switch (lora_region)
    {
    case REGION_US915:
        maxPower = 10;
        break;
    case REGION_EU868:
        if (txpower > 7)
        {                    // Only 0-7 are valid for EU868
            status &= ~0x02; // Power not accepted
            DEBUG_PRINTLN("TXPower " << static_cast<int>(txpower)
                                     << " not valid for EU868, must be 0-7");
        }
        else
        {
            // Convert index to dBm according to LoRaWAN specification
            int power_dbm = 14 - (2 * txpower); // 14dBm is the maximum power
            DEBUG_PRINTLN("TXPower " << static_cast<int>(txpower)
                                     << " corresponds to " << power_dbm << "dBm");
        }
        break;
        // Other regions as needed
    }

    if (txpower > maxPower)
    {
        status &= ~0x02; // Power not accepted
        DEBUG_PRINTLN("TXPower " << static_cast<int>(txpower) << " not valid for this region, max=" << maxPower);
    }

    // Check validity of the channel mask according to the region
    bool validChannelMask = false;

    // Check according to ChMaskCntl and region
    switch (lora_region)
    {
    case REGION_EU868:
        // For EU868 we normally have 8 channels, ChMaskCntl 0-5
        if (chmaskcntl > 5)
        {                    // Only 0-5 are valid in EU868
            status &= ~0x01; // Bit 0 = Channel mask ACK
            DEBUG_PRINTLN("ChMaskCntl " << (int)chmaskcntl << " not valid for EU868");
            DEBUG_PRINTLN("Invalid channel mask: ChMaskCntl=" << (int)chmaskcntl << ", ChMask=0x" << std::hex << chmask << std::dec);
        }
        else
        {
            // Interpret ChMaskCntl according to EU868 specification
            switch (chmaskcntl)
            {
            case 0: // Applies ChMask directly
                validChannelMask = true;
                break;
            case 1: // All channels ON
                chmask = 0xFFFF;
                validChannelMask = true;
                break;
            case 2: // All channels OFF
                chmask = 0x0000;
                validChannelMask = true;
                break;
            case 3: // Channel 16 ON, others according to ChMask
                chmask |= (1 << 15);
                validChannelMask = true;
                break;
            case 4: // Channel 16 OFF, others according to ChMask
                chmask &= ~(1 << 15);
                validChannelMask = true;
                break;
            case 5: // RFU (Reserved for Future Use)
                status &= ~0x01;
                validChannelMask = false;
                DEBUG_PRINTLN("ChMaskCntl 5 reserved for future use");
                break;
            }
        }
        break;

    case REGION_US915:
        // US915 has special handling with 72 channels
        if (chmaskcntl <= 7)
        {
            validChannelMask = true;

            // Verify specific rules for US915
            if (chmaskcntl == 7 && chmask == 0)
            {
                validChannelMask = false;
                DEBUG_PRINTLN("Invalid channel for US915 with ChMaskCntl=7");
            }
        }
        break;

        // Other regions as needed
    }

    if (!validChannelMask)
    {
        status &= ~0x01; // Channel mask not accepted
        DEBUG_PRINTLN("Invalid channel mask: ChMaskCntl=" << static_cast<int>(chmaskcntl) << ", ChMask=0x" << std::hex << chmask << std::dec);
    }

    // If everything is okay, apply the changes
    if (status == 0b111)
    {
        // Apply Data Rate (map DR to SF according to the region)
        int sf;
        float bw = 125.0; // Default value

        // Map specific to the region
        switch (lora_region)
        {
        case REGION_EU868:
            // For EU868: DR0=SF12/125kHz, DR1=SF11/125kHz, ..., DR6=SF7/250kHz
            if (dr > 7)
            {                    // DR0-DR7 are valid
                status &= ~0x04; // Invalid DR
                DEBUG_PRINTLN("DR " << (int)dr << " is not valid for EU868, max=7");
            }
            else
            {
                // Correct mapping of DR to SF/BW for EU868
                if (dr < 6)
                {
                    sf = 12 - dr;
                    bw = 125.0;
                }
                else if (dr == 6)
                {
                    sf = 7;
                    bw = 250.0;
                }
                else
                { // dr == 7
                    sf = 7;
                    bw = 125.0;
                }
            }

            break;

        case REGION_US915:
            // For US915: DR0=SF10/125kHz, DR1=SF9/125kHz, ..., DR3=SF7/125kHz, DR4=SF8/500kHz
            if (dr <= 3)
            {
                sf = 10 - dr;
                bw = 125.0;
            }
            else
            {
                sf = 8;
                bw = 500.0;
            }
            break;

        default:
            // Generic mapping if no specific implementation exists
            sf = dr <= 6 ? (12 - dr) : 7;
            bw = dr == 6 ? 250.0 : 125.0;
        }

        // Apply TX power according to the region
        int power;
        switch (lora_region)
        {
        case REGION_EU868:
            // EU868: 0=MaxEIRP, 1=MaxEIRP-2, 2=MaxEIRP-4, ..., 7=MaxEIRP-14
            power = MAX_POWER[lora_region] - (txpower * 2);
            break;

        case REGION_US915:
            // US915: 0=30dBm, 1=28dBm, 2=26dBm, ..., 10=10dBm
            power = 30 - (txpower * 2);
            break;

        default:
            // Generic mapping if no specific implementation exists
            power = MAX_POWER[lora_region] - (txpower * 2);
        }

        // Limit to safe values
        if (sf < 7)
            sf = 7;
        if (sf > 12)
            sf = 12;
        if (power < 2)
            power = 2;
        if (power > MAX_POWER[lora_region])
            power = MAX_POWER[lora_region];

        DEBUG_PRINTLN("Updating radio parameters:");
        DEBUG_PRINTLN("  SF: " << sf);
        DEBUG_PRINTLN("  BW: " << bw << "kHz");
        DEBUG_PRINTLN("  Power: " << (MAX_POWER[lora_region] - 2 * txpower) << "dBm");

        // Apply configuration to the radio
        pimpl->rfm->setSpreadingFactor(sf);
        current_sf = sf;
        pimpl->rfm->setBandwidth(bw);
        current_bw = bw;
        pimpl->rfm->setTxPower(power, true);
        pimpl->txPower = power;
        updateDataRateFromSF(); // Update DR from SF

        // Apply channel mask if valid
        if (validChannelMask)
        {
            // For EU868 with ChMaskCntl=0, apply directly the ChMask
            if (lora_region == REGION_EU868 && chmaskcntl == 0)
            {
                for (int i = 0; i < 16; i++)
                {
                    if (i < 8)
                    { // We only have 8 channels in EU868
                        // If the bit is enabled, activate the channel
                        bool enabled = (chmask & (1 << i)) != 0;
                        if (enabled)
                        {
                            channelFrequencies[i] = BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region];
                        }
                        else
                        {
                            channelFrequencies[i] = 0; // Disable channel
                        }
                    }
                }
            }

            // For other region/ChMaskCntl combinations, implement as needed
        }

        DEBUG_PRINTLN("Final status of ADR command: " << std::bitset<3>(status).to_string());

        // Apply the number of repetitions (nbRep)
        if (nbRep > 0)
        {
            // Store for use in transmissions
            current_nbRep = nbRep;
            DEBUG_PRINTLN("Number of repetitions set to " << (int)nbRep);
        }

        // Reset ADR counter since we received and applied a response
        adrAckCounter = 0;

        DEBUG_PRINTLN("ADR params applied: SF=" << sf << ", BW=" << bw << "kHz, TXPower=" << power << "dBm, NbRep=" << static_cast<int>(nbRep));
    }
    else
    {
        DEBUG_PRINTLN("ADR params rejected, status=" << std::bitset<3>(status));
    }

    // Añadir respuesta
    response.push_back(MAC_LINK_ADR_ANS);
    response.push_back(status);
}

void LoRaWAN::sendADRStatistics()
{
    if (!pendingMACResponses.empty() || !joined)
        return;

    // Send information about signal conditions to assist ADR
    std::vector<uint8_t> data;

    // Collect statistical data
    float avgSnr = pimpl->rfm->getSNR();
    int avgRssi = pimpl->rfm->getRSSI();

    // Custom packet with statistics (port 2)
    data.push_back(0x01); // Type of statistical message

    // Add average SNR (in a custom format)
    int snrValue = static_cast<int>(avgSnr * 4); // *4 for precision of 0.25 dB
    data.push_back(snrValue & 0xFF);

    // Add average RSSI (in dBm)
    data.push_back(static_cast<uint8_t>(avgRssi & 0xFF));

    // Send statistics as unconfirmed data
    send(data, 2, false);

    DEBUG_PRINTLN("Sent ADR statistics: SNR=" << avgSnr << "dB, RSSI=" << avgRssi << "dBm");
}

// Update TX parameters when ADR_ACK_DELAY is exceeded without response
void LoRaWAN::updateTxParamsForADR()
{
    // Get current SF and power
    int currentSF = pimpl->rfm->getSpreadingFactor();

    // Increment SF (reduce DR) to improve range
    if (currentSF < 12)
    {
        currentSF++;
        pimpl->rfm->setSpreadingFactor(currentSF);
        current_sf = currentSF;
        DEBUG_PRINTLN("ADR: Increasing SF to " << currentSF << " due to lack of response");
    }

    // We could also increase TX power if needed
    if (current_power < 14)
    {
        current_power += 2;
        pimpl->rfm->setTxPower(current_power, true);
        DEBUG_PRINTLN("ADR: Increasing TX power to " << current_power << " dBm");
    }

    // Reset the counter to allow time for the new configuration
    adrAckCounter = ADR_ACK_LIMIT;
}

void LoRaWAN::setupRxWindows()
{
    // Record the time when transmission finished
    pimpl->txEndTime = std::chrono::steady_clock::now();

    // Prepare for RX1 window
    pimpl->rxState = RX_WAIT_1;
    DEBUG_PRINTLN("Waiting for RX1 window (opening in " << RECEIVE_DELAY1 << " ms)");
}

/**
 * Open the RX1 window (modified to use rx1DrOffset)
 */
void LoRaWAN::openRX1Window()
{
    DEBUG_PRINTLN("Opening RX1 window on frequency " << channelFrequencies[current_channel] << " MHz");

    // Configure radio for RX1: same frequency, adjust SF based on rx1DrOffset
    pimpl->rfm->standbyMode();
    pimpl->rfm->setFrequency(channelFrequencies[current_channel]);

    // Calculate SF for RX1 based on the offset
    int rx1_sf = current_sf;

    // Apply the RX1 offset based on region
    int rx1_dr = 0;
    switch (lora_region)
    {
    case REGION_EU868:
        // For EU868, downlink DR = uplink_dr - rx1_dr_offset
        // (limited to the valid DR range)
        rx1_dr = std::max(0, std::min(7, current_dr - rx1DrOffset));

        // Convert DR to SF
        if (rx1_dr < 6)
        {
            rx1_sf = 12 - rx1_dr;
        }
        else if (rx1_dr == 6)
        {
            rx1_sf = 7; // SF7/250kHz
        }
        else
        {
            rx1_sf = 7; // SF7/FSK
        }
        break;

    case REGION_US915:
        // For US915, apply specific rule
        // ... implementation for US915 ...
        break;

        // Other regions as needed

    default:
        // Generic implementation if no specific rule
        rx1_sf = current_sf;
        break;
    }

    // Apply RX1 configuration
    pimpl->rfm->setSpreadingFactor(rx1_sf);
    pimpl->rfm->setBandwidth(current_bw);
    pimpl->rfm->setCodingRate(current_cr);
    pimpl->rfm->setPreambleLength(current_preamble);
    pimpl->rfm->setInvertIQ(true); // Always invert IQ for downlink
    pimpl->rfm->setContinuousReceive();

    // Update state
    pimpl->rxState = RX_WINDOW_1;
    pimpl->rxWindowStart = std::chrono::steady_clock::now();

    DEBUG_PRINTLN("RX1 window opened (SF" << rx1_sf << ", "
                                          << channelFrequencies[current_channel] << " MHz)");
}

/**
 * Open the RX2 window (using custom rx2DataRate if configured)
 */
void LoRaWAN::openRX2Window()
{
    DEBUG_PRINTLN("Opening RX2 window on frequency " << RX2_FREQ[lora_region] << " MHz");

    // Configure radio for RX2: frequency and SF determined by rx2DataRate if configured
    pimpl->rfm->standbyMode();
    pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);

    // Determine SF for RX2 based on rx2DataRate (if configured via RX_PARAM_SETUP_REQ)
    int rx2_sf = RX2_SF[lora_region];   // Default value for the region
    float rx2_bw = RX2_BW[lora_region]; // Default value for the region

    // If rx2DataRate was configured via MAC command, use it
    if (rx2DataRate > 0)
    {
        switch (lora_region)
        {
        case REGION_EU868:
            if (rx2DataRate < 6)
            {
                rx2_sf = 12 - rx2DataRate;
                rx2_bw = 125.0f;
            }
            else if (rx2DataRate == 6)
            {
                rx2_sf = 7;
                rx2_bw = 250.0f;
            }
            else
            {
                rx2_sf = 7;
                rx2_bw = 125.0f;
            }
            break;

        case REGION_US915:
            // Specific implementation for US915
            // ... code for US915 ...
            break;

            // Other regions as needed
        }
    }

    // Apply RX2 configuration
    pimpl->rfm->setSpreadingFactor(rx2_sf);
    pimpl->rfm->setBandwidth(rx2_bw);
    pimpl->rfm->setCodingRate(RX2_CR[lora_region]);
    pimpl->rfm->setPreambleLength(RX2_PREAMBLE[lora_region]);
    pimpl->rfm->setInvertIQ(true); // Always invert IQ for downlink
    pimpl->rfm->setContinuousReceive();

    // Update state
    pimpl->rxState = RX_WINDOW_2;
    pimpl->rxWindowStart = std::chrono::steady_clock::now();

    DEBUG_PRINTLN("RX2 window opened (SF" << rx2_sf << ", "
                                          << RX2_FREQ[lora_region] << " MHz)");
}

// Método para actualizar el estado de las ventanas de recepción
void LoRaWAN::updateRxWindows() {
    // If we're not joined or not waiting/in an RX window, do nothing
    if (!joined || pimpl->rxState == RX_IDLE) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsedSinceTx = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - pimpl->txEndTime).count();

    // Process according to current state
    switch (pimpl->rxState) {
        case RX_WAIT_1:
            // Comprobar si es hora de abrir la ventana RX1
            if (elapsedSinceTx >= RECEIVE_DELAY1) {
                DEBUG_PRINTLN("Opening RX1 window on frequency " << channelFrequencies[current_channel] << " MHz after " << elapsedSinceTx << " ms (should be " << RECEIVE_DELAY1 << " ms)");

                // Configure radio for RX1: same frequency, adjust SF based on rx1DrOffset
                pimpl->rfm->standbyMode();
                pimpl->rfm->setFrequency(channelFrequencies[current_channel]);
                
                // Calculate SF for RX1 based on the offset
                int rx1_sf = current_sf;
                if (rx1DrOffset > 0) {
                    rx1_sf = std::min(rx1_sf + rx1DrOffset, 12); // Adjust SF based on offset
                }
                
                pimpl->rfm->setSpreadingFactor(rx1_sf);
                pimpl->rfm->setBandwidth(current_bw);
                pimpl->rfm->setCodingRate(current_cr);
                pimpl->rfm->setPreambleLength(current_preamble);
                pimpl->rfm->setInvertIQ(true);  // Always invert IQ for downlink
                pimpl->rfm->setContinuousReceive();
                
                // Update state
                pimpl->rxState = RX_WINDOW_1;
                pimpl->rxWindowStart = now;
                
                DEBUG_PRINTLN("RX1 window opened (SF" << rx1_sf << ", " 
                             << channelFrequencies[current_channel] << " MHz Timestamp: "
                             << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() << ");");
            }
            break;
            
        case RX_WINDOW_1:
            // Check if RX1 window has expired
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now - pimpl->rxWindowStart).count() >= WINDOW_DURATION) {
                
                // If nothing was received in RX1, prepare for RX2
                if (elapsedSinceTx < RECEIVE_DELAY2) {
                    pimpl->rxState = RX_WAIT_2;
                    DEBUG_PRINTLN("RX1 window closed, waiting for RX2 window Timestamp: "
                                 << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
                } else {
                    // If RX2 time has passed, open directly
                    openRX2Window();
                }
            }
            break;
            
        case RX_WAIT_2:
            // Check if it's time to open the RX2 window
            if (elapsedSinceTx >= RECEIVE_DELAY2) {
                DEBUG_PRINTLN("Opening RX2 window after " << elapsedSinceTx << " ms (should be " << RECEIVE_DELAY2 << " ms)");
                openRX2Window();
            }
            break;
            
        case RX_WINDOW_2:
            // Check if RX2 window has expired
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                now - pimpl->rxWindowStart).count() >= WINDOW_DURATION) {
                
                // RX2 window closed, revert to appropriate mode based on the class
                if (currentClass == DeviceClass::CLASS_C) {
                    // For Class C, maintain continuous reception on RX2
                    pimpl->rxState = RX_CONTINUOUS;
                    DEBUG_PRINTLN("RX2 window closed, returning to continuous reception (Class C) Timestamp: "
                                 << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
                    
                    // No configuration changes, we are already in RX2
                } else {
                    // For Class A, revert to standby until next TX
                    pimpl->rfm->standbyMode();
                    pimpl->rxState = RX_IDLE;
                    DEBUG_PRINTLN("RX2 window closed, standby mode until next TX (Class A) Timestamp: "
                                 << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
                }
            }
            break;
            
        case RX_CONTINUOUS:
            // For Class C, ensure we are in continuous RX mode with RX2 config
            break;
            
        default:
            break;
    }
}

// MMethod to handle confirmations
void LoRaWAN::handleConfirmation()
{
    // If we're not waiting for ACK, do nothing
    if (confirmState != ConfirmationState::WAITING_ACK)
    {
        return;
    }

    // Check if enough time has passed to retry
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastConfirmAttempt).count();

    // Retry every 5 seconds
    if (elapsed >= 5 && confirmRetries < MAX_RETRIES)
    {
        DEBUG_PRINTLN("No ACK received, retrying confirmed send: "
                      << (confirmRetries + 1) << "/" << MAX_RETRIES);

        // Before resending, temporarily change the state to allow the new send
        ConfirmationState oldState = confirmState;
        confirmState = ConfirmationState::NONE;

        // Save a local copy of pendingAck and ackPort
        std::vector<uint8_t> localPendingAck = pendingAck;
        uint8_t localAckPort = ackPort;

        // Resend the same message that is pending
        bool result = send(localPendingAck, localAckPort, true);

        if (!result)
        {
            DEBUG_PRINTLN("Error resending confirmed message");
            // If the send failed, restore the previous state to try again later
            confirmState = oldState;
        }
        // No need to restore the state if the send was successful,
        // because send() already set the state correctly
    }
    else if (confirmRetries >= MAX_RETRIES)
    {
        DEBUG_PRINTLN("Maximum number of retries reached (" << MAX_RETRIES
                                                            << "). Message not confirmed.");

        // Reset the confirmation state
        resetConfirmationState();
    }
}

// Método para enviar un ACK
void LoRaWAN::sendAck()
{
    if (confirmState != ConfirmationState::ACK_PENDING)
    {
        return;
    }

    DEBUG_PRINTLN("Sending ACK for confirmed message");

    // Create an empty message for ACK
    std::vector<uint8_t> emptyPayload;

    // Save old state for verification
    ConfirmationState oldState = confirmState;

    // Temporarily: force send without confirmation and with ACK
    confirmState = ConfirmationState::ACK_PENDING;

    // Send as unconfirmed message (the ACK is implicit in the ACK bit of MHDR)
    bool result = send(emptyPayload, 0, false, true);

    DEBUG_PRINTLN("ACK send result: " << (result ? "SUCCESS" : "FAIL"));

    // Reset the confirmation state
    resetConfirmationState();

    confirmState = oldState;
}

// Método para resetear el estado de confirmación
void LoRaWAN::resetConfirmationState()
{
    DEBUG_PRINTLN("Resetting confirmation state");
    confirmState = ConfirmationState::NONE;
    confirmRetries = 0;
    pendingAck.clear();
    ackPort = 0;
    needsAck = false;
    // Do not reset lastFcntDown, it helps us track server responses
}

// MMethod to handle a correct reception
void LoRaWAN::handleReceivedMessage(const std::vector<uint8_t> &payload, Message &msg)
{
    // Verify MHDR to know message type
    uint8_t mhdr = payload[0];

    // Join Accept (0x20) - process differently
    if ((mhdr & 0xE0) == 0x20)
    {
        DEBUG_PRINTLN("Received JOIN ACCEPT message");
        if (processJoinAccept(payload))
        {
            // Successful join, no message to return to user
            msg.payload.clear();
            msg.port = 0;
            msg.confirmed = false;
            return;
        }
        // If processing failed, return empty message
        msg.payload.clear();
        return;
    }

    // From here on is a normal data message (uplink/downlink)

    // Extract downlink counter (FCnt) from payload
    uint16_t fcnt = payload[6] | (payload[7] << 8);

    // Verify if confirmed and check ACK bit
    bool needsAck = ((mhdr & 0xE0) == 0xA0); // 0xA0 = Confirmed Data Down
    bool isAck = (mhdr & 0x20);              // ACK bit activated

    // Extract FPort
    msg.port = (payload.size() > 8) ? payload[8] : 0;
    msg.confirmed = ((mhdr & 0xE0) == 0xA0);

    // Reset ADR counter upon receiving any downlink
    if (adrEnabled)
    {
        adrAckCounter = 0;
        DEBUG_PRINTLN("ADR: Resetting counter due to downlink reception");
    }

    // If we have a message that needs ACK, mark it as pending
    if (needsAck)
    {
        confirmState = ConfirmationState::ACK_PENDING;
        DEBUG_PRINTLN("Confirmed message received, ACK pending");

        // If we are in Class C, send ACK immediately
        if (currentClass == DeviceClass::CLASS_C)
        {
            sendAck();
        }
    }

    // If we receive an ACK for a pending message, reset the state
    if (isAck && confirmState == ConfirmationState::WAITING_ACK)
    {
        DEBUG_PRINTLN("ACK received for pending confirmed message");
        resetConfirmationState();
    }

    // Save the last counter
    lastFcntDown = fcnt;
    // Update the downlink counter in the implementation
    pimpl->downlinkCounter = fcnt;
    DEBUG_PRINTLN("FCnt extracted from downlink: " << fcnt);

    // Extract and decode payload if it exists
    if (payload.size() > 9)
    {
        std::vector<uint8_t> encrypted(payload.begin() + 9, payload.end() - 4);
        msg.payload = decryptPayload(encrypted, msg.port);

        DEBUG_PRINT("LoRaWAN message decrypted: Port=" << (int)msg.port
                                                       << ", Type=" << (msg.confirmed ? "Confirmed" : "Unconfirmed")
                                                       << ", Payload=");
        for (const auto &b : msg.payload)
        {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0')
                                 << (int)b << " ");
        }
        DEBUG_PRINTLN(std::dec);
    }

    // If we receive data on port 3 (common LinkADR) or port 0 (MAC commands)
    if (msg.port == 3)
    {
        DEBUG_PRINTLN("Processing LinkADR command on port 3");

        // Verify that we have enough bytes for a LinkADR (minimum 5)
        if (msg.payload.size() >= 5)
        {
            std::vector<uint8_t> macCommands;
            macCommands.push_back(MAC_LINK_ADR_REQ);

            // Only add the first 5 bytes (what a LinkADR needs)
            // 0x03 (CID) + DataRate_TXPower (1) + ChMask (2) + Redundancy (1) = 5 bytes
            macCommands.insert(macCommands.end(),
                               msg.payload.begin(),
                               msg.payload.begin() + std::min(size_t(5), msg.payload.size()));

            std::vector<uint8_t> macResponse;
            processMACCommands(macCommands, macResponse);

            if (!macResponse.empty())
            {
                pendingMACResponses = macResponse;
                DEBUG_PRINTLN("LinkADR response prepared: " << std::hex << (int)macResponse[0] << " " << (int)macResponse[1] << std::dec);
            }
        }
    }

    // If we receive a confirmed message, send ACK immediately if we are in class C
    if (needsAck && currentClass == DeviceClass::CLASS_C)
    {
        sendAck();
    }

    // Extract FCtrl (byte 5) to verify ACK
    uint8_t fctrl = payload[5];
    isAck = (fctrl & 0x20) != 0;  // Bit 5 = ACK
    
    DEBUG_PRINTLN("FCtrl: 0x" << std::hex << (int)fctrl << std::dec 
                 << " (ACK=" << (isAck ? "Yes" : "No") << ")");

    if (isAck && confirmState == ConfirmationState::WAITING_ACK) {
        DEBUG_PRINTLN("ACK received for pending confirmed message");
        resetConfirmationState();
    }
}

bool LoRaWAN::processJoinAccept(const std::vector<uint8_t> &data)
{
    // Create a non-const copy of the data that we can modify
    std::vector<uint8_t> response = data;

    // Delegate processing to the Impl structure
    bool result = pimpl->processJoinAccept(response);

    if (result)
    {
        joined = true;
        DEBUG_PRINTLN("Join Accept processed successfully");
    }
    else
    {
        DEBUG_PRINTLN("Join Accept processing failed");
    }

    return result;
}

/**
 * @brief Request to server info about the device link quality
 *
 * Sends a MAC LinkCheckReq command to the server. The response will arrive in a future downlink
 * with the margin and number of gateways that received the last uplink. The application must
 * register a reception callback to process the response.
 * 
 * @return true if the command was scheduled correctly, false otherwise
 */
void LoRaWAN::requestLinkCheck()
{
    if (!joined)
    {
        DEBUG_PRINTLN("Error: Cannot request LinkCheck without being joined to the network");
        return;
    }

    // Schedule the command for the next uplink
    if (pendingMACResponses.size() < 15)
    { // Maximum 15 bytes in FOpts
        pendingMACResponses.push_back(MAC_LINK_CHECK_REQ);
        DEBUG_PRINTLN("LinkCheckReq scheduled for next uplink");
    }
    else
    {
        DEBUG_PRINTLN("Error: No space in FOpts to add LinkCheckReq");
    }
}

void LoRaWAN::updateDataRateFromSF()
{
    // For EU868
    if (lora_region == REGION_EU868)
    {
        int sf = pimpl->rfm->getSpreadingFactor();
        float bw = pimpl->rfm->getBandwidth();

        if (bw == 125.0f)
        {
            if (sf >= 7 && sf <= 12)
            {
                current_dr = 12 - sf; // DR0-DR5
            }
            else
            {
                current_dr = 7; // DR7 (FSK)
            }
        }
        else if (bw == 250.0f && sf == 7)
        {
            current_dr = 6; // DR6
        }

        DEBUG_PRINTLN("Data Rate updated: DR" << current_dr);
    }
    // Add additional configurations for other regions as necessary
}