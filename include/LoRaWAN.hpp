/**
 * @file LoRaWAN.hpp
 * @brief Class declaration for LoRaWAN protocol communication.
 * 
 * This header file defines the LoRaWAN class, which provides a complete
 * implementation of the LoRaWAN protocol for communication with LoRaWAN networks.
 * 
 * The class supports both Over-The-Air Activation (OTAA) and Activation By 
 * Personalization (ABP), Class A and Class C device operation, and provides
 * a comprehensive set of methods for sending and receiving messages, managing
 * MAC commands, and controlling radio parameters.
 * 
 * @author Sergio Pérez
 * @date 2023/03/08
 * @version 1.0
 * 
 * @see LoRaWAN.cpp for the implementation
 */

#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <functional>
#include <chrono>
#include "SPIInterface.hpp"

// LoRaWAN MAC commands
#define MAC_LINK_CHECK_REQ 0x02
#define MAC_LINK_ADR_REQ 0x03
#define MAC_LINK_ADR_ANS 0x03
#define MAC_DUTY_CYCLE_REQ 0x04
#define MAC_DUTY_CYCLE_ANS 0x04
#define MAC_RX_PARAM_SETUP_REQ 0x05
#define MAC_RX_PARAM_SETUP_ANS 0x05
#define MAC_DEV_STATUS_REQ 0x06
#define MAC_DEV_STATUS_ANS 0x06
#define MAC_NEW_CHANNEL_REQ 0x07
#define MAC_NEW_CHANNEL_ANS 0x07
#define MAC_RX_TIMING_SETUP_REQ 0x08
#define MAC_RX_TIMING_SETUP_ANS 0x08
#define MAC_TX_PARAM_SETUP_REQ 0x09
#define MAC_TX_PARAM_SETUP_ANS 0x09
#define MAC_DL_CHANNEL_REQ 0x0A
#define MAC_DL_CHANNEL_ANS 0x0A
#define MAC_REKEY_CONF 0x0B
#define MAC_ADR_PARAM_SETUP_REQ 0x0C
#define MAC_ADR_PARAM_SETUP_ANS 0x0C
#define MAC_DEVICE_TIME_REQ 0x0D
#define MAC_DEVICE_TIME_ANS 0x0D
#define MAC_REJOIN_PARAM_REQ 0x0E
#define MAC_REJOIN_PARAM_ANS 0x0E
#define MAC_PING_SLOT_INFO_REQ 0x10
#define MAC_PING_SLOT_INFO_ANS 0x10
#define MAC_PING_SLOT_CHANNEL_REQ 0x11
#define MAC_PING_SLOT_FREQ_ANS 0x11
#define MAC_BEACON_TIMING_REQ 0x12
#define MAC_BEACON_TIMING_ANS 0x12
#define MAC_BEACON_FREQ_REQ 0x13
#define MAC_BEACON_FREQ_ANS 0x13
#define MAC_LINK_CHECK_ANS 0x02

/**
 * @brief Class that provides LoRaWAN protocol implementation.
 * 
 * The LoRaWAN class implements the LoRaWAN protocol for communications with 
 * LoRaWAN networks. It supports both OTAA and ABP activation methods,
 * Class A and Class C device operation, and provides methods for sending 
 * and receiving messages, managing MAC commands, and controlling various 
 * radio parameters.
 */
class LoRaWAN {
public:
    /**
     * @brief LoRaWAN supported regions.
     */
    enum Region {
        REGION_EU868 = 0, /**< European Union 863-870 MHz band */
        REGION_US915,     /**< United States 902-928 MHz band */
        REGION_AU915,     /**< Australia 915-928 MHz band */
        REGION_EU433,     /**< European Union 433 MHz band */
        REGIONS           /**< Number of regions */
    };

    /**
     * @brief LoRaWAN device classes.
     */
    enum DeviceClass {
        CLASS_A, /**< Class A device - lowest power consumption, bidirectional */
        CLASS_B, /**< Class B device - synchronized beacons, bidirectional with scheduled receive slots */
        CLASS_C  /**< Class C device - continuous receive, bidirectional with lowest latency */
    };

    /**
     * @brief LoRaWAN join modes.
     */
    enum JoinMode {
        OTAA, /**< Over-The-Air Activation - dynamic session keys */
        ABP   /**< Activation By Personalization - static session keys */
    };

    /**
     * @brief Represents the state of message confirmations.
     */
    enum ConfirmationState {
        NONE,        /**< No confirmation pending */
        WAITING_ACK, /**< Waiting for acknowledgment of sent message */
        ACK_PENDING  /**< Need to acknowledge a received message */
    };

    /**
     * @brief Represents the state of receive windows.
     */
    enum RxWindowState {
        RX_IDLE,        /**< Not in a receive window */
        RX_WAIT_1,      /**< Waiting to open RX1 window */
        RX_WINDOW_1,    /**< In RX1 window */
        RX_WAIT_2,      /**< Waiting to open RX2 window */
        RX_WINDOW_2,    /**< In RX2 window */
        RX_CONTINUOUS   /**< Continuous reception (Class C) */
    };

    /**
     * @brief Structure representing a LoRaWAN message.
     */
    struct Message {
        std::vector<uint8_t> payload; /**< Message payload */
        uint8_t port;                 /**< Message port */
        bool confirmed;               /**< Whether the message is confirmed */
    };

    /**
     * @brief Callback type for received messages.
     */
    typedef std::function<void(const Message&)> ReceiveCallback;
    
    /**
     * @brief Callback type for join events.
     */
    typedef std::function<void(bool)> JoinCallback;

    /**
     * @brief Default constructor.
     * 
     * Initializes a LoRaWAN instance with default SPI interface (CH341SPI).
     */
    LoRaWAN();

    /**
     * @brief Constructor with custom SPI interface.
     * 
     * @param spi_interface A unique pointer to an SPIInterface implementation
     */
    explicit LoRaWAN(std::unique_ptr<SPIInterface> spi_interface);

    /**
     * @brief Destructor.
     */
    ~LoRaWAN();

    /**
     * @brief Initialize the LoRaWAN stack and radio.
     * 
     * @param deviceIndex Optional device index for SPI
     * @return true if initialization succeeded, false otherwise
     */
    bool init(int deviceIndex = 0);

    /**
     * @brief Set the device class (A, B, or C).
     * 
     * @param deviceClass The device class to set
     */
    void setDeviceClass(DeviceClass deviceClass);

    /**
     * @brief Set the Device EUI.
     * 
     * @param devEUI Device EUI as a hexadecimal string
     */
    void setDevEUI(const std::string& devEUI);

    /**
     * @brief Set the Application EUI.
     * 
     * @param appEUI Application EUI as a hexadecimal string
     */
    void setAppEUI(const std::string& appEUI);

    /**
     * @brief Set the Application Key.
     * 
     * @param appKey Application Key as a hexadecimal string
     */
    void setAppKey(const std::string& appKey);

    /**
     * @brief Set the Device Address.
     * 
     * Used for ABP activation.
     * 
     * @param devAddr Device Address as a hexadecimal string
     */
    void setDevAddr(const std::string& devAddr);

    /**
     * @brief Set the Network Session Key.
     * 
     * Used for ABP activation.
     * 
     * @param nwkSKey Network Session Key as a hexadecimal string
     */
    void setNwkSKey(const std::string& nwkSKey);

    /**
     * @brief Set the Application Session Key.
     * 
     * Used for ABP activation.
     * 
     * @param appSKey Application Session Key as a hexadecimal string
     */
    void setAppSKey(const std::string& appSKey);

    /**
     * @brief Join a LoRaWAN network.
     * 
     * @param mode Join mode (OTAA or ABP)
     * @param timeout Timeout in milliseconds
     * @return true if join succeeded, false otherwise
     */
    bool join(JoinMode mode, unsigned long timeout = 10000);

    /**
     * @brief Encrypt a payload.
     * 
     * @param payload The payload to encrypt
     * @param port The port number
     * @return Encrypted payload
     */
    std::vector<uint8_t> encryptPayload(const std::vector<uint8_t>& payload, uint8_t port);

    /**
     * @brief Decrypt a payload.
     * 
     * @param payload The encrypted payload
     * @param port The port number
     * @return Decrypted payload
     */
    std::vector<uint8_t> decryptPayload(const std::vector<uint8_t>& payload, uint8_t port);

    /**
     * @brief Calculate the time on air for a payload.
     * 
     * @param payload_size Size of the payload in bytes
     * @return Time on air in milliseconds
     */
    float calculateTimeOnAir(size_t payload_size);

    /**
     * @brief Check if duty cycle allows transmission.
     * 
     * @param frequency Frequency in MHz
     * @param payload_size Size of the payload in bytes
     * @return true if transmission is allowed, false otherwise
     */
    bool checkDutyCycle(float frequency, size_t payload_size);

    /**
     * @brief Get the duty cycle usage for a channel.
     * 
     * @param channel Channel index
     * @return Duty cycle usage as a percentage
     */
    float getDutyCycleUsage(int channel);

    /**
     * @brief Reset the duty cycle usage.
     */
    void resetDutyCycle();

    /**
     * @brief Send a message.
     * 
     * @param data The payload to send
     * @param port The port number
     * @param confirmed Whether the message should be confirmed
     * @param force_duty_cycle Whether to force transmission even if duty cycle limits are reached
     * @return true if the message was sent successfully, false otherwise
     */
    bool send(const std::vector<uint8_t>& data, uint8_t port, bool confirmed = false, bool force_duty_cycle = false);

    /**
     * @brief Update the LoRaWAN state.
     * 
     * Should be called regularly to handle received messages and other events.
     */
    void update();

    /**
     * @brief Receive a message.
     * 
     * @param message Reference to a Message structure where the received message will be stored
     * @param timeout Timeout in milliseconds
     * @return true if a message was received, false otherwise
     */
    bool receive(Message& message, unsigned long timeout = 1000);

    /**
     * @brief Set a callback for received messages.
     * 
     * @param callback Function to call when a message is received
     */
    void onReceive(std::function<void(const Message&)> callback);

    /**
     * @brief Set a callback for join events.
     * 
     * @param callback Function to call when a join event occurs
     */
    void onJoin(std::function<void(bool)> callback);

    /**
     * @brief Set the LoRaWAN region.
     * 
     * @param region The region to set
     */
    void setRegion(int region);

    /**
     * @brief Get the current LoRaWAN region.
     * 
     * @return The current region
     */
    int getRegion() const;

    /**
     * @brief Get the current frequency.
     * 
     * @return The current frequency in MHz
     */
    float getFrequency() const;

    /**
     * @brief Set the frequency.
     * 
     * @param freq_mhz The frequency in MHz
     */
    void setFrequency(float freq_mhz);

    /**
     * @brief Get the channel from a frequency.
     * 
     * @param freq_mhz The frequency in MHz
     * @return The channel index, or -1 if not found
     */
    int getChannelFromFrequency(float freq_mhz) const;

    /**
     * @brief Get the frequency for a channel.
     * 
     * @param channel The channel index
     * @return The frequency in MHz, or 0 if invalid
     */
    float getFrequencyFromChannel(int channel) const;

    /**
     * @brief Set the current channel.
     * 
     * @param channel The channel index
     */
    void setChannel(uint8_t channel);

    /**
     * @brief Get the current channel.
     * 
     * @return The current channel index
     */
    uint8_t getChannel() const;

    /**
     * @brief Enable or disable single channel mode.
     * 
     * @param enable Whether to enable single channel mode
     * @param freq_mhz The frequency in MHz
     * @param sf Spreading factor (7-12)
     * @param bw Bandwidth in kHz (125, 250, or 500)
     * @param cr Coding rate (5-8 for 4/5 to 4/8)
     * @param power Transmission power in dBm
     * @param preamble Preamble length
     */
    void setSingleChannel(bool enable, float freq_mhz = 868.1, int sf = 9, 
                          int bw = 125, int cr = 5, int power = 14, int preamble = 8);

    /**
     * @brief Check if single channel mode is enabled.
     * 
     * @return true if single channel mode is enabled, false otherwise
     */
    bool getSingleChannel() const;

    /**
     * @brief Get the single channel frequency.
     * 
     * @return The single channel frequency in MHz
     */
    float getSingleChannelFrequency() const;

    /**
     * @brief Set the transmission power.
     * 
     * @param power The transmission power in dBm
     */
    void setTxPower(int8_t power);

    /**
     * @brief Get the RSSI (Received Signal Strength Indicator).
     * 
     * @return The RSSI in dBm
     */
    int getRSSI() const;

    /**
     * @brief Get the SNR (Signal-to-Noise Ratio).
     * 
     * @return The SNR in dB
     */
    int getSNR() const;

    /**
     * @brief Get the frame counter.
     * 
     * @return The current uplink frame counter
     */
    uint32_t getFrameCounter() const;

    /**
     * @brief Set the frame counter.
     * 
     * @param counter The new frame counter value
     */
    void setFrameCounter(uint32_t counter);

    /**
     * @brief Wake the radio from sleep mode.
     */
    void wake();

    /**
     * @brief Put the radio in sleep mode.
     */
    void sleep();

    /**
     * @brief Validate the session keys.
     * 
     * @return true if the keys are valid, false otherwise
     */
    bool validateKeys() const;

    /**
     * @brief Enable or disable ADR (Adaptive Data Rate).
     * 
     * @param enable Whether to enable ADR
     */
    void enableADR(bool enable);

    /**
     * @brief Check if ADR is enabled.
     * 
     * @return true if ADR is enabled, false otherwise
     */
    bool isADREnabled() const;

    /**
     * @brief Reset the LoRaWAN session.
     * 
     * Clears all session keys and counters.
     */
    void resetSession();

    /**
     * @brief Apply ADR settings.
     * 
     * @param dataRate The data rate index
     * @param txPower The transmission power index
     * @param channelMask The channel mask
     */
    void applyADRSettings(uint8_t dataRate, uint8_t txPower, const std::vector<uint8_t>& channelMask);

    /**
     * @brief Process MAC commands.
     * 
     * @param commands The MAC commands to process
     * @param response The response MAC commands
     */
    void processMACCommands(const std::vector<uint8_t>& commands, std::vector<uint8_t>& response);

    /**
     * @brief Process LinkADRReq MAC command.
     * 
     * @param cmd The MAC command
     * @param index The index in the command
     * @param response The response MAC command
     */
    void processLinkADRReq(const std::vector<uint8_t>& cmd, size_t index, std::vector<uint8_t>& response);

    /**
     * @brief Send ADR statistics.
     */
    void sendADRStatistics();

    /**
     * @brief Update transmission parameters for ADR.
     */
    void updateTxParamsForADR();

    /**
     * @brief Set up receive windows.
     */
    void setupRxWindows();

    /**
     * @brief Open RX1 window.
     */
    void openRX1Window();

    /**
     * @brief Open RX2 window.
     */
    void openRX2Window();

    /**
     * @brief Update receive windows.
     */
    void updateRxWindows();

    /**
     * @brief Handle confirmations.
     */
    void handleConfirmation();

    /**
     * @brief Send an acknowledgment.
     */
    void sendAck();

    /**
     * @brief Reset the confirmation state.
     */
    void resetConfirmationState();

    /**
     * @brief Handle a received message.
     * 
     * @param payload The received payload
     * @param msg Reference to a Message structure where the processed message will be stored
     */
    void handleReceivedMessage(const std::vector<uint8_t>& payload, Message& msg);

    /**
     * @brief Process a join accept message.
     * 
     * @param data The join accept message data
     * @return true if the processing succeeded, false otherwise
     */
    bool processJoinAccept(const std::vector<uint8_t>& data);

    /**
     * @brief Request a link check.
     */
    void requestLinkCheck();

    /**
     * @brief Update the data rate from the spreading factor.
     */
    void updateDataRateFromSF();

    /**
     * @brief Set verbose mode.
     * 
     * @param verbose Whether to enable verbose mode
     */
    static void setVerbose(bool verbose) { isVerbose = verbose; }

    /**
     * @brief Get verbose mode.
     * 
     * @return true if verbose mode is enabled, false otherwise
     */
    static bool getVerbose() { return isVerbose; }

private:
    // Forward declaration of implementation structure
    struct Impl;
    std::unique_ptr<Impl> pimpl;

    // Radio parameters for RX windows
    int current_sf;
    float current_bw;
    int current_cr;
    int current_preamble;
    int current_power;
    int current_channel;
    int current_lna;
    int current_sync_word;
    uint8_t current_nbRep;

    // RX Window parameters
    uint8_t rx1DrOffset = 0;
    uint8_t rx2DataRate = 0;

    // LoRaWAN state and options
    bool joined;
    DeviceClass currentClass;
    JoinMode joinMode;
    bool adrEnabled;
    uint8_t adrAckCounter;
    std::vector<uint8_t> pendingMACResponses;
    
    // Confirmation state variables
    ConfirmationState confirmState = NONE;
    int confirmRetries = 0;
    std::chrono::steady_clock::time_point lastConfirmAttempt;
    std::vector<uint8_t> pendingAck;
    uint8_t ackPort;
    bool needsAck = false;
    uint16_t lastFcntDown = 0;

    // Other constants
    static constexpr int MAX_CHANNELS = 16;
    
    // Channels and duty cycle tracking
    float channelFrequencies[MAX_CHANNELS];
    std::chrono::steady_clock::time_point lastChannelUse[MAX_CHANNELS];
    float channelAirTime[MAX_CHANNELS];

    // Region selection and constants
    int lora_region = REGION_EU868;

    // Single channel mode parameters
    bool one_channel_gateway = false;
    float one_channel_freq = 868.1;
    int one_channel_sf = 9;
    int one_channel_bw = 125;
    int one_channel_cr = 5;
    int one_channel_power = 14;
    int one_channel_preamble = 8;

    // Timing constants
    static constexpr unsigned long RECEIVE_DELAY1 = 1000;
    static constexpr unsigned long RECEIVE_DELAY2 = 2000;
    static constexpr unsigned long WINDOW_DURATION = 500;
    
    // ADR constants
    static constexpr uint8_t ADR_ACK_LIMIT = 64;
    static constexpr uint8_t ADR_ACK_DELAY = 32;
    static constexpr uint8_t MAX_RETRIES = 8;

    
    // Callbacks
    ReceiveCallback receiveCallback = nullptr;
    JoinCallback joinCallback = nullptr;

    // Base frequencies for regions (MHz)
    static constexpr float BASE_FREQ[REGIONS] = { 
        868.1f,  // EU868
        902.3f,  // US915
        915.2f,  // AU915
        433.175f // EU433
    };

    // Channel step for regions (MHz)
    static constexpr float CHANNEL_STEP[REGIONS] = {
        0.2f,    // EU868
        0.2f,    // US915
        0.2f,    // AU915
        0.2f     // EU433
    };

    // Maximum transmit power by region (dBm)
    static constexpr int MAX_POWER[REGIONS] = {
        14,      // EU868
        30,      // US915
        30,      // AU915
        14       // EU433
    };

    // RX2 parameters for each region
    static constexpr float RX2_FREQ[REGIONS] = {
        869.525f, // EU868
        923.3f,   // US915
        923.3f,   // AU915
        434.665f  // EU433
    };

    static constexpr int RX2_SF[REGIONS] = {
        9,       // EU868
        12,      // US915
        12,      // AU915
        9        // EU433
    };

    static constexpr float RX2_BW[REGIONS] = {
        125.0f,  // EU868
        500.0f,  // US915
        500.0f,  // AU915
        125.0f   // EU433
    };

    static constexpr int RX2_CR[REGIONS] = {
        5,       // EU868
        5,       // US915
        5,       // AU915
        5        // EU433
    };

    static constexpr int RX2_PREAMBLE[REGIONS] = {
        8,       // EU868
        8,       // US915
        8,       // AU915
        8        // EU433
    };

    // Static members
    static bool isVerbose;

    // Current data rate
    uint8_t current_dr = 0;
};
