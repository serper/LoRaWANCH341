#pragma once

#include <string>
#include <functional>
#include <cstdint>
#include <vector>
#include <memory>
#include <array>
#include <chrono>
#include "SPIInterface.hpp"

class LoRaWAN {
public:
    // LoRa Regions
    static constexpr uint8_t REGION_EU433 = 0;
    static constexpr uint8_t REGION_EU868 = 1;
    static constexpr uint8_t REGION_US915 = 2;
    static constexpr uint8_t REGION_AU915 = 3;
    static constexpr uint8_t REGION_AS923 = 4;
    static constexpr uint8_t REGION_KR920 = 5;
    static constexpr uint8_t REGION_IN865 = 6;
    static constexpr uint8_t REGION_CN470 = 7;
    static constexpr uint8_t REGION_CN779 = 8;
    static constexpr uint8_t REGION_EU433OLD = 9;
    static constexpr uint8_t REGION_AU915OLD = 10;
    static constexpr uint8_t REGION_CN470PREQUEL = 11;
    static constexpr uint8_t REGION_AS923JP = 12;
    static constexpr uint8_t REGION_AS923KR = 13;
    static constexpr uint8_t REGIONS = 14;

    // LoRa Regions base frequency
    static constexpr float BASE_FREQ[REGIONS] = {
        433.05, // EU433
        868.1,  // EU868
        903.9,  // US915
        915.2,  // AU915
        923.2,  // AS923
        920.9,  // KR920
        865.1,  // IN865
        470.3,  // CN470
        779.5,  // CN779
        433.05, // EU433
        915.2,  // AU915OLD
        470.3,  // CN470PREQUEL
        923.2,  // AS923JP
        920.9   // AS923KR
    };

    // LoRa Regions channel step
    static constexpr float CHANNEL_STEP[REGIONS] = {
        0.1, // EU433
        0.2, // EU868
        0.2, // US915
        0.2, // AU915
        0.2, // AS923
        0.2, // KR920
        0.2, // IN865
        0.6, // CN470
        1.6, // CN779
        0.1, // EU433
        0.2, // AU915OLD
        0.6, // CN470PREQUEL
        0.6, // AS923JP
        0.2  // AS923KR
    };

    // LoRa max channels
    static constexpr uint8_t CHANNELS = 8;
    static constexpr int MAX_CHANNELS = 16;  // Soporte hasta 16 canales

    // LoRa Bandwidths
    static constexpr float LORA_BW[10] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};

    // LoRa Bandwidths
    static constexpr float BW_7_8_KHZ = 7.8;
    static constexpr float BW_10_4_KHZ = 10.4;
    static constexpr float BW_15_6_KHZ = 15.6;
    static constexpr float BW_20_8_KHZ = 20.8;
    static constexpr float BW_31_25_KHZ = 31.25;
    static constexpr float BW_41_7_KHZ = 41.7;
    static constexpr float BW_62_5_KHZ = 62.5;
    static constexpr float BW_125_KHZ = 125.0;
    static constexpr float BW_250_KHZ = 250.0;
    static constexpr float BW_500_KHZ = 500.0;

    // LoRa Spreading Factors
    static constexpr int SF_6 = 6;
    static constexpr int SF_7 = 7;
    static constexpr int SF_8 = 8;
    static constexpr int SF_9 = 9;
    static constexpr int SF_10 = 10;
    static constexpr int SF_11 = 11;
    static constexpr int SF_12 = 12;

    // LoRa Coding Rates
    static constexpr int CR_5 = 5;
    static constexpr int CR_6 = 6;
    static constexpr int CR_7 = 7;
    static constexpr int CR_8 = 8;

    // LNA Gain
    static constexpr uint8_t LNA_MAX_GAIN = 0x23;
    static constexpr uint8_t LNA_HIGH_GAIN = 0x20;
    static constexpr uint8_t LNA_MED_GAIN = 0x13;
    static constexpr uint8_t LNA_LOW_GAIN = 0x03;
    static constexpr uint8_t LNA_OFF = 0x00;

    // LoRa Duty Cycle
    static constexpr float DUTY_CYCLE = 1.0 / 3600.0; // 1 hour

    enum class DeviceClass {
        CLASS_A,
        CLASS_C
    };

    enum class JoinMode {
        OTAA,
        ABP
    };

    struct Message {
        std::vector<uint8_t> payload;
        uint8_t port;
        bool confirmed;
    };

    // Constructor estándar (usa CH341SPI por defecto)
    LoRaWAN();
    
    // Nuevo constructor que acepta un SPIInterface específico
    LoRaWAN(std::unique_ptr<SPIInterface> spi_interface);
    
    ~LoRaWAN();

    // Inicialización
    bool init(int deviceIndex = 0);
    void setDeviceClass(DeviceClass deviceClass);
    
    // Configuración OTAA
    void setDevEUI(const std::string& devEUI);
    void setAppEUI(const std::string& appEUI);
    void setAppKey(const std::string& appKey);
    
    // Configuración ABP
    void setDevAddr(const std::string& devAddr);
    void setNwkSKey(const std::string& nwkSKey);
    void setAppSKey(const std::string& appSKey);

    // Gestión de conexión
    bool join(JoinMode mode = JoinMode::OTAA, unsigned long timeout = 20000);
    bool isJoined() const;
    void disconnect();

    // Transmisión y recepción
    bool send(const std::vector<uint8_t>& data, uint8_t port = 1, bool confirmed = false, bool force_duty_cycle = false);
    bool receive(Message& message, unsigned long timeout = 5000);
    
    // Callbacks
    void onReceive(std::function<void(const Message&)> callback);
    void onJoin(std::function<void(bool success)> callback);
    
    // Configuración de red
    void setDataRate(uint8_t dr);
    void setTxPower(int8_t power);
    void setChannel(uint8_t channel);
    
    // Diagnóstico
    int getRSSI() const;
    int getSNR() const;
    uint32_t getFrameCounter() const;
    void setFrameCounter(uint32_t counter);

    // Control de energía
    void sleep();
    void wake();

    // Actualización (debe llamarse periódicamente)
    void update();

    // Nuevos métodos para gestión de radio
    void setRegion(int region);
    int getRegion() const;
    float getFrequency() const;
    void setFrequency(float freq_mhz);
    int getChannelFromFrequency(float freq_mhz) const;
    float getFrequencyFromChannel(int channel) const;
    void setChannel(int channel);
    int getChannel() const;
    
    int getSpreadingFactor() const;
    void setSpreadingFactor(int sf);
    
    float getBandwidth() const;
    void setBandwidth(float bw_khz);
    
    int getCodingRate() const;
    void setCodingRate(int denominator);
    
    // Métodos para diagnóstico adicional
    bool testRadio();
    float readTemperature();
    bool calibrateTemperature(float actual_temp);
    
    // Método para reiniciar completamente la sesión
    void resetSession();
    
    // Métodos para gestión de duty cycle
    float calculateTimeOnAir(size_t payload_size);
    bool checkDutyCycle(float frequency, size_t payload_size);
    float getDutyCycleUsage(int channel);
    void resetDutyCycle();

    // Control de verbosidad
    static void setVerbose(bool verbose) { isVerbose = verbose; }
    static bool getVerbose() { return isVerbose; }

private:
    int lora_region = -1;

    // Atributos privados para gestión de radio
    int current_channel = 0;
    int current_sf = 9;
    int current_bw = 125;
    int current_cr = 5;
    int current_power = 14;
    int current_lna = 0x23;
    int current_sync_word = 0x34;
    int current_preamble = 8;
    int current_iq = 0;

    // Support to one channel gateway
    bool one_channel_gateway = false;
    float one_channel_freq = 868.1;

    // Atributos privados para gestión de LoRaWAN
    float duty_cycle[CHANNELS] = {0};
    unsigned long last_tx = 0;
    unsigned long last_rx = 0;

    // Atributos privados para gestión interna
    struct Impl;
    std::unique_ptr<Impl> pimpl;
    
    // Callbacks
    std::function<void(const Message&)> receiveCallback;
    std::function<void(bool)> joinCallback;
    
    // Estado
    bool joined;
    DeviceClass currentClass;
    JoinMode joinMode;
    
    // Métodos privados de ayuda
    bool processReceive();
    void handleRxTimeout();
    bool validateKeys() const;
    void setupRxWindows();
    void switchToClassC();
    
    // Nuevos métodos privados
    bool processJoinAccept(const std::vector<uint8_t>& data);
    bool verifyMIC(const std::vector<uint8_t>& data);
    std::vector<uint8_t> encryptPayload(const std::vector<uint8_t>& payload, uint8_t port);
    std::vector<uint8_t> decryptPayload(const std::vector<uint8_t>& payload, uint8_t port);
    
    // Variables para seguimiento de duty cycle
    std::array<std::chrono::steady_clock::time_point, MAX_CHANNELS> lastChannelUse;
    std::array<float, MAX_CHANNELS> channelAirTime;
    std::array<float, MAX_CHANNELS> channelFrequencies;

    // Control de verbosidad para mensajes de depuración
    static bool isVerbose;
};
