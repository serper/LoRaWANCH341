#pragma once

#include <string>
#include <functional>
#include <cstdint>
#include <vector>
#include <memory>
#include <array>
#include <chrono>
#include "SPIInterface.hpp"
#include <deque>
#include <chrono>

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
    
    // LoRa Regions RX2 frequency
    static constexpr float RX2_FREQ[REGIONS] = {
        434.665, // EU433
        869.525, // EU868
        923.3,   // US915
        923.3,   // AU915
        923.2,   // AS923
        921.9,   // KR920
        866.1,   // IN865
        505.3,   // CN470
        786.5,   // CN779
        434.665, // EU433
        923.3,   // AU915OLD
        505.3,   // CN470PREQUEL
        923.2,   // AS923JP
        921.9    // AS923KR
    };
    
    // LoRa Regions RX2 Spread Factor
    static constexpr uint8_t RX2_SF[REGIONS] = {
        7, // EU433
        9, // EU868
        8, // US915
        8, // AU915
        8, // AS923
        8, // KR920
        8, // IN865
        7, // CN470
        7, // CN779
        7, // EU433
        8, // AU915OLD
        7, // CN470PREQUEL
        8, // AS923JP
        8  // AS923KR
    };
    
    // LoRa Regions RX2 Bandwidth
    static constexpr float RX2_BW[REGIONS] = {
        125.0, // EU433
        125.0, // EU868
        500.0, // US915
        500.0, // AU915
        500.0, // AS923
        500.0, // KR920
        500.0, // IN865
        125.0, // CN470
        125.0, // CN779
        125.0, // EU433
        500.0, // AU915OLD
        125.0, // CN470PREQUEL
        500.0, // AS923JP
        500.0  // AS923KR
    };
    
    // LoRa Regions RX2 Coding Rate
    static constexpr uint8_t RX2_CR[REGIONS] = {
        1, // EU433
        1, // EU868
        4, // US915
        4, // AU915
        4, // AS923
        4, // KR920
        4, // IN865
        1, // CN470
        1, // CN779
        1, // EU433
        4, // AU915OLD
        1, // CN470PREQUEL
        4, // AS923JP
        4  // AS923KR
    };
    
    // LoRa Regions RX2 preamble length
    static constexpr uint16_t RX2_PREAMBLE[REGIONS] = {
        8, // EU433
        8, // EU868
        8, // US915
        8, // AU915
        8, // AS923
        8, // KR920
        8, // IN865
        8, // CN470
        8, // CN779
        8, // EU433
        8, // AU915OLD
        8, // CN470PREQUEL
        8, // AS923JP
        8  // AS923KR
    };

    // Constantes ajustadas para sincronizar con los tiempos del gateway
    // static constexpr unsigned long RECEIVE_DELAY1 = 1000; // 1 segundo exacto
    // static constexpr unsigned long RECEIVE_DELAY2 = 2000; // 2 segundos exactos
    // static constexpr unsigned long WINDOW_DURATION = 200; // 200ms por ventana
    static constexpr unsigned long RECEIVE_DELAY1 = 4000; // 4 segundo exacto
    static constexpr unsigned long RECEIVE_DELAY2 = 2000; // 2 segundos exactos
    static constexpr unsigned long WINDOW_DURATION = 2000; // 200ms por ventana

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
    
    // LoRaWAN TTN Max TX Power by region
    static constexpr int MAX_POWER[REGIONS] = {
        16, // EU433
        14, // EU868
        30, // US915
        30, // AU915
        30, // AS923
        27, // KR920
        27, // IN865
        14, // CN470
        14, // CN779
        16, // EU433
        30, // AU915OLD
        14, // CN470PREQUEL
        30, // AS923JP
        27  // AS923KR
    };
    
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
    
    // Estado de las ventanas de recepción
    enum RxWindowState
    {
        RX_IDLE,      // No esperando ninguna ventana
        RX_WAIT_1,    // Esperando que se abra RX1
        RX_WINDOW_1,  // En ventana RX1
        RX_WAIT_2,    // Esperando que se abra RX2
        RX_WINDOW_2,  // En ventana RX2
        RX_CONTINUOUS // Escucha continua (solo Clase C)
    };
    
    // Añadir después de las otras enumeraciones
    enum class ConfirmationState {
        NONE,           // Sin confirmación pendiente
        WAITING_ACK,    // Esperando ACK para un mensaje enviado
        ACK_RECEIVED,   // ACK recibido correctamente
        ACK_PENDING     // Necesitamos enviar un ACK al servidor
    };
    
    struct Message {
        std::vector<uint8_t> payload;
        uint8_t port;
        bool confirmed;
    };
    
    // Constructor estándar (usa CH341SPI por defecto)
    LoRaWAN();
    
    // Constructor que acepta un SPIInterface específico
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
    uint8_t getChannel() const;
    
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
    void setSingleChannel(bool enable, float freq_mhz = 868.1, int sf = 9, int bw = 125, int cr = 5, int power = 14, int preamble = 8);
    bool getSingleChannel() const;
    float getSingleChannelFrequency() const;
    
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
    
    // ADR control
    void enableADR(bool enable = true);
    bool isADREnabled() const;
    void applyADRSettings(uint8_t dataRate, uint8_t txPower, const std::vector<uint8_t>& channelMask);
    
    private:
    int lora_region = REGION_EU868;
    
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
    int one_channel_sf = 9;
    int one_channel_bw = 125;
    int one_channel_cr = 5;
    int one_channel_power = 14;
    int one_channel_preamble = 8;
    
    // Atributos privados para gestión de LoRaWAN
    float duty_cycle[CHANNELS] = {0};
    
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
    
    // Métodos para gestión de ventanas de recepción
    void updateRxWindows();
    void openRX2Window();
    
    // ADR related variables
    bool adrEnabled = false;
    int adrAckCounter = 0;
    static constexpr int ADR_ACK_LIMIT = 64;      // Enviar ADR ACK REQ después de estos paquetes sin respuesta
    static constexpr int ADR_ACK_DELAY = 32;      // Reducir DR después de estos paquetes adicionales
    
    // MAC commands
    static constexpr uint8_t MAC_LINK_CHECK_REQ = 0x02;
    static constexpr uint8_t MAC_LINK_CHECK_ANS = 0x02;
    static constexpr uint8_t MAC_LINK_ADR_REQ = 0x03;
    static constexpr uint8_t MAC_LINK_ADR_ANS = 0x03;
    static constexpr uint8_t MAC_DUTY_CYCLE_REQ = 0x04;
    static constexpr uint8_t MAC_DUTY_CYCLE_ANS = 0x04;
    static constexpr uint8_t MAC_RX_PARAM_SETUP_REQ = 0x05;
    static constexpr uint8_t MAC_RX_PARAM_SETUP_ANS = 0x05;
    static constexpr uint8_t MAC_DEV_STATUS_REQ = 0x06;
    static constexpr uint8_t MAC_DEV_STATUS_ANS = 0x06;
    static constexpr uint8_t MAC_NEW_CHANNEL_REQ = 0x07;
    static constexpr uint8_t MAC_NEW_CHANNEL_ANS = 0x07;
    static constexpr uint8_t MAC_RX_TIMING_SETUP_REQ = 0x08;
    static constexpr uint8_t MAC_RX_TIMING_SETUP_ANS = 0x08;
    static constexpr uint8_t MAC_TX_PARAM_SETUP_REQ = 0x09;
    static constexpr uint8_t MAC_TX_PARAM_SETUP_ANS = 0x09;
    
    // Procesamiento de comandos MAC
    void processMACCommands(const std::vector<uint8_t>& commands, std::vector<uint8_t>& response);
    void processLinkADRReq(const std::vector<uint8_t>& cmd, size_t index, std::vector<uint8_t>& response);
    void updateTxParamsForADR();
    
    // Almacenamiento para respuestas MAC pendientes
    std::vector<uint8_t> pendingMACResponses;
    
    // Variables para ADR
    int current_nbRep = 1; // Repeticiones de transmisión por defecto
    uint8_t rx1DrOffset = 0; // Offset de data rate para RX1
    uint8_t rx2DataRate = 0; // Data rate para RX2
    
    // Métodos adicionales para ADR
    void sendADRStatistics();
    void requestLinkCheck();
    
    // Extensión de la implementación de Impl
    struct ImplStats {
        // Variables para estadísticas
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
    
    // Variables para gestión de confirmaciones
    ConfirmationState confirmState = ConfirmationState::NONE;
    int confirmRetries = 0;
    static constexpr int MAX_RETRIES = 8;
    std::chrono::steady_clock::time_point lastConfirmAttempt;
    std::vector<uint8_t> pendingAck;
    uint8_t ackPort = 0;
    bool needsAck = false;
    uint16_t lastFcntDown = 0;
    
    // Métodos para gestión de confirmaciones
    void handleConfirmation();
    void sendAck();
    void resetConfirmationState();
    
    void handleReceivedMessage(const std::vector<uint8_t>& payload, Message& msg);
};
