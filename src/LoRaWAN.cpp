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
// Con CH341SPI
auto ch341_spi = SPIFactory::createCH341SPI(0, true);
auto rfm = RFM95(std::move(ch341_spi));
#else
#if defined(RFM_USE_LINUX_SPI)
// Con SPI de Linux
auto linux_spi = SPIFactory::createLinuxSPI("/dev/spidev0.0", 1000000);
auto rfm = RFM95(std::move(linux_spi));
#endif // defined(RFM_USE_LINUX_SPI)
#endif // defined(RFM_USE_CH341)

bool LoRaWAN::isVerbose = false;

// Helper para debug condicional
#define DEBUG_PRINT(x) do { if(LoRaWAN::getVerbose()) { std::cout << x; } } while(0)
#define DEBUG_PRINTLN(x) do { if(LoRaWAN::getVerbose()) { std::cout << x << std::endl; } } while(0)
#define DEBUG_HEX(x) do { if(LoRaWAN::getVerbose()) { std::cout << std::hex << (x) << std::dec; } } while(0)

struct LoRaWAN::Impl {
    std::unique_ptr<RFM95> rfm;
    std::queue<Message> rxQueue;
    std::mutex queueMutex;
    
    // Claves y direcciones
    std::array<uint8_t, 8> devEUI;
    std::array<uint8_t, 8> appEUI;
    std::array<uint8_t, 16> appKey;
    std::array<uint8_t, 4> devAddr;
    std::array<uint8_t, 16> nwkSKey;
    std::array<uint8_t, 16> appSKey;
    
    // Contadores
    uint32_t uplinkCounter;
    uint32_t downlinkCounter;
    
    // Configuración
    uint8_t dataRate;
    int8_t txPower;
    uint8_t channel;
    
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

    // Función para construir paquete Join Request
    std::vector<uint8_t> buildJoinRequest() {
        std::vector<uint8_t> packet;
        packet.reserve(23);
        
        // MHDR (Join-request = 0x00)
        packet.push_back(0x00);
        
        // AppEUI - enviar en little-endian
        for(int i = 7; i >= 0; i--) {
            packet.push_back(appEUI[i]);
        }
        
        // DevEUI - enviar en little-endian
        for(int i = 7; i >= 0; i--) {
            packet.push_back(devEUI[i]);
        }
        
        // DevNonce (random, little-endian)
        lastDevNonce = generateDevNonce();
        packet.push_back(lastDevNonce & 0xFF);
        packet.push_back((lastDevNonce >> 8) & 0xFF);
        
        // Calcular y añadir MIC
        calculateMIC(packet);

        // Debug detallado del paquete
        DEBUG_PRINTLN("Join Request details (all in LE):");
        DEBUG_PRINT("MHDR: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(packet[0]) << "\n");
        
        DEBUG_PRINT("AppEUI (LE): ");
        for(int i = 1; i <= 8; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT("\n");
        
        DEBUG_PRINT("DevEUI (LE): ");
        for(int i = 9; i <= 16; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT("\n");
        
        DEBUG_PRINT("DevNonce: ");
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(packet[17]) << " "
                  << std::setw(2) << static_cast<int>(packet[18]) << "\n");
        
        DEBUG_PRINT("MIC: ");
        for(size_t i = packet.size()-4; i < packet.size(); i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);
        
        return packet;
    }

    // Función para calcular MIC (Message Integrity Code)
    void calculateMIC(std::vector<uint8_t>& packet) {
        // Debug los datos antes de calcular el MIC
        DEBUG_PRINT("Calculating MIC for data: ");
        for(size_t i = 0; i < packet.size(); i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(packet[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        DEBUG_PRINT("Using Key: ");
        // Para Join Request usamos AppKey, para data usamos NwkSKey
        bool isJoinRequest = (packet[0] & 0xE0) == 0x00;
        const auto& key = isJoinRequest ? appKey : nwkSKey;
        
        for(size_t i = 0; i < 16; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(key[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // No verificar tamaño para paquetes de datos
        if (isJoinRequest && packet.size() != 19) {
            DEBUG_PRINTLN("Invalid packet size for Join Request: " << packet.size());
            return;
        }

        // Si es un paquete de datos, usar el algoritmo de MIC específico para datos
        std::array<uint8_t, 16> cmac;
        if (isJoinRequest) {
            cmac = AESCMAC::calculate(packet, key);
        } else {
            // Para paquetes de datos, el MIC se calcula sobre:
            // B0 | MHDR | FHDR | FPort | FRMPayload
            std::vector<uint8_t> micData;
            micData.reserve(1 + 16 + packet.size()); // B0 + mensaje

            // Block B0
            micData.push_back(0x49); // Block B0
            micData.insert(micData.end(), 4, 0x00); // 0x00^4
            micData.push_back(0x00); // Dir = 0 para uplink
            micData.insert(micData.end(), packet.begin() + 1, packet.begin() + 5);
            micData.insert(micData.end(), packet.begin() + 6, packet.begin() + 8);
            micData.push_back(0x00); // 0x00
            micData.push_back(static_cast<uint8_t>(packet.size())); // len(Data)

            // Añadir el mensaje completo después del bloque B0
            micData.insert(micData.end(), packet.begin(), packet.end());

            // Debug del bloque de datos para MIC
            DEBUG_PRINT("MIC calculation data block: ");
            for(const auto& byte : micData) {
                DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                          << static_cast<int>(byte) << " ");
            }
            DEBUG_PRINT(std::dec << std::endl);

            cmac = AESCMAC::calculate(micData, key);
        }
        
        // Debug el CMAC calculado
        DEBUG_PRINT("Full CMAC: ");
        for(size_t i = 0; i < 16; i++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(cmac[i]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // Los primeros 4 bytes del CMAC son el MIC
        packet.insert(packet.end(), cmac.begin(), cmac.begin() + 4);
    }

    bool processJoinAccept(std::vector<uint8_t>& response) {
        if (response.size() < 17) { // MHDR(1) + AppNonce(3) + NetID(3) + DevAddr(4) + DLSettings(1) + RxDelay(1) + MIC(4)
            DEBUG_PRINTLN("Join Accept: Invalid packet size");
            return false;
        }

        // 1. Descifrar el Join Accept usando AppKey
        std::vector<uint8_t> decrypted(response.size());
        decrypted[0] = response[0]; // MHDR no se cifra
        
        // Descifrar el resto en bloques de 16 bytes
        for (size_t i = 1; i < response.size(); i += 16) {
            size_t block_size = std::min(size_t(16), response.size() - i);
            std::array<uint8_t, 16> block, out;
            std::copy(response.begin() + i, response.begin() + i + block_size, block.begin());
            AESCMAC::aes_encrypt(block.data(), appKey.data(), out.data());
            std::copy(out.begin(), out.begin() + block_size, decrypted.begin() + i);
        }

        // 2. Verificar MIC
        std::vector<uint8_t> mic_data(decrypted.begin(), decrypted.end() - 4);
        std::array<uint8_t, 16> calculated_mic = AESCMAC::calculate(mic_data, appKey);
        
        for (int i = 0; i < 4; i++) {
            if (calculated_mic[i] != decrypted[decrypted.size() - 4 + i]) {
                DEBUG_PRINTLN("Join Accept: Invalid MIC");
                return false;
            }
        }

        // 3. Extraer parámetros en orden correcto
        [[maybe_unused]] uint32_t appNonce = (decrypted[3] << 16) | (decrypted[2] << 8) | decrypted[1];
        [[maybe_unused]] uint32_t netId = (decrypted[6] << 16) | (decrypted[5] << 8) | decrypted[4];
        
        // DevAddr (little-endian como viene en el mensaje)
        devAddr[0] = decrypted[7];
        devAddr[1] = decrypted[8];
        devAddr[2] = decrypted[9];
        devAddr[3] = decrypted[10];

        uint8_t dlSettings = decrypted[11];
        [[maybe_unused]] uint8_t rxDelay = decrypted[12];

        // 4. Derivar claves de sesión
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

        // Debug de la entrada para NwkSKey
        DEBUG_PRINT("NwkSKey Input: ");
        for(const auto& byte : keyInput) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // Generar NwkSKey
        AESCMAC::aes_encrypt(keyInput.data(), appKey.data(), nwkSKey.data());

        // Debug NwkSKey resultante
        DEBUG_PRINT("NwkSKey: ");
        for(const auto& byte : nwkSKey) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // AppSKey = aes128_encrypt(AppKey, 0x02|AppNonce|NetID|DevNonce|pad16)
        keyInput[0] = 0x02; // Solo cambia el primer byte
        AESCMAC::aes_encrypt(keyInput.data(), appKey.data(), appSKey.data());

        // Debug AppSKey resultante
        DEBUG_PRINT("AppSKey: ");
        for(const auto& byte : appSKey) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(byte) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);

        // 5. Configurar parámetros
        dataRate = (dlSettings >> 4) & 0x0F;
        txPower = dlSettings & 0x0F;
        
        // Reset contadores
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
    uint16_t generateDevNonce() {
        // Si no hay nonces previos, empezar desde un valor base
        uint16_t newNonce;
        if (usedNonces.empty()) {
            newNonce = 0x0001;
        } else {
            newNonce = lastDevNonce + 1;
            
            // Si llegamos al máximo, empezar desde 0x0001 de nuevo
            if (newNonce == 0) {
                newNonce = 0x0001;
            }
        }
        
        lastDevNonce = newNonce;
        usedNonces.push_back(newNonce);
        
        // Debug
        DEBUG_PRINTLN("Generated DevNonce: 0x" << std::hex << newNonce << std::dec);
        
        return newNonce;
    }

    void resetDevNonces()
    {
        usedNonces.clear();
        lastDevNonce = 0x0100;
        SessionManager::clearSession(sessionFile);
    }

    // Añadir estadísticas ADR
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
    pimpl(new Impl(SPIFactory::createCH341SPI(0))), // Usa CH341SPI por defecto
    joined(false),
    currentClass(DeviceClass::CLASS_A),
    joinMode(JoinMode::OTAA),
    adrEnabled(false), // Inicializar ADR deshabilitado
    adrAckCounter(0)   // Inicializar contador ADR
{
    // Inicializar registros de duty cycle
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Iniciar como hace 24 horas
        channelAirTime[i] = 0.0f;
    }
    
    // Configurar frecuencias de canales para la región seleccionada
    for (int i = 0; i < MAX_CHANNELS; i++) {
        channelFrequencies[i] = (i < 8) ? BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region] : 0.0f;
    }
}

LoRaWAN::LoRaWAN(std::unique_ptr<SPIInterface> spi_interface) : 
    pimpl(new Impl(std::move(spi_interface))), // Usa la interfaz proporcionada
    joined(false),
    currentClass(DeviceClass::CLASS_A),
    joinMode(JoinMode::OTAA),
    adrEnabled(false), // Inicializar ADR deshabilitado
    adrAckCounter(0)   // Inicializar contador ADR
{
    // Inicializar registros de duty cycle
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Iniciar como hace 24 horas
        channelAirTime[i] = 0.0f;
    }
    
    // Configurar frecuencias de canales para la región seleccionada
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
    pimpl->rfm->setTxPower(14, true);
    pimpl->rfm->setSpreadingFactor(9);
    pimpl->rfm->setBandwidth(125.0);
    pimpl->rfm->setCodingRate(5);
    pimpl->rfm->setPreambleLength(8);
    pimpl->rfm->setSyncWord(0x34); // LoRaWAN sync word
    return true;
}

void LoRaWAN::setDeviceClass(DeviceClass deviceClass) {
    currentClass = deviceClass;
    
    // Si cambiamos a Clase C, configurar inmediatamente para recepción continua en RX2
    if (deviceClass == DeviceClass::CLASS_C && joined) {
        DEBUG_PRINTLN("Configurando modo Clase C (recepción continua en 869.525 MHz)");
        
        // Configurar radio para ventana RX2
        pimpl->rfm->standbyMode();
        pimpl->rfm->setFrequency(869.525); // Frecuencia RX2 para EU868
        
        pimpl->rfm->setSpreadingFactor(9);
        pimpl->rfm->setBandwidth(125.0);
        pimpl->rfm->setInvertIQ(true);  // Invertir IQ para downlink
        pimpl->rfm->setLNA(1, true);
        
        // Iniciar recepción continua
        pimpl->rfm->setContinuousReceive();
    }
}

void LoRaWAN::setDevEUI(const std::string& devEUI) {
    DEBUG_PRINTLN("Setting DevEUI: " << devEUI);
    
    // Convertir string hex a bytes en orden natural
    for(size_t i = 0; i < 8 && (i*2+1) < devEUI.length(); i++) {
        std::string byteStr = devEUI.substr(i*2, 2);
        pimpl->devEUI[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: mostrar DevEUI como se almacenó
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
    
    // Convertir string hex a bytes en orden natural
    for(size_t i = 0; i < 8 && (i*2+1) < appEUI.length(); i++) {
        std::string byteStr = appEUI.substr(i*2, 2);
        pimpl->appEUI[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: mostrar AppEUI como se almacenó
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
    
    // Convertir string hex a bytes sin modificar el orden
    for(size_t i = 0; i < 16 && i*2+1 < appKey.length(); i++) {
        std::string byteStr = appKey.substr(i*2, 2);
        pimpl->appKey[i] = std::stoi(byteStr, nullptr, 16);
    }
    
    // Debug: mostrar AppKey almacenado
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
    // Intentar cargar sesión existente primero SOLAMENTE si no se forzó un reset previo
    if (!joined && pimpl->loadSessionData()) {
        joined = true;
        DEBUG_PRINTLN("Restored previous session");
        return true;
    }
    
    // Si no hay sesión válida, hacer join normal
    DEBUG_PRINTLN("Performing new OTAA join...");
    joinMode = mode;
    
    if (mode == JoinMode::OTAA) {
        // Configuración de radio para Join Request
        pimpl->rfm->standbyMode();  // Cambiamos . por ->
        pimpl->rfm->setFrequency(868.1);
        pimpl->rfm->setTxPower(14, true);
        pimpl->rfm->setSpreadingFactor(9);
        pimpl->rfm->setBandwidth(125.0);
        pimpl->rfm->setCodingRate(5);
        pimpl->rfm->setPreambleLength(8);
        pimpl->rfm->setInvertIQ(false);
        pimpl->rfm->setSyncWord(0x34);

        // Limpiar flags de interrupción
        pimpl->rfm->clearIRQFlags();

        // Preparar y enviar Join Request
        auto joinRequest = pimpl->buildJoinRequest();
        
        if (!pimpl->rfm->send(joinRequest)) {
            DEBUG_PRINTLN("Failed to send Join Request");
            return false;
        }

        // Configuración RX1
        pimpl->rfm->standbyMode();
        pimpl->rfm->setFrequency(868.1);
        pimpl->rfm->setSpreadingFactor(9);
        pimpl->rfm->setBandwidth(125.0);
        pimpl->rfm->setInvertIQ(true);
        pimpl->rfm->setLNA(1, true);
        
        // Iniciar recepción continua
        pimpl->rfm->setContinuousReceive();
        
        // Primera ventana RX (RX1) - 5 segundos después del uplink, pero iniciamos la recepción desde el principio
        DEBUG_PRINTLN("Opening RX1 window...");
        // Esperar por el paquete durante 1 segundo
        auto start = std::chrono::steady_clock::now();
        bool received = false;
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count() < 7000) { // 5 segundos de espera + 2 segundos de recepción = 7 segundos total

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
                            // Si el join fue exitoso, guardar sesión
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
            // Segunda ventana RX (RX2)
            DEBUG_PRINTLN("Opening RX2 window...");
            
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(869.525);
            pimpl->rfm->setSpreadingFactor(12);
            pimpl->rfm->setBandwidth(125.0);
            pimpl->rfm->setInvertIQ(true);
            pimpl->rfm->setLNA(1, true);
            
            // Limpiar flags antes de RX2
            pimpl->rfm->clearIRQFlags();
            pimpl->rfm->setContinuousReceive();
            
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count() < 10000) { // Los 7 segundos de RX1 + 3 segundos de RX2
                
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
                                // Si el join fue exitoso, guardar sesión
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
    
    // Para ABP, solo validar que tenemos las keys necesarias
    joined = validateKeys();
    return joined;
}

std::vector<uint8_t> LoRaWAN::encryptPayload(const std::vector<uint8_t>& payload, uint8_t port) {
    if (payload.empty()) return payload;
    
    // Seleccionar la clave correcta según el puerto
    const auto& key = (port == 0) ? pimpl->nwkSKey : pimpl->appSKey;
    
    // Crear bloque A para el cifrado AES-CTR
    std::array<uint8_t, 16> block_a;
    block_a[0] = 0x01;  // Dirección de mensaje: 0x00 downlink, 0x01 uplink
    block_a[1] = 0x00;  // Padding
    block_a[2] = 0x00;  // Padding
    block_a[3] = 0x00;  // Padding
    
    // Copiar DevAddr (little endian)
    std::copy(pimpl->devAddr.begin(), pimpl->devAddr.end(), block_a.begin() + 4);
    
    // Frame counter
    uint32_t fcnt = pimpl->uplinkCounter;
    block_a[8] = fcnt & 0xFF;
    block_a[9] = (fcnt >> 8) & 0xFF;
    block_a[10] = (fcnt >> 16) & 0xFF;
    block_a[11] = (fcnt >> 24) & 0xFF;
    
    block_a[12] = 0x00;  // 0x00 para contador alto
    block_a[13] = 0x00;  // Padding
    block_a[14] = 0x00;  // Padding
    block_a[15] = 0x01;  // Block counter starts at 1
    
    std::vector<uint8_t> encrypted;
    encrypted.reserve(payload.size());
    
    // Procesar el payload en bloques de 16 bytes
    for (size_t i = 0; i < payload.size(); i += 16) {
        // Generar el bloque S usando AES-128
        std::array<uint8_t, 16> s;
        AESCMAC::aes_encrypt(block_a.data(), key.data(), s.data());
        
        // XOR con el payload
        size_t block_size = std::min(size_t(16), payload.size() - i);
        for (size_t j = 0; j < block_size; j++) {
            encrypted.push_back(payload[i + j] ^ s[j]);
        }
        
        // Incrementar contador de bloque
        block_a[15]++;
    }
    
    return encrypted;
}

std::vector<uint8_t> LoRaWAN::decryptPayload(const std::vector<uint8_t>& payload, uint8_t port) {
    if (payload.empty()) return payload;
    
    // Seleccionar la clave correcta según el puerto
    const auto& key = (port == 0) ? pimpl->nwkSKey : pimpl->appSKey;
    
    // Crear bloque A para el descifrado AES-CTR
    std::array<uint8_t, 16> block_a;
    block_a[0] = 0x01;  // Tipo de bloque
    block_a[1] = 0x00;  // Padding
    block_a[2] = 0x00;  // Padding
    block_a[3] = 0x00;  // Padding
    block_a[4] = 0x00;  // Padding
    block_a[5] = 0x01;  // Dirección = 0x01 para downlink (CORREGIDO)
    
    // Copiar DevAddr (little endian)
    std::copy(pimpl->devAddr.begin(), pimpl->devAddr.end(), block_a.begin() + 6);
    
    // Frame counter (del downlink)
    uint16_t fcnt = pimpl->downlinkCounter;
    block_a[10] = fcnt & 0xFF;
    block_a[11] = (fcnt >> 8) & 0xFF;
    block_a[12] = 0x00;  // FCnt MSB (32 bits)
    block_a[13] = 0x00;  // FCnt MSB (32 bits)
    block_a[14] = 0x00;  // Padding
    block_a[15] = 0x01;  // Contador de bloque
    
    // Debug para ver los parámetros de descifrado
    DEBUG_PRINTLN("Decryption parameters:\n");
    DEBUG_PRINTLN("  Direction: Downlink (1)\n");
    DEBUG_PRINT("  DevAddr: ");
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(pimpl->devAddr[i]) << " ");
    }
    DEBUG_PRINT("\n  FCnt: " << std::dec << fcnt << " (0x" 
              << std::hex << fcnt << ")\n");
    DEBUG_PRINT("  Key: ");
    for (const auto& byte : key) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);
    
    // Vector para almacenar los datos descifrados
    std::vector<uint8_t> decrypted;
    decrypted.reserve(payload.size());
    
    // Procesar el payload en bloques de 16 bytes
    for (size_t i = 0; i < payload.size(); i += 16) {
        // Generar el bloque S usando AES-128
        std::array<uint8_t, 16> s;
        AESCMAC::aes_encrypt(block_a.data(), key.data(), s.data());
        
        // Debug del bloque S antes del XOR
        DEBUG_PRINT("Bloque S para XOR: ");
        for (int j = 0; j < 16; j++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                     << static_cast<int>(s[j]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);
        
        // Debug del payload cifrado
        DEBUG_PRINT("Payload cifrado para XOR: ");
        for (size_t j = 0; j < std::min(size_t(16), payload.size() - i); j++) {
            DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                     << static_cast<int>(payload[i + j]) << " ");
        }
        DEBUG_PRINT(std::dec << std::endl);
        
        // XOR con el payload cifrado
        size_t block_size = std::min(size_t(16), payload.size() - i);
        for (size_t j = 0; j < block_size; j++) {
            decrypted.push_back(payload[i + j] ^ s[j]);
        }
        
        // Incrementar contador de bloque
        block_a[15]++;
    }
    
    // Debug: mostrar resultado del descifrado
    DEBUG_PRINT("Decrypted payload: ");
    for (const auto& byte : decrypted) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);
    
    return decrypted;
}

float LoRaWAN::calculateTimeOnAir(size_t payload_size) {
    // Extraer parámetros actuales
    int sf = pimpl->rfm->getSpreadingFactor();  // Cambiamos . por ->
    float bw = pimpl->rfm->getBandwidth() * 1000; // Convertir de kHz a Hz
    int cr = pimpl->rfm->getCodingRate();
    
    // Calcular número de símbolos en preámbulo
    int preambleSymbols = pimpl->rfm->getPreambleLength() + 4.25;
    
    // Calcular duración de símbolo (ms)
    double symbolDuration = std::pow(2.0, static_cast<double>(sf)) / bw;
    
    // Calcular tamaño en bits con overhead y codificación
    size_t packet_size = payload_size + 13; // Data + LoRaWAN overhead
    double payloadSymbols = 8 + std::max(std::ceil((8 * packet_size - 4 * sf + 28 + 16) / (4 * sf)) * (cr + 4), 0.0);
    
    // Tiempo total en ms = (preámbulo + payload) * duración de símbolo
    float timeOnAir = (preambleSymbols + payloadSymbols) * symbolDuration * 1000;
    
    DEBUG_PRINTLN("Calculated time on air: " << timeOnAir << " ms");
    DEBUG_PRINTLN("Parameters: SF=" << sf << ", BW=" << (bw/1000) << "kHz, CR=4/" << cr);
    DEBUG_PRINTLN("Symbols: preamble=" << preambleSymbols << ", payload=" << payloadSymbols);
    DEBUG_PRINTLN("Symbol duration: " << (symbolDuration*1000) << " ms");
    
    return timeOnAir;
}

bool LoRaWAN::checkDutyCycle(float frequency, size_t payload_size) {
    // Identificar el canal basado en la frecuencia
    int channel = -1;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (std::abs(frequency - channelFrequencies[i]) < 0.01) {
            channel = i;
            break;
        }   
    }
    
    if (channel == -1) {
        // Canal no encontrado, usar canal 0 por defecto
        channel = 0;
    }
    
    // Obtener tiempo actual
    auto now = std::chrono::steady_clock::now();
    
    // Calcular tiempo transcurrido desde último uso
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastChannelUse[channel]).count();
    
    // Calcular tiempo de aire requerido
    float airTime = calculateTimeOnAir(payload_size);
    
    // Calcular tiempo mínimo de espera según duty cycle (1%)
    // t_wait = (t_air / duty_cycle) - t_air
    float minWaitTime = (airTime / 0.01) - airTime;
    
    // Verificar si ha pasado suficiente tiempo
    if (elapsed < minWaitTime) {
        DEBUG_PRINTLN("Duty cycle restriction: need to wait " 
                  << (minWaitTime - elapsed) << " ms more on channel " 
                  << channel << " (" << frequency << " MHz)");
        return false;
    }
    
    // Registrar uso del canal
    lastChannelUse[channel] = now;
    channelAirTime[channel] += airTime;
    
    return true;
}

float LoRaWAN::getDutyCycleUsage(int channel) {
    if(channel < 0 || channel >= MAX_CHANNELS) {
        return 0.0f; // Canal inválido
    }
    
    // Calcular tiempo transcurrido desde último uso
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastChannelUse[channel]).count();
    
    // Si ha pasado más de 1 hora, considerar que el duty cycle es 0
    if(elapsed_ms > 3600000) {
        return 0.0f;
    }
    
    // Calcular uso actual del duty cycle como porcentaje
    return (channelAirTime[channel] / 36000.0f) * 100.0f; // Porcentaje de uso en última hora
}

void LoRaWAN::resetDutyCycle() {
    auto now = std::chrono::steady_clock::now();
    for(int i = 0; i < MAX_CHANNELS; i++) {
        lastChannelUse[i] = now - std::chrono::hours(24); // Iniciar como hace 24 horas
        channelAirTime[i] = 0.0f;
    }
}

bool LoRaWAN::send(const std::vector<uint8_t>& data, uint8_t port, bool confirmed, bool force_duty_cycle) {
    if (!joined) return false;

    // Debug del payload original
    DEBUG_PRINTLN("Preparing uplink packet:");
    DEBUG_PRINT("Data to send: ");
    for(const auto& byte : data) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);

    // Configurar radio para uplink
    pimpl->rfm->standbyMode();
    
    // Seleccionar canal según mejor disponibilidad de duty cycle
    float lowestUsage = 100.0f;
    int bestChannel = 0;

    // Si es gateway de un canal, usar esa frecuencia
    if (one_channel_gateway) {
        pimpl->rfm->setFrequency(one_channel_freq);
    } else {
        // Buscar el canal con menor uso de duty cycle
        for (int i = 0; i < 8; i++) {  // Solo los primeros 8 canales
            float usage = getDutyCycleUsage(i);
            if (usage < lowestUsage && channelFrequencies[i] > 0) {
                lowestUsage = usage;
                bestChannel = i;
            }
        }
        pimpl->rfm->setFrequency(channelFrequencies[bestChannel]);
        DEBUG_PRINTLN("Seleccionado canal " << bestChannel << " con frecuencia " 
                   << channelFrequencies[bestChannel] << " MHz (uso: " << lowestUsage << "%)");
    }
    
    pimpl->rfm->setSpreadingFactor(9);
    pimpl->rfm->setBandwidth(125.0);
    pimpl->rfm->setCodingRate(5);
    pimpl->rfm->setPreambleLength(8);
    pimpl->rfm->setInvertIQ(false);
    pimpl->rfm->setSyncWord(0x34);

    // Debug de las claves de sesión
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
    
    // Construir paquete LoRaWAN estrictamente según la especificación 1.0.3
    std::vector<uint8_t> packet;
    packet.reserve(data.size() + 13);
    
    // MHDR (1 byte)
    packet.push_back(confirmed ? 0x80 : 0x40);

    // FHDR en el orden correcto según la especificación sección 4.3.1:
    // 1. DevAddr (4 bytes)
    packet.insert(packet.end(), pimpl->devAddr.begin(), pimpl->devAddr.end());
    
    // 2. FCtrl (1 byte) - ANTES del FCnt
    // Configurar FCtrl para ADR
    uint8_t fctrl = 0x00;
    if (adrEnabled) {
        fctrl |= 0x80; // bit 7 = ADR
        
        // Comprobar si necesitamos enviar ADR ACK request
        if (adrAckCounter >= ADR_ACK_LIMIT) {
            fctrl |= 0x40; // bit 6 = ADR ACK REQ
            DEBUG_PRINTLN("Enviando ADR ACK request (contador: " << adrAckCounter << ")");
        }
    }
    packet.push_back(fctrl);
    
    // 3. FCnt (2 bytes, little-endian) - DESPUÉS del FCtrl
    packet.push_back(pimpl->uplinkCounter & 0xFF);        // FCnt LSB
    packet.push_back((pimpl->uplinkCounter >> 8) & 0xFF); // FCnt MSB

    // Si tenemos respuestas MAC pendientes, incluirlas en FOpts
    if (!pendingMACResponses.empty()) {
        // Verificar que se puedan incluir (máx 15 bytes en FOpts)
        if (pendingMACResponses.size() <= 15) {
            // Actualizar FCtrl para indicar la longitud de FOpts
            fctrl |= pendingMACResponses.size() & 0x0F;
            packet[5] = fctrl;
            
            // Insertar respuestas MAC después de FCnt
            packet.insert(packet.begin() + 8, pendingMACResponses.begin(), pendingMACResponses.end());
            
            DEBUG_PRINTLN("Incluyendo " << pendingMACResponses.size() << " bytes de respuestas MAC");
            pendingMACResponses.clear();
        }
    }

    // 4. FPort (1 byte)
    packet.push_back(port);
    
    // 5. FRMPayload (encriptado)
    auto encrypted = encryptPayload(data, port);
    packet.insert(packet.end(), encrypted.begin(), encrypted.end());

    // Debug detallado del paquete antes del MIC
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
    
    // Cálculo del MIC según spec 4.4
    // B0 block exactamente como lo define la especificación
    std::array<uint8_t, 16> b0;
    std::fill(b0.begin(), b0.end(), 0);
    b0[0] = 0x49;  // Código de bloque para MIC
    // bytes 1-4 son 0x00 (reservado)
    b0[5] = 0x00;  // Dir = 0 para uplink
    
    // DevAddr (4 bytes, little-endian)
    std::copy(pimpl->devAddr.begin(), pimpl->devAddr.end(), b0.begin() + 6);
    
    // FCnt (4 bytes, little-endian, los 2 bytes superiores son 0)
    b0[10] = pimpl->uplinkCounter & 0xFF;
    b0[11] = (pimpl->uplinkCounter >> 8) & 0xFF;
    // bytes 12-13 son 0x00 (FCntMSB = 0)
    
    // byte 14 es 0x00 (siempre 0 para mensajes pequeños)
    b0[15] = static_cast<uint8_t>(packet.size());  // Longitud del mensaje

    // Debug del bloque B0
    DEBUG_PRINT("B0 block for MIC: ");
    for(const auto& byte : b0) {
        DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(byte) << " ");
    }
    DEBUG_PRINT(std::dec << std::endl);

    // Construir el bloque para CMAC
    std::vector<uint8_t> cmacData;
    cmacData.insert(cmacData.end(), b0.begin(), b0.end());
    cmacData.insert(cmacData.end(), packet.begin(), packet.end());
    
    // Calcular CMAC con NwkSKey
    auto cmac = AESCMAC::calculate(cmacData, pimpl->nwkSKey);
    
    // Añadir los primeros 4 bytes como MIC
    packet.insert(packet.end(), cmac.begin(), cmac.begin() + 4);
    
    // Calcular el tamaño del paquete para estimar tiempo de aire
    size_t packetSize = data.size() + 13; // Datos + overhead LoRaWAN
    
    // Verificar duty cycle si no está forzado
    float frequency = 868.1; // Para gateway one-channel, siempre usamos esta frecuencia
    
    if (!force_duty_cycle && !checkDutyCycle(frequency, packetSize)) {
        DEBUG_PRINTLN("Duty cycle restriction active, delaying transmission");
        // Esperar el tiempo necesario
        int channel = 0; // Single channel gateway siempre usa canal 0
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
    
    // Debug de los parámetros radio para verificación
    DEBUG_PRINTLN("Radio parameters:");
    DEBUG_PRINTLN("  Frequency: " << pimpl->rfm->getFrequency() << " MHz");
    DEBUG_PRINTLN("  SF: " << pimpl->rfm->getSpreadingFactor());
    DEBUG_PRINTLN("  BW: " << pimpl->rfm->getBandwidth() << " kHz");
    DEBUG_PRINTLN("  CR: 4/" << pimpl->rfm->getCodingRate());
    DEBUG_PRINTLN("  Power: " << pimpl->rfm->getTxPower() << " dBm");
    DEBUG_PRINTLN("Enviando paquete...");
    
    // Asegurar que el paquete se envía correctamente comprobando estado del radio
    pimpl->rfm->clearIRQFlags();
    uint8_t opMode = pimpl->rfm->readRegister(RFM95::REG_OP_MODE);
    DEBUG_PRINTLN("Modo previo a TX: 0x" << std::hex << (int)opMode << std::dec);
    
    // Transmitir el paquete
    bool result = pimpl->rfm->send(packet);
    
    // Comprobar resultado incluso si el flag no se actualiza
    if (result) {
        DEBUG_PRINTLN("Envío de paquete completado");
        pimpl->rfm->standbyMode();
        // Incrementar contador y guardar sesión
        pimpl->uplinkCounter++;
        
        // Incrementar contador ADR si está habilitado
        if (adrEnabled) {
            adrAckCounter++;
            
            // Reducir DR (aumentar SF) si hace tiempo que no recibimos respuesta
            if (adrAckCounter > ADR_ACK_LIMIT + ADR_ACK_DELAY) {
                updateTxParamsForADR();
            }
        }
        
        pimpl->saveSessionData();
        // Esperamos 6 segundos para cumplir duty cycle (1% en 868.1 MHz)
        DEBUG_PRINTLN("Esperando 6 segundos para duty cycle...");
        std::this_thread::sleep_for(std::chrono::seconds(6));
        
        // Volver a modo recepción continua con la configuración adecuada según la clase
        if (currentClass == DeviceClass::CLASS_C) {
            DEBUG_PRINTLN("Configurando recepción continua en RX2 (869.525 MHz, Clase C)");
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);
            pimpl->rfm->setSpreadingFactor(9);  // SF9 según lo observado
            pimpl->rfm->setBandwidth(125.0);
            pimpl->rfm->setInvertIQ(true);      // IQ invertido para downlink
            pimpl->rfm->setContinuousReceive();
        } else {
            // Para Clase A, configurar RX1 normalmente
            DEBUG_PRINTLN("Volviendo a modo standby (Clase A)");
            pimpl->rfm->standbyMode();
        }
    } else {
        DEBUG_PRINTLN("Error al enviar el paquete");
        
        // Incluso en caso de error, asegurarse de que volvemos a recepción
        pimpl->rfm->setContinuousReceive();
    }
    
    return result;
}

void LoRaWAN::update() {
    if (!joined) return;

    // Asegurarse de que estamos en modo de recepción continua
    uint8_t opMode = pimpl->rfm->readRegister(RFM95::REG_OP_MODE);
    
    // Solo reconfigurar si no estamos ya en modo RX continuo
    if ((opMode & 0x07) != RFM95::MODE_RX_CONTINUOUS) {
        // Si estamos en Clase C, siempre escuchar en RX2
        if (currentClass == DeviceClass::CLASS_C) {
            // Configurar para RX2
            pimpl->rfm->standbyMode();
            pimpl->rfm->setFrequency(RX2_FREQ[lora_region]);
            pimpl->rfm->setSpreadingFactor(9); // SF9 según lo observado
            pimpl->rfm->setBandwidth(125.0);
            pimpl->rfm->setInvertIQ(true);  // Invertir IQ para downlink
            pimpl->rfm->setContinuousReceive();
            DEBUG_PRINTLN("Radio reconfigurada para RX2 continuo en " << RX2_FREQ[lora_region] << " MHz (SF9)");
        } else {
            // Para Clase A, configurar en la frecuencia principal
            pimpl->rfm->standbyMode();
            if (one_channel_gateway)
            {
                pimpl->rfm->setFrequency(one_channel_freq);
            } else {
                // Buscar el canal con menor uso de duty cycle
                float lowestUsage = 100.0f;
                int bestChannel = 0;
                for (int i = 0; i < 8; i++) { // Solo los primeros 8 canales
                    float usage = getDutyCycleUsage(i);
                    if (usage < lowestUsage && channelFrequencies[i] > 0) {
                        lowestUsage = usage;
                        bestChannel = i;
                    }
                }
                pimpl->rfm->setFrequency(channelFrequencies[bestChannel]);
                DEBUG_PRINTLN("Seleccionado canal " << bestChannel << " con frecuencia "
                                                    << channelFrequencies[bestChannel] << " MHz (uso: " << lowestUsage << "%)");
            }
            pimpl->rfm->setSpreadingFactor(9);
            pimpl->rfm->setBandwidth(125.0);
            pimpl->rfm->setInvertIQ(true);
            pimpl->rfm->setContinuousReceive();
        }
    }

    // Comprobar si hay datos recibidos verificando IRQ flags
    uint8_t flags = pimpl->rfm->getIRQFlags();
    
    if (flags & RFM95::IRQ_RX_DONE_MASK) {
        DEBUG_PRINTLN("¡Detectada recepción de paquete!");
        
        // Verificar si hay error CRC
        if (flags & RFM95::IRQ_PAYLOAD_CRC_ERROR_MASK) {
            DEBUG_PRINTLN("Error CRC en paquete recibido");
        } else {
            // Mostrar información detallada del paquete
            int rssi = pimpl->rfm->getRSSI();
            float snr = pimpl->rfm->getSNR();
            
            auto payload = pimpl->rfm->readPayload();
            if (!payload.empty()) {
                DEBUG_PRINTLN("Paquete recibido: " << payload.size() << " bytes, RSSI: " 
                          << rssi << " dBm, SNR: " << snr << " dB");
                DEBUG_PRINT("Hex: ");
                for (const auto& b : payload) {
                    DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << (int)b);
                }
                DEBUG_PRINTLN(std::dec);
                
                // Procesar el paquete si es un formato LoRaWAN válido
                if (payload.size() >= 13) {
                    Message msg;
                    uint8_t mhdr = payload[0];
                    
                    // Verificar si es downlink (0x60 = unconfirmed, 0xA0 = confirmed)
                    if ((mhdr & 0xE0) == 0x60 || (mhdr & 0xE0) == 0xA0) {
                        msg.confirmed = (mhdr & 0x20) != 0;
                        
                        // Extraer DevAddr para verificación
                        std::array<uint8_t, 4> recvDevAddr;
                        std::copy(payload.begin() + 1, payload.begin() + 5, recvDevAddr.begin());
                        
                        // Verificar que el DevAddr coincide con el nuestro
                        bool addressMatch = std::equal(recvDevAddr.begin(), recvDevAddr.end(), pimpl->devAddr.begin());
                        
                        if (addressMatch) {
                            // Recolectar estadísticas para ADR (antes de cualquier otro procesamiento)
                            int rssi = pimpl->rfm->getRSSI();
                            float snr = pimpl->rfm->getSNR();
                            pimpl->addSnrSample(snr);
                            pimpl->addRssiSample(rssi);

                            // Resetear contador ADR al recibir cualquier downlink
                            if (adrEnabled) {
                                adrAckCounter = 0;
                                DEBUG_PRINTLN("ADR: Resetear contador por recepción de downlink");
                            }

                            // Extraer FCnt del mensaje (bytes 6-7)
                            uint16_t fcnt = payload[6] | (payload[7] << 8);
                            // Actualizar el contador de downlink
                            pimpl->downlinkCounter = fcnt;
                            DEBUG_PRINTLN("FCnt extraído del downlink: " << fcnt);

                            // Añadir estos logs para más información
                            DEBUG_PRINTLN("Valor hexadecimal de FCnt bytes: " 
                                      << std::hex << (int)payload[6] << " " << (int)payload[7] << std::dec);
                            DEBUG_PRINT("DevAddr bytes (hex): ");
                            for (int i = 0; i < 4; i++) {
                                DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                                         << static_cast<int>(pimpl->devAddr[i]) << " ");
                            }
                            DEBUG_PRINTLN(std::dec);
                            
                            // Extraer FPort
                            msg.port = (payload.size() > 8) ? payload[8] : 0;
                            
                            // Extraer y decriptar payload usando el nuevo FCnt
                            if (payload.size() > 9) {
                                std::vector<uint8_t> encrypted(payload.begin() + 9, payload.end() - 4);
                                msg.payload = decryptPayload(encrypted, msg.port);
                                DEBUG_PRINT("Mensaje LoRaWAN descifrado: Puerto=" << (int)msg.port 
                                          << ", Tipo=" << (msg.confirmed ? "Confirmado" : "No Confirmado")
                                          << ", Payload=");
                                for (const auto& b : msg.payload) {
                                    DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') 
                                             << (int)b << " ");
                                }
                                DEBUG_PRINTLN(std::dec);
                            }
                            
                            // Notificar mediante callback
                            if (receiveCallback) {
                                receiveCallback(msg);
                            } else {
                                // Guardar en la cola
                                std::lock_guard<std::mutex> lock(pimpl->queueMutex);
                                pimpl->rxQueue.push(msg);
                            }
                            
                            // Verificar si hay comandos MAC (FPort = 0)
                            if (payload.size() > 8 && payload[8] == 0) {
                                // FPort 0 indica que el payload son comandos MAC
                                std::vector<uint8_t> encrypted(payload.begin() + 9, payload.end() - 4);
                                std::vector<uint8_t> macCommands = decryptPayload(encrypted, 0);
                                
                                DEBUG_PRINT("Recibidos comandos MAC: ");
                                for (const auto& b : macCommands) {
                                    DEBUG_PRINT(std::hex << std::setw(2) << std::setfill('0') << (int)b << " ");
                                }
                                DEBUG_PRINTLN(std::dec);
                                
                                // Procesar comandos y generar respuesta
                                std::vector<uint8_t> macResponse;
                                processMACCommands(macCommands, macResponse);
                                
                                // Almacenar respuesta para incluirla en el próximo uplink
                                if (!macResponse.empty()) {
                                    pendingMACResponses = macResponse;
                                }
                                
                                // También resetear contador ADR ya que recibimos un downlink
                                if (adrEnabled) {
                                    adrAckCounter = 0;
                                }
                            }
                        } else {
                            DEBUG_PRINTLN("DevAddr no coincide, ignorando paquete");
                        }
                    }
                }
            }
        }
        
        // Limpiar flag y reconfigurar para seguir recibiendo
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
        DEBUG_PRINTLN("Frecuencia no válida o no permitida en ningún canal");
    }
}

int LoRaWAN::getChannelFromFrequency(float freq_mhz) const {
    if (lora_region < 0 || lora_region >= REGIONS) {
        DEBUG_PRINTLN("Región LoRa no válida");
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
        // Establecer la frecuencia correspondiente en el radio
        pimpl->rfm->setFrequency(channelFrequencies[channel]);
    } else {
        DEBUG_PRINTLN("Canal no válido o desactivado: " << channel);
    }
}

uint8_t LoRaWAN::getChannel() const {
    return pimpl->channel; // Devolver directamente el canal almacenado
}

void LoRaWAN::setOneChannelGateway(bool enable, float freq_mhz) {
    one_channel_gateway = enable;
    one_channel_freq = freq_mhz;
}

bool LoRaWAN::getOneChannelGateway() const {
    return one_channel_gateway;
}

float LoRaWAN::getOneChannelFrequency() const {
    return one_channel_freq;
}

void LoRaWAN::setTxPower(int8_t power) {
    // Limitar entre 2 dBm y el máximo permitido por la región
    if (power < 2) power = 2; // La mayoría de los módulos LoRa no van por debajo de 2 dBm
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
    // Validar DevAddr - no debe ser todo ceros
    bool validDevAddr = false;
    for (const auto& byte : pimpl->devAddr) {
        if (byte != 0) {
            validDevAddr = true;
            break;
        }
    }
    if (!validDevAddr) {
        DEBUG_PRINTLN("ABP validation failed: DevAddr is all zeros");
        std::cerr << "Error: DevAddr no válida para ABP" << std::endl;
        return false;
    }
    
    // Validar NwkSKey - no debe ser todo ceros
    bool validNwkSKey = false;
    for (const auto& byte : pimpl->nwkSKey) {
        if (byte != 0) {
            validNwkSKey = true;
            break;
        }
    }
    if (!validNwkSKey) {
        DEBUG_PRINTLN("ABP validation failed: NwkSKey is all zeros");
        std::cerr << "Error: NwkSKey no válida para ABP" << std::endl;
        return false;
    }
    
    // Validar AppSKey - no debe ser todo ceros
    bool validAppSKey = false;
    for (const auto& byte : pimpl->appSKey) {
        if (byte != 0) {
            validAppSKey = true;
            break;
        }
    }
    if (!validAppSKey) {
        DEBUG_PRINTLN("ABP validation failed: AppSKey is all zeros");
        std::cerr << "Error: AppSKey no válida para ABP" << std::endl;
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
    // Limpiar claves de sesión
    pimpl->devAddr.fill(0);
    pimpl->nwkSKey.fill(0);
    pimpl->appSKey.fill(0);

    // Reiniciar flags y contadores
    pimpl->uplinkCounter = 0;
    pimpl->downlinkCounter = 0;
    joined = false;

    // Borrar el archivo de sesión si existe
    SessionManager::clearSession(pimpl->sessionFile);

    // Reiniciar DevNonces
    pimpl->resetDevNonces();

    // También reiniciar estadísticas ADR
    pimpl->snrHistory.clear();
    pimpl->rssiHistory.clear();
    adrAckCounter = 0;

    DEBUG_PRINTLN("LoRaWAN session reset successfully");
}

void LoRaWAN::applyADRSettings(uint8_t dataRate, uint8_t txPower, const std::vector<uint8_t>& channelMask) {

    // Mapear DR a SF/BW según la región
    int sf = 9; // Valor predeterminado
    float bw = 125.0f;

    // Determinar SF y BW según la región y el DR
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
            
        // Añadir otras regiones según sea necesario
        default:
            // Mapeo genérico
            sf = dataRate <= 6 ? (12 - dataRate) : 7;
            bw = dataRate == 6 ? 250.0f : 125.0f;
    }

    // Determinar potencia según la región
    int power = 14; // Valor predeterminado
    switch (lora_region) {
        case REGION_EU868:
            power = MAX_POWER[lora_region] - (txPower * 2);
            break;
        // Añadir otras regiones según sea necesario
        default:
            power = MAX_POWER[lora_region] - (txPower * 2);
    }
    
    // Limitar a valores seguros
    if (sf < 7) sf = 7;
    if (sf > 12) sf = 12;
    if (power < 2) power = 2;
    if (power > MAX_POWER[lora_region]) power = MAX_POWER[lora_region];
    
    // Aplicar configuración al radio usando el pimpl (esto se hace en LoRaWAN.cpp)
    pimpl->rfm->setSpreadingFactor(sf);
    pimpl->rfm->setBandwidth(bw);
    pimpl->rfm->setTxPower(power, true);
}

// Método para solicitar un LinkCheckReq en el próximo uplink
void LoRaWAN::requestLinkCheck()
{
    // Añadir comando LinkCheckReq a las respuestas pendientes
    if (pendingMACResponses.size() < 15)
    { // Máximo 15 bytes en FOpts
        pendingMACResponses.push_back(MAC_LINK_CHECK_REQ);
        DEBUG_PRINTLN("LinkCheckReq programado para el próximo uplink");
    }
}

void LoRaWAN::processMACCommands(const std::vector<uint8_t> &commands, std::vector<uint8_t> &response)
{
    size_t index = 0;

    while (index < commands.size())
    {
        uint8_t cmd = commands[index++];

        switch (cmd)
        {
        case MAC_LINK_ADR_REQ:
            if (index + 4 <= commands.size())
            {
                processLinkADRReq(commands, index - 1, response);
                index += 4; // Avanzar a después de los 4 bytes de parámetros
            }
            break;

        case MAC_DUTY_CYCLE_REQ:
            if (index < commands.size())
            {
                uint8_t maxDutyCycle = commands[index++];
                float dutyCycle = maxDutyCycle == 0 ? 1.0 : 1.0 / (1 << maxDutyCycle);

                DEBUG_PRINTLN("DutyCycle: MaxDutyCycle=" << static_cast<int>(maxDutyCycle)
                                                         << ", duty cycle=" << (dutyCycle * 100) << "%");

                // Añadir respuesta - siempre aceptamos el duty cycle
                response.push_back(MAC_DUTY_CYCLE_ANS);
            }
            break;

        case MAC_DEV_STATUS_REQ:
            // Responder con estado del dispositivo (batería y margen de señal)
            {
                response.push_back(MAC_DEV_STATUS_ANS);

                // Batería: 0=externo, 1-254=nivel, 255=no medible
                uint8_t battery = 0xFF; // Alimentación externa por defecto

                // Margen de señal: SNR en pasos de 2dB, desde -32dB a +31dB
                int snr = getSNR();
                uint8_t margin = 0;
                if (snr >= -32 && snr <= 31)
                {
                    margin = (snr + 32) / 2;
                }

                response.push_back(battery);
                response.push_back(margin);

                DEBUG_PRINTLN("DevStatus: Battery=" << static_cast<int>(battery)
                                                    << ", Margin=" << static_cast<int>(margin) << " (SNR=" << snr << "dB)");
            }
            break;

        default:
            // Omitir comandos no reconocidos
            DEBUG_PRINTLN("Comando MAC no reconocido: 0x" << std::hex << static_cast<int>(cmd));
            break;
        }
    }
}

void LoRaWAN::processLinkADRReq(const std::vector<uint8_t> &cmd, size_t index, std::vector<uint8_t> &response)
{
    if (index + 4 >= cmd.size())
        return;

    // Interpretar comando ADR
    uint8_t datarate_txpower = cmd[index + 1];
    uint8_t dr = (datarate_txpower >> 4) & 0x0F;
    uint8_t txpower = datarate_txpower & 0x0F;
    uint16_t chmask = (cmd[index + 3] << 8) | cmd[index + 2];
    uint8_t redundancy = cmd[index + 4];
    uint8_t chmaskcntl = (redundancy >> 4) & 0x07;
    uint8_t nbRep = redundancy & 0x0F;

    DEBUG_PRINTLN("LinkADRReq: DR=" << static_cast<int>(dr)
                                    << ", TXPower=" << static_cast<int>(txpower)
                                    << ", ChMask=0x" << std::hex << chmask << std::dec
                                    << ", ChMaskCntl=" << static_cast<int>(chmaskcntl)
                                    << ", NbRep=" << static_cast<int>(nbRep));

    // Construir respuesta
    uint8_t status = 0b111; // Bits: Channel mask OK, Data rate OK, Power OK

    // Verificar validez del data rate según la región
    int maxDR = 5; // Por defecto para la mayoría de regiones
    switch (lora_region)
    {
    case REGION_US915:
    case REGION_AU915:
        maxDR = 4;
        break;
    case REGION_EU868:
    case REGION_EU433:
        maxDR = 7; // DR0-7 para EU868 (SF12-SF7 + modulation variants)
        break;
        // Otras regiones según sea necesario
    }

    if (dr > maxDR)
    {
        status &= ~0x04; // DR no aceptado
        DEBUG_PRINTLN("DR " << static_cast<int>(dr) << " no válido para esta región, max=" << maxDR);
    }

    // Verificar validez de la potencia según la región
    int maxPower = 7; // Valor por defecto para EU868
    switch (lora_region)
    {
    case REGION_US915:
        maxPower = 10;
        break;
    case REGION_EU868:
        maxPower = 7;
        break;
        // Otras regiones según sea necesario
    }

    if (txpower > maxPower)
    {
        status &= ~0x02; // Power no aceptado
        DEBUG_PRINTLN("TXPower " << static_cast<int>(txpower) << " no válido para esta región, max=" << maxPower);
    }

    // Verificar validez de la máscara de canales según la región
    bool validChannelMask = false;

    // Verificar según ChMaskCntl y región
    switch (lora_region)
    {
    case REGION_EU868:
        // Para EU868 normalmente tenemos 8 canales, ChMaskCntl 0-5
        if (chmaskcntl <= 5)
        {
            validChannelMask = true;

            // Para ChMaskCntl=0, verificar que al menos un canal está activo
            if (chmaskcntl == 0 && chmask == 0)
            {
                validChannelMask = false;
                DEBUG_PRINTLN("Se requiere al menos un canal activo en ChMask");
            }
        }
        break;

    case REGION_US915:
        // US915 tiene un manejo especial con 72 canales
        if (chmaskcntl <= 7)
        {
            validChannelMask = true;

            // Verificar reglas específicas para US915
            if (chmaskcntl == 7 && chmask == 0)
            {
                validChannelMask = false;
                DEBUG_PRINTLN("Canal inválido para US915 con ChMaskCntl=7");
            }
        }
        break;

        // Otras regiones según sea necesario
    }

    if (!validChannelMask)
    {
        status &= ~0x01; // Channel mask no aceptada
        DEBUG_PRINTLN("Máscara de canales no válida: ChMaskCntl=" << static_cast<int>(chmaskcntl) << ", ChMask=0x" << std::hex << chmask << std::dec);
    }

    // Si todo está bien, aplicar los cambios
    if (status == 0b111)
    {
        // Aplicar Data Rate (mapear DR a SF según la región)
        int sf;
        float bw = 125.0; // Por defecto

        // Mapeo específico por región
        switch (lora_region)
        {
        case REGION_EU868:
            // Para EU868: DR0=SF12/125kHz, DR1=SF11/125kHz, ..., DR6=SF7/250kHz
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
            { // DR7
                sf = 7;
                bw = 125.0;
                // FSK no soportado, usar LoRa equivalente
            }
            break;

        case REGION_US915:
            // Para US915: DR0=SF10/125kHz, DR1=SF9/125kHz, ..., DR3=SF7/125kHz, DR4=SF8/500kHz
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
            // Mapeo genérico si no hay implementación específica
            sf = dr <= 6 ? (12 - dr) : 7;
            bw = dr == 6 ? 250.0 : 125.0;
        }

        // Aplicar potencia TX según la región
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
            // Mapeo genérico
            power = MAX_POWER[lora_region] - (txpower * 2);
        }

        // Limitar a valores seguros
        if (sf < 7)
            sf = 7;
        if (sf > 12)
            sf = 12;
        if (power < 2)
            power = 2;
        if (power > MAX_POWER[lora_region])
            power = MAX_POWER[lora_region];

        // Aplicar configuración al radio
        pimpl->rfm->setSpreadingFactor(sf);
        pimpl->rfm->setBandwidth(bw);
        pimpl->rfm->setTxPower(power, true);

        // Aplicar máscara de canales si es válida
        if (validChannelMask)
        {
            // Para EU868 con ChMaskCntl=0, aplicar directamente
            if (lora_region == REGION_EU868 && chmaskcntl == 0)
            {
                for (int i = 0; i < 16; i++)
                {
                    if (i < 8)
                    { // Solo tenemos 8 canales en EU868
                        // Si el bit está activado, habilitamos el canal
                        bool enabled = (chmask & (1 << i)) != 0;
                        if (enabled)
                        {
                            channelFrequencies[i] = BASE_FREQ[lora_region] + i * CHANNEL_STEP[lora_region];
                        }
                        else
                        {
                            channelFrequencies[i] = 0; // Desactivar canal
                        }
                    }
                }
            }

            // Para otras combinaciones de región/ChMaskCntl, implementar según necesidad
        }

        // Aplicar número de repeticiones (nbRep)
        if (nbRep > 0)
        {
            // Almacenar para uso en transmisiones
            current_nbRep = nbRep;
        }

        // Resetear contador ADR ya que recibimos y aplicamos una respuesta
        adrAckCounter = 0;

        DEBUG_PRINTLN("ADR params aplicados: SF=" << sf << ", BW=" << bw << "kHz, TXPower=" << power << "dBm, NbRep=" << static_cast<int>(nbRep));
    }
    else
    {
        DEBUG_PRINTLN("ADR params rechazados, status=" << std::bitset<3>(status));
    }

    // Añadir respuesta
    response.push_back(MAC_LINK_ADR_ANS);
    response.push_back(status);
}

void LoRaWAN::sendADRStatistics()
{
    if (!pendingMACResponses.empty() || !joined)
        return;

    // Enviar información sobre condiciones de señal para ayudar al ADR
    std::vector<uint8_t> data;

    // Recolectar datos estadísticos
    float avgSnr = pimpl->rfm->getSNR();
    int avgRssi = pimpl->rfm->getRSSI();

    // Paquete personalizado con estadísticas (puerto 2)
    data.push_back(0x01); // Tipo de mensaje estadístico

    // Añadir SNR promedio (en un formato personalizado)
    int snrValue = static_cast<int>(avgSnr * 4); // *4 para precisión de 0.25 dB
    data.push_back(snrValue & 0xFF);

    // Añadir RSSI promedio
    data.push_back(static_cast<uint8_t>(avgRssi & 0xFF));

    // Enviar estadísticas
    send(data, 2, false);

    DEBUG_PRINTLN("Enviadas estadísticas ADR: SNR=" << avgSnr << "dB, RSSI=" << avgRssi << "dBm");
}

// Actualizar parámetros TX cuando se excede el ADR_ACK_DELAY sin respuesta
void LoRaWAN::updateTxParamsForADR()
{
    // Obtener SF actual
    int currentSF = pimpl->rfm->getSpreadingFactor();

    // Incrementar SF (reducir DR) para mejorar alcance
    if (currentSF < 12)
    {
        currentSF++;
        pimpl->rfm->setSpreadingFactor(currentSF);
        DEBUG_PRINTLN("ADR: Aumentando SF a " << currentSF << " debido a falta de respuesta");
    }

    // También podríamos aumentar la potencia TX si fuera necesario
    int currentPower = pimpl->rfm->getTxPower();
    if (currentPower < 14)
    {
        pimpl->rfm->setTxPower(currentPower + 2, true);
        DEBUG_PRINTLN("ADR: Aumentando potencia TX a " << (currentPower + 2) << " dBm");
    }

    // Resetear el contador para dar tiempo a la nueva configuración
    adrAckCounter = ADR_ACK_LIMIT;
}
