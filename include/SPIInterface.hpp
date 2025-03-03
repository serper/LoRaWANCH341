#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

/**
 * Interfaz abstracta para comunicación SPI
 */
class SPIInterface {
public:
    // Constantes para los modos de pines (usadas en pinMode)
    static constexpr uint8_t INPUT = 0;
    static constexpr uint8_t OUTPUT = 1;
    static constexpr uint8_t INPUT_PULLUP = 2;

    // Constructor virtual
    virtual ~SPIInterface() = default;

    // Métodos básicos para la comunicación SPI
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0) = 0;
    
    // Métodos para control de GPIO (necesarios para RFM95)
    virtual bool digitalWrite(uint8_t pin, bool value) = 0;
    virtual bool digitalRead(uint8_t pin) = 0;
    virtual bool pinMode(uint8_t pin, uint8_t mode) = 0;
    
    // Opcional: para interrupciones
    using InterruptCallback = std::function<void(void)>;
    virtual bool setInterruptCallback(InterruptCallback callback) = 0;
    virtual bool enableInterrupt(bool enable) = 0;
};

/**
 * Factory para crear instancias específicas de SPI
 */
class SPIFactory {
public:
    static std::unique_ptr<SPIInterface> createCH341SPI(int device_index = 0, bool lsb_first = false);
    static std::unique_ptr<SPIInterface> createLinuxSPI(const std::string& device = std::string("/dev/spidev0.0"), 
                                                      uint32_t speed = 1000000,
                                                      uint8_t mode = 0);
};
