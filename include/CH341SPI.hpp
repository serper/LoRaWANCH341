#pragma once

#include "SPIInterface.hpp"
#include <libusb.h>
#include <vector>
#include <string>
#include <thread>

/**
 * Implementación de SPIInterface usando CH341
 */
class CH341SPI : public SPIInterface {
public:
    // Constructor específico para CH341
    CH341SPI(int device_index = 0, bool lsb_first = false);
    ~CH341SPI() override;

    // Implementación de SPIInterface
    bool open() override;
    void close() override;
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0) override;
    bool digitalWrite(uint8_t pin, bool value) override;
    bool digitalRead(uint8_t pin) override;
    bool pinMode(uint8_t pin, uint8_t mode) override;
    bool setInterruptCallback(InterruptCallback callback) override;
    bool enableInterrupt(bool enable) override;
    
    // Métodos específicos de CH341
    // Constantes para los pines del CH341
    static const uint8_t PIN_D0 = 0x01;
    static const uint8_t PIN_D1 = 0x02;
    static const uint8_t PIN_D2 = 0x04;
    static const uint8_t PIN_D3 = 0x08;
    static const uint8_t PIN_D4 = 0x10;
    static const uint8_t PIN_D5 = 0x20;
    
private:
    // Atributos específicos de CH341
    libusb_device_handle *device;
    libusb_context *context;
    int device_index;
    bool lsb_first;
    uint8_t _gpio_direction;
    uint8_t _gpio_output;
    InterruptCallback interruptCallback;
    bool interruptEnabled;
    std::thread interruptThread;
    bool threadRunning;
    
    // Métodos privados
    bool configStream();
    bool enablePins(bool enable);
    uint8_t swapBits(uint8_t byte);
    void interruptMonitoringThread();
};