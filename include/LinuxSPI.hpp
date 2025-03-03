#pragma once

#include "SPIInterface.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <map>

/**
 * Implementación de SPIInterface usando spidev de Linux
 */
class LinuxSPI : public SPIInterface {
public:
    LinuxSPI(const std::string& device = "/dev/spidev0.0", 
             uint32_t speed = 1000000,
             uint8_t mode = 0);
    ~LinuxSPI() override;

    // Implementación de SPIInterface
    bool open() override;
    void close() override;
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0) override;
    bool digitalWrite(uint8_t pin, bool value) override;
    bool digitalRead(uint8_t pin) override;
    bool pinMode(uint8_t pin, uint8_t mode) override;
    bool setInterruptCallback(InterruptCallback callback) override;
    bool enableInterrupt(bool enable) override;
    
private:
    std::string device_path;
    uint32_t speed_hz;
    uint8_t spi_mode;
    int fd; // File descriptor para el dispositivo SPI
    
    // Para control de pines GPIO
    std::string gpio_export_path;
    std::string gpio_unexport_path;
    std::map<uint8_t, std::string> gpio_pin_paths;
    
    // Para interrupciones
    InterruptCallback interruptCallback;
    std::atomic<bool> interrupt_running;
    std::thread interrupt_thread;
    int interrupt_pin;
    
    // Métodos privados para manejo de GPIO
    bool exportGPIO(uint8_t pin);
    bool unexportGPIO(uint8_t pin);
    bool setGPIODirection(uint8_t pin, const std::string& direction);
    bool writeGPIOValue(uint8_t pin, bool value);
    bool readGPIOValue(uint8_t pin);
    void interruptThread();
};
