/**
 * @brief Implementation of SPIInterface using Linux spidev
 * 
 * This class provides an implementation of the SPIInterface using the spidev
 * interface available in Linux. It allows communication with SPI devices and
 * control of GPIO pins for interrupt handling.
 * 
 * @note This class is designed to work on Linux systems with spidev support.
 * 
 * @param device The SPI device path (default: "/dev/spidev0.0").
 * @param speed The SPI communication speed in Hz (default: 1,000,000 Hz).
 * @param mode The SPI mode (default: 0).
 * 
 * @author Sergio Pérez
 * @date 2025
 */
#pragma once

#include "SPIInterface.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <map>

/**
 * @class LinuxSPI
 * @brief A class to interface with SPI devices on Linux systems.
 * 
 * This class provides methods to open, close, and transfer data over an SPI
 * interface, as well as control GPIO pins and handle interrupts.
 */
class LinuxSPI : public SPIInterface {
    public:
    /**
     * @brief Constructs a new LinuxSPI object.
     * 
     * @param device The SPI device path (default is "/dev/spidev0.0").
     * @param speed The SPI communication speed in Hz (default is 1,000,000 Hz).
     * @param mode The SPI mode (default is 0).
     */
    LinuxSPI(const std::string& device = "/dev/spidev0.0", 
        uint32_t speed = 1000000,
        uint8_t mode = 0);

    /**
     * @brief Destroys the LinuxSPI object.
     */
    ~LinuxSPI();
        
    /**
     * @brief Opens the SPI device.
     * 
     * @return true if the device was successfully opened, false otherwise.
     */
    bool open();
    
    /**
     * @brief Closes the SPI device.
     */
    void close();
    
    /**
     * @brief Transfers data over the SPI interface.
     * 
     * @param write_data The data to be written to the SPI device.
     * @param read_length The number of bytes to read from the SPI device (default is 0).
     * @return A vector containing the data read from the SPI device.
     */
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0);
    
    /**
     * @brief Sets the value of a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @param value The value to set (true for high, false for low).
     * @return true if the value was successfully set, false otherwise.
     */
    bool digitalWrite(uint8_t pin, bool value);
    
    /**
     * @brief Reads the value of a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @return The value of the GPIO pin (true for high, false for low).
     */
    bool digitalRead(uint8_t pin);
    
    /**
     * @brief Sets the mode of a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @param mode The mode to set (e.g., input or output).
     * @return true if the mode was successfully set, false otherwise.
     */
    bool pinMode(uint8_t pin, uint8_t mode);
    
    /**
     * @brief Sets the callback function for GPIO interrupts.
     * 
     * @param callback The callback function to be called on an interrupt.
     * @return true if the callback was successfully set, false otherwise.
     */
    bool setInterruptCallback(InterruptCallback callback);
    
    /**
     * @brief Enables or disables GPIO interrupts.
     * 
     * @param enable true to enable interrupts, false to disable.
     * @return true if the operation was successful, false otherwise.
     */
    bool enableInterrupt(bool enable);
    
private:
std::string device_path;
    uint32_t speed_hz;
    uint8_t spi_mode;
    int fd; // File descriptor para el dispositivo SPI
    
    // GPIO paths
    std::string gpio_export_path;
    std::string gpio_unexport_path;
    std::map<uint8_t, std::string> gpio_pin_paths;
    
    // Interrupt handling
    InterruptCallback interruptCallback;
    std::atomic<bool> interrupt_running;
    std::thread interrupt_thread;
    int interrupt_pin;
    
    /**
     * @brief Exports a GPIO pin for use.
     * 
     * @param pin The GPIO pin number.
     * @return true if the pin was successfully exported, false otherwise.
     */
    bool exportGPIO(uint8_t pin);

    /**
     * @brief Unexports a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @return true if the pin was successfully unexported, false otherwise.
     */
    bool unexportGPIO(uint8_t pin);

    /**
     * @brief Sets the direction of a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @param direction The direction to set ("in" for input, "out" for output).
     * @return true if the direction was successfully set, false otherwise.
     */
    bool setGPIODirection(uint8_t pin, const std::string& direction);

    /**
     * @brief Writes a value to a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @param value The value to write (true for high, false for low).
     * @return true if the value was successfully written, false otherwise.
     */
    bool writeGPIOValue(uint8_t pin, bool value);

    /**
     * @brief Reads the value of a GPIO pin.
     * 
     * @param pin The GPIO pin number.
     * @return The value of the GPIO pin (true for high, false for low).
     */
    bool readGPIOValue(uint8_t pin);

    void interruptThread();
};




