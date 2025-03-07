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
#include <fstream>
#include <fcntl.h>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

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
     * @brief Configure interrupt settings for a GPIO pin
     * 
     * @param pin The pin number
     * @param enable True to enable the interrupt, false to disable it
     * @return True if the operation was successful, false otherwise
     */
    bool configureInterrupt(uint8_t pin, bool enable) override {
        // Basic implementation using sysfs
        // In a real implementation, would use proper GPIO interrupts
        std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(pin);
        
        // Check if GPIO exists
        if (access(gpio_path.c_str(), F_OK) == -1) {
            // Need to export
            std::ofstream export_file("/sys/class/gpio/export");
            if (!export_file.is_open()) {
                return false;
            }
            export_file << pin;
            export_file.close();
        }
        
        // Configure edge for interrupt
        std::string edge_path = gpio_path + "/edge";
        std::ofstream edge_file(edge_path);
        if (!edge_file.is_open()) {
            return false;
        }
        edge_file << (enable ? "both" : "none");
        edge_file.close();
        
        return true;
    }
    
    /**
     * @brief Set interrupt callback
     * 
     * @param callback Function to call when interrupt occurs
     * @return True if successful, false otherwise
     */
    bool setInterruptCallback(InterruptCallback callback) override;
    
    /**
     * @brief Enable or disable interrupts
     * 
     * @param enable True to enable interrupts, false to disable them
     * @return True if successful, false otherwise
     */
    bool enableInterrupt(bool enable) override;
    
    /**
     * @brief Check if the device is active/connected
     * 
     * @return True if the device is active, false otherwise
     */
    bool isActive() const override {
        return fd >= 0;
    }

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
    InterruptCallback interrupt_callback;
    bool interrupt_enabled = false;
    
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




