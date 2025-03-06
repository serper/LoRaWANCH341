/***
 * Abstract interface for SPI communication
 * 
 * @file SPIInterface.hpp
 * @brief Header file for the SPIInterface class
 */
#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

/**
 * @brief   Abstract interface for SPI communication
 * 
 * This class defines an abstract interface for SPI communication, including basic
 * methods for managing SPI communication and GPIO controls.
 */
class SPIInterface {
public:
    /***
     * Constants for GPIO pin modes
     */
    static constexpr uint8_t INPUT = 0;
    static constexpr uint8_t OUTPUT = 1;
    static constexpr uint8_t INPUT_PULLUP = 2;

    /***
     * Destructor for SPIInterface, ensures proper cleanup of resources.
     */
    virtual ~SPIInterface() = default;

    /***
     * Opens the SPI connection.
     * @return True if the connection was successfully opened, false otherwise.
     */
    virtual bool open() = 0;
    
    /***
     * Closes the SPI connection.
     * Ensures that the resources are properly released.
     */
    virtual void close() = 0;
    
    /***
     * Transfers data over SPI.
     * @param write_data The data to write to the SPI device.
     * @param read_length The number of bytes to read from the SPI device.
     * @return A vector containing the data read from the SPI device.
     */
    virtual std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0) = 0;
    
    /***
     * Writes a digital value to a specified pin.
     * @param pin The pin number.
     * @param value The value to write (true for high, false for low).
     * @return True if the operation was successful, false otherwise.
     */
    virtual bool digitalWrite(uint8_t pin, bool value) = 0;
    
    /***
     * Reads a digital value from a specified pin.
     * @param pin The pin number.
     * @return The digital value read from the pin (true for high, false for low).
     */
    virtual bool digitalRead(uint8_t pin) = 0;

    /***
     * Sets the mode of a specified pin.
     * @param pin The pin number.
     * @param mode The mode to set (e.g., input, output).
     * @return True if the operation was successful, false otherwise.
     */
    virtual bool pinMode(uint8_t pin, uint8_t mode) = 0;
    
    /***
     * Configures the interrupt settings for a pin.
     * @param pin The pin number.
     * @param enable True to enable the interrupt, false to disable it.
     * @return True if the operation was successful, false otherwise.
     */
    virtual bool configureInterrupt(uint8_t pin, bool enable) = 0;
    
    /***
     * Callback function for handling interrupts.
     */
    using InterruptCallback = std::function<void(void)>;
    virtual bool setInterruptCallback(InterruptCallback callback) = 0;

    /***
     * Enables or disables interrupts.
     * @param enable True to enable interrupts, false to disable them.
     * @return True if the operation was successful, false otherwise.
     */
    virtual bool enableInterrupt(bool enable) = 0;
    
    /***
     * Checks if the SPI device is currently active.
     * @return True if the device is active, false otherwise.
     */
    virtual bool isActive() const = 0;
};

/**
 * @brief Factory class for creating SPI interfaces
 * 
 * This class provides factory methods for creating SPI interfaces, allowing
 * the user to select the desired SPI implementation.
 */
class SPIFactory {
public:
    static std::unique_ptr<SPIInterface> createCH341SPI(int device_index = 0, bool lsb_first = false);
    static std::unique_ptr<SPIInterface> createLinuxSPI(const std::string& device = std::string("/dev/spidev0.0"), 
                                                      uint32_t speed = 1000000,
                                                      uint8_t mode = 0);
    /***
     * Creates an SPI interface for a specific device type.
     * @param device_type The type of SPI device to create (e.g., "CH341", "Linux").
     * @param device_index Optional parameter for device index.
     * @param lsb_first Optional parameter to set LSB first mode for CH341 devices.
     * @return A unique pointer to an SPIInterface, or nullptr if the type is unsupported.
     */
    static std::unique_ptr<SPIInterface> createSPIInterface(const std::string& device_type, int device_index = 0, bool lsb_first = false);
    
    /***
     * Releases any resources used by the factory.
     */
    static void cleanupResources();
    
    /***
     * Destructor for SPIFactory, ensures proper cleanup of resources.
     */
    ~SPIFactory() = default;
};
