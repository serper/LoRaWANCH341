/**
 * @file CH341SPI.hpp
 * @brief Header file for the CH341SPI class, which implements the SPIInterface for the CH341 USB to SPI adapter.
 * 
 * @author Sergio Pérez
 * @date 2025
 * 
 * @license MIT License
 * Copyright (c) 2025 Sergio Pérez
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include "SPIInterface.hpp"
#include <libusb.h>
#include <vector>
#include <string>
#include <thread>

/**
 * @class CH341SPI
 * @brief A class to interface with the CH341 USB to SPI adapter.
 */
class CH341SPI : public SPIInterface {
public:
    /**
     * @brief Constructor for CH341SPI.
     * @param device_index Index of the CH341 device to use (default is 0).
     * @param lsb_first Set to true if least significant bit should be sent first (default is false).
     */
    CH341SPI(int device_index = 0, bool lsb_first = false);

    /**
     * @brief Destructor for CH341SPI.
     */
    ~CH341SPI();

    /**
     * @brief Opens the SPI connection.
     * @return True if the connection was successfully opened, false otherwise.
     */
    bool open();

    /**
     * @brief Closes the SPI connection.
     */
    void close();

    /**
     * @brief Transfers data over SPI.
     * @param write_data Data to be written to the SPI bus.
     * @param read_length Number of bytes to read from the SPI bus (default is 0).
     * @return A vector containing the data read from the SPI bus.
     */
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& write_data, size_t read_length = 0);

    /**
     * @brief Writes a digital value to a specified pin.
     * @param pin The pin number.
     * @param value The value to write (true for high, false for low).
     * @return True if the operation was successful, false otherwise.
     */
    bool digitalWrite(uint8_t pin, bool value);

    /**
     * @brief Reads a digital value from a specified pin.
     * @param pin The pin number.
     * @return The value read from the pin (true for high, false for low).
     */
    bool digitalRead(uint8_t pin);

    /**
     * @brief Sets the mode of a specified pin.
     * @param pin The pin number.
     * @param mode The mode to set (e.g., input, output).
     * @return True if the operation was successful, false otherwise.
     */
    bool pinMode(uint8_t pin, uint8_t mode);

    /**
     * @brief Configure interrupt settings for a pin
     * 
     * @param pin The pin number
     * @param enable True to enable the interrupt, false to disable it
     * @return True if the operation was successful, false otherwise
     */
    bool configureInterrupt(uint8_t pin, bool enable) override {
        // CH341 doesn't support native interrupts, would need external handling
        return false;
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
        return is_open;
    }

    // Constants for CH341 pins
    static const uint8_t PIN_D0 = 0x01;
    static const uint8_t PIN_D1 = 0x02;
    static const uint8_t PIN_D2 = 0x04;
    static const uint8_t PIN_D3 = 0x08;
    static const uint8_t PIN_D4 = 0x10;
    static const uint8_t PIN_D5 = 0x20;

private:
    libusb_device_handle *device; ///< Handle to the libusb device.
    libusb_context *context; ///< Context for libusb.
    int device_index; ///< Index of the CH341 device.
    bool lsb_first; ///< Flag to indicate if LSB should be sent first.
    bool is_open; ///< Flag to indicate whether the device is open and active
    uint8_t _gpio_direction; ///< Direction of GPIO pins.
    uint8_t _gpio_output; ///< Output state of GPIO pins.
    InterruptCallback interruptCallback; ///< Callback function for interrupts.
    bool interruptEnabled; ///< Flag to indicate if interrupts are enabled.
    std::thread interruptThread; ///< Thread for monitoring interrupts.
    bool threadRunning; ///< Flag to indicate if the interrupt monitoring thread is running.

    /**
     * @brief Configures the SPI stream.
     * @return True if the configuration was successful, false otherwise.
     */
    bool configStream();

    /**
     * @brief Enables or disables the pins.
     * @param enable Set to true to enable pins, false to disable.
     * @return True if the operation was successful, false otherwise.
     */
    bool enablePins(bool enable);

    /**
     * @brief Swaps the bits of a byte.
     * @param byte The byte whose bits are to be swapped.
     * @return The byte with swapped bits.
     */
    uint8_t swapBits(uint8_t byte);

    /**
     * @brief Thread function for monitoring interrupts.
     */
    void interruptMonitoringThread();
};