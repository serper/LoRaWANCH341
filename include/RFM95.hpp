#ifndef RFM95_HPP
#define RFM95_HPP

#include "CH341SPI.hpp"
#include "SPIInterface.hpp"
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <memory>

/**
 * RFM95 LoRa Module Handler
 */
class RFM95
{
public:
    // RFM95 Register Addresses
    static constexpr uint8_t REG_FIFO = 0x00;
    static constexpr uint8_t REG_OP_MODE = 0x01;
    static constexpr uint8_t REG_FRF_MSB = 0x06;
    static constexpr uint8_t REG_FRF_MID = 0x07;
    static constexpr uint8_t REG_FRF_LSB = 0x08;
    static constexpr uint8_t REG_PA_CONFIG = 0x09;
    static constexpr uint8_t REG_PA_RAMP = 0x0A;
    static constexpr uint8_t REG_OCP = 0x0B;
    static constexpr uint8_t REG_LNA = 0x0C;
    static constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    static constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
    static constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
    static constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
    static constexpr uint8_t REG_IRQ_FLAGS_MASK = 0x11;
    static constexpr uint8_t REG_IRQ_FLAGS = 0x12;
    static constexpr uint8_t REG_RX_NB_BYTES = 0x13;
    static constexpr uint8_t REG_PKT_SNR_VALUE = 0x19;
    static constexpr uint8_t REG_PKT_RSSI_VALUE = 0x1A;
    static constexpr uint8_t REG_MODEM_CONFIG_1 = 0x1D;
    static constexpr uint8_t REG_MODEM_CONFIG_2 = 0x1E;
    static constexpr uint8_t REG_PREAMBLE_MSB = 0x20;
    static constexpr uint8_t REG_PREAMBLE_LSB = 0x21;
    static constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
    static constexpr uint8_t REG_MODEM_CONFIG_3 = 0x26;
    static constexpr uint8_t REG_FREQ_ERROR_MSB = 0x28;
    static constexpr uint8_t REG_FREQ_ERROR_MID = 0x29;
    static constexpr uint8_t REG_FREQ_ERROR_LSB = 0x2A;
    static constexpr uint8_t REG_RSSI_WIDEBAND = 0x2C;
    static constexpr uint8_t REG_DETECTION_OPTIMIZE = 0x31;
    static constexpr uint8_t REG_INVERTIQ = 0x33;
    static constexpr uint8_t REG_DETECTION_THRESHOLD = 0x37;
    static constexpr uint8_t REG_SYNC_WORD = 0x39;
    static constexpr uint8_t REG_INVERTIQ2 = 0x3B;
    static constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;
    static constexpr uint8_t REG_DIO_MAPPING_2 = 0x41;
    static constexpr uint8_t REG_VERSION = 0x42;
    static constexpr uint8_t REG_PA_DAC = 0x4D;

    // RFM95 Operation Modes
    static constexpr uint8_t MODE_SLEEP = 0x00;
    static constexpr uint8_t MODE_STDBY = 0x01;
    static constexpr uint8_t MODE_TX = 0x03;
    static constexpr uint8_t MODE_RX_CONTINUOUS = 0x05;
    static constexpr uint8_t MODE_RX_SINGLE = 0x06;

    // PA Config
    static constexpr uint8_t PA_BOOST = 0x80;

    // IRQ Flags
    static constexpr uint8_t IRQ_CAD_DONE_MASK = 0x01;
    static constexpr uint8_t IRQ_CAD_DETECTED_MASK = 0x02;
    static constexpr uint8_t IRQ_RX_TIMEOUT_MASK = 0x04;
    static constexpr uint8_t IRQ_TX_DONE_MASK = 0x08;
    static constexpr uint8_t IRQ_VALID_HEADER_MASK = 0x10;
    static constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20;
    static constexpr uint8_t IRQ_RX_DONE_MASK = 0x40;
    static constexpr uint8_t IRQ_TX_TIMEOUT_MASK = 0x80;

    // DIO Mapping
    static constexpr uint8_t DIO0_RX_DONE = 0x00;
    static constexpr uint8_t DIO0_TX_DONE = 0x40;
    static constexpr uint8_t DIO1_RX_TIMEOUT = 0x00;
    static constexpr uint8_t DIO3_TX_DONE = 0x40; // 01 para DIO3
    static constexpr uint8_t DIO4_RX_DONE = 0x00; // 00 para DIO4
    static constexpr uint8_t DIO_TX_PIN = 0x03;
    static constexpr uint8_t DIO_RX_PIN = 0x04;

    /**
     * Constructor
     *
     * @param device_index Index of CH341 device to use (default: 0)
     */
    RFM95(int device_index = 0);

    /**
     * Constructor
     *
     * @param spi_interface Unique pointer to SPI interface implementation
     */
    RFM95(std::unique_ptr<SPIInterface> spi_interface);

    /**
     * Destructor
     */
    ~RFM95();

    /**
     * Initialize RFM95 module
     *
     * @return true if successful, false otherwise
     */
    bool begin();

    /**
     * Close the connection
     */
    void end();

    /**
     * Set frequency in MHz
     *
     * @param freq_mhz Frequency in MHz
     */
    void setFrequency(float freq_mhz);

    /**
     * Get current frequency in MHz
     *
     * @return Frequency in MHz
     */
    float getFrequency();

    /**
     * Set transmit power level
     *
     * @param level Power level in dBm (2-20 for PA_BOOST, 0-15 for RFO)
     * @param use_pa_boost Use PA_BOOST output pin
     */
    void setTxPower(int level, bool use_pa_boost = true);

    /**
     * Get current transmit power level
     *
     * @return Power level
     */
    int getTxPower();

    /**
     * Set spreading factor (6-12)
     *
     * @param sf Spreading factor
     */
    void setSpreadingFactor(int sf);

    /**
     * Get current spreading factor
     *
     * @return Spreading factor
     */
    int getSpreadingFactor();

    /**
     * Set bandwidth in kHz (7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500)
     *
     * @param bw_khz Bandwidth in kHz
     */
    void setBandwidth(float bw_khz);

    /**
     * Get current bandwidth in kHz
     *
     * @return Bandwidth in kHz
     */
    float getBandwidth();

    /**
     * Set coding rate denominator (5-8, giving rates of 4/5, 4/6, 4/7, 4/8)
     *
     * @param denominator Coding rate denominator
     */
    void setCodingRate(int denominator);

    /**
     * Get current coding rate denominator
     *
     * @return Coding rate denominator
     */
    int getCodingRate();

    /**
     * Set preamble length (6-65535)
     *
     * @param length Preamble length
     */
    void setPreambleLength(int length);

    /**
     * Get current preamble length
     *
     * @return Preamble length
     */
    int getPreambleLength();

    /**
     * Set IQ inversion (used for LoRaWAN downlinks)
     *
     * @param invert True to invert IQ, False for normal operation
     */
    void setInvertIQ(bool invert = false);

    /**
     * Check if IQ inversion is enabled
     *
     * @return True if IQ inversion is enabled
     */
    bool getInvertIQ();

    /**
     * Set sync word
     *
     * @param sync_word Sync word
     */
    void setSyncWord(uint8_t sync_word);

    /**
     * Get current sync word
     *
     * @return Sync word
     */
    uint8_t getSyncWord();

    /**
     * Set LNA gain and boost
     *
     * @param lna_gain LNA gain
     * @param lna_boost LNA boost
     */
    void setLNA(int lna_gain = -1, bool lna_boost = true);

    /**
     * Get current LNA settings
     *
     * @return LNA settings
     */
    uint8_t getLNA();

    /**
     * Enable or disable automatic gain control
     *
     * @param enable True to enable, False to disable
     */
    void setAutoAGC(bool enable);

    /**
     * Check if automatic gain control is enabled
     *
     * @return True if AGC is enabled
     */
    bool getAutoAGC();

    /**
     * Clear all IRQ flags
     */
    void clearIRQFlags();

    /**
     * Get current IRQ flags
     *
     * @return IRQ flags
     */
    uint8_t getIRQFlags();

    /**
     * Clear TX done flag
     */
    void clearIRQFlagTxDone();

    /**
     * Clear RX done flag
     */
    void clearIRQFlagRxDone();

    /**
     * Check if RX done flag is set
     *
     * @return True if RX done
     */
    bool getRxDone();

    /**
     * Check if TX done flag is set
     *
     * @return True if TX done
     */
    bool getTxDone();

    /**
     * Check if RX error flag is set
     *
     * @return True if RX error
     */
    bool getRxError();

    /**
     * Check if valid header flag is set
     *
     * @return True if valid header
     */
    bool getValidHeader();

    /**
     * Check if CAD done flag is set
     *
     * @return True if CAD done
     */
    bool getCADDone();

    /**
     * Check if CAD detected flag is set
     *
     * @return True if CAD detected
     */
    bool getCADDetected();

    /**
     * Check if payload CRC error flag is set
     *
     * @return True if payload CRC error
     */
    bool getPayloadCRCError();

    /**
     * Enable or disable LoRa mode
     *
     * @param enable True to enable, False to disable
     */
    void setLoRaMode(bool enable = true);

    /**
     * Send data packet
     *
     * @param data Data to send (max 255 bytes)
     * @param invert_iq True to send with inverted IQ (LoRaWAN downlinks)
     * @return True if send successful
     */
    bool send(const std::vector<uint8_t> &data, bool invert_iq = false);

    /**
     * Receive data packet
     *
     * @param timeout Maximum time to wait for packet in seconds
     * @param invert_iq True to receive with inverted IQ
     * @return Received data or null vector if timeout or error
     */
    std::vector<uint8_t> receive(float timeout = 5.0, bool invert_iq = false);

    /**
     * Set continuous receive mode
     */
    void setContinuousReceive();

    /**
     * Set standby mode
     */
    void standbyMode();

    /**
     * Set sleep mode
     */
    void sleepMode();

    /**
     * Reset RX pointer
     */
    void resetPtrRx();

    /**
     * Get current RX FIFO address
     *
     * @return RX FIFO address
     */
    uint8_t getFifoRxCurrentAddr();

    /**
     * Get number of received bytes
     *
     * @return Number of bytes
     */
    uint8_t getRxNbBytes();

    /**
     * Read received data packet
     *
     * @return Received data
     */
    std::vector<uint8_t> readPayload();

    /**
     * Get current RSSI in dBm
     *
     * @return RSSI in dBm
     */
    float getRSSI();

    /**
     * Get last packet SNR in dB
     *
     * @return SNR in dB
     */
    float getSNR();

    /**
     * Read a register value
     *
     * @param address Register address
     * @return Register value
     */
    uint8_t readRegister(uint8_t address);

    /**
     * Write a register value
     *
     * @param address Register address
     * @param value Value to write
     */
    void writeRegister(uint8_t address, uint8_t value);

    /**
     * Put the module in continuous receive mode
     */
    void receiveMode();

    /**
     * Configure DIO pins for TX/RX indication
     *
     * @param _dio3 DIO3 mapping
     * @param _dio4 DIO4 mapping
     */
    void setDIOMapping(uint8_t _dio3 = 0x40, uint8_t _dio4 = 0x00);

    /**
     * Calibrate temperature sensor with a reference temperature
     *
     * @param actual_temp Actual temperature measured with external sensor in Celsius
     * @return True if calibration successful
     */
    bool calibrateTemperature(float actual_temp);

    /**
     * Read calibrated temperature
     *
     * @return Temperature in Celsius
     */
    float readTemperature();

    /**
     * Configure beacon mode with automatic periodic transmission
     *
     * @param interval_ms Interval between transmissions in milliseconds
     * @param payload Data to transmit in each beacon
     * @return True if successful
     */
    bool setBeaconMode(int interval_ms, const std::vector<uint8_t> &payload);

    /**
     * Stop beacon mode
     */
    void stopBeaconMode();

    /**
     * Check and print current operating mode
     */
    void checkOperatingMode();

    /**
     * Check and print IRQ flags
     */
    void checkIRQFlags();

    /**
     * Print key registers for debugging
     */
    void printRegisters();

    /**
     * Test basic SPI communication
     *
     * @return True if test successful
     */
    bool testCommunication();

    /**
     * Read VERSION register (0x42) directly
     *
     * @return VERSION register value
     */
    uint8_t readVersionRegister();

private:
    std::unique_ptr<SPIInterface> spi; // Usar la interfaz abstracta
};

#endif // RFM95_HPP