/**
 * @file RFM95.cpp
 * @brief Implementation of the RFM95 LoRa module interface.
 * 
 * This file contains the implementation of the RFM95 class, which provides an interface
 * to interact with the RFM95 LoRa module using an SPI interface. The class supports
 * various functionalities such as setting frequency, transmit power, spreading factor,
 * bandwidth, coding rate, preamble length, and more.
 * 
 * The class also provides methods for sending and receiving data, configuring interrupt
 * flags, and reading various status registers.
 * 
 * @note This implementation assumes the use of a specific SPI interface, which is
 * provided as a unique pointer to an SPIInterface object.
 * 
 * @author Sergio Pérez
 * @date 2025
 */

#include "RFM95.hpp"
#include "SPIInterface.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <algorithm>

RFM95::RFM95(std::unique_ptr<SPIInterface> spi_interface)
    : spi(std::move(spi_interface))
{
    // ...initialization code...
}

RFM95::RFM95(int device_index)
    : spi(SPIFactory::createCH341SPI(device_index))
{
    // ...initialization code...
}

RFM95::~RFM95()
{
    end();
}

bool RFM95::begin()
{
    if (!spi->open())
    {
        return false;
    }

    // Reset device to initial state
    sleepMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Read VERSION register
    uint8_t version = readVersionRegister();

    if (version != 0x12)
    {
        return false;
    }

    // Set sleep mode and LoRa mode
    writeRegister(REG_OP_MODE, 0x80); // LoRa mode
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // Set modem config
    writeRegister(REG_MODEM_CONFIG_1, 0x72);
    writeRegister(REG_MODEM_CONFIG_2, 0x70);
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // Set transmit power
    writeRegister(REG_PA_CONFIG, 0x8F);
    writeRegister(REG_PA_DAC, 0x87);

    // Set LNA
    writeRegister(REG_LNA, 0x23);

    // Set up FIFO
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // Go to standby
    standbyMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

void RFM95::end()
{
    spi->close();
}

void RFM95::setFrequency(float freq_mhz)
{
    uint32_t frf = static_cast<uint32_t>((freq_mhz * 524288.0) / 32.0);

    // Write the three bytes
    writeRegister(REG_FRF_MSB, (frf >> 16) & 0xFF);
    writeRegister(REG_FRF_MID, (frf >> 8) & 0xFF);
    writeRegister(REG_FRF_LSB, frf & 0xFF);
}

float RFM95::getFrequency()
{
    // Read the three bytes from the registers
    uint32_t msb = static_cast<uint32_t>(readRegister(REG_FRF_MSB));
    uint32_t mid = static_cast<uint32_t>(readRegister(REG_FRF_MID));
    uint32_t lsb = static_cast<uint32_t>(readRegister(REG_FRF_LSB));

    // Combine the bytes to form the FRF value
    uint32_t frf = (msb << 16) | (mid << 8) | lsb;

    // Calculate the frequency using the formula from the datasheet
    float freq_mhz = (frf * 32.0) / 524288.0;

    return freq_mhz;
}

void RFM95::setTxPower(int level, bool use_pa_boost)
{
    if (use_pa_boost)
    {
        level = std::max(2, std::min(level, 20));
        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
    else
    {
        level = std::max(0, std::min(level, 15));
        writeRegister(REG_PA_CONFIG, level);
    }
}

int RFM95::getTxPower()
{
    uint8_t pa = readRegister(REG_PA_CONFIG);
    if (pa & PA_BOOST)
    {
        return (pa & 0x0F) + 2;
    }
    return pa & 0x0F;
}

void RFM95::setSpreadingFactor(int sf)
{
    sf = std::max(6, std::min(sf, 12));

    if (sf == 6)
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xC5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
    }
    else
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xC3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    }

    uint8_t reg2 = readRegister(REG_MODEM_CONFIG_2);
    reg2 = (reg2 & 0x0F) | ((sf << 4) & 0xF0);
    writeRegister(REG_MODEM_CONFIG_2, reg2);
}

int RFM95::getSpreadingFactor()
{
    uint8_t reg2 = readRegister(REG_MODEM_CONFIG_2);
    return (reg2 >> 4) & 0x0F;
}

void RFM95::setBandwidth(float bw_khz)
{
    const float bws[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
    uint8_t bw_value = 9; // Default to 500kHz

    for (int i = 0; i < 10; i++)
    {
        if (bw_khz <= bws[i])
        {
            bw_value = i;
            break;
        }
    }

    uint8_t reg1 = readRegister(REG_MODEM_CONFIG_1);
    reg1 = (reg1 & 0x0F) | (bw_value << 4);
    writeRegister(REG_MODEM_CONFIG_1, reg1);
}

float RFM95::getBandwidth()
{
    const float bws[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
    uint8_t reg1 = readRegister(REG_MODEM_CONFIG_1);
    uint8_t bw_value = (reg1 >> 4) & 0x0F;
    return bws[bw_value < 10 ? bw_value : 9];
}

void RFM95::setCodingRate(int denominator)
{
    denominator = std::max(5, std::min(denominator, 8));

    uint8_t cr = denominator - 4;
    uint8_t reg1 = readRegister(REG_MODEM_CONFIG_1);
    reg1 = (reg1 & 0xF1) | (cr << 1);
    writeRegister(REG_MODEM_CONFIG_1, reg1);
}

int RFM95::getCodingRate()
{
    uint8_t reg1 = readRegister(REG_MODEM_CONFIG_1);
    uint8_t cr = (reg1 >> 1) & 0x07;
    return cr + 4;
}

void RFM95::setPreambleLength(int length)
{
    writeRegister(REG_PREAMBLE_MSB, (length >> 8) & 0xFF);
    writeRegister(REG_PREAMBLE_LSB, length & 0xFF);
}

int RFM95::getPreambleLength()
{
    uint16_t msb = readRegister(REG_PREAMBLE_MSB);
    uint16_t lsb = readRegister(REG_PREAMBLE_LSB);
    return (msb << 8) | lsb;
}

void RFM95::setInvertIQ(bool invert)
{
    if (invert)
    {
        // Setup for inverted IQ
        writeRegister(REG_INVERTIQ, 0x66);
        writeRegister(REG_INVERTIQ2, 0x19);
    }
    else
    {
        // Setup for normal IQ
        writeRegister(REG_INVERTIQ, 0x27);
        writeRegister(REG_INVERTIQ2, 0x1D);
    }
}

bool RFM95::getInvertIQ()
{
    return (readRegister(REG_INVERTIQ) & 0x40) != 0;
}

void RFM95::setSyncWord(uint8_t sync_word)
{
    writeRegister(REG_SYNC_WORD, sync_word);
}

uint8_t RFM95::getSyncWord()
{
    return readRegister(REG_SYNC_WORD);
}

void RFM95::setLNA(int lna_gain, bool lna_boost)
{
    if (lna_gain >= 0)
    {
        setAutoAGC(false);
    }
    else
    {
        setAutoAGC(true);
    }

    if (lna_boost)
    {
        writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
    }
    else
    {
        writeRegister(REG_LNA, readRegister(REG_LNA) & 0xFC);
    }
}

uint8_t RFM95::getLNA()
{
    return readRegister(REG_LNA);
}

void RFM95::setAutoAGC(bool enable)
{
    if (enable)
    {
        writeRegister(REG_MODEM_CONFIG_3, readRegister(REG_MODEM_CONFIG_3) | 0x04);
    }
    else
    {
        writeRegister(REG_MODEM_CONFIG_3, readRegister(REG_MODEM_CONFIG_3) & ~0x04);
    }
}

bool RFM95::getAutoAGC()
{
    return (readRegister(REG_MODEM_CONFIG_3) & 0x04) != 0;
}

void RFM95::clearIRQFlags()
{
    writeRegister(REG_IRQ_FLAGS, 0xFF);
}

uint8_t RFM95::getIRQFlags()
{
    return readRegister(REG_IRQ_FLAGS);
}

void RFM95::clearIRQFlagTxDone()
{
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

void RFM95::clearIRQFlagRxDone()
{
    writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
}

bool RFM95::getRxDone()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) != 0;
}

bool RFM95::getTxDone()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) != 0;
}

bool RFM95::getRxError()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0;
}

bool RFM95::getValidHeader()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_VALID_HEADER_MASK) != 0;
}

bool RFM95::getCADDone()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DONE_MASK) != 0;
}

bool RFM95::getCADDetected()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_CAD_DETECTED_MASK) != 0;
}

bool RFM95::getPayloadCRCError()
{
    return (readRegister(REG_IRQ_FLAGS) & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0;
}

void RFM95::setLoRaMode(bool enable)
{
    uint8_t mode = readRegister(REG_OP_MODE);
    if (enable)
    {
        mode = (mode & 0x7F) | 0x80; // Set bit 7 for LoRa mode
    }
    else
    {
        mode = mode & 0x7F; // Clear bit 7 for FSK mode
    }
    writeRegister(REG_OP_MODE, mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Wait for mode change
}

bool RFM95::send(const std::vector<uint8_t> &data, bool invert_iq)
{
    if (data.size() > 255)
    {
        return false;
    }

    // Configure IQ mode
    setInvertIQ(invert_iq);

    setDIOMapping(0x40, 0x40); // DIO3=01, DIO4=01 (TxDone)

    // Enter standby mode
    standbyMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Configure DIO3 for TxDone
    uint8_t current = readRegister(REG_DIO_MAPPING_1);
    writeRegister(REG_DIO_MAPPING_1, (current & 0x3F) | 0x40); // 01 for DIO3

    // Clear IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    // Write data
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    for (uint8_t byte : data)
    {
        writeRegister(REG_FIFO, byte);
    }
    writeRegister(REG_PAYLOAD_LENGTH, data.size());

    // Start TX
    writeRegister(REG_OP_MODE, MODE_TX);

    // Wait for TX done
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        uint8_t flags = readRegister(REG_IRQ_FLAGS);
        if (flags & IRQ_TX_DONE_MASK)
        {
            writeRegister(REG_IRQ_FLAGS, 0xFF); // Clear flags
            standbyMode();
            // Restore normal IQ mode if it was inverted
            if (invert_iq)
            {
                setInvertIQ(false);
            }
            return true;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 2000)
        {
            // Timeout after 2 seconds
            // Restore normal IQ mode if it was inverted
            if (invert_iq)
            {
                setInvertIQ(false);
            }
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

std::vector<uint8_t> RFM95::receive(float timeout, bool invert_iq)
{
    // Configure IQ mode
    setInvertIQ(invert_iq);

    // Enter receive mode
    writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);
    setDIOMapping(0x40, 0xC0); // DIO3=01, DIO4=11

    // Clear IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);


    // Wait for RX done or timeout
    auto start_time = std::chrono::steady_clock::now();
    while (true)
    {
        uint8_t irq_flags = readRegister(REG_IRQ_FLAGS);

        if (irq_flags != 0)
        {

            // Decode flags
            if (irq_flags & IRQ_RX_DONE_MASK)
            {

                // Check for CRC error
                if (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)
                {
                    writeRegister(REG_IRQ_FLAGS, 0xFF); // Clear flags
                    continue;                           // Try again
                }

                // Read packet
                uint8_t length = readRegister(REG_RX_NB_BYTES);

                if (length > 0)
                {
                    uint8_t current_addr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
                    writeRegister(REG_FIFO_ADDR_PTR, current_addr);

                    std::vector<uint8_t> data;
                    data.reserve(length);
                    for (int i = 0; i < length; i++)
                    {
                        data.push_back(readRegister(REG_FIFO));
                    }

                    writeRegister(REG_IRQ_FLAGS, 0xFF); // Clear flags

                    // Restore normal IQ mode if it was inverted
                    if (invert_iq)
                    {
                        setInvertIQ(false);
                    }

                    return data;
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout * 1000)
        {
            writeRegister(REG_IRQ_FLAGS, 0xFF); // Clear flags

            // Restore normal IQ mode if it was inverted
            if (invert_iq)
            {
                setInvertIQ(false);
            }

            return std::vector<uint8_t>(); // Return empty vector on timeout
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void RFM95::setContinuousReceive()
{
    // Put the module in standby mode first
    standbyMode();
    
    // Configure FIFO RX
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_ADDR));
    
    // Configure DIO for reception
    uint8_t dio_mapping = readRegister(REG_DIO_MAPPING_1);
    dio_mapping &= 0x3F;  // Clear DIO0 bits (bits 6-7)
    dio_mapping |= DIO0_RX_DONE;  // DIO0 = 0 (RX_DONE)
    writeRegister(REG_DIO_MAPPING_1, dio_mapping);
    
    // Clear interrupt flags
    clearIRQFlags();
    
    // Change to RX_CONTINUOUS mode
    uint8_t opmode = readRegister(REG_OP_MODE);
    opmode = (opmode & 0xF8) | MODE_RX_CONTINUOUS;  // Modes: 1=STDBY, 5=RXCONT, 3=TX
    writeRegister(REG_OP_MODE, opmode);
    
    // Debug: verify that the mode was changed correctly
    opmode = readRegister(REG_OP_MODE);
    if ((opmode & 0x07) != MODE_RX_CONTINUOUS) {
        std::cerr << "Error: Could not change to RX_CONTINUOUS mode" << std::endl;
    }
}

void RFM95::standbyMode()
{
    writeRegister(REG_OP_MODE, MODE_STDBY);
}

void RFM95::sleepMode()
{
    uint8_t current = readRegister(REG_OP_MODE);
    writeRegister(REG_OP_MODE, (current & 0xF8) | MODE_SLEEP);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void RFM95::resetPtrRx()
{
    writeRegister(REG_FIFO_ADDR_PTR, 0);
}

uint8_t RFM95::getFifoRxCurrentAddr()
{
    return readRegister(REG_FIFO_RX_CURRENT_ADDR);
}

uint8_t RFM95::getRxNbBytes()
{
    return readRegister(REG_RX_NB_BYTES);
}

std::vector<uint8_t> RFM95::readPayload()
{
    uint8_t length = readRegister(REG_RX_NB_BYTES);
    if (length > 0)
    {
        uint8_t current_addr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        writeRegister(REG_FIFO_ADDR_PTR, current_addr);

        std::vector<uint8_t> data;
        data.reserve(length);
        for (int i = 0; i < length; i++)
        {
            data.push_back(readRegister(REG_FIFO));
        }

        return data;
    }
    return std::vector<uint8_t>();
}

float RFM95::getRSSI()
{
    return -137 + readRegister(REG_PKT_RSSI_VALUE);
}

float RFM95::getSNR()
{
    int8_t snr = static_cast<int8_t>(readRegister(REG_PKT_SNR_VALUE));
    return snr * 0.25f;
}

uint8_t RFM95::readRegister(uint8_t address)
{
    std::vector<uint8_t> cmd = {static_cast<uint8_t>(address & 0x7F)};
    std::vector<uint8_t> response = spi->transfer(cmd, 1);
    if (!response.empty())
    {
        return response[0];
    }
    return 0;
}

void RFM95::writeRegister(uint8_t address, uint8_t value)
{
    std::vector<uint8_t> cmd = {static_cast<uint8_t>(address | 0x80), value};
    spi->transfer(cmd, 0);
}

void RFM95::receiveMode()
{
    // Clear FIFO
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // Reset IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    // Enable LNA boost
    writeRegister(REG_LNA, 0x23);

    // Set detection optimize and threshold for SF7-12
    writeRegister(REG_DETECTION_OPTIMIZE, 0xC3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0A);

    // Enter RX continuous mode
    writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);
}

void RFM95::setDIOMapping(uint8_t _dio3, uint8_t _dio4)
{
    // RegDioMapping1 (0x40)
    uint8_t dio_map1 = readRegister(REG_DIO_MAPPING_1);
    dio_map1 = (dio_map1 & 0x3F) | (_dio3 & 0xC0);
    writeRegister(REG_DIO_MAPPING_1, dio_map1);

    // RegDioMapping2 (0x41)
    uint8_t dio_map2 = readRegister(REG_DIO_MAPPING_2);
    dio_map2 = (dio_map2 & 0x3F) | (_dio4 & 0xC0);
    writeRegister(REG_DIO_MAPPING_2, dio_map2);

    // Enable interrupts
    writeRegister(REG_IRQ_FLAGS_MASK, 0x00); // IRQ mask
    writeRegister(REG_IRQ_FLAGS, 0xFF);      // Clear flags
}

bool RFM95::calibrateTemperature(float actual_temp)
{
    try
    {
        // Save current mode
        uint8_t current_mode = readRegister(REG_OP_MODE);

        // Set FSK mode
        writeRegister(REG_OP_MODE, 0x00); // FSK + sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Store calibration value
        uint8_t cal_value = static_cast<uint8_t>(actual_temp);
        writeRegister(0x3B, cal_value);
                //   << static_cast<int>(cal_value) << std::dec << std::endl;

        // Restore mode
        writeRegister(REG_OP_MODE, current_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error in temperature calibration: " << e.what() << std::endl;
        return false;
    }
}

float RFM95::readTemperature()
{
    try
    {
        // Save current mode
        uint8_t current_mode = readRegister(REG_OP_MODE);

        // Set FSK mode
        writeRegister(REG_OP_MODE, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read calibration value
        uint8_t cal_value = readRegister(0x3B);

        // Read current value
        uint8_t raw_temp = readRegister(0x3C);

        // Calculate actual temperature using calibration
        float temp = raw_temp + cal_value;

        // Restore mode
        writeRegister(REG_OP_MODE, current_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return temp;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading temperature: " << e.what() << std::endl;
        return 0.0f;
    }
}

bool RFM95::setBeaconMode(int interval_ms, const std::vector<uint8_t> &payload)
{
    if (payload.size() > 255)
    {
        return false;
    }

    setDIOMapping(0x40, 0x40); // DIO3=01, DIO4=01

    // Configure FIFO
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // Write payload
    for (uint8_t byte : payload)
    {
        writeRegister(REG_FIFO, byte);
    }
    writeRegister(REG_PAYLOAD_LENGTH, payload.size());

    // Set beacon interval
    uint16_t period = interval_ms / 1000;      // Convert to seconds
    writeRegister(0x24, (period >> 8) & 0xFF); // REG_BEACON_PERIOD MSB
    writeRegister(0x25, period & 0xFF);        // REG_BEACON_PERIOD LSB

    // Enable beacon mode
    uint8_t mode = readRegister(REG_OP_MODE);
    mode = (mode & 0x7F) | 0x80; // Set LoRa mode
    mode = (mode & 0xF8) | 0x03; // Set beacon mode
    writeRegister(REG_OP_MODE, mode);

    return true;
}

void RFM95::stopBeaconMode()
{
    standbyMode();
}

void RFM95::checkOperatingMode()
{
    uint8_t mode = readRegister(REG_OP_MODE);
    std::cout << "Operating Mode: 0x" << std::hex << static_cast<int>(mode) << std::dec << std::endl;
    std::cout << "  LoRa Mode: " << ((mode & 0x80) ? "Yes" : "No") << std::endl;
    std::cout << "  Mode: " << static_cast<int>(mode & 0x07) << std::endl; // 0=Sleep, 1=Standby, 2=FSTX, 3=TX, 4=FSRX, 5=RX
}

void RFM95::checkIRQFlags()
{
    uint8_t flags = readRegister(REG_IRQ_FLAGS);
    std::cout << "IRQ Flags: 0x" << std::hex << static_cast<int>(flags) << std::dec << std::endl;
    std::cout << "  RxTimeout: " << ((flags & 0x80) ? "true" : "false") << std::endl;
    std::cout << "  RxDone: " << ((flags & 0x40) ? "true" : "false") << std::endl;
    std::cout << "  PayloadCrcError: " << ((flags & 0x20) ? "true" : "false") << std::endl;
    std::cout << "  ValidHeader: " << ((flags & 0x10) ? "true" : "false") << std::endl;
    std::cout << "  TxDone: " << ((flags & 0x08) ? "true" : "false") << std::endl;
    std::cout << "  CadDone: " << ((flags & 0x04) ? "true" : "false") << std::endl;
    std::cout << "  FhssChangeChannel: " << ((flags & 0x02) ? "true" : "false") << std::endl;
    std::cout << "  CadDetected: " << ((flags & 0x01) ? "true" : "false") << std::endl;
}

void RFM95::printRegisters()
{
    std::cout << "\nRegister values:" << std::endl;
    std::cout << "OP_MODE: 0x" << std::hex << static_cast<int>(readRegister(REG_OP_MODE)) << std::dec << std::endl;
    std::cout << "IRQ_FLAGS: 0x" << std::hex << static_cast<int>(readRegister(REG_IRQ_FLAGS)) << std::dec << std::endl;
    std::cout << "MODEM_CONFIG_1: 0x" << std::hex << static_cast<int>(readRegister(REG_MODEM_CONFIG_1)) << std::dec << std::endl;
    std::cout << "MODEM_CONFIG_2: 0x" << std::hex << static_cast<int>(readRegister(REG_MODEM_CONFIG_2)) << std::dec << std::endl;
    std::cout << "MODEM_CONFIG_3: 0x" << std::hex << static_cast<int>(readRegister(REG_MODEM_CONFIG_3)) << std::dec << std::endl;
    std::cout << "PA_CONFIG: 0x" << std::hex << static_cast<int>(readRegister(REG_PA_CONFIG)) << std::dec << std::endl;
}

bool RFM95::testCommunication()
{
    // Write a test value to sync word register
    uint8_t test_value = 0x42;
    writeRegister(REG_SYNC_WORD, test_value);

    // Read it back
    uint8_t read_value = readRegister(REG_SYNC_WORD);

    return test_value == read_value;
}

uint8_t RFM95::readVersionRegister()
{
    try
    {
        std::vector<uint8_t> response = spi->transfer({0x42}, 1);
        if (!response.empty())
        {
            return response[0];
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading VERSION register: " << e.what() << std::endl;
        return 0;
    }
}