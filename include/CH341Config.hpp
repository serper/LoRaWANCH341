#ifndef CH341_CONFIG_HPP
#define CH341_CONFIG_HPP

#include <cstdint>

namespace CH341Config
{
    // Device identifiers
    constexpr uint16_t VENDOR_ID = 0x1A86;
    constexpr uint16_t PRODUCT_ID = 0x5512;

    // Endpoints
    constexpr uint8_t BULK_WRITE_EP = 0x02;
    constexpr uint8_t BULK_READ_EP = 0x82;

    // Packet Configuration
    constexpr uint8_t PACKET_LENGTH = 0x20;
    constexpr uint16_t MAX_PACKETS = 256;
    constexpr uint16_t MAX_PACKET_LEN = PACKET_LENGTH * MAX_PACKETS;

    // Pin mapping for CH341F
    constexpr uint8_t PIN_MISO = 0x02;
    constexpr uint8_t PIN_MOSI = 0x04;
    constexpr uint8_t PIN_SCK = 0x08;
    constexpr uint8_t PIN_CS = 0x20;

    // Commands
    constexpr uint8_t CMD_SPI_STREAM = 0xA8;
    constexpr uint8_t CMD_UIO_STREAM = 0xAB;
    constexpr uint8_t CMD_UIO_STM_OUT = 0x80;
    constexpr uint8_t CMD_UIO_STM_DIR = 0x40;
    constexpr uint8_t CMD_UIO_STM_END = 0x20;
    constexpr uint8_t CMD_I2C_STREAM = 0xAA;
    constexpr uint8_t CMD_I2C_STM_SET = 0x60;
    constexpr uint8_t CMD_I2C_STM_END = 0x00;

    // Timeouts
    constexpr unsigned int USB_TIMEOUT = 1000;
}

#endif // CH341_CONFIG_HPP