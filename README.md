# LoRaWAN CH341 Implementation

This project provides a C++ implementation of the LoRaWAN protocol using the CH341 USB-to-SPI converter to interface with a RFM95/SX1276 LoRa radio module.

## Features

- Full LoRaWAN 1.0.3 protocol implementation
- Support for Class A and Class C devices
- OTAA and ABP activation methods
- Support for multiple regions (EU868, US915, etc.)
- MAC commands implementation
- Adaptive Data Rate (ADR) support
- Channel management
- Duty cycle management
- Confirmed/unconfirmed messages
- Linux and Windows support via CH341 USB interface

## Hardware Requirements

- CH341A/CH341F USB adapter
- RFM95W, SX1276, or compatible LoRa module
- Any USB-capable device running Linux or Windows

## Dependencies

- libusb-1.0
- OpenSSL (for AES encryption)
- cJSON (for configuration and session management)
- C++17 compatible compiler

## Building

## Current Implementation Status

The project currently supports the following LoRaWAN features:

- **Network Activation**
    - Over-the-Air Activation (OTAA)
    - Activation By Personalization (ABP)

- **Message Handling**
    - Uplink and downlink communication
    - Class A and Class C device operation

- **Network Management**
    - Regional channel and power restrictions
    - Adaptive Data Rate (ADR) with performance statistics
    - Duty cycle management

The implementation adheres to LoRaWAN protocol specifications and continues to evolve with additional features.

## Configuration

The project uses a JSON configuration file (`config.json`) to set up device parameters and connection settings. A sample configuration file (`config.json.sample`) is provided as reference.

### Configuration Structure

```json
{
    "device": {
        "devEUI": "Device Extended Unique Identifier",
        "appEUI": "Application Extended Unique Identifier",
        "appKey": "Application Key"
    },
    "connection": {
        "spi_type": "ch341",
        "device_index": 0,
        "spi_device": "/dev/spidev0.0",
        "spi_speed": 1000000
    },
    "options": {
        "force_reset": false,
        "send_interval": 30,
        "verbose": false
    }
}
```
### Use
### Implementation Examples

#### CH341 SPI Interface
```cpp
// Create SPI interface using CH341
auto spi_interface = SPIFactory::createCH341SPI(device_index, true);
LoRaWAN lorawan(std::move(spi_interface));
```
or more easy
```cpp
LoRaWAN lorawan();
```

#### Native Linux SPI Interface
```cpp
// Create SPI interface using native Linux SPI
auto spi_interface = SPIFactory::createLinuxSPI(spi_device, spi_speed);
LoRaWAN lorawan(std::move(spi_interface));
```

The `SPIFactory` class provides factory methods to create appropriate SPI interfaces based on your hardware configuration. Choose the implementation that matches your setup.

### Parameters

#### Device Settings
- `devEUI`: Device Extended Unique Identifier (16 characters)
- `appEUI`: Application Extended Unique Identifier (16 characters)
- `appKey`: Application Key (32 characters)

#### Connection Settings
- `spi_type`: SPI interface type (currently supports "ch341")
- `device_index`: Device index number
- `spi_device`: SPI device path
- `spi_speed`: SPI communication speed in Hz

#### Options
- `force_reset`: Enable/disable force reset
- `send_interval`: Message sending interval in seconds
- `verbose`: Enable/disable verbose logging

## Getting Started

1. Copy `config.json.sample` to `config.json`
2. Update the configuration with your device credentials
3. Configure the connection settings according to your setup
4. Adjust options as needed

## Requirements

- CH341 compatible hardware or native linux SPI
- Access to LoRaWAN network
- Proper device registration with network server

## Dependencies

- libusb-dev (for CH341 communication)
- libjson-cpp-dev (for JSON configuration parsing)
- libspi-dev (for SPI communication)
- libopenssl
- C++17 compatible compiler
- CMake build system (version 3.10 or higher)