# LoRaWAN CH341 Project

This project implements a LoRaWAN device communication using CH341 SPI interface.

As an example of using this library you can see main.cpp

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

- libch341-dev (for CH341 communication)
- libjson-cpp-dev (for JSON configuration parsing)
- libspi-dev (for SPI communication)
- C++17 compatible compiler
- CMake build system (version 3.10 or higher)