#include "LoRaWAN.hpp"
#include "ConfigManager.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstdio>
#include <string>
#include <cstring>
#include <memory>
#include <iomanip>
#include <array>
#include <vector>
#include "SPIInterface.hpp"

// Helper for conditional debug
#define DEBUG_PRINT(x) do { if(LoRaWAN::getVerbose()) { std::cout << x; } } while(0)
#define DEBUG_PRINTLN(x) do { if(LoRaWAN::getVerbose()) { std::cout << x << std::endl; } } while(0)
#define DEBUG_HEX(x) do { if(LoRaWAN::getVerbose()) { std::cout << std::hex << (x) << std::dec; } } while(0)

LoRaWAN lora;

void resetAndRejoin(LoRaWAN& lora, const std::string& devEUI, const std::string& appEUI, const std::string& appKey) {
    // Delete the session file
    std::remove("lorawan_session.json");
    
    std::cout << "Forcing new OTAA join..." << std::endl;
    
    // Reset the internal LoRaWAN session
    lora.resetSession();
    
    // Configure LoRaWAN with config values
    lora.setDevEUI(devEUI);
    lora.setAppEUI(appEUI);
    lora.setAppKey(appKey);
    
    // Force new JOIN
    bool joined = lora.join(LoRaWAN::JoinMode::OTAA);
    if (joined) {
        std::cout << "JOINED SUCCESSFULLY WITH NEW SESSION" << std::endl;
    } else {
        std::cerr << "ERROR: Join failed" << std::endl;
    }
}

void showHelp() {
    std::cout << "Usage: LoRaWANCH341 [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -o, --one-channel  Single channel mode (868.1 MHz, SF9, BW 125 KHz)" << std::endl;
    std::cout << "  -r, --reset         Force LoRaWAN session reset" << std::endl;
    std::cout << "  -c, --config        Specify configuration file (default: config.json)" << std::endl;
    std::cout << "  -v, --verbose       Enable detailed debug messages" << std::endl;
    std::cout << "  --spi=<type>        SPI type: 'ch341' or 'linux' (overrides config.json)" << std::endl;
    std::cout << "  --device=<path>     Linux SPI device path (overrides config.json)" << std::endl;
    std::cout << "  --device-index=<n>  CH341 device index (0,1,2...) (overrides config.json)" << std::endl;
    std::cout << "  --speed=<hz>        SPI bus speed in Hz (overrides config.json)" << std::endl;
    std::cout << "  -h, --help          Show this help" << std::endl;
}

// receiveCallback
void receiveCallback(const LoRaWAN::Message& message)
{
    std::cout << "Received " << (message.confirmed ? "confirmed" : "unconfirmed") << " message on port " << static_cast<int>(message.port) << ": ";
    for (uint8_t byte : message.payload) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

int main(int argc, char* argv[])
{
    // Default initial values
    bool one_channel = false;
    bool forceReset = false;
    bool verbose = false; // Default silent mode
    std::string configPath = "config.json";
    
    // Variables for command line options that override config.json
    bool hasSpiType = false;
    std::string cmdSpiType;
    bool hasDevicePath = false;
    std::string cmdDevicePath;
    bool hasDeviceIndex = false;
    int cmdDeviceIndex = 0;
    bool hasSpeed = false;
    uint32_t cmdSpeed = 0;

    // Process command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            showHelp();
            return 0;
        } 
        else if (arg == "-o" || arg == "--one-channel") {
            one_channel = true;
        }
        else if (arg == "-r" || arg == "--reset") {
            forceReset = true;
        } 
        else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                configPath = argv[i + 1];
                i++; // Skip the next argument
            } else {
                std::cerr << "Error: Missing configuration file path" << std::endl;
                return 1;
            }
        }
        else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            std::cout << "Verbose mode activated" << std::endl;
        } 
        else if (arg.find("--spi=") == 0) {
            cmdSpiType = arg.substr(6);
            hasSpiType = true;
            if (cmdSpiType != "ch341" && cmdSpiType != "linux") {
                std::cerr << "Error: Invalid SPI type. Use 'ch341' or 'linux'" << std::endl;
                return 1;
            }
        }
        else if (arg.find("--device=") == 0) {
            cmdDevicePath = arg.substr(9);
            hasDevicePath = true;
        }
        else if (arg.find("--device-index=") == 0) {
            try {
                cmdDeviceIndex = std::stoi(arg.substr(15));
                hasDeviceIndex = true;
            } catch (...) {
                std::cerr << "Error: Invalid device index" << std::endl;
                return 1;
            }
        }
        else if (arg.find("--speed=") == 0) {
            try {
                cmdSpeed = std::stoul(arg.substr(8));
                hasSpeed = true;
            } catch (...) {
                std::cerr << "Error: Invalid SPI speed" << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            showHelp();
            return 1;
        }
    }

    // Load configuration
    std::cout << "Loading configuration from: " << configPath << std::endl;
    ConfigManager config(configPath);
    if (!config.loadConfig()) {
        std::cerr << "Error loading configuration. Using default values." << std::endl;
    }

    // Read connection parameters from config.json
    std::string spi_type = config.getNestedString("connection.spi_type", "ch341");
    std::string spi_device = config.getNestedString("connection.spi_device", "/dev/spidev0.0");
    int device_index = config.getNestedInt("connection.device_index", 0);
    uint32_t spi_speed = config.getNestedInt("connection.spi_speed", 1000000);
    
    // Override with command line values if provided
    if (hasSpiType) {
        spi_type = cmdSpiType;
        DEBUG_PRINTLN("Overriding SPI type with command line value: " << spi_type);
    }
    
    if (hasDevicePath) {
        spi_device = cmdDevicePath;
        DEBUG_PRINTLN("Overriding SPI device path with command line value: " << spi_device);
    }
    
    if (hasDeviceIndex) {
        device_index = cmdDeviceIndex;
        DEBUG_PRINTLN("Overriding CH341 device index with command line value: " << device_index);
    }
    
    if (hasSpeed) {
        spi_speed = cmdSpeed;
        DEBUG_PRINTLN("Overriding SPI speed with command line value: " << spi_speed);
    }
    
    // Read LoRaWAN credentials
    std::string devEUI = config.getNestedString("device.devEUI", "");
    std::string appEUI = config.getNestedString("device.appEUI", "");
    std::string appKey = config.getNestedString("device.appKey", "");
    
    // Read additional options
    bool configForceReset = config.getNestedBool("options.force_reset", false);
    bool configVerbose = config.getNestedBool("options.verbose", false);
    int sendInterval = config.getNestedInt("options.send_interval", 60);
    
    // Command line option takes priority
    forceReset = forceReset || configForceReset;
    verbose = verbose || configVerbose;
    
    // Display final configuration
    DEBUG_PRINTLN("Final configuration:");
    DEBUG_PRINT("  SPI: " << spi_type);
    if (spi_type == "ch341") {
        DEBUG_PRINTLN(" (index: " << device_index << ")");
    } else if (spi_type == "linux") {
        DEBUG_PRINTLN(" (device: " << spi_device << ", speed: " << spi_speed << " Hz)");
    } else {
        DEBUG_PRINTLN(" (unknown)");
    }

    DEBUG_PRINTLN(std::endl << "  DevEUI: " << devEUI);
    DEBUG_PRINTLN("  AppEUI: " << appEUI);
    DEBUG_PRINTLN("  AppKey: " << appKey);
    DEBUG_PRINTLN("  Send interval: " << sendInterval << " seconds");
    DEBUG_PRINTLN("  Force reset: " << (forceReset ? "Yes" : "No"));
    DEBUG_PRINTLN("  Verbose: " << (verbose ? "Yes" : "No"));

    // Create the corresponding SPI instance
    std::unique_ptr<SPIInterface> spi_interface;
    
    if (spi_type == "ch341") {
        DEBUG_PRINTLN("Using CH341 as SPI interface (device #" << device_index << ")");
        spi_interface = SPIFactory::createCH341SPI(device_index, true);
    } 
    else if (spi_type == "linux") {
        DEBUG_PRINTLN("Using native Linux SPI: " << spi_device << " at " << spi_speed << " Hz");
        spi_interface = SPIFactory::createLinuxSPI(spi_device, spi_speed);
    }
    else {
        std::cerr << "Unsupported SPI type: " << spi_type << std::endl;
        return 1;
    }

    // Create LoRaWAN instance with the selected SPI
    LoRaWAN lorawan(std::move(spi_interface));

    // Set verbose mode for all components
    LoRaWAN::setVerbose(verbose);

    // Initialize
    if (!lorawan.init())
    {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }

    // Configure single-channel mode if requested
    if (one_channel) {
        std::cout << "Setting up single-channel mode..." << std::endl;
        lorawan.setSingleChannel(true, 868.1, 9, 125);
    }

    // Configure LoRaWAN
    lorawan.setDevEUI(devEUI);
    lorawan.setAppEUI(appEUI);
    lorawan.setAppKey(appKey);

    // If reset was requested, force it now
    if (forceReset) {
        resetAndRejoin(lorawan, devEUI, appEUI, appKey);
    } 
    // If not, do normal join
    else if (lorawan.join(LoRaWAN::JoinMode::OTAA)) {
        std::cout << "Joined successfully" << std::endl;
    } else {
        std::cout << "Join failed, forcing reset and rejoin" << std::endl;
        resetAndRejoin(lorawan, devEUI, appEUI, appKey);
    }

    // Explicitly switch to Class C and configure to listen on RX2
    std::cout << "Switching to Class C mode for continuous reception at 869.525 MHz..." << std::endl;
    lorawan.setDeviceClass(LoRaWAN::DeviceClass::CLASS_C);
    lorawan.enableADR(true);

    // Variable to count consecutive failed attempts
    int failedAttempts = 0;
    
    // Set receive callback
    lorawan.onReceive(receiveCallback);

    lorawan.requestLinkCheck();
    
    // In the main loop, display information about the current frequency
    while (true)
    {
        // Send data
        std::vector<uint8_t> data = {1, 2, 3, 4};
        if (lorawan.send(data, 1, false)) {
            std::cout << "Message sent successfully" << std::endl;
            failedAttempts = 0; // Reset failure counter
        } else {
            std::cout << "Failed to send message" << std::endl;
            failedAttempts++;
            
            // If too many consecutive failures, reset the session
            if (failedAttempts >= 3) {
                std::cout << "Too many failed attempts, resetting session..." << std::endl;
                resetAndRejoin(lorawan, devEUI, appEUI, appKey);
                failedAttempts = 0;
            }
        }
        
        // Show current listening frequency
        std::cout << "Listening on: " << lorawan.getFrequency() << " MHz" << std::endl;
        
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < sendInterval)
        {
            lorawan.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return 0;
}