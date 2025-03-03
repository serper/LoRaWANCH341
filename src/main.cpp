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

// Helper para debug condicional
#define DEBUG_PRINT(x) do { if(LoRaWAN::getVerbose()) { std::cout << x; } } while(0)
#define DEBUG_PRINTLN(x) do { if(LoRaWAN::getVerbose()) { std::cout << x << std::endl; } } while(0)
#define DEBUG_HEX(x) do { if(LoRaWAN::getVerbose()) { std::cout << std::hex << (x) << std::dec; } } while(0)

LoRaWAN lora;

// Añadir un método para reiniciar la sesión y forzar un nuevo join
void resetAndRejoin(LoRaWAN& lora, const std::string& devEUI, const std::string& appEUI, const std::string& appKey) {
    // Borrar el archivo de sesión
    std::remove("lorawan_session.json");
    
    std::cout << "Forzando nuevo OTAA join..." << std::endl;
    
    // Reiniciar la sesión interna de LoRaWAN
    lora.resetSession();
    
    // Configurar LoRaWAN con los valores del config
    lora.setDevEUI(devEUI);
    lora.setAppEUI(appEUI);
    lora.setAppKey(appKey);
    
    // Forzar JOIN nuevo
    bool joined = lora.join(LoRaWAN::JoinMode::OTAA);
    if (joined) {
        std::cout << "JOINED SUCCESSFULLY WITH NEW SESSION" << std::endl;
    } else {
        std::cerr << "ERROR: Join failed" << std::endl;
    }
}

void showHelp() {
    std::cout << "Uso: LoRaWANCH341 [opciones]" << std::endl;
    std::cout << "Opciones:" << std::endl;
    std::cout << "  -r, --reset         Forzar reinicio de sesión LoRaWAN" << std::endl;
    std::cout << "  -c, --config        Especificar archivo de configuración (por defecto: config.json)" << std::endl;
    std::cout << "  -v, --verbose       Activar mensajes de depuración detallados" << std::endl;
    std::cout << "  --spi=<tipo>        Tipo de SPI: 'ch341' o 'linux' (sobreescribe config.json)" << std::endl;
    std::cout << "  --device=<ruta>     Ruta al dispositivo SPI Linux (sobreescribe config.json)" << std::endl;
    std::cout << "  --device-index=<n>  Índice del dispositivo CH341 (0,1,2...) (sobreescribe config.json)" << std::endl;
    std::cout << "  --speed=<hz>        Velocidad del bus SPI en Hz (sobreescribe config.json)" << std::endl;
    std::cout << "  -h, --help          Mostrar esta ayuda" << std::endl;
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

// Modificar main para procesar argumentos de línea de comandos y cargar la configuración
int main(int argc, char* argv[])
{
    // Valores iniciales predeterminados
    bool forceReset = false;
    bool verbose = false; // Por defecto, modo silencioso
    std::string configPath = "config.json";
    
    // Variables para opciones de línea de comandos que sobrescriben el config.json
    bool hasSpiType = false;
    std::string cmdSpiType;
    bool hasDevicePath = false;
    std::string cmdDevicePath;
    bool hasDeviceIndex = false;
    int cmdDeviceIndex = 0;
    bool hasSpeed = false;
    uint32_t cmdSpeed = 0;

    // Procesar argumentos de línea de comandos
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            showHelp();
            return 0;
        } 
        else if (arg == "-r" || arg == "--reset") {
            forceReset = true;
        } 
        else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                configPath = argv[i + 1];
                i++; // Saltar el siguiente argumento
            } else {
                std::cerr << "Error: Falta ruta del archivo de configuración" << std::endl;
                return 1;
            }
        }
        else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            std::cout << "Modo verbose activado" << std::endl;
        } 
        else if (arg.find("--spi=") == 0) {
            cmdSpiType = arg.substr(6);
            hasSpiType = true;
            if (cmdSpiType != "ch341" && cmdSpiType != "linux") {
                std::cerr << "Error: Tipo de SPI no válido. Use 'ch341' o 'linux'" << std::endl;
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
                std::cerr << "Error: Índice de dispositivo no válido" << std::endl;
                return 1;
            }
        }
        else if (arg.find("--speed=") == 0) {
            try {
                cmdSpeed = std::stoul(arg.substr(8));
                hasSpeed = true;
            } catch (...) {
                std::cerr << "Error: Velocidad SPI inválida" << std::endl;
                return 1;
            }
        }
        else {
            std::cerr << "Argumento desconocido: " << arg << std::endl;
            showHelp();
            return 1;
        }
    }

    // Cargar configuración
    std::cout << "Cargando configuración desde: " << configPath << std::endl;
    ConfigManager config(configPath);
    if (!config.loadConfig()) {
        std::cerr << "Error al cargar la configuración. Usando valores por defecto." << std::endl;
    }

    // Leer parámetros de conexión desde config.json
    std::string spi_type = config.getNestedString("connection.spi_type", "ch341");
    std::string spi_device = config.getNestedString("connection.spi_device", "/dev/spidev0.0");
    int device_index = config.getNestedInt("connection.device_index", 0);
    uint32_t spi_speed = config.getNestedInt("connection.spi_speed", 1000000);
    
    // Sobreescribir con valores de línea de comandos si se proporcionaron
    if (hasSpiType) {
        spi_type = cmdSpiType;
        DEBUG_PRINTLN("Sobreescribiendo tipo SPI con valor de línea de comandos: " << spi_type);
    }
    
    if (hasDevicePath) {
        spi_device = cmdDevicePath;
        DEBUG_PRINTLN("Sobreescribiendo ruta de dispositivo SPI con valor de línea de comandos: " << spi_device);
    }
    
    if (hasDeviceIndex) {
        device_index = cmdDeviceIndex;
        DEBUG_PRINTLN("Sobreescribiendo índice de dispositivo CH341 con valor de línea de comandos: " << device_index);
    }
    
    if (hasSpeed) {
        spi_speed = cmdSpeed;
        DEBUG_PRINTLN("Sobreescribiendo velocidad SPI con valor de línea de comandos: " << spi_speed);
    }
    
    // Leer credenciales LoRaWAN
    std::string devEUI = config.getNestedString("device.devEUI", "");
    std::string appEUI = config.getNestedString("device.appEUI", "");
    std::string appKey = config.getNestedString("device.appKey", "");
    
    // Leer opciones adicionales
    bool configForceReset = config.getNestedBool("options.force_reset", false);
    bool configVerbose = config.getNestedBool("options.verbose", false);
    int sendInterval = config.getNestedInt("options.send_interval", 30);
    
    // La opción de línea de comandos tiene prioridad
    forceReset = forceReset || configForceReset;
    verbose = verbose || configVerbose;
    
    // Mostrar la configuración final
    DEBUG_PRINTLN("Configuración final:" << std::endl);
    std::cout << "  SPI: " << spi_type;
    if (spi_type == "ch341") {
        DEBUG_PRINTLN(" (índice: " << device_index << ")");
    } else if (spi_type == "linux") {
        DEBUG_PRINTLN(" (dispositivo: " << spi_device << ", velocidad: " << spi_speed << " Hz)");
    } else {
        DEBUG_PRINTLN(" (desconocido)");
    }

    DEBUG_PRINTLN(std::endl << "  DevEUI: " << devEUI);
    DEBUG_PRINTLN("  AppEUI: " << appEUI);
    DEBUG_PRINTLN("  AppKey: " << appKey);
    DEBUG_PRINTLN("  Intervalo de envío: " << sendInterval << " segundos");
    DEBUG_PRINTLN("  Forzar reset: " << (forceReset ? "Sí" : "No"));
    DEBUG_PRINTLN("  Verbose: " << (verbose ? "Sí" : "No"));

    // Crear la instancia de SPI correspondiente
    std::unique_ptr<SPIInterface> spi_interface;
    
    if (spi_type == "ch341") {
        DEBUG_PRINTLN("Usando CH341 como interfaz SPI (dispositivo #" << device_index << ")");
        spi_interface = SPIFactory::createCH341SPI(device_index, true);
    } 
    else if (spi_type == "linux") {
        DEBUG_PRINTLN("Usando Linux SPI nativo: " << spi_device << " a " << spi_speed << " Hz");
        spi_interface = SPIFactory::createLinuxSPI(spi_device, spi_speed);
    }
    else {
        std::cerr << "Tipo de SPI no soportado: " << spi_type << std::endl;
        return 1;
    }

    // Crear la instancia de LoRaWAN con el SPI seleccionado
    LoRaWAN lorawan(std::move(spi_interface));

    // Establecer el modo verboso para todos los componentes
    LoRaWAN::setVerbose(verbose);

    // Inicializar
    if (!lorawan.init())
    {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }

    // Configurar LoRaWAN
    lorawan.setDevEUI(devEUI);
    lorawan.setAppEUI(appEUI);
    lorawan.setAppKey(appKey);

    // Si se solicitó el reinicio, forzarlo ahora
    if (forceReset) {
        resetAndRejoin(lorawan, devEUI, appEUI, appKey);
    } 
    // Si no, hacer join normal
    else if (lorawan.join(LoRaWAN::JoinMode::OTAA)) {
        std::cout << "Joined successfully" << std::endl;
    } else {
        std::cout << "Join failed, forcing reset and rejoin" << std::endl;
        resetAndRejoin(lorawan, devEUI, appEUI, appKey);
    }

    // Pasamos explícitamente a Clase C y configuramos para escuchar en RX2
    std::cout << "Cambiando a modo Class C para recepción continua en 869.525 MHz..." << std::endl;
    lorawan.setDeviceClass(LoRaWAN::DeviceClass::CLASS_C);

    // Variable para contar intentos fallidos consecutivos
    int failedAttempts = 0;
    
    // Establecer el callback de recepción
    lorawan.onReceive(receiveCallback);

    // En el loop principal, muestra información sobre la frecuencia actual
    while (true)
    {
        // Enviar datos
        std::vector<uint8_t> data = {1, 2, 3, 4};
        if (lorawan.send(data, 1, true)) {
            std::cout << "Message sent successfully" << std::endl;
            failedAttempts = 0; // Reiniciar contador de fallos
        } else {
            std::cout << "Failed to send message" << std::endl;
            failedAttempts++;
            
            // Si hay demasiados fallos consecutivos, reiniciar sesión
            if (failedAttempts >= 3) {
                std::cout << "Too many failed attempts, resetting session..." << std::endl;
                resetAndRejoin(lorawan, devEUI, appEUI, appKey);
                failedAttempts = 0;
            }
        }
        
        // Mostrar la frecuencia de escucha actual
        std::cout << "Escuchando en: " << lorawan.getFrequency() << " MHz" << std::endl;
        
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < sendInterval)
        {
            lorawan.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return 0;
}