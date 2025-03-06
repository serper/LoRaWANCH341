
/**
 * @file CH341SPI.cpp
 * @brief Implementation of the CH341SPI class for SPI communication using the CH341 USB-SPI adapter.
 * 
 * This file contains the implementation of the CH341SPI class, which provides methods for
 * initializing, configuring, and communicating with SPI devices using the CH341 USB-SPI adapter.
 * It uses the libusb library for USB communication.
 * 
 * @author Sergio Pérez
 * @date 2025
 */

#include "CH341SPI.hpp"
#include "CH341Config.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

CH341SPI::CH341SPI(int device_index, bool lsb_first)
    : device(nullptr),
      context(nullptr),
      device_index(device_index),
      lsb_first(lsb_first),
      interruptEnabled(false),
      threadRunning(false)
{
    // Initialize libusb context
    int ret = libusb_init(&context);
    if (ret != 0)
    {
        std::cerr << "Failed to initialize libusb: " << libusb_error_name(ret) << std::endl;
    }
}

CH341SPI::~CH341SPI()
{
    // Stop interrupt thread if running
    if (threadRunning)
    {
        threadRunning = false;
        if (interruptThread.joinable())
        {
            interruptThread.join();
        }
    }

    // Ensure device is closed
    close();

    // Free libusb context
    if (context)
    {
        libusb_exit(context);
        context = nullptr;
    }
}

bool CH341SPI::open()
{
    if (!context)
    {
        std::cerr << "LibUSB not initialized" << std::endl;
        return false;
    }

    try
    {
        // Get list of devices
        libusb_device **device_list;
        ssize_t count = libusb_get_device_list(context, &device_list);
        if (count < 0)
        {
            std::cerr << "Failed to get device list: " << libusb_error_name(count) << std::endl;
            return false;
        }

        // Find CH341 devices
        std::vector<libusb_device *> ch341_devices;
        for (ssize_t i = 0; i < count; i++)
        {
            libusb_device *dev = device_list[i];
            libusb_device_descriptor desc;

            int ret = libusb_get_device_descriptor(dev, &desc);
            if (ret < 0)
                continue;

            if (desc.idVendor == CH341Config::VENDOR_ID &&
                desc.idProduct == CH341Config::PRODUCT_ID)
            {
                ch341_devices.push_back(dev);
            }
        }

        if (ch341_devices.empty())
        {
            std::cerr << "No CH341 devices found" << std::endl;
            libusb_free_device_list(device_list, 1);
            return false;
        }

        if (device_index >= static_cast<int>(ch341_devices.size()))
        {
            std::cerr << "Device index " << device_index << " out of range, only "
                      << ch341_devices.size() << " devices found" << std::endl;
            libusb_free_device_list(device_list, 1);
            return false;
        }

        // Open selected device
        int ret = libusb_open(ch341_devices[device_index], &device);
        libusb_free_device_list(device_list, 1);

        if (ret != 0)
        {
            std::cerr << "Failed to open device: " << libusb_error_name(ret) << std::endl;
            return false;
        }

        // Set configuration
        ret = libusb_set_configuration(device, 1);
        if (ret != 0)
        {
            std::cerr << "Failed to set configuration: " << libusb_error_name(ret) << std::endl;
            libusb_close(device);
            device = nullptr;
            return false;
        }

        // Claim interface
        ret = libusb_claim_interface(device, 0);
        if (ret != 0)
        {
            std::cerr << "Failed to claim interface: " << libusb_error_name(ret) << std::endl;
            libusb_close(device);
            device = nullptr;
            return false;
        }

        // Configure SPI mode
        if (!configStream())
        {
            close();
            return false;
        }

        if (!enablePins(true))
        {
            close();
            return false;
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in open: " << e.what() << std::endl;
        if (device)
        {
            libusb_close(device);
            device = nullptr;
        }
        return false;
    }
}

void CH341SPI::close()
{
    if (device)
    {
        try
        {
            enablePins(false);
            libusb_release_interface(device, 0);
            libusb_close(device);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception in close: " << e.what() << std::endl;
        }
        device = nullptr;
    }
}

bool CH341SPI::configStream()
{
    if (!device)
        return false;

    try
    {
        // Configure for 100KHz (slower for stability)
        uint8_t cmd[3] = {
            CH341Config::CMD_I2C_STREAM,
            CH341Config::CMD_I2C_STM_SET | 0x01, // 100KHz
            CH341Config::CMD_I2C_STM_END};

        int transferred = 0;
        int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                       cmd, sizeof(cmd), &transferred,
                                       CH341Config::USB_TIMEOUT);

        if (ret != 0 || transferred != sizeof(cmd))
        {
            std::cerr << "Error configuring stream: " << libusb_error_name(ret) << std::endl;
            return false;
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in configStream: " << e.what() << std::endl;
        return false;
    }
}

bool CH341SPI::enablePins(bool enable)
{
    if (!device)
        return false;

    try
    {
        // Prepare command buffer
        uint8_t cmd[6] = {
            CH341Config::CMD_UIO_STREAM,
            CH341Config::CMD_UIO_STM_OUT | 0x37,                   // CS high
            CH341Config::CMD_UIO_STM_OUT | 0x37,                   // CS high
            CH341Config::CMD_UIO_STM_OUT | 0x37,                   // CS high
            static_cast<uint8_t>(CH341Config::CMD_UIO_STM_DIR | (enable ? 0x3F : 0x00)), // Set direction
            CH341Config::CMD_UIO_STM_END                           // End stream
        };

        int transferred = 0;
        int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                       cmd, sizeof(cmd), &transferred,
                                       CH341Config::USB_TIMEOUT);

        if (ret != 0 || transferred != sizeof(cmd))
        {
            std::cerr << "Error setting pins: " << libusb_error_name(ret) << std::endl;
            return false;
        }

        // Add delay similar to Python implementation
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in enablePins: " << e.what() << std::endl;
        return false;
    }
}

uint8_t CH341SPI::swapBits(uint8_t byte)
{
    // Implementación directa del algoritmo de intercambio bit a bit
    uint8_t result = 0;
    for (int i = 0; i < 8; i++)
    {
        if (byte & (1 << i))
        {
            result |= (1 << (7 - i));
        }
    }
    return result;
}

std::vector<uint8_t> CH341SPI::transfer(const std::vector<uint8_t> &write_data, size_t read_length)
{
    std::vector<uint8_t> result;

    if (!device)
    {
        return result; // Empty result indicates error
    }

    try
    {
        // CS low
        uint8_t cs_low[3] = {
            CH341Config::CMD_UIO_STREAM,
            CH341Config::CMD_UIO_STM_OUT | 0x36,
            CH341Config::CMD_UIO_STM_END};

        int transferred = 0;
        int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                       cs_low, sizeof(cs_low), &transferred,
                                       CH341Config::USB_TIMEOUT);

        if (ret != 0)
        {
            std::cerr << "Error setting CS low: " << libusb_error_name(ret) << std::endl;
            return result;
        }

        // Transferir byte a byte, como en la implementación Python
        for (size_t i = 0; i < write_data.size(); i++)
        {
            // Enviar un byte a la vez
            uint8_t cmd[2] = {CH341Config::CMD_SPI_STREAM, lsb_first ? swapBits(write_data[i]) : write_data[i]};

            ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                       cmd, sizeof(cmd), &transferred,
                                       CH341Config::USB_TIMEOUT);

            if (ret != 0)
            {
                std::cerr << "Error in SPI write: " << libusb_error_name(ret) << std::endl;
                return result;
            }

            // Leer la respuesta para este byte
            uint8_t response_byte;
            ret = libusb_bulk_transfer(device, CH341Config::BULK_READ_EP,
                                       &response_byte, 1, &transferred,
                                       CH341Config::USB_TIMEOUT);

            if (ret != 0)
            {
                std::cerr << "Error in SPI read: " << libusb_error_name(ret) << std::endl;
                return result;
            }

            // Descartar este byte de respuesta (echo del comando)
        }

        // Ahora leer los bytes solicitados
        for (size_t i = 0; i < read_length; i++)
        {
            // Enviar byte dummy (0xFF)
            uint8_t cmd[2] = {CH341Config::CMD_SPI_STREAM, 0xFF};

            ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                       cmd, sizeof(cmd), &transferred,
                                       CH341Config::USB_TIMEOUT);

            if (ret != 0)
            {
                std::cerr << "Error in SPI read byte: " << libusb_error_name(ret) << std::endl;
                return result;
            }

            // Leer la respuesta
            uint8_t response_byte;
            ret = libusb_bulk_transfer(device, CH341Config::BULK_READ_EP,
                                       &response_byte, 1, &transferred,
                                       CH341Config::USB_TIMEOUT);

            if (ret != 0)
            {
                std::cerr << "Error in SPI read byte: " << libusb_error_name(ret) << std::endl;
                return result;
            }

            // Procesar y añadir a los resultados
            result.push_back(lsb_first ? swapBits(response_byte) : response_byte);
        }

        // CS high
        uint8_t cs_high[3] = {
            CH341Config::CMD_UIO_STREAM,
            CH341Config::CMD_UIO_STM_OUT | 0x37,
            CH341Config::CMD_UIO_STM_END};

        ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                   cs_high, sizeof(cs_high), &transferred,
                                   CH341Config::USB_TIMEOUT);

        if (ret != 0)
        {
            std::cerr << "Error setting CS high: " << libusb_error_name(ret) << std::endl;
        }

        return result;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in spiTransfer: " << e.what() << std::endl;
        return std::vector<uint8_t>();
    }
}

bool CH341SPI::digitalWrite(uint8_t pin, bool value)
{
    if (!device)
        return false;

    // Asegurarse de que el pin está configurado como salida
    _gpio_direction |= pin;

    if (value)
    {
        _gpio_output |= pin; // Set the pin high
    }
    else
    {
        _gpio_output &= ~pin; // Set the pin low
    }

    // Enviar el comando GPIO al CH341
    uint8_t cmd[4] = {
        CH341Config::CMD_UIO_STREAM,
        static_cast<uint8_t>(CH341Config::CMD_UIO_STM_OUT | _gpio_output),
        static_cast<uint8_t>(CH341Config::CMD_UIO_STM_DIR | _gpio_direction),
        CH341Config::CMD_UIO_STM_END};

    int transferred = 0;
    int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                   cmd, sizeof(cmd), &transferred,
                                   CH341Config::USB_TIMEOUT);

    return (ret == 0);
}

bool CH341SPI::digitalRead(uint8_t pin)
{
    if (!device)
        return false;

    // Asegurar que el pin está configurado como entrada
    _gpio_direction &= ~pin;

    // Enviar el comando para configurar la dirección
    uint8_t dir_cmd[3] = {
        CH341Config::CMD_UIO_STREAM,
        static_cast<uint8_t>(CH341Config::CMD_UIO_STM_DIR | _gpio_direction),
        CH341Config::CMD_UIO_STM_END
    };

    int transferred = 0;
    int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                   dir_cmd, sizeof(dir_cmd), &transferred,
                                   CH341Config::USB_TIMEOUT);

    if (ret != 0)
        return false;

    // Comando para leer el estado de los pines
    uint8_t read_cmd[2] = {
        CH341Config::CMD_UIO_STREAM | 0x80, // Comando de lectura
        CH341Config::CMD_UIO_STM_END};

    ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                               read_cmd, sizeof(read_cmd), &transferred,
                               CH341Config::USB_TIMEOUT);

    if (ret != 0)
        return false;

    // Leer la respuesta
    uint8_t gpio_value;
    ret = libusb_bulk_transfer(device, CH341Config::BULK_READ_EP,
                               &gpio_value, 1, &transferred,
                               CH341Config::USB_TIMEOUT);

    if (ret != 0 || transferred != 1)
        return false;

    // Verificar si el pin específico está alto
    return (gpio_value & pin) != 0;
}

bool CH341SPI::pinMode(uint8_t pin, uint8_t mode)
{
    if (!device)
        return false;

    if (mode == OUTPUT)
    {
        _gpio_direction |= pin; // Configurar como salida
    }
    else
    {
        _gpio_direction &= ~pin; // Configurar como entrada
    }

    // Enviar el comando para configurar la dirección
    uint8_t cmd[3] = {
        static_cast<uint8_t>(CH341Config::CMD_UIO_STREAM),
        static_cast<uint8_t>(CH341Config::CMD_UIO_STM_DIR | _gpio_direction),
        CH341Config::CMD_UIO_STM_END};

    int transferred = 0;
    int ret = libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                   cmd, sizeof(cmd), &transferred,
                                   CH341Config::USB_TIMEOUT);

    return (ret == 0);
}

bool CH341SPI::setInterruptCallback(InterruptCallback callback)
{
    interruptCallback = callback;
    return true;
}

bool CH341SPI::enableInterrupt(bool enable)
{
    if (enable && !interruptEnabled)
    {
        interruptEnabled = true;
        threadRunning = true;
        // Crear un nuevo hilo que monitoree el pin INT#
        interruptThread = std::thread(&CH341SPI::interruptMonitoringThread, this);
        return true;
    }
    else if (!enable && interruptEnabled)
    {
        interruptEnabled = false;
        threadRunning = false;
        if (interruptThread.joinable())
        {
            interruptThread.join();
        }
        return true;
    }
    return false;
}

void CH341SPI::interruptMonitoringThread()
{
    // Vector para almacenar el estado del registro de interrupciones
    uint8_t cmd[2] = {CH341Config::CMD_UIO_STREAM | 0x80, CH341Config::CMD_UIO_STM_END};

    while (threadRunning)
    {
        if (device)
        {
            // Leer el estado del pin INT# (pin 7)
            int transferred = 0;

            // Enviar comando para leer los pines
            libusb_bulk_transfer(device, CH341Config::BULK_WRITE_EP,
                                 cmd, sizeof(cmd), &transferred,
                                 CH341Config::USB_TIMEOUT);

            // Leer la respuesta
            uint8_t pinState;
            if (libusb_bulk_transfer(device, CH341Config::BULK_READ_EP,
                                     &pinState, 1, &transferred,
                                     CH341Config::USB_TIMEOUT) == 0)
            {

                // Comprobar si el bit correspondiente a INT# está activo (pin 7 = bit 6)
                // Nota: Puede necesitar ajustar esta lógica según cómo esté conectado
                bool interruptTriggered = ((pinState & 0x40) == 0); // Activo a nivel bajo

                static bool lastState = false;
                // Detectar cambio de estado (flanco descendente)
                if (interruptTriggered && !lastState)
                {
                    // Llamar al callback si está configurado
                    if (interruptCallback)
                    {
                        interruptCallback();
                    }
                }
                lastState = interruptTriggered;
            }
        }

        // Dormir un tiempo corto para no saturar el bus
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}