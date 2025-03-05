#include "LinuxSPI.hpp"
#include <fcntl.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#endif
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring>
#include <map>
#include <algorithm>

LinuxSPI::LinuxSPI(const std::string& device, uint32_t speed, uint8_t mode)
    : device_path(device),
      speed_hz(speed),
      spi_mode(mode),
      fd(-1),
      interrupt_running(false),
      interrupt_pin(-1)
{
#ifdef __linux__
    gpio_export_path = "/sys/class/gpio/export";
    gpio_unexport_path = "/sys/class/gpio/unexport";
#else
    std::cerr << "Warning: LinuxSPI implementation is only available on Linux systems." << std::endl;
#endif
}

LinuxSPI::~LinuxSPI() {
    close();
}

bool LinuxSPI::open() {
#ifdef __linux__
    // Abrir el dispositivo SPI
    fd = ::open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Error: No se pudo abrir dispositivo SPI: " << device_path << std::endl;
        return false;
    }

    // Configurar modo SPI
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        std::cerr << "Error: No se pudo configurar modo SPI" << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    // Configurar bits por palabra (8 bits)
    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Error: No se pudo configurar bits por palabra" << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    // Configurar velocidad SPI
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        std::cerr << "Error: No se pudo configurar velocidad SPI" << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    return true;
#else
    std::cerr << "Error: Linux SPI not supported on this platform" << std::endl;
    return false;
#endif
}

void LinuxSPI::close() {
    // Desactivar interrupciones
    enableInterrupt(false);

#ifdef __linux__
    // Cerrar el dispositivo SPI
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }

    // Liberar todos los pines GPIO usados
    for (const auto& pin_entry : gpio_pin_paths) {
        unexportGPIO(pin_entry.first);
    }
    gpio_pin_paths.clear();
#endif
}

std::vector<uint8_t> LinuxSPI::transfer(const std::vector<uint8_t>& write_data, size_t read_length) {
#ifdef __linux__
    if (fd < 0) {
        return {};
    }

    size_t total_length = std::max(write_data.size(), read_length);
    if (total_length == 0) {
        return {};
    }

    // Preparar buffer de salida
    std::vector<uint8_t> tx_buffer(total_length, 0);
    std::copy(write_data.begin(), write_data.end(), tx_buffer.begin());

    // Preparar buffer de entrada
    std::vector<uint8_t> rx_buffer(total_length, 0);

    // Configurar estructura de transferencia SPI
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buffer.data(),
        .rx_buf = (unsigned long)rx_buffer.data(),
        .len = static_cast<uint32_t>(total_length),
        .speed_hz = speed_hz,
        .delay_usecs = 0,
        .bits_per_word = 8,
        .cs_change = 0,
        .tx_nbits = 0,
        .rx_nbits = 0,
        .pad = 0
    };

    // Realizar transferencia SPI
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cerr << "Error: Fallo en transferencia SPI" << std::endl;
        return {};
    }

    // Devolver los datos recibidos
    return rx_buffer;
#else
    // En plataformas no Linux, devolvemos un vector vacío
    std::cerr << "Error: Linux SPI not supported on this platform" << std::endl;
    return {};
#endif
}

bool LinuxSPI::exportGPIO(uint8_t pin) {
#ifdef __linux__
    std::ofstream exportFile(gpio_export_path);
    if (!exportFile.is_open()) {
        std::cerr << "Error: No se puede abrir archivo de exportación GPIO" << std::endl;
        return false;
    }

    exportFile << pin;
    exportFile.close();

    // Esperar un momento para que el sistema cree los archivos
    usleep(100000); // 100ms

    // Crear la ruta del pin
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << static_cast<int>(pin);
    gpio_pin_paths[pin] = ss.str();

    return true;
#else
    return false;
#endif
}

bool LinuxSPI::unexportGPIO(uint8_t pin) {
#ifdef __linux__
    std::ofstream unexportFile(gpio_unexport_path);
    if (!unexportFile.is_open()) {
        std::cerr << "Error: No se puede abrir archivo de desexportación GPIO" << std::endl;
        return false;
    }

    unexportFile << pin;
    unexportFile.close();

    // Eliminar la ruta del pin del mapa
    gpio_pin_paths.erase(pin);

    return true;
#else
    return false;
#endif
}

bool LinuxSPI::setGPIODirection(uint8_t pin, const std::string& direction) {
#ifdef __linux__
    // Verificar si el pin está exportado
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        if (!exportGPIO(pin)) {
            return false;
        }
    }

    // Abrir archivo de dirección
    std::string direction_path = gpio_pin_paths[pin] + "/direction";
    std::ofstream directionFile(direction_path);
    if (!directionFile.is_open()) {
        std::cerr << "Error: No se puede abrir archivo de dirección GPIO para pin " << static_cast<int>(pin) << std::endl;
        return false;
    }

    directionFile << direction;
    directionFile.close();

    return true;
#else
    return false;
#endif
}

bool LinuxSPI::writeGPIOValue(uint8_t pin, bool value) {
#ifdef __linux__
    // Verificar si el pin está exportado
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        std::cerr << "Error: Pin " << static_cast<int>(pin) << " no exportado" << std::endl;
        return false;
    }

    // Abrir archivo de valor
    std::string value_path = gpio_pin_paths[pin] + "/value";
    std::ofstream valueFile(value_path);
    if (!valueFile.is_open()) {
        std::cerr << "Error: No se puede abrir archivo de valor GPIO para pin " << static_cast<int>(pin) << std::endl;
        return false;
    }

    valueFile << (value ? "1" : "0");
    valueFile.close();

    return true;
#else
    return false;
#endif
}

bool LinuxSPI::readGPIOValue(uint8_t pin) {
#ifdef __linux__
    // Verificar si el pin está exportado
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        std::cerr << "Error: Pin " << static_cast<int>(pin) << " no exportado" << std::endl;
        return false;
    }

    // Abrir archivo de valor
    std::string value_path = gpio_pin_paths[pin] + "/value";
    std::ifstream valueFile(value_path);
    if (!valueFile.is_open()) {
        std::cerr << "Error: No se puede abrir archivo de valor GPIO para pin " << static_cast<int>(pin) << std::endl;
        return false;
    }

    char value;
    valueFile >> value;
    valueFile.close();

    return (value == '1');
#else
    return false;
#endif
}

bool LinuxSPI::digitalWrite(uint8_t pin, bool value) {
    return writeGPIOValue(pin, value);
}

bool LinuxSPI::digitalRead(uint8_t pin) {
    return readGPIOValue(pin);
}

bool LinuxSPI::pinMode(uint8_t pin, uint8_t mode) {
    std::string direction;
    switch (mode) {
        case INPUT:
            direction = "in";
            break;
        case OUTPUT:
            direction = "out";
            break;
        case INPUT_PULLUP:
            // En Linux, los pull-up se configuran diferente
            // aquí simulamos configurando como entrada normal
            direction = "in";
            break;
        default:
            std::cerr << "Error: Modo de pin no válido" << std::endl;
            return false;
    }

    return setGPIODirection(pin, direction);
}

bool LinuxSPI::setInterruptCallback(InterruptCallback callback) {
    interruptCallback = callback;
    return true;
}

bool LinuxSPI::enableInterrupt(bool enable) {
#ifdef __linux__
    if (enable && !interrupt_running) {
        // Verificamos que tenemos una callback y un pin configurado
        if (!interruptCallback || interrupt_pin < 0) {
            std::cerr << "Error: Callback o pin de interrupción no configurado" << std::endl;
            return false;
        }

        // Verificamos que el pin esté configurado como entrada
        if (gpio_pin_paths.find(interrupt_pin) == gpio_pin_paths.end()) {
            std::cerr << "Error: Pin de interrupción no configurado como GPIO" << std::endl;
            return false;
        }

        // Configurar edge para interrupciones
        std::string edge_path = gpio_pin_paths[interrupt_pin] + "/edge";
        std::ofstream edgeFile(edge_path);
        if (!edgeFile.is_open()) {
            std::cerr << "Error: No se puede configurar edge para interrupciones" << std::endl;
            return false;
        }
        edgeFile << "rising";  // Podríamos hacerlo configurable
        edgeFile.close();

        // Iniciar hilo de monitoreo
        interrupt_running = true;
        interrupt_thread = std::thread(&LinuxSPI::interruptThread, this);
        return true;
    } else if (!enable && interrupt_running) {
        // Detener hilo de monitoreo
        interrupt_running = false;
        if (interrupt_thread.joinable()) {
            interrupt_thread.join();
        }
        return true;
    }
    
    return true;  // Si ya estaba en el estado deseado
#else
    return false;
#endif
}

void LinuxSPI::interruptThread() {
#ifdef __linux__
    std::string value_path = gpio_pin_paths[interrupt_pin] + "/value";
    
    while (interrupt_running) {
        // Aquí idealmente usaríamos poll() o epoll() para esperar interrupciones
        // de forma eficiente, pero por simplicidad usamos polling
        
        if (readGPIOValue(interrupt_pin)) {
            // Llamar a la callback
            if (interruptCallback) {
                interruptCallback();
            }
            
            // Esperar un poco para evitar rebotes
            usleep(50000);  // 50ms
        }
        
        // Esperar un tiempo corto antes de volver a verificar
        usleep(10000);  // 10ms
    }
#endif
}
