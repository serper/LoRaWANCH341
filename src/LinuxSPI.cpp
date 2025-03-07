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
    // Open SPI device
    fd = ::open(device_path.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Error: Could not open SPI device: " << device_path << std::endl;
        return false;
    }

    // Configure SPI mode
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        std::cerr << "Error: Could not configure SPI mode" << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    // Configure bits per word (8 bits)
    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Error: Could not configure bits per word" << std::endl;
        ::close(fd);
        fd = -1;
        return false;
    }

    // Configure SPI speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) {
        std::cerr << "Error: Could not configure SPI speed" << std::endl;
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
    // Disable interrupts
    enableInterrupt(false);

#ifdef __linux__
    // Close SPI device
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }

    // Unexport all used GPIO pins
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

    // Prepare output buffer
    std::vector<uint8_t> tx_buffer(total_length, 0);
    std::copy(write_data.begin(), write_data.end(), tx_buffer.begin());

    // Prepare input buffer
    std::vector<uint8_t> rx_buffer(total_length, 0);

    // Configure SPI transfer structure
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

    // Execute SPI transfer
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cerr << "Error: SPI transfer failed" << std::endl;
        return {};
    }

    // Return received data
    return rx_buffer;
#else
    // On non-Linux platforms, return an empty vector
    std::cerr << "Error: Linux SPI not supported on this platform" << std::endl;
    return {};
#endif
}

bool LinuxSPI::exportGPIO(uint8_t pin) {
#ifdef __linux__
    std::ofstream exportFile(gpio_export_path);
    if (!exportFile.is_open()) {
        std::cerr << "Error: Unable to open GPIO export file" << std::endl;
        return false;
    }

    exportFile << pin;
    exportFile.close();

    // Wait a moment for the system to create the files
    usleep(100000); // 100ms

    exportFile << pin;
    exportFile.close();


    usleep(100000); // 100ms


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
        std::cerr << "Error: Unable to open GPIO unexport file" << std::endl;
        return false;
    }

    unexportFile << pin;
    unexportFile.close();

    // Remove pin's path from the map
    gpio_pin_paths.erase(pin);

    return true;
#else
    return false;
#endif
}

bool LinuxSPI::setGPIODirection(uint8_t pin, const std::string& direction) {
#ifdef __linux__
    // Verify if pin is exported
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        if (!exportGPIO(pin)) {
            return false;
        }
    }

    // Open direction file
    std::string direction_path = gpio_pin_paths[pin] + "/direction";
    std::ofstream directionFile(direction_path);
    if (!directionFile.is_open()) {
        std::cerr << "Error: Unable to open GPIO direction file for pin " 
                  << static_cast<int>(pin) << std::endl;
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
    // Verify if pin is exported
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        std::cerr << "Error: Pin " << static_cast<int>(pin) << " not exported" << std::endl;
        return false;
    }

    // Open value file
    std::string value_path = gpio_pin_paths[pin] + "/value";
    std::ofstream valueFile(value_path);
    if (!valueFile.is_open()) {
        std::cerr << "Error: Unable to open GPIO value file for pin " << static_cast<int>(pin) << std::endl;
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
    // Verify if pin is exported
    if (gpio_pin_paths.find(pin) == gpio_pin_paths.end()) {
        std::cerr << "Error: Pin " << static_cast<int>(pin) << " not exported" << std::endl;
        return false;
    }

    // Open value file
    std::string value_path = gpio_pin_paths[pin] + "/value";
    std::ifstream valueFile(value_path);
    if (!valueFile.is_open()) {
        std::cerr << "Error: Unable to open GPIO value file for pin " << static_cast<int>(pin) << std::endl;
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
            // In Linux, pull-ups are configured differently
            // here we simulate configuring as a normal input
            direction = "in";
            break;
        default:
            std::cerr << "Error: Invalid pin mode" << std::endl;
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
        // Verify that we have a callback and a pin configured
        if (!interruptCallback || interrupt_pin < 0) {
            std::cerr << "Error: Callback or interrupt pin not configured" << std::endl;
            return false;
        }

        // Verify that the pin is configured as input
        if (gpio_pin_paths.find(interrupt_pin) == gpio_pin_paths.end()) {
            std::cerr << "Error: Interrupt pin not configured as GPIO" << std::endl;
            return false;
        }

        // Configure edge for interrupts
        std::string edge_path = gpio_pin_paths[interrupt_pin] + "/edge";
        std::ofstream edgeFile(edge_path);
        if (!edgeFile.is_open()) {
            std::cerr << "Error: Unable to configure edge for interrupts" << std::endl;
            return false;
        }
        edgeFile << "rising";  // Could be configurable
        edgeFile.close();

        // Start monitoring thread
        interrupt_running = true;
        interrupt_thread = std::thread(&LinuxSPI::interruptThread, this);
        return true;
    } else if (!enable && interrupt_running) {
        // Stop monitoring thread
        interrupt_running = false;
        if (interrupt_thread.joinable()) {
            interrupt_thread.join();
        }
        return true;
    }
    
    return true;  // If already in the desired state
#else
    return false;
#endif
}

void LinuxSPI::interruptThread() {
#ifdef __linux__
    std::string value_path = gpio_pin_paths[interrupt_pin] + "/value";
    
    while (interrupt_running) {
        // Ideally, we would use poll() or epoll() to wait for interrupts
        // efficiently, but for simplicity, we use polling
        
        if (readGPIOValue(interrupt_pin)) {
            // Call the callback
            if (interruptCallback) {
                interruptCallback();
            }
            
            // Wait a bit to avoid bouncing
            usleep(50000);  // 50ms
        }
        
        // Wait a short time before checking again
        usleep(10000);  // 10ms
    }
#endif
}
