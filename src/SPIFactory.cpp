#include "SPIInterface.hpp"
#include "CH341SPI.hpp"
#include "LinuxSPI.hpp"
#include <memory>

std::unique_ptr<SPIInterface> SPIFactory::createCH341SPI(int device_index, bool lsb_first) {
    return std::make_unique<CH341SPI>(device_index, lsb_first);
}

std::unique_ptr<SPIInterface> SPIFactory::createLinuxSPI(const std::string& device, 
                                                       uint32_t speed, 
                                                       uint8_t mode) {
    return std::make_unique<LinuxSPI>(device, speed, mode);
}
