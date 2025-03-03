cmake_minimum_required(VERSION 3.10)
project(LoRaWANCH341 CXX)

# Add MSYS2 specific paths
if(MINGW)
    list(APPEND CMAKE_PREFIX_PATH 
        "C:/msys64/mingw64"
        "C:/msys64/mingw64/lib/cmake"
    )
endif()

# Find packages
find_package(PkgConfig REQUIRED)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
else()
    message(FATAL_ERROR "pkg-config not found!")
endif()

find_package(OpenSSL REQUIRED)
if(OPENSSL_FOUND)
    message(STATUS "OpenSSL Found: ${OPENSSL_VERSION}")
    message(STATUS "OpenSSL Include Dir: ${OPENSSL_INCLUDE_DIR}")
    message(STATUS "OpenSSL Libraries: ${OPENSSL_LIBRARIES}")
endif()

# Set C++ standard and flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Add debug symbols
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Add source files
set(SOURCES
    src/main.cpp
    src/AES-CMAC.cpp
    src/CH341SPI.cpp  
    src/LoRaWAN.cpp  
    src/RFM95.cpp
    src/SessionManager.cpp
    src/SPIFactory.cpp
    src/LinuxSPI.cpp
    src/ConfigManager.cpp  # Añadir esta línea
)

# Find required packages
find_package(Threads REQUIRED)
find_package(cJSON REQUIRED)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    PRIVATE
        ${CJSON_LIBRARIES}
        Threads::Threads
        ${LIBUSB_LIBRARIES}
        OpenSSL::Crypto
)

# Add include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CJSON_INCLUDE_DIRS}
        ${LIBUSB_INCLUDE_DIRS}
        ${OPENSSL_INCLUDE_DIR}
)

# Print configuration for debugging
message(STATUS "LIBUSB_FOUND: ${LIBUSB_FOUND}")
message(STATUS "LIBUSB_INCLUDE_DIRS: ${LIBUSB_INCLUDE_DIRS}")
message(STATUS "LIBUSB_LIBRARIES: ${LIBUSB_LIBRARIES}")
message(STATUS "CJSON_LIBRARIES: ${CJSON_LIBRARIES}")
message(STATUS "CJSON_INCLUDE_DIRS: ${CJSON_INCLUDE_DIRS}")