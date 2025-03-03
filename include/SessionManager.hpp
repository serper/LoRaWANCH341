#pragma once

#include <string>
#include <array>
#include <vector>
#include <cstdint>

class SessionManager {
public:
    struct SessionData {
        std::array<uint8_t, 4> devAddr;
        std::array<uint8_t, 16> nwkSKey;
        std::array<uint8_t, 16> appSKey;
        uint32_t uplinkCounter;
        uint32_t downlinkCounter;
        uint16_t lastDevNonce;  // Añadir último DevNonce usado
        std::vector<uint16_t> usedNonces;  // Añadir historial de nonces
        bool joined;
    };

    static bool saveSession(const std::string& filename, const SessionData& data);
    static bool loadSession(const std::string& filename, SessionData& data);
    static void clearSession(const std::string& filename);
};
