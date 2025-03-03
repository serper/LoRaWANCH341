#include "SessionManager.hpp"
#include <cjson/cJSON.h>
#include <fstream>
#include <sstream>
#include <iomanip>

static std::string bytesToHex(const uint8_t* data, size_t len) {
    std::stringstream ss;
    for(size_t i = 0; i < len; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(data[i]);
    }
    return ss.str();
}

static void hexToBytes(const std::string& hex, uint8_t* data, size_t len) {
    for(size_t i = 0; i < len; i++) {
        std::string byteStr = hex.substr(i*2, 2);
        data[i] = std::stoi(byteStr, nullptr, 16);
    }
}

bool SessionManager::saveSession(const std::string& filename, const SessionData& data) {
    cJSON* root = cJSON_CreateObject();
    
    // Convert binary data to hex strings
    // Store devAddr in reverse order
    std::vector<uint8_t> reversedDevAddr(data.devAddr.rbegin(), data.devAddr.rend());
    cJSON_AddStringToObject(root, "devAddr",
        bytesToHex(reversedDevAddr.data(), reversedDevAddr.size()).c_str());
    cJSON_AddStringToObject(root, "nwkSKey", 
        bytesToHex(data.nwkSKey.data(), data.nwkSKey.size()).c_str());
    cJSON_AddStringToObject(root, "appSKey", 
        bytesToHex(data.appSKey.data(), data.appSKey.size()).c_str());
    
    cJSON_AddNumberToObject(root, "uplinkCounter", data.uplinkCounter);
    cJSON_AddNumberToObject(root, "downlinkCounter", data.downlinkCounter);
    cJSON_AddBoolToObject(root, "joined", data.joined);

    char* jsonStr = cJSON_Print(root);
    std::ofstream file(filename);
    if (!file) {
        cJSON_Delete(root);
        free(jsonStr);
        return false;
    }

    file << jsonStr;
    file.close();
    
    cJSON_Delete(root);
    free(jsonStr);
    return true;
}

bool SessionManager::loadSession(const std::string& filename, SessionData& data) {
    std::ifstream file(filename);
    if (!file) {
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonStr = buffer.str();

    cJSON* root = cJSON_Parse(jsonStr.c_str());
    if (!root) {
        return false;
    }

    cJSON* item;
    if ((item = cJSON_GetObjectItem(root, "devAddr"))) {
        // Load devAddr in reverse order
        std::vector<uint8_t> reversedDevAddr(4);
        hexToBytes(item->valuestring, reversedDevAddr.data(), reversedDevAddr.size());
        std::copy(reversedDevAddr.rbegin(), reversedDevAddr.rend(), data.devAddr.begin());
    }
    if ((item = cJSON_GetObjectItem(root, "nwkSKey"))) {
        hexToBytes(item->valuestring, data.nwkSKey.data(), data.nwkSKey.size());
    }
    if ((item = cJSON_GetObjectItem(root, "appSKey"))) {
        hexToBytes(item->valuestring, data.appSKey.data(), data.appSKey.size());
    }
    if ((item = cJSON_GetObjectItem(root, "uplinkCounter"))) {
        data.uplinkCounter = item->valueint;
    }
    if ((item = cJSON_GetObjectItem(root, "downlinkCounter"))) {
        data.downlinkCounter = item->valueint;
    }
    if ((item = cJSON_GetObjectItem(root, "joined"))) {
        data.joined = item->valueint;
    }

    cJSON_Delete(root);
    return true;
}

void SessionManager::clearSession(const std::string& filename) {
    std::remove(filename.c_str());
}
