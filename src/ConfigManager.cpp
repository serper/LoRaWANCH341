#include "ConfigManager.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// Utility function to split a string by a delimiter
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    
    while (getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

ConfigManager::ConfigManager(const std::string& configFile)
    : configFilePath(configFile), root(nullptr)
{
    loadConfig();
}

ConfigManager::~ConfigManager() {
    cleanupJSON();
}

void ConfigManager::cleanupJSON() {
    if (root != nullptr) {
        cJSON_Delete(root);
        root = nullptr;
    }
}

bool ConfigManager::loadConfig() {
    cleanupJSON();
    
    // Intentar abrir el archivo de configuración
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        std::cerr << "No se pudo abrir el archivo de configuración: " << configFilePath << std::endl;
        return false;
    }
    
    // Leer el archivo completo a un string
    std::stringstream buffer;
    buffer << configFile.rdbuf();
    std::string jsonStr = buffer.str();
    configFile.close();
    
    // Parsear el JSON
    root = cJSON_Parse(jsonStr.c_str());
    if (root == nullptr) {
        const char* errorPtr = cJSON_GetErrorPtr();
        std::string errorMsg = errorPtr != nullptr ? errorPtr : "Unknown error";
        std::cerr << "Error al parsear JSON: " << errorMsg << std::endl;
        return false;
    }
    
    return true;
}

bool ConfigManager::saveConfig() {
    if (root == nullptr) {
        std::cerr << "No hay configuración para guardar" << std::endl;
        return false;
    }
    
    // Convertir el objeto cJSON a string formateado
    char* jsonStr = cJSON_Print(root);
    if (jsonStr == nullptr) {
        std::cerr << "Error al convertir configuración a string JSON" << std::endl;
        return false;
    }
    
    // Guardar en archivo
    std::ofstream configFile(configFilePath);
    if (!configFile.is_open()) {
        std::cerr << "No se pudo abrir el archivo para escritura: " << configFilePath << std::endl;
        free(jsonStr);
        return false;
    }
    
    configFile << jsonStr;
    configFile.close();
    free(jsonStr);
    
    return true;
}

std::string ConfigManager::getString(const std::string& key, const std::string& defaultValue) {
    if (root == nullptr) {
        return defaultValue;
    }
    
    cJSON* item = cJSON_GetObjectItem(root, key.c_str());
    if (item == nullptr || !cJSON_IsString(item)) {
        return defaultValue;
    }
    
    return item->valuestring;
}

int ConfigManager::getInt(const std::string& key, int defaultValue) {
    if (root == nullptr) {
        return defaultValue;
    }
    
    cJSON* item = cJSON_GetObjectItem(root, key.c_str());
    if (item == nullptr || !cJSON_IsNumber(item)) {
        return defaultValue;
    }
    
    return item->valueint;
}

bool ConfigManager::getBool(const std::string& key, bool defaultValue) {
    if (root == nullptr) {
        return defaultValue;
    }
    
    cJSON* item = cJSON_GetObjectItem(root, key.c_str());
    if (item == nullptr || !cJSON_IsBool(item)) {
        return defaultValue;
    }
    
    return cJSON_IsTrue(item);
}

cJSON* ConfigManager::getNestedItem(const std::string& path) {
    if (root == nullptr) {
        return nullptr;
    }
    
    auto parts = split(path, '.');
    if (parts.empty()) {
        return nullptr;
    }
    
    cJSON* current = root;
    
    for (size_t i = 0; i < parts.size() - 1; ++i) {
        current = cJSON_GetObjectItem(current, parts[i].c_str());
        if (current == nullptr) {
            return nullptr;
        }
    }
    
    return cJSON_GetObjectItem(current, parts.back().c_str());
}

std::string ConfigManager::getNestedString(const std::string& path, const std::string& defaultValue) {
    cJSON* item = getNestedItem(path);
    if (item == nullptr || !cJSON_IsString(item)) {
        return defaultValue;
    }
    
    return item->valuestring;
}

int ConfigManager::getNestedInt(const std::string& path, int defaultValue) {
    cJSON* item = getNestedItem(path);
    if (item == nullptr || !cJSON_IsNumber(item)) {
        return defaultValue;
    }
    
    return item->valueint;
}

bool ConfigManager::getNestedBool(const std::string& path, bool defaultValue) {
    cJSON* item = getNestedItem(path);
    if (item == nullptr || !cJSON_IsBool(item)) {
        return defaultValue;
    }
    
    return cJSON_IsTrue(item);
}

void ConfigManager::setString(const std::string& key, const std::string& value) {
    if (root == nullptr) {
        root = cJSON_CreateObject();
    }
    
    cJSON_DeleteItemFromObject(root, key.c_str());
    cJSON_AddStringToObject(root, key.c_str(), value.c_str());
}

void ConfigManager::setInt(const std::string& key, int value) {
    if (root == nullptr) {
        root = cJSON_CreateObject();
    }
    
    cJSON_DeleteItemFromObject(root, key.c_str());
    cJSON_AddNumberToObject(root, key.c_str(), value);
}

void ConfigManager::setBool(const std::string& key, bool value) {
    if (root == nullptr) {
        root = cJSON_CreateObject();
    }
    
    cJSON_DeleteItemFromObject(root, key.c_str());
    cJSON_AddBoolToObject(root, key.c_str(), value);
}
