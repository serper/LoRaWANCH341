#pragma once

#include <string>
#include <map>
#include <fstream>
#include <cjson/cJSON.h>

class ConfigManager {
public:
    ConfigManager(const std::string& configFile = "config.json");
    ~ConfigManager();

    bool loadConfig();
    std::string getString(const std::string& key, const std::string& defaultValue = "");
    int getInt(const std::string& key, int defaultValue = 0);
    bool getBool(const std::string& key, bool defaultValue = false);
    
    // Para claves anidadas como "device.devEUI", etc.
    std::string getNestedString(const std::string& path, const std::string& defaultValue = "");
    int getNestedInt(const std::string& path, int defaultValue = 0);
    bool getNestedBool(const std::string& path, bool defaultValue = false);

    // Guardar la configuración
    bool saveConfig();
    void setString(const std::string& key, const std::string& value);
    void setInt(const std::string& key, int value);
    void setBool(const std::string& key, bool value);

private:
    std::string configFilePath;
    cJSON* root;
    
    cJSON* getNestedItem(const std::string& path);
    void cleanupJSON();
};
