/**
 * @file ConfigManager.hpp
 * @brief Header file for the ConfigManager class.
 *
 * This file contains the declaration of the ConfigManager class, which is responsible for
 * loading, saving, and managing configuration settings stored in a JSON file.
 */

#pragma once

#include <string>
#include <map>
#include <fstream>
#include <cjson/cJSON.h>

/**
 * @class ConfigManager
 * @brief Manages configuration settings stored in a JSON file.
 */
class ConfigManager {
public:
    /**
     * @brief Constructs a ConfigManager object.
     * @param configFile The path to the configuration file. Defaults to "config.json".
     */
    ConfigManager(const std::string& configFile = "config.json");

    /**
     * @brief Destructs the ConfigManager object.
     */
    ~ConfigManager();

    /**
     * @brief Loads the configuration from the JSON file.
     * @return True if the configuration was successfully loaded, false otherwise.
     */
    bool loadConfig();

    /**
     * @brief Retrieves a string value from the configuration.
     * @param key The key of the configuration setting.
     * @param defaultValue The default value to return if the key is not found. Defaults to an empty string.
     * @return The string value associated with the key, or the default value if the key is not found.
     */
    std::string getString(const std::string& key, const std::string& defaultValue = "");

    /**
     * @brief Retrieves an integer value from the configuration.
     * @param key The key of the configuration setting.
     * @param defaultValue The default value to return if the key is not found. Defaults to 0.
     * @return The integer value associated with the key, or the default value if the key is not found.
     */
    int getInt(const std::string& key, int defaultValue = 0);

    /**
     * @brief Retrieves a boolean value from the configuration.
     * @param key The key of the configuration setting.
     * @param defaultValue The default value to return if the key is not found. Defaults to false.
     * @return The boolean value associated with the key, or the default value if the key is not found.
     */
    bool getBool(const std::string& key, bool defaultValue = false);

    /**
     * @brief Retrieves a nested string value from the configuration.
     * @param path The path to the nested configuration setting.
     * @param defaultValue The default value to return if the path is not found. Defaults to an empty string.
     * @return The string value associated with the path, or the default value if the path is not found.
     */
    std::string getNestedString(const std::string& path, const std::string& defaultValue = "");

    /**
     * @brief Retrieves a nested integer value from the configuration.
     * @param path The path to the nested configuration setting.
     * @param defaultValue The default value to return if the path is not found. Defaults to 0.
     * @return The integer value associated with the path, or the default value if the path is not found.
     */
    int getNestedInt(const std::string& path, int defaultValue = 0);

    /**
     * @brief Retrieves a nested boolean value from the configuration.
     * @param path The path to the nested configuration setting.
     * @param defaultValue The default value to return if the path is not found. Defaults to false.
     * @return The boolean value associated with the path, or the default value if the path is not found.
     */
    bool getNestedBool(const std::string& path, bool defaultValue = false);

    /**
     * @brief Saves the current configuration to the JSON file.
     * @return True if the configuration was successfully saved, false otherwise.
     */
    bool saveConfig();

    /**
     * @brief Sets a string value in the configuration.
     * @param key The key of the configuration setting.
     * @param value The string value to set.
     */
    void setString(const std::string& key, const std::string& value);

    /**
     * @brief Sets an integer value in the configuration.
     * @param key The key of the configuration setting.
     * @param value The integer value to set.
     */
    void setInt(const std::string& key, int value);

    /**
     * @brief Sets a boolean value in the configuration.
     * @param key The key of the configuration setting.
     * @param value The boolean value to set.
     */
    void setBool(const std::string& key, bool value);

private:
    std::string configFilePath; ///< The path to the configuration file.
    cJSON* root; ///< The root JSON object representing the configuration.

    /**
     * @brief Retrieves a nested JSON item from the configuration.
     * @param path The path to the nested JSON item.
     * @return A pointer to the nested JSON item, or nullptr if the item is not found.
     */
    cJSON* getNestedItem(const std::string& path);

    /**
     * @brief Cleans up the JSON object.
     */
    void cleanupJSON();
};
