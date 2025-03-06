/** 
 * AES-CMAC Implementation
 * This class provides methods to calculate AES-CMAC and perform AES encryption.
 */
#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include <array>

    
class AESCMAC {
public:
/***
 * @brief Calculate AES-CMAC for a given message and key
 */
static std::array<uint8_t, 16> calculate(const std::vector<uint8_t>& message, 
                                        const std::array<uint8_t, 16>& key);

/***
 * @brief Calculate AES-CMAC for a given message and key
 */
static void aes_encrypt(uint8_t* input, const uint8_t* key, uint8_t* output);

/***
 * @brief Test AES encryption on a single block
 * @param input The input block to be encrypted
 * @param key The encryption key
 * @param output The encrypted output block
 */
    void test_encrypt_block(const std::array<uint8_t, 16> &input,
                                 const std::array<uint8_t, 16> &key,
                                 std::array<uint8_t, 16> &output);

private:
    static void generate_subkey(const std::array<uint8_t, 16> &key,
                                          std::array<uint8_t, 16> &k1,
                                          std::array<uint8_t, 16> &k2);
    static void xor_block(uint8_t* output, const uint8_t* input, size_t len);
    static void left_shift(uint8_t* input, uint8_t* output);
};
