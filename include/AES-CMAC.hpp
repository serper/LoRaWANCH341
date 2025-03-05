#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>
#include <array>

class AESCMAC {
public:
    static std::array<uint8_t, 16> calculate(const std::vector<uint8_t>& message, 
                                           const std::array<uint8_t, 16>& key);
    static void aes_encrypt(uint8_t* input, const uint8_t* key, uint8_t* output);

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
