#include "AES-CMAC.hpp"
#include <openssl/evp.h>
#include <stdexcept>
#include <iostream>
#include <iomanip>

std::array<uint8_t, 16> AESCMAC::calculate(const std::vector<uint8_t>& message, 
                                          const std::array<uint8_t, 16>& key) {
    // 1. Generar subkeys K1 y K2
    std::array<uint8_t, 16> k1, k2;
    generate_subkey(key, k1, k2);

    // 2. Calcular número de bloques
    size_t n = (message.size() + 15) / 16;
    bool last_block_complete = (message.size() % 16) == 0;

    // 3. Inicializar X0 a cero
    std::array<uint8_t, 16> x = {0};
    std::array<uint8_t, 16> y;

    // 4. Procesar bloques completos excepto el último
    for(size_t i = 0; i < n-1; i++) {
        // XOR con el mensaje
        for(size_t j = 0; j < 16; j++) {
            y[j] = message[i*16 + j] ^ x[j];
        }
        // Cifrar
        aes_encrypt(y.data(), key.data(), x.data());
    }

    // 5. Procesar último bloque
    std::array<uint8_t, 16> last_block = {0};
    size_t last_block_size = message.size() - (n-1)*16;
    std::copy(message.end() - last_block_size, message.end(), last_block.begin());

    // Si el último bloque está incompleto, aplicar padding y usar K2
    if (!last_block_complete) {
        last_block[last_block_size] = 0x80; // Padding
        for(size_t i = 0; i < 16; i++) {
            y[i] = last_block[i] ^ k2[i] ^ x[i];
        }
    } else {
        // Si está completo, usar K1
        for(size_t i = 0; i < 16; i++) {
            y[i] = last_block[i] ^ k1[i] ^ x[i];
        }
    }

    // 6. Último cifrado
    std::array<uint8_t, 16> cmac;
    aes_encrypt(y.data(), key.data(), cmac.data());

    return cmac;
}

void AESCMAC::generate_subkey(const std::array<uint8_t, 16>& key,
                             std::array<uint8_t, 16>& k1,
                             std::array<uint8_t, 16>& k2) {
    // Constante Rb para AES-CMAC
    static const uint8_t const_rb = 0x87;

    // L = AES-128(key, 0^128)
    std::array<uint8_t, 16> L = {0};
    std::array<uint8_t, 16> temp = {0};
    
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) {
        throw std::runtime_error("Error creating EVP context");
    }

    int outlen;
    if (EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, key.data(), nullptr) != 1 ||
        EVP_CIPHER_CTX_set_padding(ctx, 0) != 1 ||
        EVP_EncryptUpdate(ctx, temp.data(), &outlen, L.data(), 16) != 1) {
        EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Error generating subkey");
    }
    
    L = temp;
    EVP_CIPHER_CTX_free(ctx);

    // K1 = L << 1 if MSB(L) = 0
    // K1 = (L << 1) XOR Rb if MSB(L) = 1
    bool msb = (L[0] & 0x80) != 0;
    left_shift(L.data(), k1.data());
    if(msb) {
        k1[15] ^= const_rb;
    }

    // K2 = K1 << 1 if MSB(K1) = 0
    // K2 = (K1 << 1) XOR Rb if MSB(K1) = 1
    msb = (k1[0] & 0x80) != 0;
    left_shift(k1.data(), k2.data());
    if(msb) {
        k2[15] ^= const_rb;
    }
}

void AESCMAC::left_shift(uint8_t* input, uint8_t* output) {
    uint8_t overflow = 0;
    
    for(int i = 15; i >= 0; i--) {
        output[i] = (input[i] << 1) | overflow;
        overflow = (input[i] & 0x80) ? 1 : 0;
    }
}

void AESCMAC::xor_block(uint8_t* output, const uint8_t* input, size_t len) {
    for(size_t i = 0; i < len; i++) {
        output[i] ^= input[i];
    }
}

void AESCMAC::aes_encrypt(uint8_t* input, const uint8_t* key, uint8_t* output) {
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    int outlen;
    
    if (!ctx ||
        EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, key, nullptr) != 1 ||
        EVP_CIPHER_CTX_set_padding(ctx, 0) != 1 ||
        EVP_EncryptUpdate(ctx, output, &outlen, input, 16) != 1) {
        if (ctx) EVP_CIPHER_CTX_free(ctx);
        throw std::runtime_error("Error in AES encryption");
    }
    
    EVP_CIPHER_CTX_free(ctx);
}

void AESCMAC::test_encrypt_block(const std::array<uint8_t, 16> &input,
                                 const std::array<uint8_t, 16> &key,
                                 std::array<uint8_t, 16> &output)
{
    // Mostrar entrada y clave para depuración
    std::cout << "Test encrypt input: ";
    for (const auto &b : input)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    std::cout << std::dec << std::endl;

    std::cout << "Test encrypt key: ";
    for (const auto &b : key)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    std::cout << std::dec << std::endl;

    // Realizar encriptación
    aes_encrypt(const_cast<uint8_t*>(input.data()), key.data(), output.data());

    // Mostrar resultado
    std::cout << "Test encrypt output: ";
    for (const auto &b : output)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
    std::cout << std::dec << std::endl;
}