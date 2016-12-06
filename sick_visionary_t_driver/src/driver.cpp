#include <ros/ros.h>
#include <sick_visionary_t_driver/driver.h>

#ifdef OPENSSL_ENABLED
#include <openssl/md5.h>
df
uint32_t Driver_3DCS::calculatePasswordHash(const std::string &strPassword)
{
    unsigned char dig[MD5_DIGEST_LENGTH];
    MD5((unsigned char*)strPassword.c_str(), strPassword.size(), dig);
    
    // 128 bit to 32 bit by XOR 
    uint8_t byte0 = dig[0] ^ dig[4] ^ dig[8]  ^ dig[12];
    uint8_t byte1 = dig[1] ^ dig[5] ^ dig[9]  ^ dig[13]; 
    uint8_t byte2 = dig[2] ^ dig[6] ^ dig[10] ^ dig[14]; 
    uint8_t byte3 = dig[3] ^ dig[7] ^ dig[11] ^ dig[15]; 
    
    return byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24);
}
#endif
