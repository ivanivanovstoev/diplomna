#include <cstdint>
#include <cstdlib>
void crc32(uint32_t &orig_crc, const void *buf, size_t size);
void crc32c(uint32_t &orig_crc, const void *buf, size_t size);
// This is a  reference  to function
extern void (&Crc32c)(uint32_t &orig_crc, const void *buf, size_t size);

// #if defined(_WIN32) || defined(_WIN64)
// #define Crc32c crc32c
// #else
// #define Crc32c crc32ck
// void crc32ck(uint32_t &orig_crc, const void *buf, size_t size);
// #endif
