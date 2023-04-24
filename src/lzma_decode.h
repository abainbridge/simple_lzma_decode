#pragma once

#include <stdint.h>

typedef enum {
    LZMA_OK,
    LZMA_ERROR_DATA,
    LZMA_ERROR_UNSUPPORTED,
    LZMA_ERROR_INPUT_EOF,
    LZMA_ERROR_MAYBE_FINISHED_WITHOUT_MARK
} LzmaRes;

// On success, a buffer for the decompressed data will be allocated and returned in resultBuf.
// The length of the buffer will be returned in resultLen.
LzmaRes LzmaDecode(const uint8_t *src, size_t srcLen, uint8_t **resultBuf, size_t *resultLen);
