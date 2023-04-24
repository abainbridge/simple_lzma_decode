#include "lzma_decode.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char ** argv) {
    char* ifname = "../test.bmp.lzma";
    char* ofname = "../test.bmp";

    FILE* inFile = fopen(ifname, "rb");
    fseek(inFile, 0, SEEK_END);
    size_t inFileSize = ftell(inFile);
    fseek(inFile, 0, SEEK_SET);

    char *inBuf = malloc(inFileSize);
    fread(inBuf, 1, inFileSize, inFile);

    size_t destLen;
    uint8_t *dest;
    LzmaRes err = LzmaDecode(inBuf, inFileSize, &dest, &destLen);

    FILE* outFile = fopen(ofname, "wb");
    fwrite(dest, 1, destLen, outFile);
}
