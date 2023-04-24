# simple_lzma_decode
Simplified version of Igor Pavlov's LZMA decoder.

Igor's original code is public domain and so are my changes.

The changes were approximately:
1. Remove everything that is not needed for the single-shot interface. That's the interface were you give a memory buffer containing a single, complete LZMA file, and get back a malloc'd buffer containing the complete decompressed data.
2. Remove everything that could be removed by assuming the compiler supports C99.

## IMPORTANT NOTE

If you generate an LZMA file using the lzma binary from XZ Utils 5.27, and possibly all other versions, the code here will not be able to decode it. The problem is that XZ Utils does not populate the "decompressed length" field in the LZMA header. Or rather it sets it to all 0xFF. As a result, the code does not know how big a buffer to allocate. I don't think there's any simple and efficient way to decompress such files. The only solution I know is to modify the LZMA file to add the decompressed length.
