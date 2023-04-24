// Original author: 2008-11-06 : Igor Pavlov : Public domain
// Simplified for a single use case by Andrew Bainbridge, 2023.

#include "lzma_decode.h"

#include <stdlib.h>
#include <string.h>

#define TRY(x) { int __result__ = (x); if (__result__ != 0) return __result__; }

#define kNumTopBits 24
#define kTopValue (1u << kNumTopBits)

#define kNumBitModelTotalBits 11
#define kBitModelTotal (1 << kNumBitModelTotalBits)
#define kNumMoveBits 5

#define RC_INIT_SIZE 5

#define NORMALIZE if (range < kTopValue) { range <<= 8; code = (code << 8) | (*buf++); }

#define IF_BIT_0(p) ttt = *(p); NORMALIZE; bound = (range >> kNumBitModelTotalBits) * ttt; if (code < bound)
#define UPDATE_0(p) range = bound; *(p) = (Prob)(ttt + ((kBitModelTotal - ttt) >> kNumMoveBits));
#define UPDATE_1(p) range -= bound; code -= bound; *(p) = (Prob)(ttt - (ttt >> kNumMoveBits));
#define GET_BIT2(p, i, A0, A1) IF_BIT_0(p) \
    { UPDATE_0(p); i = (i + i); A0; } else \
    { UPDATE_1(p); i = (i + i) + 1; A1; }
#define GET_BIT(p, i) GET_BIT2(p, i, ; , ;)

#define TREE_GET_BIT(probs, i) { GET_BIT((probs + i), i); }
#define TREE_DECODE(probs, limit, i) \
    { i = 1; do { TREE_GET_BIT(probs, i); } while (i < limit); i -= limit; }

#define TREE_6_DECODE(probs, i) \
    { i = 1; \
    TREE_GET_BIT(probs, i); \
    TREE_GET_BIT(probs, i); \
    TREE_GET_BIT(probs, i); \
    TREE_GET_BIT(probs, i); \
    TREE_GET_BIT(probs, i); \
    TREE_GET_BIT(probs, i); \
    i -= 0x40; }

#define NORMALIZE_CHECK if (range < kTopValue) { if (buf >= bufLimit) return DUMMY_ERROR; range <<= 8; code = (code << 8) | (*buf++); }

#define IF_BIT_0_CHECK(p) ttt = *(p); NORMALIZE_CHECK; bound = (range >> kNumBitModelTotalBits) * ttt; if (code < bound)
#define UPDATE_0_CHECK range = bound;
#define UPDATE_1_CHECK range -= bound; code -= bound;
#define GET_BIT2_CHECK(p, i, A0, A1) IF_BIT_0_CHECK(p) \
    { UPDATE_0_CHECK; i = (i + i); A0; } else \
    { UPDATE_1_CHECK; i = (i + i) + 1; A1; }
#define GET_BIT_CHECK(p, i) GET_BIT2_CHECK(p, i, ; , ;)
#define TREE_DECODE_CHECK(probs, limit, i) \
    { i = 1; do { GET_BIT_CHECK(probs + i, i) } while (i < limit); i -= limit; }


#define kNumPosBitsMax 4
#define kNumPosStatesMax (1 << kNumPosBitsMax)

#define kLenNumLowBits 3
#define kLenNumLowSymbols (1 << kLenNumLowBits)
#define kLenNumMidBits 3
#define kLenNumMidSymbols (1 << kLenNumMidBits)
#define kLenNumHighBits 8
#define kLenNumHighSymbols (1 << kLenNumHighBits)

#define LenChoice 0
#define LenChoice2 (LenChoice + 1)
#define LenLow (LenChoice2 + 1)
#define LenMid (LenLow + (kNumPosStatesMax << kLenNumLowBits))
#define LenHigh (LenMid + (kNumPosStatesMax << kLenNumMidBits))
#define kNumLenProbs (LenHigh + kLenNumHighSymbols)


#define kNumStates 12
#define kNumLitStates 7

#define kStartPosModelIndex 4
#define kEndPosModelIndex 14
#define kNumFullDistances (1 << (kEndPosModelIndex >> 1))

#define kNumPosSlotBits 6
#define kNumLenToPosStates 4

#define kNumAlignBits 4
#define kAlignTableSize (1 << kNumAlignBits)

#define kMatchMinLen 2
#define kMatchSpecLenStart (kMatchMinLen + kLenNumLowSymbols + kLenNumMidSymbols + kLenNumHighSymbols)

#define IsMatch 0
#define IsRep (IsMatch + (kNumStates << kNumPosBitsMax))
#define IsRepG0 (IsRep + kNumStates)
#define IsRepG1 (IsRepG0 + kNumStates)
#define IsRepG2 (IsRepG1 + kNumStates)
#define IsRep0Long (IsRepG2 + kNumStates)
#define PosSlot (IsRep0Long + (kNumStates << kNumPosBitsMax))
#define SpecPos (PosSlot + (kNumLenToPosStates << kNumPosSlotBits))
#define Align (SpecPos + kNumFullDistances - kEndPosModelIndex)
#define LenCoder (Align + kAlignTableSize)
#define RepLenCoder (LenCoder + kNumLenProbs)
#define Literal (RepLenCoder + kNumLenProbs)

#define LZMA_BASE_SIZE 1846
#define LZMA_LIT_SIZE 768

#if Literal != LZMA_BASE_SIZE
StopCompilingDueBUG
#endif

#define Prob uint16_t


typedef struct _Properties {
    unsigned lc, lp, pb;
    uint32_t dicSize;
} Properties;

#define LZMA_REQUIRED_INPUT_MAX 20

typedef struct {
    Properties properties;
    Prob *probs;
    uint8_t *dic;
    const uint8_t *buf;
    uint32_t range, code;
    size_t dicPos;
    size_t dicBufSize;
    uint32_t processedPos;
    uint32_t checkDicSize;
    unsigned state;
    uint32_t reps[4];
    unsigned remainLen;
    int needFlush;
    int needInitState;
    uint32_t numProbs;
    unsigned tempBufSize;
    uint8_t tempBuf[LZMA_REQUIRED_INPUT_MAX];
} Decoder;


static const uint8_t kLiteralNextStates[kNumStates * 2] = {
    0, 0, 0, 0, 1, 2, 3,  4,  5,  6,  4,  5,
    7, 7, 7, 7, 7, 7, 7, 10, 10, 10, 10, 10
};

// Decodes symbols while (buf < bufLimit). At least one symbol will be decoded.
// "buf" is without last normalization.
// Out:
//     p->remainLen:
//         < kMatchSpecLenStart : normal remain
//         = kMatchSpecLenStart : finished
//         = kMatchSpecLenStart + 1 : Flush marker
//         = kMatchSpecLenStart + 2 : State Init Marker
static int DecodeReal(Decoder *d, size_t limit, const uint8_t *bufLimit) {
    Prob *probs = d->probs;

    unsigned state = d->state;
    uint32_t rep0 = d->reps[0], rep1 = d->reps[1], rep2 = d->reps[2], rep3 = d->reps[3];
    unsigned pbMask = (1u << d->properties.pb) - 1;
    unsigned lpMask = (1u << d->properties.lp) - 1;
    unsigned lc = d->properties.lc;

    uint8_t *dic = d->dic;
    size_t dicBufSize = d->dicBufSize;
    size_t dicPos = d->dicPos;

    uint32_t processedPos = d->processedPos;
    uint32_t checkDicSize = d->checkDicSize;
    unsigned len = 0;

    const uint8_t *buf = d->buf;
    uint32_t range = d->range;
    uint32_t code = d->code;

    do {
        uint32_t bound;
        unsigned ttt;
        unsigned posState = processedPos & pbMask;

        Prob *prob = probs + IsMatch + (state << kNumPosBitsMax) + posState;
        IF_BIT_0(prob) {
            unsigned symbol;
            UPDATE_0(prob);
            prob = probs + Literal;
            if (checkDicSize != 0 || processedPos != 0)
                prob += (LZMA_LIT_SIZE * (((processedPos & lpMask) << lc) +
                (dic[(dicPos == 0 ? dicBufSize : dicPos) - 1] >> (8 - lc))));

            if (state < kNumLitStates) {
                symbol = 1;
                do { GET_BIT(prob + symbol, symbol) } while (symbol < 0x100);
            }
            else {
                unsigned matchByte = d->dic[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
                unsigned offs = 0x100;
                symbol = 1;
                do {
                    matchByte <<= 1;
                    unsigned bit = (matchByte & offs);
                    Prob *probLit = prob + offs + bit + symbol;
                    GET_BIT2(probLit, symbol, offs &= ~bit, offs &= bit)
                } while (symbol < 0x100);
            }
            dic[dicPos++] = (uint8_t)symbol;
            processedPos++;

            state = kLiteralNextStates[state];
            continue;
        }
        else {
            UPDATE_1(prob);
            prob = probs + IsRep + state;
            IF_BIT_0(prob) {
                UPDATE_0(prob);
                state += kNumStates;
                prob = probs + LenCoder;
            }
            else {
                UPDATE_1(prob);
                if (checkDicSize == 0 && processedPos == 0)
                    return LZMA_ERROR_DATA;
                prob = probs + IsRepG0 + state;
                IF_BIT_0(prob) {
                    UPDATE_0(prob);
                    prob = probs + IsRep0Long + (state << kNumPosBitsMax) + posState;
                    IF_BIT_0(prob) {
                        UPDATE_0(prob);
                        dic[dicPos] = dic[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
                        dicPos++;
                        processedPos++;
                        state = state < kNumLitStates ? 9 : 11;
                        continue;
                    }
                    UPDATE_1(prob);
                }
                else {
                    uint32_t distance;
                    UPDATE_1(prob);
                    prob = probs + IsRepG1 + state;
                    IF_BIT_0(prob) {
                        UPDATE_0(prob);
                        distance = rep1;
                    }
                    else {
                        UPDATE_1(prob);
                        prob = probs + IsRepG2 + state;
                        IF_BIT_0(prob) {
                            UPDATE_0(prob);
                            distance = rep2;
                        }
                        else {
                            UPDATE_1(prob);
                            distance = rep3;
                            rep3 = rep2;
                        }
                        rep2 = rep1;
                    }
                    rep1 = rep0;
                    rep0 = distance;
                }
                state = state < kNumLitStates ? 8 : 11;
                prob = probs + RepLenCoder;
            }
            {
                unsigned limit, offset;
                Prob *probLen = prob + LenChoice;
                IF_BIT_0(probLen) {
                    UPDATE_0(probLen);
                    probLen = prob + LenLow + (posState << kLenNumLowBits);
                    offset = 0;
                    limit = (1 << kLenNumLowBits);
                }
                else {
                    UPDATE_1(probLen);
                    probLen = prob + LenChoice2;
                    IF_BIT_0(probLen) {
                        UPDATE_0(probLen);
                        probLen = prob + LenMid + (posState << kLenNumMidBits);
                        offset = kLenNumLowSymbols;
                        limit = (1 << kLenNumMidBits);
                    }
                    else {
                        UPDATE_1(probLen);
                        probLen = prob + LenHigh;
                        offset = kLenNumLowSymbols + kLenNumMidSymbols;
                        limit = (1 << kLenNumHighBits);
                    }
                }
                TREE_DECODE(probLen, limit, len);
                len += offset;
            }

            if (state >= kNumStates) {
                uint32_t distance;
                prob = probs + PosSlot +
                        ((len < kNumLenToPosStates ? len : kNumLenToPosStates - 1) << kNumPosSlotBits);
                TREE_6_DECODE(prob, distance);
                if (distance >= kStartPosModelIndex) {
                    unsigned posSlot = (unsigned)distance;
                    int numDirectBits = (int)(((distance >> 1) - 1));
                    distance = (2 | (distance & 1));
                    if (posSlot < kEndPosModelIndex) {
                        distance <<= numDirectBits;
                        prob = probs + SpecPos + distance - posSlot - 1;
                        {
                            uint32_t mask = 1;
                            unsigned i = 1;
                            do {
                                GET_BIT2(prob + i, i, ; , distance |= mask);
                                mask <<= 1;
                            } while (--numDirectBits != 0);
                        }
                    }
                    else {
                        numDirectBits -= kNumAlignBits;
                        do {
                            NORMALIZE
                            range >>= 1;

                            {
                                code -= range;
                                uint32_t t = (0 - ((uint32_t)code >> 31)); /* (uint32_t)((Int32)code >> 31) */
                                distance = (distance << 1) + (t + 1);
                                code += range & t;
                            }
                        } while (--numDirectBits != 0);
                        prob = probs + Align;
                        distance <<= kNumAlignBits;
                        {
                            unsigned i = 1;
                            GET_BIT2(prob + i, i, ; , distance |= 1);
                            GET_BIT2(prob + i, i, ; , distance |= 2);
                            GET_BIT2(prob + i, i, ; , distance |= 4);
                            GET_BIT2(prob + i, i, ; , distance |= 8);
                        }
                        if (distance == 0xFFFFFFFFu) {
                            len += kMatchSpecLenStart;
                            state -= kNumStates;
                            break;
                        }
                    }
                }
                rep3 = rep2;
                rep2 = rep1;
                rep1 = rep0;
                rep0 = distance + 1;
                if (checkDicSize == 0) {
                    if (distance >= processedPos)
                        return LZMA_ERROR_DATA;
                }
                else if (distance >= checkDicSize)
                    return LZMA_ERROR_DATA;
                state = (state < kNumStates + kNumLitStates) ? kNumLitStates : kNumLitStates + 3;
            }

            len += kMatchMinLen;

            if (limit == dicPos)
                return LZMA_ERROR_DATA;
            {
                size_t rem = limit - dicPos;
                unsigned curLen = ((rem < len) ? (unsigned)rem : len);
                size_t pos = (dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0);

                processedPos += curLen;

                len -= curLen;
                if (pos + curLen <= dicBufSize) {
                    uint8_t *dest = dic + dicPos;
                    ptrdiff_t src = (ptrdiff_t)pos - (ptrdiff_t)dicPos;
                    const uint8_t *lim = dest + curLen;
                    dicPos += curLen;
                    do {
                        *dest = (uint8_t)*(dest + src);
                    } while (++dest != lim);
                }
                else {
                    do {
                        dic[dicPos++] = dic[pos];
                        if (++pos == dicBufSize)
                            pos = 0;
                    } while (--curLen != 0);
                }
            }
        }
    } while (dicPos < limit && buf < bufLimit);
    NORMALIZE;
    d->buf = buf;
    d->range = range;
    d->code = code;
    d->remainLen = len;
    d->dicPos = dicPos;
    d->processedPos = processedPos;
    d->reps[0] = rep0;
    d->reps[1] = rep1;
    d->reps[2] = rep2;
    d->reps[3] = rep3;
    d->state = state;

    return LZMA_OK;
}

static void WriteRem(Decoder *d, size_t limit) {
    if (d->remainLen != 0 && d->remainLen < kMatchSpecLenStart) {
        uint8_t *dic = d->dic;
        size_t dicPos = d->dicPos;
        size_t dicBufSize = d->dicBufSize;
        unsigned len = d->remainLen;
        uint32_t rep0 = d->reps[0];
        if (limit - dicPos < len)
            len = (unsigned)(limit - dicPos);

        if (d->checkDicSize == 0 && d->properties.dicSize - d->processedPos <= len)
            d->checkDicSize = d->properties.dicSize;

        d->processedPos += len;
        d->remainLen -= len;
        while (len-- != 0) {
            dic[dicPos] = dic[(dicPos - rep0) + ((dicPos < rep0) ? dicBufSize : 0)];
            dicPos++;
        }
        d->dicPos = dicPos;
    }
}

static int DecodeReal2(Decoder *d, size_t limit, const uint8_t *bufLimit) {
    do {
        size_t limit2 = limit;
        if (d->checkDicSize == 0) {
            uint32_t rem = d->properties.dicSize - d->processedPos;
            if (limit - d->dicPos > rem)
                limit2 = d->dicPos + rem;
        }
        TRY(DecodeReal(d, limit2, bufLimit));
        if (d->processedPos >= d->properties.dicSize)
            d->checkDicSize = d->properties.dicSize;
        WriteRem(d, limit);
    } while (d->dicPos < limit && d->buf < bufLimit && d->remainLen < kMatchSpecLenStart);

    if (d->remainLen > kMatchSpecLenStart)
        d->remainLen = kMatchSpecLenStart;

    return 0;
}

typedef enum {
    DUMMY_ERROR, // unexpected end of input stream
    DUMMY_LIT,
    DUMMY_MATCH,
    DUMMY_REP
} Dummy;

static Dummy TryDummy(const Decoder *d, const uint8_t *buf, size_t inSize) {
    uint32_t range = d->range;
    uint32_t code = d->code;
    const uint8_t *bufLimit = buf + inSize;
    Prob *probs = d->probs;
    unsigned state = d->state;
    Dummy res;

    uint32_t bound;
    unsigned ttt;
    unsigned posState = (d->processedPos) & ((1 << d->properties.pb) - 1);

    Prob *prob = probs + IsMatch + (state << kNumPosBitsMax) + posState;
    IF_BIT_0_CHECK(prob) {
        UPDATE_0_CHECK

        prob = probs + Literal;
        if (d->checkDicSize != 0 || d->processedPos != 0)
            prob += (LZMA_LIT_SIZE *
                ((((d->processedPos) & ((1 << (d->properties.lp)) - 1)) << d->properties.lc) +
                (d->dic[(d->dicPos == 0 ? d->dicBufSize : d->dicPos) - 1] >> (8 - d->properties.lc))));

        if (state < kNumLitStates) {
            unsigned symbol = 1;
            do { GET_BIT_CHECK(prob + symbol, symbol) } while (symbol < 0x100);
        }
        else {
            unsigned matchByte = d->dic[d->dicPos - d->reps[0] +
                    ((d->dicPos < d->reps[0]) ? d->dicBufSize : 0)];
            unsigned offs = 0x100;
            unsigned symbol = 1;
            do {
                matchByte <<= 1;
                unsigned bit = (matchByte & offs);
                Prob *probLit = prob + offs + bit + symbol;
                GET_BIT2_CHECK(probLit, symbol, offs &= ~bit, offs &= bit)
            } while (symbol < 0x100);
        }
        res = DUMMY_LIT;
    }
    else {
        unsigned len;
        UPDATE_1_CHECK;

        prob = probs + IsRep + state;
        IF_BIT_0_CHECK(prob) {
            UPDATE_0_CHECK;
            state = 0;
            prob = probs + LenCoder;
            res = DUMMY_MATCH;
        }
        else {
            UPDATE_1_CHECK;
            res = DUMMY_REP;
            prob = probs + IsRepG0 + state;
            IF_BIT_0_CHECK(prob) {
                UPDATE_0_CHECK;
                prob = probs + IsRep0Long + (state << kNumPosBitsMax) + posState;
                IF_BIT_0_CHECK(prob) {
                    UPDATE_0_CHECK;
                    NORMALIZE_CHECK;
                    return DUMMY_REP;
                }
                else {
                    UPDATE_1_CHECK;
                }
            }
            else {
                UPDATE_1_CHECK;
                prob = probs + IsRepG1 + state;
                IF_BIT_0_CHECK(prob) {
                    UPDATE_0_CHECK;
                }
                else {
                    UPDATE_1_CHECK;
                    prob = probs + IsRepG2 + state;
                    IF_BIT_0_CHECK(prob) {
                        UPDATE_0_CHECK;
                    }
                    else {
                        UPDATE_1_CHECK;
                    }
                }
            }
            state = kNumStates;
            prob = probs + RepLenCoder;
        }
        {
            unsigned limit, offset;
            Prob *probLen = prob + LenChoice;
            IF_BIT_0_CHECK(probLen) {
                UPDATE_0_CHECK;
                probLen = prob + LenLow + (posState << kLenNumLowBits);
                offset = 0;
                limit = 1 << kLenNumLowBits;
            }
            else {
                UPDATE_1_CHECK;
                probLen = prob + LenChoice2;
                IF_BIT_0_CHECK(probLen) {
                    UPDATE_0_CHECK;
                    probLen = prob + LenMid + (posState << kLenNumMidBits);
                    offset = kLenNumLowSymbols;
                    limit = 1 << kLenNumMidBits;
                }
                else {
                    UPDATE_1_CHECK;
                    probLen = prob + LenHigh;
                    offset = kLenNumLowSymbols + kLenNumMidSymbols;
                    limit = 1 << kLenNumHighBits;
                }
            }
            TREE_DECODE_CHECK(probLen, limit, len);
            len += offset;
        }

        if (state < 4) {
            unsigned posSlot;
            prob = probs + PosSlot +
                    ((len < kNumLenToPosStates ? len : kNumLenToPosStates - 1) <<
                    kNumPosSlotBits);
            TREE_DECODE_CHECK(prob, 1 << kNumPosSlotBits, posSlot);
            if (posSlot >= kStartPosModelIndex) {
                int numDirectBits = ((posSlot >> 1) - 1);

                if (posSlot < kEndPosModelIndex) {
                    prob = probs + SpecPos + ((2 | (posSlot & 1)) << numDirectBits) - posSlot - 1;
                }
                else {
                    numDirectBits -= kNumAlignBits;
                    do {
                        NORMALIZE_CHECK
                        range >>= 1;
                        code -= range & (((code - range) >> 31) - 1);
                    } while (--numDirectBits != 0);
                    prob = probs + Align;
                    numDirectBits = kNumAlignBits;
                }
                {
                    unsigned i = 1;
                    do {
                        GET_BIT_CHECK(prob + i, i);
                    } while (--numDirectBits != 0);
                }
            }
        }
    }

    NORMALIZE_CHECK;
    return res;
}

static void InitRc(Decoder *d, const uint8_t *data) {
    d->code = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | ((uint32_t)data[4]);
    d->range = 0xFFFFFFFF;
    d->needFlush = 0;
}

static void InitStateReal(Decoder *d) {
    uint32_t numProbs = Literal + ((uint32_t)LZMA_LIT_SIZE << (d->properties.lc + d->properties.lp));
    Prob *probs = d->probs;
    for (uint32_t i = 0; i < numProbs; i++)
        probs[i] = kBitModelTotal >> 1;
    d->reps[0] = d->reps[1] = d->reps[2] = d->reps[3] = 1;
    d->state = 0;
    d->needInitState = 0;
}

static LzmaRes DecodeToDic(Decoder *d, size_t dicLimit, const uint8_t *src, size_t *srcLen) {
    size_t inSize = *srcLen;
    *srcLen = 0;
    WriteRem(d, dicLimit);

    while (d->remainLen != kMatchSpecLenStart) {
        int checkEndMarkNow = 0;

        if (d->needFlush != 0) {
            for (; inSize > 0 && d->tempBufSize < RC_INIT_SIZE; (*srcLen)++, inSize--)
                d->tempBuf[d->tempBufSize++] = *src++;
            if (d->tempBufSize < RC_INIT_SIZE) {
                return LZMA_ERROR_INPUT_EOF;
            }
            if (d->tempBuf[0] != 0)
                return LZMA_ERROR_DATA;

            InitRc(d, d->tempBuf);
            d->tempBufSize = 0;
        }

        if (d->dicPos >= dicLimit) {
            if (d->remainLen == 0 && d->code == 0) {
                return LZMA_ERROR_MAYBE_FINISHED_WITHOUT_MARK;
            }
            if (d->remainLen != 0) {
                return LZMA_ERROR_DATA;
            }
            checkEndMarkNow = 1;
        }

        if (d->needInitState)
            InitStateReal(d);

        if (d->tempBufSize == 0) {
            const uint8_t *bufLimit;
            if (inSize < LZMA_REQUIRED_INPUT_MAX || checkEndMarkNow) {
                int dummyRes = TryDummy(d, src, inSize);
                if (dummyRes == DUMMY_ERROR) {
                    memcpy(d->tempBuf, src, inSize);
                    d->tempBufSize = (unsigned)inSize;
                    *srcLen += inSize;
                    return LZMA_ERROR_INPUT_EOF;
                }
                if (checkEndMarkNow && dummyRes != DUMMY_MATCH) {
                    return LZMA_ERROR_DATA;
                }
                bufLimit = src;
            }
            else
                bufLimit = src + inSize - LZMA_REQUIRED_INPUT_MAX;
            d->buf = src;
            if (DecodeReal2(d, dicLimit, bufLimit) != 0)
                return LZMA_ERROR_DATA;
            size_t processed = (size_t)(d->buf - src);
            *srcLen += processed;
            src += processed;
            inSize -= processed;
        }
        else {
            unsigned rem = d->tempBufSize, lookAhead = 0;
            while (rem < LZMA_REQUIRED_INPUT_MAX && lookAhead < inSize)
                d->tempBuf[rem++] = src[lookAhead++];
            d->tempBufSize = rem;
            if (rem < LZMA_REQUIRED_INPUT_MAX || checkEndMarkNow) {
                int dummyRes = TryDummy(d, d->tempBuf, rem);
                if (dummyRes == DUMMY_ERROR) {
                    *srcLen += lookAhead;
                    return LZMA_ERROR_INPUT_EOF;
                }
                if (checkEndMarkNow && dummyRes != DUMMY_MATCH) {
                    return LZMA_ERROR_DATA;
                }
            }
            d->buf = d->tempBuf;
            if (DecodeReal2(d, dicLimit, d->buf) != 0)
                return LZMA_ERROR_DATA;
            lookAhead -= rem - (unsigned)(d->buf - d->tempBuf);
            *srcLen += lookAhead;
            src += lookAhead;
            inSize -= lookAhead;
            d->tempBufSize = 0;
        }
    }

    return (d->code == 0) ? LZMA_OK : LZMA_ERROR_DATA;
}

#define MIN_DIC_SIZE (1 << 12)
static LzmaRes DecodeHeader(Properties *p, const uint8_t *src) {
    // Properties.
    uint8_t d = src[0];
    if (d >= (9 * 5 * 5))
        return LZMA_ERROR_UNSUPPORTED;
    p->lc = d % 9;
    d /= 9;
    p->pb = d / 5;
    p->lp = d % 5;

    // Dictionary size.
    p->dicSize = *((uint32_t *)(src + 1));
    if (p->dicSize < MIN_DIC_SIZE)
        p->dicSize = MIN_DIC_SIZE;

    return LZMA_OK;
}

LzmaRes LzmaDecode(const uint8_t *src, size_t srcLen, uint8_t **dest, size_t *destLen) {
    if (srcLen < 13 + RC_INIT_SIZE)
        return LZMA_ERROR_INPUT_EOF;

    Decoder d = { 0 };
    TRY(DecodeHeader(&d.properties, src));
    size_t decompressedSize = (size_t)*((uint64_t*)(src + 5));
    src += 13;
    srcLen -= 13;

    if (decompressedSize > 0xffffffffu)
        return LZMA_ERROR_UNSUPPORTED;

    d.numProbs = LZMA_BASE_SIZE + (LZMA_LIT_SIZE << (d.properties.lc + d.properties.lp));
    d.probs = malloc(d.numProbs * sizeof(Prob));
    d.dic = malloc(decompressedSize);
    d.dicBufSize = decompressedSize;
    d.needFlush = 1;
    d.needInitState = 1;

    LzmaRes res = DecodeToDic(&d, decompressedSize, src, &srcLen);

    if (decompressedSize != d.dicPos)
        res = LZMA_ERROR_DATA;

    if (res != LZMA_OK) {
        free(d.dic);
    }
    else {
        *dest = d.dic;
        *destLen = decompressedSize;
    }

    free(d.probs);
    return res;
}
