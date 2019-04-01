#include "Utilities.h"


void floatToByteArray(uint8_t *bytes, float f, int start)
{
    //Create new union where a float and byte array share memory addresses for conversion
    union {
        float fval;
        uint8_t bval[4];
    } floatAsBytes;

    floatAsBytes.fval = f;

    //make report equal to data in byte array from union
    for (int i = start; i < (start + 4); i++)
    {
        bytes[i] = floatAsBytes.bval[i - start];
    }
}

void shortToByteArray(uint8_t *bytes, short s, int start)
{
    //Create new union where a 16 bit int and byte array share memory addresses for conversion
    union {
        short sval;
        uint8_t bval[2];
    } shortAsBytes;

    shortAsBytes.sval = s;
    
    //make report equal to data in byte array from union
    for (int i = start; i < (start + 2); i++)
    {
        bytes[i] = shortAsBytes.bval[i - start];
    }
}

float ByteArrayToFloat(uint8_t *bytes, int start)
{
    //Create new union where a byte array shares a memory address with a float
    union {
        float fval;
        uint8_t bval[4];
    } floatAsBytes;

    //Make byte array equal to incoming report bytes
    for (int i = start; i < (start + 4); i++)
    {
        floatAsBytes.bval[i - start] = bytes[i];
    }

    //return float value of incoming array
    return floatAsBytes.fval;
}

short ByteArrayToShort(uint8_t *bytes, int start)
{
    //Make byte array equal to incoming report bytes
    union {
        short sval;
        uint8_t bval[2];
    } shortAsBytes;

    for (int i = start; i < (start + 2); i++)
    {
        shortAsBytes.bval[i - start] = bytes[i];
    }

    return shortAsBytes.sval;
}