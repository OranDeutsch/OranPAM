#ifndef UTILITIES_H
#define UTILITIES_H

#include "mbed.h"

/**
       * Appends a float to a byte array as four bytes
       * @param uint8_t bytes : Pointer to the byte array to append to
       * @param float f: the float to add to the byte array
       * @param int start : the position in the byte array to append the float valur
       */
void floatToByteArray(uint8_t *bytes, float f, int start);

/**
       * Appends a short (16 bit int) to a byte array as two bytes
       * @param uint8_t bytes : Pointer to the byte array to append to
       * @param float f: the float to add to the byte array
       * @param int start : the position in the byte array to append the short value
       */
void shortToByteArray(uint8_t *bytes, short s, int start);

/**
       * returns the corrisponding float value for four bytes in a byte array
       * @param uint8_t bytes : Pointer to the byte array to read
       * @param int start : the position in the byte array to read the float value
       * @return the corrisponding float value
       */
float ByteArrayToFloat(uint8_t *bytes, int start);

/**
       * returns the corrisponding short value for two bytes in a byte array
       * @param uint8_t bytes : Pointer to the byte array to read
       * @param int start : the position in the byte array to read the short value
       * @return the corrisponding short value
       */
short ByteArrayToShort(uint8_t *bytes, int start);

#endif