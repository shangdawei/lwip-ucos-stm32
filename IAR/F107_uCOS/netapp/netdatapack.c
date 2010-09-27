/**
* @file netdatapack.c
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date 2010Aug3 9:52:23
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#define __NETDATAPACK_C


/* Includes ------------------------------------------------------------------*/
#include "netdatapack.h"
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/
/* Various bits for floating point types --*/
/* Varies for different architectures */
typedef float   float32_t;
typedef double  float64_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
* @fn uint64_t pack754(
*         long double f,
*          unsigned bits,
*          unsigned expbits
*         )
* @brief convert IEEE754 compatibility double to U64.
* @param long double f :double.
* @param  unsigned bits :param description.
* @param  unsigned expbits :param description.
* @return uint64_t return variable description.
*/
uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit
    
    if (f == 0.0) return 0; // get this special case out of the way
    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }
    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;
    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);
    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias
    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

/**
* @fn long double unpack754(
*         uint64_t i,
*          unsigned bits,
*          unsigned expbits
*         )
* @brief function description.
* @param uint64_t i :convert U64 to IEEE754 compatibility double.
* @param  unsigned bits :param description.
* @param  unsigned expbits :param description.
* @return long double return variable description.
*/
long double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit
    if (i == 0) return 0.0;
    // pull the significand
    
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on
    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }
    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;
    return result;
}

/**
* @fn void packi16(
*         unsigned char *buf,
*          unsigned int i
*         )
* @brief convert U16 to array [Big Endian].
* @param unsigned char *buf :a pointer to char array.
* @param  unsigned int i :16bit unsigned int.
* @return void return variable description.
*/
void packi16(unsigned char *buf, unsigned int i)
{
    *buf++ = i >> 8; *buf++ = i;
}

/**
* @fn void packi32(
*         unsigned char *buf,
*          unsigned long i
*         )
* @brief convert U32 to array [Big Endian].
* @param unsigned char *buf :a pointer to char array.
* @param  unsigned long i :32bit unsigned int.
* @return void return variable description.
*/
void packi32(unsigned char *buf, unsigned long i)
{
    *buf++ = i >> 24; *buf++ = i >> 16;
    *buf++ = i >> 8;  *buf++ = i;
}

/**
* @fn unsigned int unpacki16(
*         unsigned char *buf
*         )
* @brief convert array[Big Endian] to U16.
* @param unsigned char *buf :a pointer to char array.
* @return unsigned int return U16.
*/
unsigned int unpacki16(unsigned char *buf)
{
    return (buf[0] << 8) | buf[1];
}

/**
* @fn unsigned long unpacki32(
*         unsigned char *buf
*         )
* @brief convert array[Big Endian] to U32.
* @param unsigned char *buf :a pointer to char array.
* @return unsigned int return U32.
*/
unsigned long unpacki32(unsigned char *buf)
{
    return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

/**
* @fn int32_t pack(
*         unsigned char *buf,
*          char *format,
*          ...
*         )
* @brief store data dictated by the format string in the buffer.
*  h - 16-bit              l - 32-bit
*  c - 8-bit char          f - float, 32-bit
*  s - string (16-bit length is automatically prepended)
*
* @param unsigned char *buf :a pointer to char array.
* @param  char *format :convert formatter.
* @param  ... :convert value.
* @return int32_t return array length.
*/
int32_t pack(unsigned char *buf, char *format, ...)
{
    va_list ap;
    int16_t h;
    int32_t l;
    int8_t c;
    float32_t f;
    char *s;
    int32_t size = 0, len;
    
    va_start(ap, format);
    
    for(; *format != '\0'; format++)
    {
        switch(*format)
        {
        case 'h': // 16-bit
            size += 2;
            h = (int16_t)va_arg(ap, int); // promoted
            packi16(buf, h);
            buf += 2;
            break;
        case 'l': // 32-bit
            size += 4;
            l = va_arg(ap, int32_t);
            packi32(buf, l);
            buf += 4;
            break;
        case 'c': // 8-bit
            size += 1;
            c = (int8_t)va_arg(ap, int); // promoted
            *buf++ = (c >> 0) & 0xff; 
            break;
        case 'f': // float
            size += 4;
            f = (float32_t)va_arg(ap, double); // promoted
            l = pack754_32(f); // convert to IEEE 754
            packi32(buf, l);
            buf += 4;
            break;
        case 's': // string
            s = va_arg(ap, char*);
            len = strlen(s);
            size += len + 2;
            packi16(buf, len);  // first two bytes storage string length (doesn't contain '\0')
            buf += 2;
            memcpy(buf, s, len);
            buf += len;
            break;
        }
    }
    
    va_end(ap);
    
    return size;
}

/**
* @fn void unpack(
*         unsigned char* buf,
*          char *format,
*          ...
*         )
* @brief  unpack data dictated by the format string into the buffer.
* @param unsigned char* buf :a pointer to char array.
* @param  char *format :convert formatter.
* @param  ... :convert value.
* @return void
*/
void unpack(unsigned char* buf, char *format, ...)
{
    va_list ap;
    int16_t *h;
    int32_t *l;
    int32_t pf;
    int8_t *c;
    float32_t *f;
    char *s;
    int32_t len, count, maxstrlen = 0;
    
    va_start(ap, format);
    
    for(; *format != '\0'; format++)
    {
        switch(*format)
        {
        case 'h': // 16-bit
            h = va_arg(ap, int16_t*);
            *h = unpacki16(buf);
            buf += 2;
            break;
        case 'l': // 32-bit
            l = va_arg(ap, int32_t*);
            *l = unpacki32(buf);
            buf += 4;
            break;
        case 'c': // 8-bit
            c = va_arg(ap, int8_t*);
            *c = *buf++;
            break;
        case 'f': // float
            f = va_arg(ap, float32_t*);
            pf = unpacki32(buf);
            buf += 4;
            *f = unpack754_32(pf);
            break;
        case 's': // string
            s = va_arg(ap, char*);
            len = unpacki16(buf); // get string length from array
            buf += 2;
            // if param's max length is vaild ,then using param.
            if(maxstrlen > 0 && len > maxstrlen) count = maxstrlen - 1;
            else count = len;
            memcpy(s, buf, count);
            s[count] = '\0'; // add '\0' indicate the end of string array
            buf += len;
            break;
        default:
            if(isdigit(*format)) { // track max str len
                maxstrlen = maxstrlen * 10 + (*format - '0');
            }
        }
        
        if(!isdigit(*format)) maxstrlen = 0;
    }
    
    va_end(ap);
}
/* Private functions ---------------------------------------------------------*/


/*-- File end --*/

