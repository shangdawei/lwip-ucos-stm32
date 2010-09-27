/**
* @file netdatapack.h
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date  2010Aug3 10:36:15
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#ifndef __NETDATAPACK_H
#define __NETDATAPACK_H

#ifdef __cplusplus
extern "C" { /* Make sure we have C-declarations in C++ programs */
#endif


/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define pack754_32(f)       (pack754((f), 32, 8))
#define pack754_64(f)       (pack754((f), 64, 11))
#define unpack754_32(i)     (unpack754((i), 32, 8))
#define unpack754_64(i)     (unpack754((i), 64, 11))

#ifndef __NETDATAPACK_C
/* Exported variables --------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void packi16(unsigned char *buf, unsigned int i);       /* convert U16 to array */
void packi32(unsigned char *buf, unsigned long i);      /* convert U32 to array */
unsigned int unpacki16(unsigned char *buf);             /* convert array to U16 */
unsigned long unpacki32(unsigned char *buf);            /* convert array to U32 */

int32_t pack(unsigned char *buf, char *format, ...);    /* pack struct */
void unpack(unsigned char* buf, char *format, ...);     /* unpack struct */

uint64_t pack754(long double f, unsigned bits, unsigned expbits);
long double unpack754(uint64_t i, unsigned bits, unsigned expbits);

#endif /* !defined(__NETDATAPACK_C) */


/*
#error section
-- The standard C preprocessor directive #error should be used to notify the
programmer when #define constants or macros are not present and to indicate
that a #define value is out of range. These statements are normally found in
a module¡¯s .H file. The #error directive will display the message within the
double quotes when the condition is not met.
*/


#ifdef __cplusplus
}
#endif


#endif /* #ifndef __NETDATAPACK_H */
/*-- File end --*/

