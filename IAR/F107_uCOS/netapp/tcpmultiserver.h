/**
* @file tcpmultiserver.h
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date  2010Aug2 16:00:33
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#ifndef __TCPMULTISERVER_H
#define __TCPMULTISERVER_H

#ifdef __cplusplus
extern "C" { /* Make sure we have C-declarations in C++ programs */
#endif


/* Includes ------------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


#ifndef __TCPMULTISERVER_C
/* Exported variables --------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void tcpmultiserver_init(); /* start tcpserver */

#endif /* !defined(__TCPMULTISERVER_C) */


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


#endif /* #ifndef __TCPMULTISERVER_H */
/*-- File end --*/

