/**
* @file netapp.h
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date  2010Jul31 21:18:35
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#ifndef __NETAPP_H
#define __NETAPP_H

#ifdef __cplusplus
extern "C" { /* Make sure we have C-declarations in C++ programs */
#endif


/* Includes ------------------------------------------------------------------*/
#include "netio.h"          /* Netio TCP transceiver speed test! */
#include "sample_TTCP.h"    /* TTCP test TCP establish speed. */
#include "tcpecho.h"        /* tcpecho test */
#include "udpecho.h"        /* udpecho test */
#include "tcpclient.h"      /* tcpclient test */
#include "tcpserver.h"      /* tcpserver test */
#include "tcpserver2.h"     /* tcpserver2 test */
#include "udpserver.h"      /* udpserver test */
#include "udpclient.h"      /* udpclient test */
#include "tcpmultiserver.h" /* udpmultiserver test */

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


#ifndef __NETAPP_C
/* Exported variables --------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void httpd_init(void);      /* Sample Http Web Server using LwIP Raw API */

#endif /* !defined(__NETAPP_C) */


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


#endif /* #ifndef __NETAPP_H */
/*-- File end --*/

