/**
* @file udpclient.c
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date 2010Jul31 21:27:15
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#define __UDPCLIENT_C


/* Includes ------------------------------------------------------------------*/
#include "udpclient.h"
#include <lwip/netdb.h> /* 为了解析主机名，需要包含netdb.h头文件 */
#include <lwip/sockets.h> /* 使用BSD socket，需要包含sockets.h头文件 */
#include "lwip/mem.h"
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
const char send_data[] = "This is UDP Client from RT-Thread.\n"; /* 发送用到的数据 */

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void udpclient(const char* url, int port, int count)
{
   int sock;
   struct hostent *host;
   struct sockaddr_in server_addr;

   /* 通过函数入口参数url获得host地址（如果是域名，会做域名解析） */
   host= (struct hostent *) gethostbyname(url);

   /* 创建一个socket，类型是SOCK_DGRAM，UDP类型 */
   if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
   {
       printf("Socket error\n");
       return;
   }

   /* 初始化预连接的服务端地址 */
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(port);
   server_addr.sin_addr = *((struct in_addr *)host->h_addr);
   memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

   /* 总计发送count次数据 */
   while (count)
   {
       /* 发送数据到服务远端 */
       sendto(sock, send_data, strlen(send_data), 0,
              (struct sockaddr *)&server_addr, sizeof(struct sockaddr));

       /* 线程休眠一段时间 */
       OSTimeDly(50);

       /* 计数值减一 */
       count --;
   }

   /* 关闭这个socket */
   lwip_close(sock);
}

/*-- File end --*/

