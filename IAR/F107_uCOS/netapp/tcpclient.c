/**
* @file tcpclient.c
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date 2010Jul31 20:32:54
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#define __TCPCLIENT_C


/* Includes ------------------------------------------------------------------*/
#include "tcpclient.h"
//#include "lwip/opt.h"
#include "lwip/netdb.h" /* 为了解析主机名，需要包含netdb.h头文件 */
#include "lwip/sockets.h" /* 使用BSD Socket，需要包含sockets.h头文件 */
#include "lwip/mem.h" /* 为了分配内存 */
#include "includes.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static const char send_data[] = "This is TCP Client form uC/OS-II."; /* 发送用到的数据 */

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
void tcpclient(const char* url, int port)
{
    char *recv_data;
    struct hostent *host;
    int sock, bytes_received;
    struct sockaddr_in server_addr;
    
       /* 通过函数入口参数url获得host地址（如果是域名，会做域名解析） */
   host = gethostbyname(url);

   /* 分配用于存放接收数据的缓冲 */
   recv_data = mem_malloc(1024);
   if (recv_data == NULL)
   {
       printf("No memory\n");
       return;
   }

   /* 创建一个socket，类型是SOCKET_STREAM，TCP类型 */
   if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
   {
       /* 创建socket失败 */
       printf("Socket error\n");

       /* 释放接收缓冲 */
       mem_free(recv_data);
       return;
   }

   /* 初始化预连接的服务端地址 */
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(port);
   server_addr.sin_addr = *((struct in_addr *)host->h_addr);
   memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

   /* 连接到服务端 */
   if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
   {
       /* 连接失败 */
       printf("Connect error\n");
       
       /*释放接收缓冲 */
       mem_free(recv_data);
       return;
   }
   
   while(1)
   {
       /* 从sock连接中接收最大1024字节数据 */
       bytes_received = recv(sock, recv_data, 1024, 0);
       if (bytes_received < 0)
       {
           /* 接收失败，关闭这个连接 */
           lwip_close(sock);
           
           /* 释放接收缓冲 */
           mem_free(recv_data);
           break;
       }
       
       /* 有接收到数据，把末端清零 */
       recv_data[bytes_received] = '\0';
       
       if (strcmp(recv_data , "q") == 0 || strcmp(recv_data , "Q") == 0)
       {
           /* 如果是首字母是q或Q，关闭这个连接 */
           lwip_close(sock);
           
           /* 释放接收缓冲 */
           mem_free(recv_data);
           break;
       }
       else
       {
           /* 在控制终端显示收到的数据 */
           printf("\nRecieved data = %s " , recv_data);
       }
       
       /* 发送数据到sock连接 */
       send(sock,send_data,strlen(send_data), 0);
   }
   
   return;
}

/*-- File end --*/

