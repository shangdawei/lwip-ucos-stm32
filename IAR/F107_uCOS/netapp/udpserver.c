/**
* @file udpserver.c
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date 2010Jul31 21:32:57
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#define __UDPSERVER_C


/* Includes ------------------------------------------------------------------*/
#include "udpserver.h"
#include <lwip/sockets.h> /* 使用BSD socket，需要包含sockets.h头文件 */
#include <includes.h>
#include "lwip/mem.h"

#include "app_cfg.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#ifndef UDPSERVER_STK_SIZE          /* UDPSERVER 使用堆栈大小 */
#define UDPSERVER_STK_SIZE  300
#endif

#ifndef LWIP_UDPSERVER_PRIO         /* UDPSERVER 任务优先级 */
#define LWIP_UDPSERVER_PRIO 16
#endif

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static OS_STK   UdpServerStack[UDPSERVER_STK_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void udpserv(void* paramemter);
static void udpserver_thread(void* arg);

/* Private functions ---------------------------------------------------------*/
static void udpserv(void* paramemter)
{
   int sock;
   int bytes_read;
   char *recv_data;
   u32_t addr_len;
   struct sockaddr_in server_addr, client_addr;

   /* 分配接收用的数据缓冲 */
   recv_data = mem_malloc(1024);
   if (recv_data == NULL)
   {
       /* 分配内存失败，返回 */
       printf("No memory\n");
       return;
   }

   /* 创建一个socket，类型是SOCK_DGRAM，UDP类型 */
   if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
   {
       printf("Socket error\n");

       /* 释放接收用的数据缓冲 */
       mem_free(recv_data);
       return;
   }

   /* 初始化服务端地址 */
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(5000);
   server_addr.sin_addr.s_addr = INADDR_ANY;
   memset(&(server_addr.sin_zero),0, sizeof(server_addr.sin_zero));

   /* 绑定socket到服务端地址 */
   if (bind(sock,(struct sockaddr *)&server_addr,
            sizeof(struct sockaddr)) == -1)
   {
       /* 绑定地址失败 */
       printf("Bind error\n");

       /* 释放接收用的数据缓冲 */
       mem_free(recv_data);
       return;
   }

   addr_len = sizeof(struct sockaddr);
   printf("UDPServer Waiting for client on port 5000...\n");

   while (1)
   {
       /* 从sock中收取最大1024字节数据 */
       bytes_read = recvfrom(sock, recv_data, 1024, 0,
                             (struct sockaddr *)&client_addr, &addr_len);
       /* UDP不同于TCP，它基本不会出现收取的数据失败的情况，除非设置了超时等待 */

       recv_data[bytes_read] = '\0'; /* 把末端清零 */

       /* 输出接收的数据 */
       printf("\n(%s , %d) said : ",inet_ntoa(client_addr.sin_addr),
                  ntohs(client_addr.sin_port));
       printf("%s", recv_data);

       /* 如果接收数据是exit，退出 */
       if (strcmp(recv_data, "exit") == 0)
       {
           printf("receive \"exit\" command, exit udpserver task.\n");
           lwip_close(sock);

           /* 释放接收用的数据缓冲 */
           mem_free(recv_data);
           break;
       }
   }

   return;
}

static void udpserver_thread(void* arg)
{
    udpserv(arg);
    
    printf("Delete Udpserver thread.\n");
    OSTaskDel(OS_PRIO_SELF);
}


/**
* @brief start udpserver
*/
void udpserver_init()
{
    CPU_INT08U  os_err;
    
    OSTaskCreate(udpserver_thread, 
                 NULL, 
                 &UdpServerStack[UDPSERVER_STK_SIZE - 1], 
                 LWIP_UDPSERVER_PRIO
                     );
#if (OS_TASK_NAME_SIZE >= 11)
    OSTaskNameSet(LWIP_UDPSERVER_PRIO, (CPU_INT08U *)"udpserver", &os_err);
#endif
}

/*-- File end --*/

