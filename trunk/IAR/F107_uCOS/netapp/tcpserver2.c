/**
* @file tcpserver2.c
* @brief A brief file description.
* @details
*     A more elaborated file description.
* @author Wang Mengyin
* @date 2010Jul31 21:00:58
* @note
*               Copyright 2010 Wang Mengyin.ALL RIGHTS RESERVED.
*                            http://tigerwang202.blogbus.com
*    This software is provided under license and contains proprietary and
* confidential material which is the property of Company Name tech.
*/


#define __TCPSERVER2_C


/* Includes ------------------------------------------------------------------*/
#include "tcpserver2.h"
#include "lwip/mem.h" /* 为了分配内存 */
#include <lwip/sockets.h> /* 使用BSD Socket接口必须包含sockets.h这个头文件 */
#include "includes.h"

#include "netdatapack.h" /* 为了实现数据类型转换 */
//#include "stm3210c_eval_lcd.h"

#include "app_cfg.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#ifndef TCPSERVER2_STK_SIZE          /* tcpserver 使用堆栈大小 */
#define TCPSERVER2_STK_SIZE  300
#endif

#ifndef LWIP_TCPSERVER2_PRIO         /* tcpserver 任务优先级 */
#define LWIP_TCPSERVER2_PRIO 15
#endif

#ifndef RECV_BUFFER_SIZE            /* 接收缓存区大小 */
#define RECV_BUFFER_SIZE    512
#endif

#ifndef TCPSERVER2_PORT_NO
#define TCPSERVER2_PORT_NO   5011    /* tcpserver 使用端口号 */
#endif

#define CUSTOM_HEADER_LEN       5       /* 数据包头长度 */

#define CUSTOM_MAGIC_NUM        0x0711  /* 幻数 :) */

#define CUSTOM_MAGIC_OFFSET     0       /* 数据包幻数的偏移地址 */
#define CUSTOM_DATA_LEN_OFFSET  2       /* 数据包长度的偏移地址 */
#define CUSTOM_CMD_OFFSET       4       /* 数据包命令的偏移地址 */


#define CUSTOM_CMD_LED          1       /* 设置LED状态 */
#define CUSTOM_CMD_ADC          2       /* 获取ADC状态 */
#define CUSTOM_CMD_LCD          3       /* 设置LCD状态 */
#define CUSTOM_CMD_ECHO         4       /* 获取Echo信息 */ 

#define CUSTOM_ADC_LEN          2       /* 设置ADC数据包长度 */
#define CUSTOM_LCD_LEN          (16 * 4 + 1) /* 设置LCD数据包长度*/
#define CUSTOM_ECHO_LEN         (strlen(send_data) + 2) /* 设置echo数据包长度 */
/* Private macro -------------------------------------------------------------*/
#define CUSTOM_GET_LENGTH(a)    unpacki16(((unsigned char* )(a)) + CUSTOM_DATA_LEN_OFFSET)

/* Private variables ---------------------------------------------------------*/
static const char send_data[] = "This is TCP Server from uC/OS-II."; /* 发送用到的数据 */
static OS_STK   TcpServerStack[TCPSERVER2_STK_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void tcpserv(void* parameter);
static void tcpserver_thread(void* arg);
static int blocking_lwip_recv(int socket, void* mem, int len);
/* Private functions ---------------------------------------------------------*/

/**
* @fn static int processCMD(
*         unsigned char* buf
*         )
* @brief parse received data.
* @param unsigned char* buf :a pointer to received data buffer.
* @return int return send data length.
*/
static int processCMD(unsigned char* buf)
{
    int send_len = 0;
    u16 magic_num, body_length, adc;
    u8 cmd, led, lcd_dispbuf[CUSTOM_LCD_LEN];
    unpack(buf, "hhc", &magic_num, &body_length, &cmd); // 获取头信息
    
#if 1
    // debug
    printf("Custom TCP data header:\n");
    printf("Byte[0 1] : magic number = 0x%04X\n", magic_num);
    printf("Byte[2 3] : body length  = 0x%04X\n", body_length);
    printf("Byte[4  ] : command      = 0x%04X\n", cmd);
#endif
    
    switch(cmd)
    {
    case CUSTOM_CMD_LED:
        {
            unpack(buf + CUSTOM_HEADER_LEN, "c", &led); // 获取LCD状态
            if(led != 0)
                BSP_LED_On(2); /* led 2 on */
            else
                BSP_LED_Off(2); /* led 2 off */
            printf("set led[2] %s\n", led != 0 ? "ON": "OFF");
        }
        break;
    case CUSTOM_CMD_ADC:
        {
            adc = BSP_ADC_GetStatus(1); // 读取ADC值
            // 生成发送数据包
            send_len = pack(buf, "hhch", 
                            CUSTOM_MAGIC_NUM,  /* Byte[0 1]    magic number */
                            CUSTOM_ADC_LEN,    /* Byte[2 3]    length       */
                            CUSTOM_CMD_ADC,    /* Byte[4  ]    command      */
                            adc                /* Byte[5 6]    adc data     */);
            printf("send ADC value: %d\n", adc);
        }
        break;
    case CUSTOM_CMD_LCD:
        {
            unpack(buf + CUSTOM_HEADER_LEN, "s", &lcd_dispbuf); // 获取LCD的字符
            LCD_Clear();
            LCD_Puts(lcd_dispbuf); /* show received string in 128*64 lcd */
            printf("lcd display: %s \n", lcd_dispbuf);
        }
        break;
    case CUSTOM_CMD_ECHO:
        {
            // 设置回显字符串
            send_len = pack(buf + CUSTOM_HEADER_LEN, "s", send_data);
            // 写入数据包头
            send_len += pack(buf, "hhc",
                            CUSTOM_MAGIC_NUM,  /* Byte[0 1]  magic number */
                            send_len,          /* Byte[2 3]  length       */
                            CUSTOM_CMD_ECHO    /* Byte[4  ]  command      */);
            printf("send echo string: %s \n", send_data);
        }
        break;
    default:
        {   /* undefined CMD, I don't think that will happen. */
            printf("error unknown command interpret.\n");
        }
    }
    return send_len;
}

/**
* @fn static int blocking_lwip_recv(
*         int socket,
*          void* mem,
*          int len
*         )
* @brief receive certain length of data form socket, note functon will block until
* given data length is received.
* @param int socket :socket.
* @param  void* mem :pointer to data received buffer.
* @param  int len :received data length.
* @return int return received data length, length should equal to len given,
* length value <= 0 indicates connect error.
*/
static int blocking_lwip_recv(int socket, void* mem, int len)
{
    int readlen ,offset = 0, ret;
    fd_set fds;

// lwip dose not implement MSG_WAITALL, so we can't use block recv directly.    
//    return lwip_recv(socket, mem, len, MSG_WAITALL);
    while(1)
    {
        FD_ZERO(&fds);
        FD_SET(socket, &fds);
        if((ret = lwip_select(socket + 1, &fds, 0, 0, 0)) <= 0)
        {   /* select error. */
            printf("block receive select error.\n");
            return ret;
        }
        
        readlen = lwip_recv(socket, ((char*) mem) + offset, len - offset, 0);
        if(readlen <= 0)
        {   /* connect error, client has close the connection. */
            if(readlen < 0)
                printf("block receive error.\n");
            return readlen;
        }
        if(readlen == (len - offset))
        {   /* we have get all residuary data */
            offset = len;
            break;
        }
        offset += readlen;
    } // end of while(1)
    return offset;
}

/**
* @brief tcpserver accept tcp connect receive data and output on debug thermal.
* @param void* parameter :unused.
*/
static void tcpserv(void* parameter)
{
   char *recv_data; /* 用于接收的指针，后面会做一次动态分配以请求可用内存 */
   u32_t sin_size;
   int sock, connected, bytes_received, datalen;
   struct sockaddr_in server_addr, client_addr;
   bool stop = FALSE; /* 停止标志 */

   recv_data = mem_malloc(RECV_BUFFER_SIZE); /* 分配接收用的数据缓冲 */
   if (recv_data == NULL)
   {
       printf("No memory\n");
       return;
   }

   /* 一个socket在使用前，需要预先创建出来，指定SOCK_STREAM为TCP的socket */
   if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
   {
       /* 创建失败的错误处理 */
       printf("Socket error\n");

       /* 释放已分配的接收缓冲 */
       mem_free(recv_data);
       return;
   }

   /* 初始化服务端地址 */
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(TCPSERVER2_PORT_NO); /* 服务端工作的端口 */
   server_addr.sin_addr.s_addr = INADDR_ANY;
   memset(&(server_addr.sin_zero),8, sizeof(server_addr.sin_zero));

   /* 绑定socket到服务端地址 */
   if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
   {
       /* 绑定失败 */
       printf("Unable to bind\n");

       /* 释放已分配的接收缓冲 */
       mem_free(recv_data);
       return;
   }

   /* 在socket上进行监听 */
   if (listen(sock, 5) == -1)
   {
       printf("Listen error\n");

       /* release recv buffer */
       mem_free(recv_data);
       return;
   }

   printf("\nTCPServer Waiting for client on port %d...\n", TCPSERVER2_PORT_NO);
   while(stop != TRUE)
   {
       sin_size = sizeof(struct sockaddr_in);

       /* 接受一个客户端连接socket的请求，这个函数调用是阻塞式的 */
       connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
       /* 返回的是连接成功的socket */

       /* 接受返回的client_addr指向了客户端的地址信息 */
       printf("I got a connection from (%s , %d)\n",
                  inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));

       /* 客户端连接的处理 */
       while (1)
       {
           // 接收自定义数据包的头部分
            bytes_received = blocking_lwip_recv(connected, recv_data, CUSTOM_HEADER_LEN);
            
            if(bytes_received != CUSTOM_HEADER_LEN)
                break;
            
            bytes_received = 0;
            // 解析头数据获取剩余接收数据长度
            if((datalen = CUSTOM_GET_LENGTH(recv_data)) > 0)
            {   // 接收剩余数据
                bytes_received = blocking_lwip_recv(connected, recv_data + CUSTOM_HEADER_LEN, datalen);
                
                if(bytes_received != datalen) // 接收到的数据长度与剩余数据长度不等
                    break;
            }
            
            // 处理接收到的数据
            bytes_received = processCMD((unsigned char*)recv_data);
            
            if(bytes_received > 0) // 需要向客户端返回数据
                if(lwip_send(connected, recv_data, bytes_received, 0) < 0)
                    break;
            
       } // end of while(1)
       
       printf("close connection form (%s , %d).\n",
              inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));
       lwip_close(connected); // 服务器关闭连接
   } // end of while(stop != TRUE)

   /* 释放接收缓冲 */
   mem_free(recv_data);

   return ;
}

/**
* @brief tcpserver thread
* @param void *arg :unused.
*/
static void tcpserver_thread(void *arg)
{
    tcpserv(arg);
    
    printf("Delete Tcpserver thread.\n");
    OSTaskDel(OS_PRIO_SELF);
}

/**
* @brief start tcpserver
*/
void tcpserver2_init()
{
    CPU_INT08U  os_err;
    
    OSTaskCreate(tcpserver_thread, 
                 NULL, 
                 &TcpServerStack[TCPSERVER2_STK_SIZE - 1], 
                 LWIP_TCPSERVER2_PRIO
                     );
#if (OS_TASK_NAME_SIZE >= 11)
    OSTaskNameSet(LWIP_TCPSERVER2_PRIO, (CPU_INT08U *)"tcpserver", &os_err);
#endif
}

/*-- File end --*/

