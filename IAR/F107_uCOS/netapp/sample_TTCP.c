/*
*********************************************************************************************************
*                                     lwIP TCP/IP Stack & uC/OS-II RTOS
*                                         port for Freescale MCF5223x
*
* File : sample_http.c
*
* By   : Ming Zeng <vivimillin@gmail.com>, Changxing Lin
*        Tsinghua Freescale Embedded System Center, Beijing, P.R.China, <2008.01>
*********************************************************************************************************
*/
 
//#include "includes.h"
//#include "cc.h"
#include <string.h>

#include "udp.h"

#include "tcp.h"


#define	PCATTCP_VERSION				"2.01.01.08"

#define	IPPORT_TTCP					5001

typedef
struct _TEST_BLOCK
{
	u16_t m_bRunningUDP;

  u16_t remote_port;
  
  struct in_addr remote_ip;
  
    char *           m_pBuf;              // ptr to dynamic buffer (aligned)
  	s32_t         m_nBufferSize;       // length of buffer

    u32_t           m_tStart;
    u32_t           m_tFinish;

    u32_t			m_numCalls;          // # of I/O system calls
    u32_t			m_nbytes;         // bytes on net
} 
TEST_BLOCK, *PTEST_BLOCK;


void TTCP_InitStatistics(PTEST_BLOCK pTBLK)
{
   pTBLK->m_numCalls = 0;       // # of I/O system calls
   pTBLK->m_nbytes = 0;      // bytes on net

   pTBLK->m_tStart = OSTimeGet();//GetTickCount();
}


void TTCP_LogStatistics(PTEST_BLOCK pTBLK)
{
	float realt;                 // user, real time (seconds)

	pTBLK->m_tFinish = OSTimeGet();//GetTickCount();

	realt = ((float )pTBLK->m_tFinish - (float )pTBLK->m_tStart)/100;//1000

#if USE_UDP
	printf( "\r\n  Statistics  : UDP <- %s:%d\r\n",
	inet_ntoa( pTBLK->remote_ip ),
	htons( pTBLK->remote_port ) );
#else
	printf( "\r\n  Statistics  : TCP <- %s:%d\r\n",
	inet_ntoa( pTBLK->remote_ip ),
	htons( pTBLK->remote_port ) );
#endif

   printf( 
      "%ld bytes in %ld real seconds = %ld KB/sec +++\r\n",
      (u32_t)pTBLK->m_nbytes,
      (u32_t)realt,
      (u32_t)(pTBLK->m_nbytes/(realt*1024.0))
      );

   printf( "numCalls: %ld; msec/call: %ld; calls/sec: %ld\r\n",
      (u32_t)pTBLK->m_numCalls,
      (u32_t)(1024.0 * realt/((u32_t)pTBLK->m_numCalls)),
      (u32_t)(((u32_t)pTBLK->m_numCalls)/realt)
      );
}

static void
TTCP_errTCP(void *arg, err_t err)
{
	PTEST_BLOCK pTBLK = arg;
	
	TTCP_LogStatistics(pTBLK);
	
	mem_free(pTBLK);
}

static void
TTCP_closeTCP(struct tcp_pcb *pcb, PTEST_BLOCK pTBLK)
{
	TTCP_LogStatistics(pTBLK);

	tcp_arg(pcb, NULL);
	
	tcp_recv(pcb, NULL);

	tcp_close(pcb);
	
	mem_free(pTBLK);
}

static err_t
TTCP_recvTCP(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	char *data;
	PTEST_BLOCK pTBLK = arg;

	if(p != NULL)
	{
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		data = p->payload;

		pTBLK->m_numCalls++;

		pTBLK->m_nbytes += p->tot_len;

		//LED3_TOGGLE;

		pbuf_free(p);
	}
	else
	{
		TTCP_closeTCP(pcb, pTBLK);
	}
	
	return ERR_OK;
}


static err_t
TTCP_initTCP(void *arg, struct tcp_pcb *pcb, err_t err)
{
	PTEST_BLOCK pTBLK;

	/* Allocate memory for the structure that holds the state of the connection. */
	pTBLK = mem_malloc(sizeof(TEST_BLOCK));

	if(pTBLK == NULL)
	{
		puts( "*** TTCP Error: mem_malloc(TEST_BLOCK) Failed.\r\n");
		return ERR_MEM;
	}


	memset( pTBLK, 0x00, sizeof( TEST_BLOCK ) );

	memcpy(&(pTBLK->remote_ip), &(pcb->remote_ip), sizeof(struct in_addr));
	
	pTBLK->remote_port = pcb->remote_port;
	
	/* Tell TCP that this is the structure we wish to be passed for our
	callbacks. */
	tcp_arg(pcb, pTBLK);

	/* Tell TCP that we wish to be informed of incoming data by a call
	to the TTCP_recvTCP() function. */
	tcp_recv(pcb, TTCP_recvTCP);

	tcp_err(pcb, TTCP_errTCP);
	
	puts( "**************\r\n" );
	printf( "  Listening...: On port %d\r\n", IPPORT_TTCP );
	
	printf( "\r\n  Accept      : TCP <- %s:%d\r\n",
	inet_ntoa( pTBLK->remote_ip ),
	htons( pTBLK->remote_port ) );

	TTCP_InitStatistics(pTBLK);
	
	puts( "  Receive Mode: Sinking (discarding) Data\r\n" );
	
	return ERR_OK;
}

void TTCP_init( void )
{
	struct tcp_pcb *pcb;
	
    printf( "PCAUSA Test TCP Utility V%s\r\n", PCATTCP_VERSION );

	pcb = tcp_new();

	tcp_bind(pcb, IP_ADDR_ANY, IPPORT_TTCP);//5001

	pcb = tcp_listen(pcb);

	if (pcb == NULL)
	{
		puts( "*** TTCP Error: tcp_listen Failed.\r\n");
		return;
	}
		
	tcp_accept(pcb, TTCP_initTCP);
	
	puts( "TCP Receive Test\r\n" );
}

