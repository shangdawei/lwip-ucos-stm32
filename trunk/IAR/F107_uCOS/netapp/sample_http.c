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
 
#include "sample_http.h"
//#include "tcp.h"

#if LWIP_TCP

//by millin 2008-01 add counter & runtimer
u32_t runtime=0;
u32_t counter=0;

/*-----------------------------------------------------------------------------------*/
static void
conn_err(void *arg, err_t err)
{
  struct http_state *hs;

  hs = arg;
  mem_free(hs);
}
/*-----------------------------------------------------------------------------------*/
static void
close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  mem_free(hs);
  tcp_close(pcb);
}
/*-----------------------------------------------------------------------------------*/
static void
send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
  err_t err;
  u16_t len;

  /* We cannot send more data than space available in the send
     buffer. */     
  if(tcp_sndbuf(pcb) < hs->left) {
    len = tcp_sndbuf(pcb);
  } else {
    len = hs->left;
  }

  do {
    err = tcp_write(pcb, hs->file, len, 0);
    if(err == ERR_MEM) {
      len /= 2;
    }
  } while(err == ERR_MEM && len > 1);  
  
  if(err == ERR_OK) {
    hs->file += len;
    hs->left -= len;
  }
}
/*-----------------------------------------------------------------------------------*/
static err_t
http_poll(void *arg, struct tcp_pcb *pcb)
{
  struct http_state *hs;

  hs = arg;
  
  /*  printf("Polll\n");*/
  if(hs == NULL) {
    /*    printf("Null, close\n");*/
    tcp_abort(pcb);
    return ERR_ABRT;
  } else {
    ++hs->retries;
    if(hs->retries == 4) {
      tcp_abort(pcb);
      return ERR_ABRT;
    }
    send_data(pcb, hs);
  }

  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
static err_t
http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  struct http_state *hs;

  hs = arg;

  hs->retries = 0;
  
  if(hs->left > 0) {
    send_data(pcb, hs);
  } else {
    close_conn(pcb, hs);
  }

  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
static err_t
http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  int i;
  char *data;
  struct http_state *hs;
  //added by millin, 2008.01
  u8_t hour,min,sec;
  u16_t day;
  

  hs = arg;

  if(err == ERR_OK && p != NULL) {

    /* Inform TCP that we have taken the data. */
    tcp_recved(pcb, p->tot_len);
    
    if(hs->file == NULL) {
      data = p->payload;
      
      if(*data =='G') {
		for(i = 0; i < 40; i++) {
	  		if( ((char *)data + 4)[i] == ' ' || ((char *)data + 4)[i] == '\r' || ((char *)data + 4)[i] == '\n') {
	    				((char *)data + 4)[i] = 0;
	  		}// if
		}//for
	
		// if it was GET dsp.jpg
		if ((*(data+5) =='E') && (*(data+6) =='V') && (*(data+7) =='B')) {
			hs->file = (char *)&jpeg;
			hs->left = sizeof(jpeg)+1;
		}
		// GET other files
		else {
			hs->file = (char *)&demo;
			hs->left = sizeof(demo)+1;

			//-----------------------------------------------------------------
			//counter
			counter ++;
			sprintf(&demo[459],"%.6d",counter);
			demo[465] = 0x20;
			//runtime
			runtime = OSTimeGet() / 100;
			sec = (u8_t)runtime % 60;
			min = (u8_t)((runtime / 60) % 60);
			hour = (u8_t)((runtime / 3600) % 24);
			day = (u16_t)((runtime / 3600) / 24);
			sprintf(&demo[515],"%.3d:%.2d:%.2d:%.2d",day,hour,min,sec);
			demo[527] = 0x20;
			//-----------------------------------------------------------------
		}	
		
		pbuf_free(p);
		send_data(pcb, hs);

		/* Tell TCP that we wish be to informed of data that has been
	   	successfully sent by a call to the http_sent() function. */
		tcp_sent(pcb, http_sent);
      
      } //if(*data =='G')
      
      else {
			pbuf_free(p);
			close_conn(pcb, hs);
      } //else if(*data =='G')
	
	} //if(hs->file == NULL) 
	
	else {
      		pbuf_free(p);
    }
  }

  if(err == ERR_OK && p == NULL) {
    close_conn(pcb, hs);
  }
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
static err_t
http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  struct http_state *hs;

  tcp_setprio(pcb, TCP_PRIO_MIN);
  
  /* Allocate memory for the structure that holds the state of the
     connection. */
  hs = mem_malloc(sizeof(struct http_state));

  if(hs == NULL) {
    return ERR_MEM;
  }
  
  /* Initialize the structure. */
  hs->file = NULL;
  hs->left = 0;
  hs->retries = 0;
  
  /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
  tcp_arg(pcb, hs);

  /* Tell TCP that we wish to be informed of incoming data by a call
     to the http_recv() function. */
  tcp_recv(pcb, http_recv);

  tcp_err(pcb, conn_err);
  
  tcp_poll(pcb, http_poll, 4);
  return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
void
httpd_init(void)
{
  struct tcp_pcb *pcb;

  //by millin 2008.01
  //OSTimeSet((u32_t) 0x00000000);
  counter = (u32_t) 0x00000000;
    
  pcb = tcp_new();
  tcp_bind(pcb, IP_ADDR_ANY, 80);
  pcb = tcp_listen(pcb);
  tcp_accept(pcb, http_accept);

}

#endif //LWIP_TCP
