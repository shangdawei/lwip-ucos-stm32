/**
  ******************************************************************************
  * @file    stm3210c_eval_lcd.h
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   This file contains all the functions prototypes for the lcd firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210C_EVAL_LCD_H
#define __STM3210C_EVAL_LCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* set LCD command */
#define Reset_LCD         0xe2

#define Set_Start_Line_X  0x40  // Line Address : 0
#define Set_Page_Addr_X   0xb0  // Page Address : 0
#define Set_ColH_Addr_X   0x10
#define Set_ColL_Addr_X   0x0

#define LCD_Reset   0xe2

#define Display_Off   0xae
#define Display_On    0xaf
#define Set_ADC_Normal    0xa0
#define Set_ADC_Reverse   0xa1
#define Set_LCD_Bias_7    0xa3
#define Set_LCD_Bias_9    0xa2
#define RMW_Mode_En       0xe0
#define RMW_Mode_Dis      0xee
#define COM_Scan_Dir_Normal 0xc0
#define COM_Scan_Dir_Reverse  0xc8

#define Set_Resistor_Ratio_X   0x20
#define Set_Ref_Vol_Mode       0x81
//#define Set_Ref_Vol_Reg        0x20
#define Set_Ref_Vol_Reg        0x1F

#define Display_Normal    0xa6
#define Display_Reverse   0xa7
#define Display_All_On    0xa5
#define Display_All_Normal  0xa4


// ZYMG12864
/*A0=0  -- cmd*/
//#define LCD_Command  *((volatile unsigned char * )0x6c000000)
//#define LCD_Command  *((volatile unsigned char * )0x6c000000)
/*A0=1 -- data*/
//#define LCD_Data  *((volatile unsigned char * )0x6c000001)
//#define LCD_Data  *((volatile unsigned char * )0x6c400000)



/*define the constant for display digital char*/
#define	D0		0
#define	D1		1
#define	D2		2
#define	D3		3
#define	D4		4
#define	D5		5
#define	D6		6
#define	D7		7
#define	D8		8
#define	D9		9
#define	DPoint	10	//"."
#define DDash	20	//"-"
#define DColon	21	//":"

#define Da	30
#define Db	31
#define Dc	32
#define Dd	33

#define De	34
#define Df	35
#define Dg	36
#define Dh	37
#define Di	38
#define Dj	39
#define Dk	40
#define Dl	41
#define Dm	42
#define Dn	43
#define Do	44
#define Dp	45
#define Dq	46
#define Dr	47
#define Ds	48
#define Dt	49
#define Du	50

#define Dv	51
#define Dw	52
#define Dx	53
#define Dy	54
#define Dz	55

#define DA	56
#define DB	57
#define DC	58
#define DD	59
#define DE	60
#define DF	61
#define DG	62
#define DH	63
#define DI	64
#define DJ	65
#define DK	66
#define DL	67
#define DM	68
#define DN	69
#define DO	70
#define DP	71
#define DQ	72
#define DR	73
#define DS	74
#define DT	75
#define DU	76
#define DV	77
#define DW	78
#define DX	79
#define DY	80
#define DZ	81

//#define LCD_CS_Low()  GPIOG->BSRR = 0x10000000
//#define LCD_CS_High()  GPIOG->BSRR = 0x00001000
//#define LCD_CS_Low() (*(vu32*)(PERIPH_BB_BASE + (GPIOE_BASE-PERIPH_BASE + 0x10)*32 +18*4)) = 1
//#define LCD_CS_High() (*(vu32*)(PERIPH_BB_BASE + (GPIOE_BASE-PERIPH_BASE + 0x10)*32 +2*4)) = 1
//#define LCD_A22_Low()  GPIOE->BSRR = 0x00400000
//#define LCD_A22_High()  GPIOE->BSRR = 0x00000040
//#define LCD_CMD_WR_EN()     {GPIOC->ODR &= (~0x00000280);} 
#define LCD_Pin_CorD    GPIO_Pin_8
#define LCD_Pin_WR      GPIO_Pin_7
#define LCD_Pin_RD      GPIO_Pin_6
#define LCD_Pin_CS      GPIO_Pin_9

#define Line1   0
#define Line2   2
#define Line3   4
#define Line4   6

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void STM3210C_LK_LCD_Init(void);
void LCD_SetTextColor(vu16 Color);
void LCD_SetBackColor(vu16 Color);
void LCD_ClearLine(u8 Line);
void LCD_Clear(void);
void LCD_SetCursor(u8 Xpos, u16 Ypos);
void LCD_DrawChar(u8 Xpos, u8 Ypos, u8 offset);
void LCD_DisplayChar(u8 Line, u16 Column, u8 Ascii);
void LCD_DisplayStringLine(u8 Line, u8 *ptr);
void LCD_SetDisplayWindow(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(u8 Xpos, u16 Ypos, u16 Length, u8 Direction);
void LCD_DrawRect(u8 Xpos, u16 Ypos, u8 Height, u16 Width);
void LCD_DrawCircle(u8 Xpos, u16 Ypos, u16 Radius);
void LCD_DrawMonoPict(uc32 *Pict);
void LCD_WriteBMP(u32 BmpAddress);
void LCD_Puts(u8 *ptr);

/*----- Medium layer function -----*/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
u16 LCD_ReadRAM(void);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

/*----- Low layer function -----*/
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);

void delay(void);
void LCD_Draw_ST_Logo(void);
void LCD_Cursor(void);
u8 LCD_DrawString(u8 Xpos, u8 Ypos, u8 *c, u8 length);

void Converse_Logo(void);
void LCD_CMD(u8 cmd);
void LCD_DAT(const u8 dat);


#endif /* __STM3210C_EVAL_LCD_H */
/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */ 
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
