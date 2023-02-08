/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/13 4:26p $
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NUC100Series.h"
#include "GPIO.h"
#include "SYS.h"
#include "string.h"
#define PLL_CLOCK           50000000


#define RXBUFSIZE 1024 
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);
int main()
{
	/* Unlock protected registers */
	SYS_UnlockReg();
	
	/* Init System, peripheral clock and multi-function I/O */
	SYS_Init();
	
	/* Lock protected registers */
	SYS_LockReg();
	
	/* Init UART0 for printf and testing */
	UART0_Init();

	GPIO_SetMode(PA,BIT12,GPIO_PMD_OUTPUT);
	GPIO_SetMode(PA,BIT13,GPIO_PMD_OUTPUT);
	GPIO_SetMode(PA,BIT14,GPIO_PMD_OUTPUT);
	
	UART_FunctionTest(); 
	while(1);
}

void UART02_IRQHandler(void) 
{
    UART_TEST_HANDLE();
}

//what is call back function?
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->ISR; //ask
		uint8_t *txt=g_u8RecData; 
	
		char RGB_LED[6][20];
		strcpy(RGB_LED[0],"blue on");
		strcpy(RGB_LED[1],"blue off");
		strcpy(RGB_LED[2],"green on");
		strcpy(RGB_LED[3],"green off");
		strcpy(RGB_LED[4],"red on");
		strcpy(RGB_LED[5],"red off");
	
    if(u32IntSts & UART_ISR_RDA_INT_Msk) //
    {
				//if(!g_u8RecData[0]) printf("Input:");
				u8InChar = UART_READ(UART0);
			  if(u8InChar == '0') //input 0 to end.
          {
                g_bWait = FALSE;
          }
      if(u8InChar==0X0D){
						g_u8RecData[g_u32comRtail]='\0'; //end of string
						printf("Input:%s\n",(char*)txt);
				
						if(strcmp(RGB_LED[0],(char*)txt)==0)
						{
								PA12=0;
						}
						if(strcmp(RGB_LED[1],(char*)txt)==0)
						{
								PA12=1;
						}
						if(strcmp(RGB_LED[2],(char*)txt)==0)
						{
								PA13=0;
						}
						if(strcmp(RGB_LED[3],(char*)txt)==0)
						{
								PA13=1;
						}
						if(strcmp(RGB_LED[4],(char*)txt)==0)
						{
								PA14=0;
						}
						if(strcmp(RGB_LED[5],(char*)txt)==0)
						{
								PA14=1;
						}
						//reset
						g_u8RecData[0]=0;
						g_u32comRtail=0;
						g_u32comRbytes=0;
				}					
			else if(g_u32comRbytes < RXBUFSIZE)
        {
            g_u8RecData[g_u32comRtail] = u8InChar; //put the char in g_u8RecData
            g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1); //check oversize
            g_u32comRbytes++;
        }
		}
}
void UART_FunctionTest()
{
	/*
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");
	*/
    /*
        Using a RS232 cable to connect UART0 and PC.
        UART0 is set to debug port. UART0 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART0 will print the received char on screen.
    */
	
		printf("LAB2-UART\n");
	
    /* Enable Interrupt and install the call back function */
    UART_EnableInt(UART0, ( UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    while(g_bWait);

    /* Disable Interrupt */
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    g_bWait = TRUE;
    printf("\nUART Sample Demo End.\n");

}

