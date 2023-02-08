/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/13 10:13a $
 * @brief    Implement timer counting in periodic mode.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"


#define PLL_CLOCK           50000000



void OpenKeyPad(void)
{
  GPIO_SetMode(PA, BIT0, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT1, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT3, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT4, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT5, GPIO_PMD_QUASI);						
}

uint8_t ScanKey(void)
{
  PA0=1; PA1=1; PA2=0; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 1;
	if (PA4==0) return 4;
	if (PA5==0) return 7;
  PA0=1; PA1=0; PA2=1; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 2;
	if (PA4==0) return 5;
	if (PA5==0) return 8;
  PA0=0; PA1=1; PA2=1; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 3;
	if (PA4==0) return 6;
	if (PA5==0) return 9;
	return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC100Series.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }
}


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

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);    
    
    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable Timer 0~3 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);    

    /* Select Timer 0~3 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, NULL);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART0_Init(void)
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;
		uint32_t count[3] = {0};
		uint32_t iscounta = 1;
		uint32_t iscountb = 1;
		uint32_t counta = 1;
		uint32_t countb = 1;
		uint32_t countc = 1;
		uint32_t last_time;
		uint32_t scan_key;
		
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 6);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Clear Timer0 ~ Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);
		
		OpenKeyPad();

		
    /* Check Timer0 ~ Timer3 interrupt counts */
    printf("# Timer interrupt counts :\n");
		last_time = u32InitCount;
    while(1)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
						scan_key = ScanKey();
						switch(scan_key){
								case 1:
										iscounta ^= 1;
										break;
								case 2:
										iscountb ^= 1;
										break;
						}
            u32InitCount = g_au32TMRINTCount[0];
            if(last_time != u32InitCount){
								last_time = u32InitCount;
								if(iscounta){
										counta = (counta + 1) % 3;
										if(!counta)
												count[0]++;
								}
								if(iscountb){
										countb = (countb + 1) % 2;
										if(!countb)
												count[1]++;
								}
								countc = (countc + 1) % 6;
								if(!countc)
										count[2]++;
						}
						printf("\rTimer1:%3d    Timer2:%3d    Counter:%3d",
												count[0], count[1], count[2]);
						
				}
    }

    //printf("*** PASS ***\n");

    //while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
