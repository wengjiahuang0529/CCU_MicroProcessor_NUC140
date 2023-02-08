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
																																					
    /* Select UART module clock source */																						//Ask the different between HXT , HCLK , HIRC 						
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));		//!!Select Timer clock source !!

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
	
    /* Unlock protected registers */
    SYS_UnlockReg();
	
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
	
    /* Lock protected registers */
    SYS_LockReg();
	
    /* Init UART0 for printf */
    UART0_Init();
	
    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);						//Go to the definition to check.
    TIMER_EnableInt(TIMER0);															//when the time is up make enable Interrupt
	
    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);									//enable corresponding function.	This is very important
	
    /* Clear Timer0 ~ Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0]= 0;																
    u32InitCount = g_au32TMRINTCount[0];
		
    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);  																//When count to zero will interrupt and into the corresponding func.

    /* Check Timer0 ~ Timer3 interrupt counts */
    printf("# Timer interrupt counts :\n");
    while(1)
    { 		
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            printf("TMR0:%3d\n",g_au32TMRINTCount[0]);
            u32InitCount = g_au32TMRINTCount[0];
        }
    }
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
