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


void GPAB_IRQHandler(void){
    /* To check if PA interrupt occurred */
	if(GPIO_GET_INT_FLAG(PA, BIT0)) 				//key3
		{
				GPIO_CLR_INT_FLAG(PA, BIT0);
        printf("Key3 be pressed\n");
			
    }
		else if(GPIO_GET_INT_FLAG(PA, BIT1)) 	//key2
		{
				GPIO_CLR_INT_FLAG(PA, BIT1);
				printf("Key2 be pressed\n");
		}
		else if(GPIO_GET_INT_FLAG(PA, BIT2))	//key1
		{
				GPIO_CLR_INT_FLAG(PA, BIT2);
				printf("Key1 be pressed\n");
		}
}

void SYS_Init(void){
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART0_Init(void){
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
int main(void){
    volatile uint32_t u32InitCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
	
	/*  Configure PE.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
	GPIO_SetMode(PA, BIT0, GPIO_PMD_QUASI);
	GPIO_SetMode(PA, BIT1, GPIO_PMD_QUASI);
	GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
  GPIO_SetMode(PA, BIT3, GPIO_PMD_QUASI);
	GPIO_SetMode(PA, BIT4, GPIO_PMD_QUASI);
	GPIO_SetMode(PA, BIT5, GPIO_PMD_QUASI);
	
 	GPIO_EnableInt(PA, 0, GPIO_INT_FALLING);
	GPIO_EnableInt(PA, 1, GPIO_INT_FALLING);
	GPIO_EnableInt(PA, 2, GPIO_INT_FALLING);
	
  NVIC_EnableIRQ(GPAB_IRQn);
		
	PA3=0; PA4=1; PA5=1;

	/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0 | BIT1 | BIT2);

		printf("Hi\n");
		
		while(1);
   
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

