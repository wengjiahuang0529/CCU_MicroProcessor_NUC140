/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/13 10:17a $
 * @brief
 *           Show how to generate time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC100Series.h"


#define PLL_CLOCK           50000000
#define RXBUFSIZE           1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8IsWDTTimeoutINT = 0;
char msg[RXBUFSIZE] = {0};
volatile uint8_t alarm = 0;

/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_NUC100Series.s.
 */
void WDT_IRQHandler(void){
    if(WDT_GET_TIMEOUT_INT_FLAG() == 1){
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsWDTTimeoutINT = 1;

        //printf("\nwatch dog timer occurred!!!");
		strcpy(msg ,"\nwatch dog timer occurred!!!");
		UART_EnableInt(UART0, (UART_IER_THRE_IEN_Msk));
		
		if(!alarm){
			WDT_CLEAR_RESET_FLAG();
            WDT_RESET_COUNTER();
			//printf("\nNo problem~~~");
			strcpy(msg ,"\n\rNo problem~~~");
		}else{
			//printf("\nalarm!!!~~~Reset!!!\n\n");
			strcpy(msg ,"\n\ralarm!!!~~~Reset!!!\n\n");
			UART_EnableInt(UART0, (UART_IER_THRE_IEN_Msk));
		}
    }
}

void GPAB_IRQHandler(void){
    /* To check if PA interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT3)){
		GPIO_CLR_INT_FLAG(PA, BIT3);
		alarm ^= 1;
		//printf("\nchange!!!");
		strcpy(msg ,"\n\rchange!!!");
		UART_EnableInt(UART0, (UART_IER_THRE_IEN_Msk));
	}else{
        /* Un-expected interrupt. Just clear all PA, PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
    }
}

void UART02_IRQHandler(void){
	uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->ISR;
	int i;
	
    if(u32IntSts & UART_ISR_THRE_INT_Msk){
		for(i = 0;i < strlen(msg);i++){
			u8InChar = msg[i];
			UART_WRITE(UART0, u8InChar);
		}	
		memset(msg,'\0',1024);
		UART_DisableInt(UART0, ( UART_IER_THRE_IEN_Msk));
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

    /* Enable external XTAL 12MHz clock and internal 10 kHz */
    CLK_EnableXtalRC((CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk));

    /* Waiting for clock ready */
    CLK_WaitClockReady((CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);    
    
    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    
    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);    
        
    /* Select WDT module clock source */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, NULL);

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

void GPIO_Init(){
	GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
	
    GPIO_SetMode(PA, BIT3, GPIO_PMD_QUASI);
    GPIO_EnableInt(PA, 3, GPIO_INT_FALLING);
	

    NVIC_EnableIRQ(GPAB_IRQn);

    PA->ISRC = PA->ISRC;
    PB->ISRC = PB->ISRC;
	/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_64);
    GPIO_ENABLE_DEBOUNCE(PA, BIT3);
    PA2=0; PA3=1;
}



/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void){
	int delay = 5000000;
	
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
	
	/* Init GPIO for printf */
	GPIO_Init();

    printf("\rStart Lab5");

    /* Use PA.0 to check time-out period time */
    GPIO_SetMode(PA, 0, GPIO_PMD_OUTPUT);
    PA0 = 1;
    PA0 = 0;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Enable WDT time-out reset function and select time-out interval to 2^14 * WDT clock then start WDT counting */
    g_u8IsWDTTimeoutINT = 0;
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);
	
	memset(msg,'\0',1024);

    while(1){		
		if(!alarm){
			printf("\nsafe!");
		}else{
			printf("\nalarm!!!");
		}
	CLK_SysTickDelay(delay);	
	}
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

