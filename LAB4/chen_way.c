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

#define PLL_CLOCK           50000000 //50Mhz->XTAL=12Mhz, 12Mhz->XTAL=12Mhz, 4Mhz->XTAL=4Mhz

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[3] = {0};//timer counter
volatile int t_flag[3] = {0} ,t_cmd[3] = {0};//interrupt state

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC100Series.s.
 */
void TMR0_IRQHandler(void){
    if(TIMER_GetIntFlag(TIMER0) == 1){ //1 is time-out interrupt occered, 0 is not
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);//clear timer time-out interrupt flag 0000 0000 0000 0001
			if(!t_cmd[0]){//t_cmd[0]=0			
				g_au32TMRINTCount[0]++;
				t_flag[0] = 1;
			}       
    }
}


void TMR3_IRQHandler(void){
    if(TIMER_GetIntFlag(TIMER3) == 1){
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);
			if(!t_cmd[1]){			
				g_au32TMRINTCount[1]++;
				t_flag[1] = 1;
			}		       
    }
}

void TMR1_IRQHandler(void){
    if(TIMER_GetIntFlag(TIMER1) == 1){
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
			if(!t_cmd[2]){			
				g_au32TMRINTCount[2]++;
				t_flag[2] = 1;
			}		       
    }
}

void GPAB_IRQHandler(void){//gpio_INT
    // To check if PA interrupt occurred 
    if(GPIO_GET_INT_FLAG(PA, BIT0)){
       GPIO_CLR_INT_FLAG(PA, BIT0);
			g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = 0;
			t_cmd[2] = 1;
    }
		else if(GPIO_GET_INT_FLAG(PA, BIT1)){
			GPIO_CLR_INT_FLAG(PA, BIT1);
			t_cmd[1] = !t_cmd[1];
		}
		else if(GPIO_GET_INT_FLAG(PA, BIT2)){
			GPIO_CLR_INT_FLAG(PA, BIT2);
			t_cmd[0] = !t_cmd[0];
		}
		else{
      //Un-expected interrupt. Just clear all PA, PB interrupts 
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
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

    /* Enable Timer 0 ,3 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);        
    CLK_EnableModuleClock(TMR3_MODULE);    
		CLK_EnableModuleClock(TMR1_MODULE);
		
    /* Select Timer 0 ,3 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, NULL);//XTAL=12Mhz
		CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, NULL);//???
	//CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HIRC, NULL);//HIRC=22.1184Mhz	
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HXT, NULL);

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
void count_t(void){
		printf("\r                                  ");
		printf("count: %d\r\n",g_au32TMRINTCount[2]);
}
void print_time(void){
		//printf("\r                                  ");
		printf("\rtimer0:%d\ttimer1:%d" ,g_au32TMRINTCount[0] ,g_au32TMRINTCount[1]);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void){

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d MHz\n", SystemCoreClock/1000000);
    printf("+------------------------------------------------+\n");
		printf("|                  Start Timer:                  |\n");
    printf("+------------------------------------------------+\n\n");


    /* Open Timer0 in periodic mode, enable interrupt and 2 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 2);
    TIMER_EnableInt(TIMER0);
    /* Open Timer3 in periodic mode, enable interrupt and 3 interrupt ticks per second */
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 3);
    TIMER_EnableInt(TIMER3);
		/* Open Timer3 in periodic mode, enable interrupt and 1 interrupt ticks per second */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER1);

    /* Enable Timer0 ,Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);//Interrupt Request 
    NVIC_EnableIRQ(TMR3_IRQn);
		NVIC_EnableIRQ(TMR1_IRQn);
		
    /* Clear Timer0 ,Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2]= 0;
	
    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
		TIMER_Start(TIMER1);
	/*  Configure PE.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
		GPIO_SetMode(PA, BIT0, GPIO_PMD_QUASI);//why not INPUT??
		GPIO_SetMode(PA, BIT1, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT3, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT4, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT5, GPIO_PMD_QUASI);
		GPIO_EnableInt(PA, 0, GPIO_INT_FALLING);
		GPIO_EnableInt(PA, 1, GPIO_INT_FALLING);
		GPIO_EnableInt(PA, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPAB_IRQn);//Enable External Interrupt
		PA3=0; 
		PA4=1; 
		PA5=1;
	
	/* Enable interrupt de-bounce and select sampling cycle time is 16 clocks*/ 
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_16);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0 | BIT1 | BIT2);
		
    printf("# Timer interrupt counts :\n");
	/* Check Timer0 ~ Timer3 interrupt counts */
    while(1){//waiting for interrupt
			if(t_flag[2]==1){
				t_flag[2] = 0;
				count_t();
				print_time();
			}
			if(t_flag[0] == 1){
				t_flag[0] = 0;
				print_time();
			}
			if(t_flag[1] == 1){
				t_flag[1] = 0;
				print_time();
			}	
			if(t_cmd[2] == 1){
				t_cmd[2] = 0;
				print_time();
			}
   }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
