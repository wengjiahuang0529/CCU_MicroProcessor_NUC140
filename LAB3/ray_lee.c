/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 1 $
 * $Date: 14/12/08 11:49a $
 * @brief    Perform A/D Conversion with ADC single mode.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "NUC100Series.h"


#define PLL_CLOCK       50000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcSingleModeTest(void);
void ADC_Init(void);
void TMR0_IRQHandler(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
volatile uint32_t g_au32TMRINTCount = 0;
volatile uint32_t printflag = 0;


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

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));
		
		/* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
		
		/* Select Timer 0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, NULL);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Disable the GPA0 - GPA3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, 0xF);

    /* Configure the GPA0 - GPA3 ADC analog input pins */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA0_Msk | SYS_GPA_MFP_PA1_Msk | SYS_GPA_MFP_PA2_Msk | SYS_GPA_MFP_PA3_Msk) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 | SYS_GPA_MFP_PA1_ADC1 | SYS_GPA_MFP_PA2_ADC2 | SYS_GPA_MFP_PA3_ADC3 ;
    SYS->ALT_MFP1 = 0;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init ADC                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_Init()
{
		/* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 1 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << 1);

    /* Power on ADC module */
    ADC_POWER_ON(ADC);
	
		/* Clear the A/D interrupt flag for safe */
		ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the ADC interrupt */
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);
}

void GPIO_Init()
{
		GPIO_SetMode(PC, BIT12, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT13, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT14, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT15, GPIO_PMD_OUTPUT);
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC100Series.s.
 */

void TMR0_IRQHandler()
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
        printflag = 1;
    }
}


void AdcSingleModeTest()
{
    volatile int32_t  i32ConversionData;
		int i=-1;
		int j;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADC single mode sample code                     |\n");
    printf("+----------------------------------------------------------------------+\n");
		
		while(1){
			g_u32AdcIntFlag = 0;
			ADC_START_CONV(ADC);

		/* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function)*/
			while(g_u32AdcIntFlag == 0);
						
		/* Get the conversion result of the ADC channel 1 */
			i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 1);
			printf("Conversion result of channel 1: %d\ndelay value: %f s\n", i32ConversionData, ((float)i32ConversionData*5000)/10000000);
			i = i>3 ? 0: i+1;
			*(&PC12+i) = 0;
			//delay
			for(j=0;j<(i32ConversionData)*5500;j++){};
			*(&PC12+i) = 1;
	}
}


/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* clear the A/D conversion flag */
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
		volatile uint32_t u32InitCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);
	
		GPIO_Init();
	
		ADC_Init();

		/* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

		/* Clear Timer0 interrupt counts to 0 */
    g_au32TMRINTCount = 0;
    u32InitCount = g_au32TMRINTCount;

		/* Start Timer0 counting */
    TIMER_Start(TIMER0);



    /* Single Mode test */
    AdcSingleModeTest();
		
		
		
		/* Disable the ADC interrupt */
    ADC_DisableInt(ADC, ADC_ADF_INT);

    /* Disable ADC module */
    ADC_Close(ADC);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while(1);

}



