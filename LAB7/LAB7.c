/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/04/20 2:56p $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"

#define PLL_CLOCK           50000000

#define TEST_COUNT             64
//This is a list of some of the registers available on the ADXL345.
char POWER_CTL = 0x2d;  //Power Control Register (0x2d,0x08)
char DATA_FORMAT = 0x31;   //(0x31,0x0b)
char FIFO_CTL=0x38;  //(0x38, 0x80)
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

int i,j;
/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

void SPI_write(uint8_t address,int8_t data)
{
		SPI_SET_SS0_LOW(SPI2);		//SPI2->SSR |=0x1;
	
		SPI2->TX[0] =address;
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));
		SPI2->TX[0] =data;
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));
		SPI_ClearTxFIFO(SPI2);
	
		SPI_SET_SS0_HIGH(SPI2);
}
int8_t SPI_read(uint8_t address)
{
		SPI_SET_SS0_LOW(SPI2);		//SPI2->SSR |=0x1;	
	
		SPI2->TX[0] =(0x80|address);//0x80=1000 0000
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));
		SPI_TRIGGER(SPI2);
		while(SPI_IS_BUSY(SPI2));	
	
		SPI_SET_SS0_HIGH(SPI2);
	
		return SPI_READ_RX0(SPI2);
}

float axisdata[3]={0};
/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    while(1){
			axisdata[0]=((float)((SPI_read(DATAX1) << 8) | SPI_read(DATAX0))/256);
			axisdata[1]=((float)((SPI_read(DATAY1) << 8) | SPI_read(DATAY0))/256);
			axisdata[2]=((float)((SPI_read(DATAZ1) << 8) | SPI_read(DATAZ0))/256);
			for( i=0;i<2000000;i++){}
			printf("x=%.2f	y=%.2f	z=%.2f\n",axisdata[0],axisdata[1],axisdata[2]);
		}
    while(1);
}

void SYS_Init(void)
{
    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI2 */
		//******
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL1_SPI2_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
		
    /* Enable SPI2 peripheral clock */
		//******
    CLK_EnableModuleClock(SPI2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Setup SPI2 multi-function pins */
    SYS->GPD_MFP = SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PD0_SPI2_SS0 | SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;//////******/////////

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void SPI_Init(void)
{
		int offset[3],loop=20;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
		printf("\nStart...\n");
		
    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_2, 0x08, 2000000);
		
		SPI2->CNTRL |=0x846; //???????????????
		SPI_DisableAutoSS(SPI2);
		SPI_SET_SS0_HIGH(SPI2);
	
		//write
		SPI_write(DATA_FORMAT,0x0b); 
		SPI_write(POWER_CTL,0x08);
		SPI_write(FIFO_CTL,0x80);
		for(i=0;i<3;i++)
				SPI_write(0x1e+i,0);
		
		//identify
		printf("device id: %x\n",(uint8_t)SPI_read(0x00));
		for(j=0;j<3000000;j++){}
		
		for(j=0;j<loop;j++)
		{
					for(i=0;i<3;i++){
							offset[i] += ((SPI_read(0x33+(2*i))<<8) | SPI_read(0x32+(2*i)));
					}
		}
		SPI_write(0x1e,-1*offset[0]/(4*loop));
		SPI_write(0x1f,-1*offset[1]/(4*loop));
		SPI_write(0x20,-1*(offset[2]-256)/(4*loop));
}




