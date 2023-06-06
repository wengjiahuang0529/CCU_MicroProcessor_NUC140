/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/12/29 3:22p $
 * @brief
 *           Show a I2C Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define g_u8DeviceAddr 0x53
#define offsetx 0x1E 
#define offsety 0x1F
#define offsetz 0x20
volatile uint8_t g_DataAddr;
volatile uint8_t g_au8MstTxData;
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t Read;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);////(I2C)->I2CSTATUS

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
				I2C_SET_DATA(I2C0, g_DataAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
    } 
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
		else if(u32Status == 0x40)                  // SLA+R has been transmitted and ACK has been received 
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
		else if(u32Status == 0x48)                  //SLA+R has been transmitted and NOT ACK has been received.
    {
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    }
		else if(u32Status == 0x58)                  // DATA has been received and NACK has been returned 
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
        g_u8MstEndFlag = 1;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_DataAddr);
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
       			if(I2C0->I2CDAT != g_au8MstTxData){ 				//Write register
								I2C_SET_DATA(I2C0, g_au8MstTxData);
								I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
						}
						else{																				//Write data
								I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
								g_u8MstEndFlag = 1;
						}
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
		else if(u32Status == 0x40)                  // SLA+R has been transmitted and ACK has been received 
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
		else if(u32Status == 0x48)                  //SLA+R has been transmitted and NOT ACK has been received.
    {
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_STO_SI);
    }
		else if(u32Status == 0x58)                  // DATA has been received and NACK has been returned 
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
        g_u8MstEndFlag = 1;
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

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP = SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C_Write(uint8_t slvaddr,uint8_t Data)
{
				g_DataAddr = slvaddr;
				g_au8MstTxData=Data;

				g_u8MstEndFlag = 0;

        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

        /* Wait I2C Tx Finish */
        while(g_u8MstEndFlag == 0);

}

int8_t I2C_Read(uint8_t slvaddr)
{
				g_DataAddr = slvaddr;
				g_u8MstEndFlag = 0;
	
				/* I2C function to read data to slave */
				s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
	
				/* I2C as master sends START signal */
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);
			
				while(g_u8MstEndFlag == 0);
				return (I2C_GET_DATA(I2C0)); //I2C0->I2CDAT
}

void I2C0_Init(void)
{	
		int offset[3];
		int t=10;
		int i,j;
		printf("\nADXL init \n");
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
	
		I2C_Write(0x2D,0x08); //POWER_CTL(0x2D): 0x08
	
		I2C_Write(0x31,0x0b);	//DATA_FORMAT(0x31): 0x0B
		//The default configuration of the interrupt pins is active high.
		//It can be changed to active low by setting the INT_INVERT bit.

		I2C_Write(0x38,0x80);	//FIFO_CTL(0x38): 0x80  
		//The watermark bit is set when the number of samples in FIFO equals the value stored in the samples bits

			for(i=0;i<t;i++) //set the offset value. First, we need sample the value.
		{
				for(j=0;j<3;j++){
						offset[j] += ((I2C_Read(0x33+(2*j))<<8) | I2C_Read(0x32+(2*j))); 
					//The output data is twos complement, with DATAx0 as the least significant byte and DATAx1 as the most significant byte
				}
		}
		
		//Offset xyz
		I2C_Write(offsetx,-1*(offset[0])/(4*t));
		I2C_Write(offsety,-1*(offset[1])/(4*t));
		I2C_Write(offsetz,-1*(offset[2]-256)/(4*t));
		
		
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
		float axis[3]={0};
		int i;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("| NUC100 I2C Driver Sample Code(Master) for access Slave |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("+--------------------------------------------------------+\n");

    printf("Configure I2C0 as a master.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PA.8), I2C0_SCL(PA.9)\n");

    /* Init I2C0 */
    I2C0_Init();
		
		printf("Device Id: %x\n",(uint8_t)I2C_Read(0x00));
				
			while(1){
			for(i=0;i<3;i++){
				axis[i]=((float)((I2C_Read(0x33+2*i) << 8) | I2C_Read(0x32+2*i))/256);
			}
			CLK_SysTickDelay(10000000);
			printf("x=%.2f	y=%.2f	z=%.2f\n",axis[0],axis[1],axis[2]);
		}

    s_I2C0HandlerFn = NULL;

    /* Close I2C0 */
    I2C0_Close();

    while(1);
}



