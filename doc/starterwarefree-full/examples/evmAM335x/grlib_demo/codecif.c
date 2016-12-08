/**
 * \file  codecif.c
 *
 * \brief Functions to configure the codec trough i2c or other interfaces.
 *
 * Currently only one interface type is allowed. If another interface to be
 * used, this need enhancement.
 *
*/

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "soc_AM335x.h"
#include "interrupt.h"
#include "hsi2c.h"
#include "codecif.h"

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void I2CCodecIsr(void);
static void I2CCodecSendBlocking(unsigned int baseAddr, unsigned int dataCnt);
static void I2CCodecRcvBlocking(unsigned int baseAddr, unsigned int dataCnt);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int dataIdx = 0;
volatile unsigned int txCompFlag = 1;
volatile unsigned int slaveData[3];
unsigned int savedBase;
unsigned int dataMax;

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
/**
 * \brief   Initializes the I2C interface for a codec
 *
 * \param   baseAddr      Base Address of the I2C Module Registers which
 *                        is used for the codec
 *          slaveAddr     Slave Address of the codec
 *
 * Note: This API enables the system interrupt for the given I2C module only.
 *       It does not do any pin multiplexing or global interrupt enabling. 
 *       This shall be called only after AINTC initialization.
 *
 * \return  None.
 *
 **/
void I2CCodecIfInit(unsigned int baseAddr, unsigned int slaveAddr)
{
    /* Put i2c in reset/disabled state */
    I2CMasterDisable(baseAddr);

    /* Disable the auto idle functionality */
    I2CAutoIdleDisable(baseAddr);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_1_REGS, 48000000, 12000000, 100000);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(baseAddr, slaveAddr);

    I2CMasterEnable(baseAddr);
   
    IntRegister(SYS_INT_I2C1INT, I2CCodecIsr);
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_I2C1INT);
}

/*
** Function to send data through i2c
*/
static void I2CCodecSendBlocking(unsigned int baseAddr, unsigned int dataCnt)
{
    txCompFlag = 1;
    dataIdx = 0;    
    savedBase = baseAddr;
    dataMax = dataCnt;
 
    I2CSetDataCount(baseAddr, dataCnt);

    I2CMasterControl(baseAddr, I2C_CFG_MST_TX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(baseAddr, I2C_INT_TRANSMIT_READY | I2C_INT_STOP_CONDITION);

    I2CMasterStart(baseAddr);
   
    /* Wait till the data is sent */ 
    while(txCompFlag);
}

/*
** Function to receive data from the Codec through I2C bus
*/
static void I2CCodecRcvBlocking(unsigned int baseAddr, unsigned int dataCnt)
{
    txCompFlag = 1;
    dataIdx = 0;
    savedBase = baseAddr;
    
    I2CSetDataCount(baseAddr, dataCnt);

    I2CMasterControl(baseAddr, I2C_CFG_MST_RX | I2C_CFG_STOP);

    I2CMasterIntEnableEx(baseAddr, I2C_INT_RECV_READY | I2C_INT_STOP_CONDITION);

    I2CMasterStart(baseAddr);

    /* Wait till data is received fully */
    while(txCompFlag);
}

/*
** ISR to handler i2c interrupts
*/
void I2CCodecIsr(void)
{
    unsigned int status;

    status = I2CMasterIntStatus(SOC_I2C_1_REGS);
    I2CMasterIntClearEx(SOC_I2C_1_REGS,
                        (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));

    if (status & I2C_INT_TRANSMIT_READY)
    {
         /* Put data to data transmit register of i2c */
         I2CMasterDataPut(SOC_I2C_1_REGS, slaveData[dataIdx]);
         I2CMasterIntClearEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);
         dataIdx++;
 
         if(dataMax == dataIdx)
         {
              I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY);
         }
    }

    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
         slaveData[dataIdx] = I2CMasterDataGet(SOC_I2C_1_REGS);
         I2CMasterIntClearEx(SOC_I2C_1_REGS, I2C_INT_RECV_READY);
         dataIdx++;
 
         if(dataMax == dataIdx)
         {
              I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_RECV_READY);

              /* Generate a STOP */
              I2CMasterStop(SOC_I2C_1_REGS);
         }
    }

    if (status & I2C_INT_STOP_CONDITION)
    {
         /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_STOP_CONDITION |
                                               I2C_INT_RECV_READY);
         txCompFlag = 0;
    }

    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(SOC_I2C_1_REGS, I2C_INT_TRANSMIT_READY |
                                                 I2C_INT_RECV_READY |
                                                 I2C_INT_NO_ACK |
                                                 I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(SOC_I2C_1_REGS);

         txCompFlag = 0;
    }

}

/*
** Writes a codec register with the given data value
*/
void CodecRegWrite(unsigned int baseAddr, unsigned char regAddr,
                   unsigned char regData)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address and data */
    slaveData[0] = regAddr;
    slaveData[1] = regData;

    I2CCodecSendBlocking(baseAddr, 2);
#endif
}

/*
** Reads a codec register contents
*/
unsigned char CodecRegRead(unsigned int baseAddr, unsigned char regAddr)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    slaveData[0] = regAddr;
    I2CCodecSendBlocking(baseAddr, 1);

    /* Receive the register contents in slaveData */
    I2CCodecRcvBlocking(baseAddr, 1);

#endif

    return (slaveData[0]);
}

/*
** Sets codec register bit specified in the bit mask
*/
void CodecRegBitSet(unsigned int baseAddr, unsigned char regAddr, 
                    unsigned char bitMask)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    slaveData[0] = regAddr;
    I2CCodecSendBlocking(baseAddr, 1);
  
    /* Receive the register contents in slaveData */
    I2CCodecRcvBlocking(baseAddr, 1);

    slaveData[1] =  slaveData[0] | bitMask;
    slaveData[0] = regAddr;

    I2CCodecSendBlocking(baseAddr, 2);

#endif
}

/*
** Clears codec register bits specified in the bit mask
*/
void CodecRegBitClr(unsigned int baseAddr, unsigned char regAddr,    
                    unsigned char bitMask)
{
#ifdef CODEC_INTERFACE_I2C

    /* Send the register address */
    slaveData[0] = regAddr;
    I2CCodecSendBlocking(baseAddr, 1);

    /* Receive the register contents in slaveData */
    I2CCodecRcvBlocking(baseAddr, 1);

    slaveData[1] =  slaveData[0] & ~bitMask;
    slaveData[0] = regAddr;
   
    I2CCodecSendBlocking(baseAddr, 2);

#endif
}

/***************************** End Of File ***********************************/
