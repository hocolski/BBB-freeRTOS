//*****************************************************************************
//
// usb_dev_serial.c - Main routines for the USB CDC serial example.
//
// This is part of Beaglebone Starteraware firmware package, modified and reused from revision 6288
// of the DK-LM3S9B96 Firmware Package.
//
//*****************************************************************************

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

#include "ustdlib.h"
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "hw_types.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "usblib.h"
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usb.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usb_serial_structs.h"
#include "hw_usb.h"
#include "delay.h"
#include "cache.h"
#include "mmu.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Serial Device (usb_dev_serial)</h1>
//!
//! This example application turns the evaluation kit into a virtual serial
//! port when connected to the USB host system.  The application supports the
//! USB Communication Device Class, Abstract Control Model to redirect UART0
//! traffic to and from the USB host system.  File usb_dev_serial_win2k.inf may
//! be used to install the example as a virtual COM port on a Windows2000
//! system.  For WindowsXP or Vista, usb_dev_serial.inf should be used.
//
//*****************************************************************************

//*****************************************************************************
//
// Note:
//
// This example is intended to run on Stellaris evaluation kit hardware
// where the UARTs are wired solely for TX and RX, and do not have GPIOs
// connected to act as handshake signals.  As a result, this example mimics
// the case where communication is always possible.  It reports DSR, DCD
// and CTS as high to ensure that the USB host recognizes that data can be
// sent and merely ignores the host's requested DTR and RTS states.  "TODO"
// comments in the code indicate where code would be required to add support
// for real handshakes.
//
//*****************************************************************************

//*****************************************************************************
//
// Configuration and tuning parameters.
//
//*****************************************************************************
typedef struct
{
    tUSBRingBufObject sRingBuf;
}
tUSBBufferVars;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile unsigned int g_ulUARTTxCount = 0;
volatile unsigned int g_ulUARTRxCount = 0;
#ifdef DEBUG
unsigned int g_ulUARTRxErrors = 0;
#endif

//*****************************************************************************
//
// Defines required to redirect UART0 via USB.
//
//*****************************************************************************
#define USB_UART_BASE           SOC_UART_0_REGS

//*****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//*****************************************************************************
#define DEFAULT_BIT_RATE        115200
#define BAUD_RATE_115200          (115200)
#define UART_MODULE_INPUT_CLK     (48000000)


//*****************************************************************************
//
// Flag indicating whether or not we are currently sending a Break condition.
//
//*****************************************************************************
static tBoolean g_bSendingBreak = false;

//*****************************************************************************
//
// Global system tick counter
//
//*****************************************************************************
volatile unsigned int g_ulSysTickCount = 0;

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_STATUS_UPDATE   0x00000002
volatile unsigned int g_ulFlags = 0;
char *g_pcStatus;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile tBoolean g_bUSBConfigured = false;

//*****************************************************************************
//
// Internal function prototypes.
//
//*****************************************************************************
static void USBUARTPrimeTransmit(unsigned int ulBase);
static void CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, int lErrors);
static void SetControlLineState(unsigned short usState);
static tBoolean SetLineCoding(tLineCoding *psLineCoding);
static void GetLineCoding(tLineCoding *psLineCoding);
static void SendBreak(tBoolean bSend);
static void MMUConfigAndEnable(void);

#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)

/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned int ulLine)
{
    while(1)
    {
    }
}
#endif


//*****************************************************************************
//
// This function is called whenever serial data is received from the UART.
// It is passed the accumulated error flags from each character received in
// this interrupt and determines from them whether or not an interrupt
// notification to the host is required.
//
// If a notification is required and the control interrupt endpoint is idle,
// we send the notification immediately.  If the endpoint is not idle, we
// accumulate the errors in a global variable which will be checked on
// completion of the previous notification and used to send a second one
// if necessary.
//
//*****************************************************************************

static void
CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, int lErrors)
{
    unsigned short usSerialState;

    //
    // Clear our USB serial state.  Since we are faking the handshakes, always
    // set the TXCARRIER (DSR) and RXCARRIER (DCD) bits.
    //
    usSerialState = USB_CDC_SERIAL_STATE_TXCARRIER |
                    USB_CDC_SERIAL_STATE_RXCARRIER;

    //
    // Are any error bits set?
    //
    if(lErrors)
    {
        //
        // At least one error is being notified so translate from our hardware
        // error bits into the correct state markers for the USB notification.
        //
        if(lErrors & UART_OVERRUN_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_OVERRUN;
        }

        if(lErrors & UART_PARITY_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_PARITY;
        }

        if(lErrors & UART_FRAMING_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_FRAMING;
        }

        if(lErrors & UART_BREAK_DETECTED_ERROR)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_BREAK;
        }

       // Call the CDC driver to notify the state change.
        USBDCDCSerialStateChange((void *)psDevice, usSerialState);
    }
}

/*
** Function to setup MMU. This function Maps three regions (1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                     MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                      MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

/*
** A wrapper function performing FIFO configurations.
*/

static void UartFIFOConfigure(void)
{
    unsigned int fifoConfig = 0;

    /* Setting the TX and RX FIFO Trigger levels as 1. No DMA enabled. */
    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_1,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  1,
                                  1,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_0_ENABLE);

    /* Configuring the FIFO settings. */
    UARTFIFOConfig(SOC_UART_0_REGS, fifoConfig);
}


/*
** A wrapper function performing Baud Rate settings.
*/
static void UartBaudRateSet(void)
{
    unsigned int divisorValue = 0;

    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         BAUD_RATE_115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(SOC_UART_0_REGS, divisorValue);
}

//*****************************************************************************
//
// Read as many characters from the UART FIFO as we can and move them into
// the CDC transmit buffer.
//
// \return Returns UART error flags read during data reception.
//
//*****************************************************************************

static int
ReadUARTData(void)
{
     int lChar, lErrors;
    unsigned char ucChar;
    unsigned int ulSpace;

    //
    // Clear our error indicator.
    //
    lErrors = 0;

    //
    // How much space do we have in the buffer?
    //
    ulSpace = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);

    //
    // Read data from the UART FIFO until there is none left or we run
    // out of space in our receive buffer.
    //
    while(ulSpace && UARTCharsAvail(USB_UART_BASE))
    {
        //
        // Read a character from the UART FIFO into the ring buffer if no
        // errors are reported.
        //
        lChar = UARTCharGetNonBlocking(USB_UART_BASE);

        //
        // If the character did not contain any error notifications,
        // copy it to the output buffer.
        //
        if(!(lChar & ~0xFF))
        {
            ucChar = (unsigned char)(lChar & 0xFF);
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer,
                           (unsigned char *)&ucChar, 1);

            //
            // Decrement the number of bytes we know the buffer can accept.
            //
            ulSpace--;
        }
        else
        {
#ifdef DEBUG
            //
            // Increment our receive error counter.
            //
            g_ulUARTRxErrors++;
#endif
            //
            // Update our error accumulator.
            //
            lErrors |= lChar;
        }

        //
        // Update our count of bytes received via the UART.
        //
        g_ulUARTRxCount++;
    }

    //
    // Pass back the accumulated error indicators.
    //
    return(lErrors);
}

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the USB UART's transmit FIFO.
//
//*****************************************************************************

static void
USBUARTPrimeTransmit(unsigned int ulBase)
{
   unsigned int ulRead;
    unsigned char ucChar;

    //
    // If we are currently sending a break condition, don't receive any
    // more data. We will resume transmission once the break is turned off.
    //
    if(g_bSendingBreak)
    {
        return;
    }

    //
    // If there is space in the UART FIFO, try to read some characters
    // from the receive buffer to fill it again.
    //
    while(UARTSpaceAvail(ulBase))
    {
        //
        // Get a character from the buffer.
        //
        ulRead = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);

        //
        // Did we get a character?
        //
        if(ulRead)
        {
            //
            // Place the character in the UART transmit FIFO.
            //
            UARTCharPut(ulBase, ucChar);

            //
            // Update our count of bytes transmitted via the UART.
            //
            g_ulUARTTxCount++;
        }
        else
        {
            //
            // We ran out of characters so exit the function.
            //
            return;
        }
    }
}

//*****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system time.
    //
    g_ulSysTickCount++;
}

void UARTHandleError(void)
{
    while((UARTRxErrorGet(USB_UART_BASE)))
    {
        /* Read a byte from the RBR if RBR has data.*/
       UARTCharGetNonBlocking(USB_UART_BASE);
    }

}


//*****************************************************************************
//
// Interrupt handler for the UART which we are redirecting via USB.
//
//*****************************************************************************

void
USBUARTIntHandler(void)
{
    unsigned int ulInts;
    int lErrors;

    //
    // Get and clear the current interrupt source(s)
    //
    ulInts = UARTIntIdentityGet(SOC_UART_0_REGS);



        /* Check if the cause is receiver line error condition.*/
        if( UART_INTID_RX_LINE_STAT_ERROR == ulInts || UART_INTID_CHAR_TIMEOUT == ulInts)
        {
            UARTHandleError();
        }

        //
        // Are we being interrupted because the TX FIFO has space available?
        //
        if(ulInts == UART_INTID_TX_THRES_REACH)
        {
            //
            // Move as many bytes as we can into the transmit FIFO.
            //
            USBUARTPrimeTransmit(USB_UART_BASE);

            //
            // If the output buffer is empty, turn off the transmit interrupt.
            //
            if(!USBBufferDataAvailable(&g_sRxBuffer))
            {
                UARTIntDisable(USB_UART_BASE, UART_INTID_TX_THRES_REACH);
            }
        }

        //
        // Handle receive interrupts.
        //
        if(ulInts  == UART_INTID_RX_THRES_REACH)
        {
            //
            // Read the UART's characters into the buffer.
            //

            lErrors = ReadUARTData();

            //
            // Check to see if we need to notify the host of any errors we just
            // detected.
            //
            CheckForSerialStateChange(&g_sCDCDevice, lErrors);
        }

}

//*****************************************************************************
//
// Set the state of the RS232 RTS and DTR signals.
//
//*****************************************************************************
static void
SetControlLineState(unsigned short usState)
{
    //
    // TODO: If configured with GPIOs controlling the handshake lines,
    // set them appropriately depending upon the flags passed in the wValue
    // field of the request structure passed.
    //
}

//*****************************************************************************
//
// Set the communication parameters to use on the UART.
//
//*****************************************************************************
static tBoolean
SetLineCoding(tLineCoding *psLineCoding)
{
    unsigned int ulConfig, ulParity;
    tBoolean bRetcode;

    //
    // Assume everything is OK until we detect any problem.
    //
    bRetcode = true;

    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ucDatabits)
    {
        case 5:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_5;
            break;
        }

        case 6:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_6;
            break;
        }

        case 7:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_7;
            break;
        }

        case 8:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_8;
            break;
        }

        default:
        {
            ulConfig = UART_FRAME_WORD_LENGTH_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, we set no parity and return an error.
    //
    switch(psLineCoding->ucParity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ulParity = UART_PARITY_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ulParity = UART_ODD_PARITY;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ulParity = UART_EVEN_PARITY;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
             ulParity = UART_PARITY_REPR_1;
             break;
        }

        case USB_CDC_PARITY_SPACE:
        {
             ulParity = UART_PARITY_REPR_0;
             break;
        }

        default:
        {
            ulParity =  UART_PARITY_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  Our hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ucStop, we set up for 1 stop bit but
    // return an error in case the caller needs to Stall or otherwise report
    // this back to the host.
    //
    switch(psLineCoding->ucStop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            ulConfig |= UART_FRAME_NUM_STB_1;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            ulConfig |= UART_FRAME_NUM_STB_1_5_2;
            break;
        }

        //
        // Other cases are either invalid values of ucStop or values that we
        // cannot support so set 1 stop bit but return an error.
        //
        default:
        {
            ulConfig = UART_FRAME_NUM_STB_1;
            bRetcode |= false;
            break;
        }
    }



    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS, ulConfig, ulParity);
    //
    // Let the caller know if we had a problem or not.
    //
    return(bRetcode);
}

//*****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//*****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    //
    // Get the current line coding set in the UART.
    //
    psLineCoding->ulRate = DEFAULT_BIT_RATE;
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;

}

//*****************************************************************************
//
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//
//*****************************************************************************
static void
SendBreak(tBoolean bSend)
{
    //
    // Are we being asked to start or stop the break condition?
    //
    if(!bSend)
    {
        //
        // Remove the break condition on the line.
        //
        UARTBreakCtl(USB_UART_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {
        //
        // Start sending a break condition on the line.
        //
        UARTBreakCtl(USB_UART_BASE, true);
        g_bSendingBreak = true;
    }
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned int
ControlHandler(void *pvCBData, unsigned int ulEvent,
               unsigned int ulMsgValue, void *pvMsgData)
{
    unsigned char ulIntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            ulIntsOff = IntDisable();
            g_pcStatus = "Host connected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            ulIntsOff = IntDisable();
            g_pcStatus = "Host disconnected.";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            IntEnable(ulIntsOff);
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            SetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            SetControlLineState((unsigned short)ulMsgValue);
            break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            SendBreak(true);
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            SendBreak(false);
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned int
TxHandler(void *pvCBData, unsigned int ulEvent, unsigned int ulMsgValue,
          void *pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned int
RxHandler(void *pvCBData, unsigned int ulEvent, unsigned int ulMsgValue,
          void *pvMsgData)
{
    //unsigned int ulCount;

    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Feed some characters into the UART TX FIFO and enable the
            // interrupt so we are told when there is more space.
            //
              USBUARTPrimeTransmit(USB_UART_BASE);
              UARTIntEnable(USB_UART_BASE, UART_INTID_TX_THRES_REACH);
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            return(0);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }

    return(0);
}

static void UARTAINTCConfigure(void)
{
    /* Registering the Interrupt Service Routine(ISR). */
        IntRegister(SYS_INT_UART0INT, USBUARTIntHandler);

        /* Setting the priority for the system interrupt in AINTC. */
        IntPrioritySet(SYS_INT_UART0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

        /* Enabling the system interrupt in AINTC. */
        IntSystemEnable(SYS_INT_UART0INT);



}

static void USB0AINTCConfigure(void)
{

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_USB0);
}

static void USBInterruptEnable(void)
{
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Configuring AINTC to receive USB0 interrupts. */
    USB0AINTCConfigure();

    UARTAINTCConfigure();
}


//*****************************************************************************
//
// This is the main application entry function.
//
//*****************************************************************************
int
main(void)
{
    unsigned int intFlags = 0;
    unsigned int Intstatus;

        MMUConfigAndEnable();

    //
    // Not configured initially.
    //
    g_bUSBConfigured = false;

    //
    //configures arm interrupt controller to generate raster interrupt
    //
    USBInterruptEnable();

    //
    //Delay timer setup
    //
    DelayTimerSetup();

    //
    //Initialize the Rx and TX Buffers
    //
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Performing the Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Performing a module reset. */
    UARTModuleReset(SOC_UART_0_REGS);

    /* Performing Baud Rate settings. */
    UartBaudRateSet();

   /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(SOC_UART_0_REGS, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(SOC_UART_0_REGS);

    /* Disabling Break Control. */
    UARTBreakCtl(SOC_UART_0_REGS, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(SOC_UART_0_REGS, UART16x_OPER_MODE);

     /* Performing FIFO configurations. */
    UartFIFOConfigure();

    /* Preparing the 'intFlags' variable to be passed as an argument.*/
    intFlags |= (UART_INT_LINE_STAT | UART_INT_RHR_CTI);

    /* Enable the Interrupts in UART.*/
    UARTIntEnable(SOC_UART_0_REGS, intFlags);

    //
    // Main application loop.
    //
     while(1)
    {

        //
        // Have we been asked to update the status display?
        //
        if(g_ulFlags & COMMAND_STATUS_UPDATE)
        {
            //
            // Clear the command flag
            //
            Intstatus = IntDisable();
            g_ulFlags &= ~COMMAND_STATUS_UPDATE;
            IntEnable(Intstatus);
        }
    }
}
