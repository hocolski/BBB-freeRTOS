/*****************************************************************************
*
* usb_device_host_msc.c - USB mass storage host and device application.
*
* This is part of Starterware AM 335X  Firmware Package.
*
*****************************************************************************/

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

#include <string.h>
#include "ff.h"
#include "hw_types.h"
#include "hw_usb.h"
#include "interrupt.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "usblib.h"
#include "usbmsc.h"
#include "usbhost.h"
#include "usbhmsc.h"
#include "usb-ids.h"
#include "usbdevice.h"
#include "usbdmsc.h"
#include "usb_msc_structs.h"
#include "delay.h"
#include "cmdline.h"
#include "cppi41dma.h"
#include "cache.h"
#include "mmu.h"
#include "uart_irda_cir.h"
#include "uartStdio.h"
#include "consoleUtils.h"


#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=(16*1024)
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024]__attribute__((aligned(16*1024)));
#endif

unsigned char *dataBuffer;


#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)

static void MMUConfigAndEnable(void);
void USBDeviceIntHandlerInternal(unsigned int ulIndex, unsigned int ulStatus,
                                                    unsigned int *endPStatus);
void USBHostIntHandlerInternal(unsigned int ulIndex, unsigned int ulStatus,
                                                    unsigned int *endPStatus);

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Mass Storage Class Device and Host (usb_device_host_msc)</h1>
//!
//! This example application demonstrates reading a file system from a USB mass
//! storage class device.  It makes use of FatFs, a FAT file system driver.  It
//! provides a simple command console for issuing commands to view
//! and navigate the file system on the mass storage device.
//!
//! When the program is started a message will be printed to the
//! console.  Type ``help'' for command help.
//!
//! This device side of the example application turns the evaluation board into
//! a USB mass storage class device.  The application will use the RAM on EVM
//! for the storage media for the mass storage device.
//
//! For additional details about FatFs, see the following site:
//! http://elm-chan.org/fsw/ff/00index_e.html
//
//*****************************************************************************

#define USB_INSTANCE_Device 0
#define USB_INSTANCE_Host   1

#define FatFS_Drive_Index   0


//*****************************************************************************
//
// The number of SysTick ticks per second.
//
//*****************************************************************************
#define TICKS_PER_SECOND 100
#define MS_PER_SYSTICK (1000 / TICKS_PER_SECOND)

//*****************************************************************************
//
// Our running system tick counter and a global used to determine the time
// elapsed since last call to GetTickms().
//
//*****************************************************************************
unsigned int g_ulSysTickCount;
unsigned int g_ulLastTick;

//*****************************************************************************
//
// Defines the size of the buffers that hold the path, or temporary data from
// the memory card.  There are two buffers allocated of this size.  The buffer
// size must be large enough to hold the longest expected full path name,
// including the file name, and a trailing null character.
//
//*****************************************************************************
#define PATH_BUF_SIZE   80

//*****************************************************************************
//
// Defines the size of the buffer that holds the command line.
//
//*****************************************************************************
#define CMD_BUF_SIZE    64

//*****************************************************************************
//
// This buffer holds the full path to the current working directory.  Initially
// it is root ("/").
//
//*****************************************************************************
static char g_cCwdBuf[PATH_BUF_SIZE] = "/";

//*****************************************************************************
//
// A temporary data buffer used when manipulating file paths, or reading data
// from the memory card.
//
//*****************************************************************************
static char g_cTmpBuf[PATH_BUF_SIZE];

//*****************************************************************************
//
// The buffer that holds the command line.
//
//*****************************************************************************
static char g_cCmdBuf[CMD_BUF_SIZE];

//*****************************************************************************
//
// Current FAT fs state.
//
//*****************************************************************************
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;

//*****************************************************************************
//
// A structure that holds a mapping between an FRESULT numerical code,
// and a string representation.  FRESULT codes are returned from the FatFs
// FAT file system driver.
//
//*****************************************************************************
typedef struct
{
    FRESULT fresult;
    char *pcResultStr;
}
tFresultString;

//*****************************************************************************
//
// A macro to make it easy to add result codes to the table.
//
//*****************************************************************************
#define FRESULT_ENTRY(f)        { (f), (#f) }

//*****************************************************************************
//
// A table that holds a mapping between the numerical FRESULT code and
// it's name as a string.  This is used for looking up error codes for
// printing to the console.
//
//*****************************************************************************
tFresultString g_sFresultStrings[] =
{
    FRESULT_ENTRY(FR_OK),
    FRESULT_ENTRY(FR_NOT_READY),
    FRESULT_ENTRY(FR_NO_FILE),
    FRESULT_ENTRY(FR_NO_PATH),
    FRESULT_ENTRY(FR_INVALID_NAME),
    FRESULT_ENTRY(FR_INVALID_DRIVE),
    FRESULT_ENTRY(FR_DENIED),
    FRESULT_ENTRY(FR_EXIST),
    FRESULT_ENTRY(FR_RW_ERROR),
    FRESULT_ENTRY(FR_WRITE_PROTECTED),
    FRESULT_ENTRY(FR_NOT_ENABLED),
    FRESULT_ENTRY(FR_NO_FILESYSTEM),
    FRESULT_ENTRY(FR_INVALID_OBJECT),
    FRESULT_ENTRY(FR_MKFS_ABORTED)
};

//*****************************************************************************
//
// A macro that holds the number of result codes.
//
//*****************************************************************************
#define NUM_FRESULT_CODES (sizeof(g_sFresultStrings) / sizeof(tFresultString))

//*****************************************************************************
//
// The size of the host controller's memory pool in bytes.
//
//*****************************************************************************
#define HCD_MEMORY_SIZE         128

//*****************************************************************************
//
// The memory pool to provide to the Host controller driver.
//
//*****************************************************************************
unsigned char g_pHCDPool[HCD_MEMORY_SIZE];

//*****************************************************************************
//
// The instance data for the MSC driver.
//
//*****************************************************************************
unsigned int g_ulMSCInstance = 0;

//*****************************************************************************
//
// Declare the USB Events driver interface.
//
//*****************************************************************************
DECLARE_EVENT_DRIVER(g_sUSBEventDriver, 0, 0, USBHCDEvents);

//*****************************************************************************
//
// The global that holds all of the host drivers in use in the application.
// In this case, only the MSC class is loaded.
//
//*****************************************************************************
static tUSBHostClassDriver const * const g_ppHostClassDrivers[] =
{
    &g_USBHostMSCClassDriver,
    &g_sUSBEventDriver
};

//*****************************************************************************
//
// This global holds the number of class drivers in the g_ppHostClassDrivers
// list.
//
//*****************************************************************************
#define NUM_CLASS_DRIVERS       (sizeof(g_ppHostClassDrivers) /               \
                                 sizeof(g_ppHostClassDrivers[0]))

//*****************************************************************************
//
// Hold the current state for the application.
//
//*****************************************************************************
typedef enum
{
    //
    // No device is present.
    //
    STATE_NO_DEVICE,

    //
    // Mass storage device is being enumerated.
    //
    STATE_DEVICE_ENUM,

    //
    // Mass storage device is ready.
    //
    STATE_DEVICE_READY,

    //
    // An unsupported device has been attached.
    //
    STATE_UNKNOWN_DEVICE,

    //
    // A power fault has occurred.
    //
    STATE_POWER_FAULT
}
tState;
volatile tState g_eState;
volatile tState g_eUIState;


//*****************************************************************************
//
// The current USB operating mode - Host, Device or unknown.
//
//*****************************************************************************
tUSBMode g_eCurrentUSBMode;


#ifdef DMA_MODE

endpointInfo epInfoHost[]=
{
    {
        USB_EP_TO_INDEX(USB_EP_1),
        CPDMA_DIR_RX,
        CPDMA_MODE_SET_TRANSPARENT,
    },

    {
        USB_EP_TO_INDEX(USB_EP_1),
        CPDMA_DIR_TX,
        CPDMA_MODE_SET_TRANSPARENT,
    },

    {
        USB_EP_TO_INDEX(USB_EP_2),
        CPDMA_DIR_RX,
        CPDMA_MODE_SET_TRANSPARENT,
    },

    {
        USB_EP_TO_INDEX(USB_EP_2),
        CPDMA_DIR_TX,
        CPDMA_MODE_SET_TRANSPARENT,
    }
};

#define NUMBER_OF_HOST_ENDPOINTS 4

#endif


void
USBCppiDmaIntHandler(void)
{
    unsigned int epStatus = 0;
    unsigned int ulIndex = 0;

  /*
     * Check for USB 0 related interrupt
     */
    ulIndex = 0;
#if defined (am335x_15x15) || defined(am335x)

    epStatus = HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_STATUS_0);

    if(epStatus)
    {
        //
        // Clear the EP interrupt status.
        //
        HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_STATUS_0) = epStatus;

#ifdef DMA_MODE
        HWREG(USBSS_BASE + USBSS_IRQ_STATUS) =
                HWREG(USBSS_BASE + USBSS_IRQ_STATUS);
#endif

        USBDeviceIntHandlerInternal(ulIndex, 0, &epStatus);
        //
        //End of Interrupts.
        //
        HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_EOI) = 0;
    }

  /*
     * Check for USB 1 related interrupt
     */
    epStatus = 0;
    ulIndex = 1;
    epStatus = HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_STATUS_0);

    if(epStatus)
    {
        //
        // Clear the EP interrupt status.
        //
        HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_STATUS_0) = epStatus;

#ifdef DMA_MODE
        HWREG(USBSS_BASE + USBSS_IRQ_STATUS) =
                HWREG(USBSS_BASE + USBSS_IRQ_STATUS);
#endif

        USBHostIntHandlerInternal(ulIndex, 0, &epStatus);
        //
        //End of Interrupts.
        //
        HWREG(g_USBInstance[ulIndex].uiSubBaseAddr + USB_0_IRQ_EOI) = 0;
    }

#ifdef DMA_MODE
        HWREG(USBSS_BASE + USBSS_IRQ_EOI) = 0;
#endif

#endif //#if defined (am335x_15x15) || defined(am335x)

}




//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned int ulLine)
{
}
#endif

//*****************************************************************************
//
// This is the handler for this SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our tick counter.
    //
    g_ulSysTickCount++;
}

//*****************************************************************************
//
// This function returns the number of ticks since the last time this function
// was called.
//
//*****************************************************************************
unsigned int
GetTickms(void)
{
    unsigned int ulRetVal;
    unsigned int ulSaved;

    ulRetVal = g_ulSysTickCount;
    ulSaved = ulRetVal;

    if(ulSaved > g_ulLastTick)
    {
        ulRetVal = ulSaved - g_ulLastTick;
    }
    else
    {
        ulRetVal = g_ulLastTick - ulSaved;
    }

    //
    // This could miss a few milliseconds but the timings here are on a
    // much larger scale.
    //
    g_ulLastTick = ulSaved;

    //
    // Return the number of milliseconds since the last time this was called.
    //
    return(ulRetVal * MS_PER_SYSTICK);
}

//*****************************************************************************
//
// USB Mode callback
//
// \param ulIndex is the zero-based index of the USB controller making the
//        callback.
// \param eMode indicates the new operating mode.
//
// This function is called by the USB library whenever an OTG mode change
// occurs and, if a connection has been made, informs us of whether we are to
// operate as a host or device.
//
// \return None.
//
//*****************************************************************************
void
ModeCallback(unsigned int ulIndex, tUSBMode eMode)
{
    //
    // Save the new mode.
    //

    g_eCurrentUSBMode = eMode;
}

//*****************************************************************************
//
// This function returns a string representation of an error code that was
// returned from a function call to FatFs.  It can be used for printing human
// readable error messages.
//
//*****************************************************************************
const char *
StringFromFresult(FRESULT fresult)
{
    unsigned int uIdx;

    //
    // Enter a loop to search the error code table for a matching error code.
    //
    for(uIdx = 0; uIdx < NUM_FRESULT_CODES; uIdx++)
    {
        //
        // If a match is found, then return the string name of the error code.
        //
        if(g_sFresultStrings[uIdx].fresult == fresult)
        {
            return(g_sFresultStrings[uIdx].pcResultStr);
        }
    }

    //
    // At this point no matching code was found, so return a string indicating
    // unknown error.
    //
    return("UNKNOWN ERROR CODE");
}

//*****************************************************************************
//
// This function implements the "ls" command.  It opens the current directory
// and enumerates through the contents, and prints a line for each item it
// finds.  It shows details such as file attributes, time and date, and the
// file size, along with the name.  It shows a summary of file sizes at the end
// along with free space.
//
//*****************************************************************************
int
Cmd_ls(int argc, char *argv[])
{
    unsigned int ulTotalSize;
    unsigned int ulFileCount;
    unsigned int ulDirCount;
    FRESULT fresult;
    FATFS *pFatFs;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Open the current directory for access.
    //
    fresult = f_opendir(&g_sDirObject, g_cCwdBuf);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    ulTotalSize = 0;
    ulFileCount = 0;
    ulDirCount = 0;

    //
    // Enter loop to enumerate through all directory entries.
    //
    while(1)
    {
        //
        // Read an entry from the directory.
        //
        fresult = f_readdir(&g_sDirObject, &g_sFileInfo);

        //
        // Check for error and return if there is a problem.
        //
        if(fresult != FR_OK)
        {
            return(fresult);
        }

        //
        // If the file name is blank, then this is the end of the listing.
        //
        if(!g_sFileInfo.fname[0])
        {
            break;
        }

        //
        // If the attribute is directory, then increment the directory count.
        //
        if(g_sFileInfo.fattrib & AM_DIR)
        {
            ulDirCount++;
        }

        //
        // Otherwise, it is a file.  Increment the file count, and add in the
        // file size to the total.
        //
        else
        {
            ulFileCount++;
            ulTotalSize += g_sFileInfo.fsize;
        }

        //
        // Print the entry information on a single line with formatting to show
        // the attributes, date, time, size, and name.
        //
        ConsoleUtilsPrintf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n",
                    (g_sFileInfo.fattrib & AM_DIR) ? 'D' : '-',
                    (g_sFileInfo.fattrib & AM_RDO) ? 'R' : '-',
                    (g_sFileInfo.fattrib & AM_HID) ? 'H' : '-',
                    (g_sFileInfo.fattrib & AM_SYS) ? 'S' : '-',
                    (g_sFileInfo.fattrib & AM_ARC) ? 'A' : '-',
                    (g_sFileInfo.fdate >> 9) + 1980,
                    (g_sFileInfo.fdate >> 5) & 15,
                     g_sFileInfo.fdate & 31,
                    (g_sFileInfo.ftime >> 11),
                    (g_sFileInfo.ftime >> 5) & 63,
                     g_sFileInfo.fsize,
                     g_sFileInfo.fname);
    }

    //
    // Print summary lines showing the file, dir, and size totals.
    //
    ConsoleUtilsPrintf("\n%4u File(s),%10u bytes total\n%4u Dir(s)",
                ulFileCount, ulTotalSize, ulDirCount);

    //
    // Get the free space.
    //
    fresult = f_getfree("/", &ulTotalSize, &pFatFs);

    //
    // Check for error and return if there is a problem.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Display the amount of free space that was calculated.
    //
    ConsoleUtilsPrintf(", %10uK bytes free\n",
        ulTotalSize * pFatFs->sects_clust / 2);

    //
    // Made it to here, return with no errors.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "cd" command.  It takes an argument that
// specifies the directory to make the current working directory.  Path
// separators must use a forward slash "/".  The argument to cd can be one of
// the following:
//
// * root ("/")
// * a fully specified path ("/my/path/to/mydir")
// * a single directory name that is in the current directory ("mydir")
// * parent directory ("..")
//
// It does not understand relative paths, so don't try something like this:
// ("../my/new/path")
//
// Once the new directory is specified, it attempts to open the directory to
// make sure it exists.  If the new path is opened successfully, then the
// current working directory (cwd) is changed to the new path.
//
//*****************************************************************************
int
Cmd_cd(int argc, char *argv[])
{
    unsigned int uIdx;
    FRESULT fresult;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Copy the current working path into a temporary buffer so it can be
    // manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If the first character is /, then this is a fully specified path, and it
    // should just be used as-is.
    //
    if(argv[1][0] == '/')
    {
        //
        // Make sure the new path is not bigger than the cwd buffer.
        //
        if(strlen(argv[1]) + 1 > sizeof(g_cCwdBuf))
        {
            ConsoleUtilsPrintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // If the new path name (in argv[1])  is not too long, then copy it
        // into the temporary buffer so it can be checked.
        //
        else
        {
            strncpy(g_cTmpBuf, argv[1], sizeof(g_cTmpBuf));
        }
    }

    //
    // If the argument is .. then attempt to remove the lowest level on the
    // CWD.
    //
    else if(!strcmp(argv[1], ".."))
    {
        //
        // Get the index to the last character in the current path.
        //
        uIdx = strlen(g_cTmpBuf) - 1;

        //
        // Back up from the end of the path name until a separator (/) is
        // found, or until we bump up to the start of the path.
        //
        while((g_cTmpBuf[uIdx] != '/') && (uIdx > 1))
        {
            //
            // Back up one character.
            //
            uIdx--;
        }

        //
        // Now we are either at the lowest level separator in the current path,
        // or at the beginning of the string (root).  So set the new end of
        // string here, effectively removing that last part of the path.
        //
        g_cTmpBuf[uIdx] = 0;
    }

    //
    // Otherwise this is just a normal path name from the current directory,
    // and it needs to be appended to the current path.
    //
    else
    {
        //
        // Test to make sure that when the new additional path is added on to
        // the current path, there is room in the buffer for the full new path.
        // It needs to include a new separator, and a trailing null character.
        //
        if(strlen(g_cTmpBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cCwdBuf))
        {
            ConsoleUtilsPrintf("Resulting path name is too long\n");
            return(0);
        }

        //
        // The new path is okay, so add the separator and then append the new
        // directory to the path.
        //
        else
        {
            //
            // If not already at the root level, then append a /
            //
            if(strcmp(g_cTmpBuf, "/"))
            {
                strcat(g_cTmpBuf, "/");
            }

            //
            // Append the new directory to the path.
            //
            strcat(g_cTmpBuf, argv[1]);
        }
    }

    //
    // At this point, a candidate new directory path is in chTmpBuf.  Try to
    // open it to make sure it is valid.
    //
    fresult = f_opendir(&g_sDirObject, g_cTmpBuf);

    //
    // If it can't be opened, then it is a bad path.  Inform user and return.
    //
    if(fresult != FR_OK)
    {
        ConsoleUtilsPrintf("cd: %s\n", g_cTmpBuf);
        return(fresult);
    }

    //
    // Otherwise, it is a valid new path, so copy it into the CWD.
    //
    else
    {
        strncpy(g_cCwdBuf, g_cTmpBuf, sizeof(g_cCwdBuf));
    }

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "pwd" command.  It simply prints the current
// working directory.
//
//*****************************************************************************
int
Cmd_pwd(int argc, char *argv[])
{
    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // Print the CWD to the console.
    //
    ConsoleUtilsPrintf("%s\n", g_cCwdBuf);

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "cat" command.  It reads the contents of a file
// and prints it to the console.  This should only be used on text files.  If
// it is used on a binary file, then a bunch of garbage is likely to printed on
// the console.
//
//*****************************************************************************
int
Cmd_cat(int argc, char *argv[])
{
    FRESULT fresult;
    unsigned short usBytesRead;

    //
    // Do not attempt to do anything if there is not a drive attached.
    //
    if(g_eState != STATE_DEVICE_READY)
    {
        return(FR_NOT_READY);
    }

    //
    // First, check to make sure that the current path (CWD), plus the file
    // name, plus a separator and trailing null, will all fit in the temporary
    // buffer that will be used to hold the file name.  The file name must be
    // fully specified, with path, to FatFs.
    //
    if(strlen(g_cCwdBuf) + strlen(argv[1]) + 1 + 1 > sizeof(g_cTmpBuf))
    {
        ConsoleUtilsPrintf("Resulting path name is too long\n");
        return(0);
    }

    //
    // Copy the current path to the temporary buffer so it can be manipulated.
    //
    strcpy(g_cTmpBuf, g_cCwdBuf);

    //
    // If not already at the root level, then append a separator.
    //
    if(strcmp("/", g_cCwdBuf))
    {
        strcat(g_cTmpBuf, "/");
    }

    //
    // Now finally, append the file name to result in a fully specified file.
    //
    strcat(g_cTmpBuf, argv[1]);

    //
    // Open the file for reading.
    //
    fresult = f_open(&g_sFileObject, g_cTmpBuf, FA_READ);

    //
    // If there was some problem opening the file, then return an error.
    //
    if(fresult != FR_OK)
    {
        return(fresult);
    }

    //
    // Enter a loop to repeatedly read data from the file and display it, until
    // the end of the file is reached.
    //
    do
    {
        //
        // Read a block of data from the file.  Read as much as can fit in the
        // temporary buffer, including a space for the trailing null.
        //
        fresult = f_read(&g_sFileObject, g_cTmpBuf, sizeof(g_cTmpBuf) - 1,
                         &usBytesRead);

        //
        // If there was an error reading, then print a newline and return the
        // error to the user.
        //
        if(fresult != FR_OK)
        {
            ConsoleUtilsPrintf("\n");
            return(fresult);
        }

        //
        // Null terminate the last block that was read to make it a null
        // terminated string that can be used with printf.
        //
        g_cTmpBuf[usBytesRead] = 0;

        //
        // Print the last chunk of the file that was received.
        //
        ConsoleUtilsPrintf("%s", g_cTmpBuf);

        //
        // Continue reading until less than the full number of bytes are read.
        // That means the end of the buffer was reached.
        //
    }
    while(usBytesRead == sizeof(g_cTmpBuf) - 1);

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This function implements the "help" command.  It prints a simple list of the
// available commands with a brief description.
//
//*****************************************************************************
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    ConsoleUtilsPrintf("\nAvailable commands\n");
    ConsoleUtilsPrintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_sCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        ConsoleUtilsPrintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

//*****************************************************************************
//
// This is the table that holds the command names, implementing functions, and
// brief description.
//
//*****************************************************************************
tCmdLineEntry g_sCmdTable[] =
{
    { "help",   Cmd_help,      " : Display list of commands" },
    { "h",      Cmd_help,   "    : alias for help" },
    { "?",      Cmd_help,   "    : alias for help" },
    { "ls",     Cmd_ls,      "   : Display list of files" },
    { "chdir",  Cmd_cd,         ": Change directory" },
    { "cd",     Cmd_cd,      "   : alias for chdir" },
    { "pwd",    Cmd_pwd,      "  : Show current working directory" },
    { "cat",    Cmd_cat,      "  : Show contents of a text file" },
    { 0, 0, 0 }
};

//*****************************************************************************
//
// This is the callback from the MSC driver.
//
// \param ulInstance is the driver instance which is needed when communicating
// with the driver.
// \param ulEvent is one of the events defined by the driver.
// \param pvData is a pointer to data passed into the initial call to register
// the callback.
//
// This function handles callback events from the MSC driver.  The only events
// currently handled are the MSC_EVENT_OPEN and MSC_EVENT_CLOSE.  This allows
// the main routine to know when an MSC device has been detected and
// enumerated and when an MSC device has been removed from the system.
//
// \return Returns \e true on success or \e false on failure.
//
//*****************************************************************************
void
MSCCallback(unsigned int ulInstance, unsigned int ulEvent, void *pvData)
{
    //
    // Determine the event.
    //
    switch(ulEvent)
    {
        //
        // Called when the device driver has successfully enumerated an MSC
        // device.
        //
        case MSC_EVENT_OPEN:
        {
            //
            // Proceed to the enumeration state.
            //
            g_eState = STATE_DEVICE_ENUM;
            break;
        }

        //
        // Called when the device driver has been unloaded due to error or
        // the device is no longer present.
        //
        case MSC_EVENT_CLOSE:
        {
            //
            // Go back to the "no device" state and wait for a new connection.
            //
            g_eState = STATE_NO_DEVICE;

            break;
        }

        default:
        {
            break;
        }
    }
}

//*****************************************************************************
//
// This is the generic callback from host stack.
//
// \param pvData is actually a pointer to a tEventInfo structure.
//
// This function will be called to inform the application when a USB event has
// occurred that is outside those related to the mass storage device.  At this
// point this is used to detect unsupported devices being inserted and removed.
// It is also used to inform the application when a power fault has occurred.
// This function is required when the g_USBGenericEventDriver is included in
// the host controller driver array that is passed in to the
// USBHCDRegisterDrivers() function.
//
// \return None.
//
//*****************************************************************************
void
USBHCDEvents(void *pvData)
{
    tEventInfo *pEventInfo;

    //
    // Cast this pointer to its actual type.
    //
    pEventInfo = (tEventInfo *)pvData;

    switch(pEventInfo->ulEvent)
    {
        //
        // New keyboard detected.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // An unknown device was detected.
            //
            g_eState = STATE_UNKNOWN_DEVICE;

            break;
        }

        //
        // Keyboard has been unplugged.
        //
        case USB_EVENT_DISCONNECTED:
        {
            //
            // Unknown device has been removed.
            //
            g_eState = STATE_NO_DEVICE;

            break;
        }

        case USB_EVENT_POWER_FAULT:
        {
            //
            // No power means no device is present.
            //
            g_eState = STATE_POWER_FAULT;

            break;
        }

        default:
        {
            break;
        }
    }
}

//*****************************************************************************
//
// This function reads a line of text from the console.  The USB host main
// function is called throughout this process to keep USB alive and well.
//
//*****************************************************************************
void
ReadLine(void)
{
    unsigned int ulIdx, ulPrompt;
    unsigned char ucChar;
    tState eStateCopy;

    //
    // Start reading at the beginning of the command buffer and print a prompt.
    //
    g_cCmdBuf[0] = '\0';
    ulIdx = 0;
    ulPrompt = 1;

    //
    // Loop forever.  This loop will be explicitly broken out of when the line
    // has been fully read.
    //
    while(1)
    {

        //
        // See if a mass storage device has been enumerated.
        //
        if(g_eState == STATE_DEVICE_ENUM)
        {
            //
            // Take it easy on the Mass storage device if it is slow to
            // start up after connecting.
            //
            if(USBHMSCDriveReady(g_ulMSCInstance) != 0)
            {
                //
                // Wait about 100ms before attempting to check if the
                // device is ready again.
                //
                 delay(100);

                break;
            }

            //
            // Reset the working directory to the root.
            //
            g_cCwdBuf[0] = '/';
            g_cCwdBuf[1] = '\0';

            //
            // Attempt to open the directory.  Some drives take longer to
            // start up than others, and this may fail (even though the USB
            // device has enumerated) if it is still initializing.
            //
            f_mount(FatFS_Drive_Index, &g_sFatFs);
            if(f_opendir(&g_sDirObject, g_cCwdBuf) == FR_OK)
            {
                //
                // The drive is fully ready, so move to that state.
                //
                g_eState = STATE_DEVICE_READY;
            }
        }

        //
        // See if the state has changed.  We make a copy of g_eUIState to
        // prevent a compiler warning about undefined order of volatile
        // accesses.
        //
        eStateCopy = g_eUIState;
        if(g_eState != eStateCopy)
        {
            //
            // Determine the new state.
            //
            switch(g_eState)
            {
                //
                // A previously connected device has been disconnected.
                //
                case STATE_NO_DEVICE:
                {
                    if(g_eUIState == STATE_UNKNOWN_DEVICE)
                    {
                        ConsoleUtilsPrintf("\nUnknown device disconnected.\n");
                    }
                    else
                    {
                        ConsoleUtilsPrintf("\nMass storage device"
                            " disconnected.\n");
                    }
                    ulPrompt = 1;
                    break;
                }

                //
                // A mass storage device is being enumerated.
                //
                case STATE_DEVICE_ENUM:
                {
                    break;
                }

                //
                // A mass storage device has been enumerated and initialized.
                //
                case STATE_DEVICE_READY:
                {
                    ConsoleUtilsPrintf("\nMass storage device connected.\n");
                    ulPrompt = 1;
                    break;
                }

                //
                // An unknown device has been connected.
                //
                case STATE_UNKNOWN_DEVICE:
                {
                    ConsoleUtilsPrintf("\nUnknown device connected.\n");
                    ulPrompt = 1;
                    break;
                }

                //
                // A power fault has occurred.
                //
                case STATE_POWER_FAULT:
                {
                    ConsoleUtilsPrintf("\nPower fault.\n");
                    ulPrompt = 1;
                    break;
                }
            }

            //
            // Save the current state.
            //
            g_eUIState = g_eState;
        }

        //
        // Print a prompt if necessary.
        //
        if(ulPrompt)
        {
            //
            // Print the prompt based on the current state.
            //
            if(g_eState == STATE_DEVICE_READY)
            {
                ConsoleUtilsPrintf("%s> %s", g_cCwdBuf, g_cCmdBuf);
            }
            else if(g_eState == STATE_UNKNOWN_DEVICE)
            {
                ConsoleUtilsPrintf("UNKNOWN> %s", g_cCmdBuf);
            }
            else
            {
                ConsoleUtilsPrintf("NODEV> %s", g_cCmdBuf);
            }

            //
            // The prompt no longer needs to be printed.
            //
            ulPrompt = 0;
        }

        //
        // Loop while there are characters that have been received from the
        // UART. This needs to be done in such a way that the USB main function
        // can continue to be called if user has not entered any data. Hence
        // console utils functions cannot be called here to get the characters
        // from console.
        //
        while(UARTCharsAvail(SOC_UART_0_REGS))
        {
            //
            // Read the next character from the UART.
            //
            ucChar = UARTGetc();

            //
            // See if this character is a backspace and there is at least one
            // character in the input line.
            //
            if((ucChar == '\b') && (ulIdx != 0))
            {
                //
                // Erase the lsat character from the input line.
                //
                ConsoleUtilsPrintf("\b \b");
                ulIdx--;
                g_cCmdBuf[ulIdx] = '\0';
            }

            //
            // See if this character is a newline.
            //
            else if((ucChar == '\r') || (ucChar == '\n'))
            {
                //
                // Return to the caller.
                //
                ConsoleUtilsPrintf("\n");
                return;
            }

            //
            // See if this character is an escape or Ctrl-U.
            //
            else if((ucChar == 0x1b) || (ucChar == 0x15))
            {
                //
                // Erase all characters in the input buffer.
                //
                while(ulIdx)
                {
                    ConsoleUtilsPrintf("\b \b");
                    ulIdx--;
                }
                g_cCmdBuf[0] = '\0';
            }

            //
            // See if this is a printable ASCII character.
            //
            else if((ucChar >= ' ') && (ucChar <= '~') &&
                    (ulIdx < (sizeof(g_cCmdBuf) - 1)))
            {
                //
                // Add this character to the input buffer.
                //
                g_cCmdBuf[ulIdx++] = ucChar;
                g_cCmdBuf[ulIdx] = '\0';
                ConsoleUtilsPrintf("%c", ucChar);
            }
        }

          USBHCDMain(USB_INSTANCE_Host, g_ulMSCInstance);
    }
}

//*****************************************************************************
//
// Sets up the AINTC Interrupt
//
//*****************************************************************************
static void USBAINTCConfigure(int usbInstance)
{
   if(usbInstance)
   {
       /* Registering the Interrupt Service Routine(ISR). */
//       IntRegister(SYS_INT_USB1, USBCppiDmaIntHandler);
       IntRegister(SYS_INT_USB1, USB1HostIntHandler);


       /* Setting the priority for the system interrupt in AINTC. */
       IntPrioritySet(SYS_INT_USB1, 0, AINTC_HOSTINT_ROUTE_IRQ);

       /* Enabling the system interrupt in AINTC. */
       IntSystemEnable(SYS_INT_USB1);
   }
   else
   {
       /* Registering the Interrupt Service Routine(ISR). */
//       IntRegister(SYS_INT_USB0, USBCppiDmaIntHandler);
       IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

       /* Setting the priority for the system interrupt in AINTC. */
       IntPrioritySet(SYS_INT_USB0, 0, AINTC_HOSTINT_ROUTE_IRQ);

       /* Enabling the system interrupt in AINTC. */
       IntSystemEnable(SYS_INT_USB0);
   }
}

static void CPDMAAINTCConfigure(void)
{

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_USBSSINT, USBCppiDmaIntHandler);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_USBSSINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_USBSSINT);
}

static void USBInterruptEnable(void)
{
    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Configuring AINTC to receive USB interrupts. */
    USBAINTCConfigure(USB_INSTANCE_Device);
    USBAINTCConfigure(USB_INSTANCE_Host);

#ifdef DMA_MODE
    CPDMAAINTCConfigure();
#endif
}

//*****************************************************************************
//
// The number of ticks to wait before falling back to the idle state.  Since
// the tick rate is 100Hz this is approximately 3 seconds.
//
//*****************************************************************************
#define USBMSC_ACTIVITY_TIMEOUT 30

//*****************************************************************************
//
// DMA Configuration.
//
//*****************************************************************************
#define NUMBER_OF_ENDPOINTS        2 //Total number of send points(RX +TX) used in this USB configuration
#define USB_MSC_BUFER_SIZE        512

//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    MSC_DEV_DISCONNECTED,

    //
    // Connected but not yet fully enumerated.
    //
    MSC_DEV_CONNECTED,

    //
    // Connected and fully enumerated but not currently handling a command.
    //
    MSC_DEV_IDLE,

    //
    // Currently reading the SD card.
    //
    MSC_DEV_READ,

    //
    // Currently writing the SD card.
    //
    MSC_DEV_WRITE,
}
g_eMSCState;

//*****************************************************************************
//
// The Flags that handle updates to the status area to avoid drawing when no
// updates are required..
//
//*****************************************************************************
#define FLAG_UPDATE_STATUS      1
static unsigned int g_ulFlags;
//static unsigned int g_ulIdleTimeout;
unsigned int g_bufferIndex = 0;

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned int
RxHandler(void *pvCBData, unsigned int ulEvent,
               unsigned int ulMsgValue, void *pvMsgData)
{
    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
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
    return(0);
}




//*****************************************************************************
//
// This function is the call back notification function provided to the USB
// library's mass storage class.
//
//*****************************************************************************
unsigned int
USBDMSCEventCallback(void *pvCBData, unsigned int ulEvent,
                     unsigned int ulMsgParam, void *pvMsgData)
{
    //
    // Reset the time out every time an event occurs.
    //
   // g_ulIdleTimeout = USBMSC_ACTIVITY_TIMEOUT;

    switch(ulEvent)
    {
        //
        // Writing to the device.
        //
        case USBD_MSC_EVENT_WRITING:
        {
            //
            // Only update if this is a change.
            //
            if(g_eMSCState != MSC_DEV_WRITE)
            {
                //
                // Go to the write state.
                //
                g_eMSCState = MSC_DEV_WRITE;

                //
                // Cause the main loop to update the screen.
                //
                g_ulFlags |= FLAG_UPDATE_STATUS;
            }

            break;
        }

        //
        // Reading from the device.
        //
        case USBD_MSC_EVENT_READING:
        {
            //
            // Only update if this is a change.
            //
            if(g_eMSCState != MSC_DEV_READ)
            {
                //
                // Go to the read state.
                //
                g_eMSCState = MSC_DEV_READ;

                //
                // Cause the main loop to update the screen.
                //
                g_ulFlags |= FLAG_UPDATE_STATUS;
            }

            break;
        }
        case USBD_MSC_EVENT_IDLE:
        default:
        {
            break;
        }
    }

    return(0);
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



//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    
    int iStatus;    
    
    /* MMU needs to be turned on to provide HW support unaligned 
       access to USB structures */
    MMUConfigAndEnable();
  
    /* Enable Data Cache */
    CacheEnable(CACHE_ALL);
  
    //
    // Initially wait for device connection.
    //
    g_eState = STATE_NO_DEVICE;
    g_eUIState = STATE_NO_DEVICE;

    /* Initialize Console Utils. */
    ConsoleUtilsInit ();

    /* Select the console type to be used. For this application, only
     * UART console type must be used, since there is a need for
     * non-blocking getc from the console, which is not available for
     * the other consoles.
     */
    ConsoleUtilsSetType(CONSOLE_UART);

    ConsoleUtilsPrintf("\n\nUSB Mass Storage Host program\n");
    ConsoleUtilsPrintf("Type \'help\' for help.\n\n");

    //
    //USB Interrupt enable
    //
    USBInterruptEnable();

    //
    //Delay timer setup
    //
    DelayTimerSetup();

    //
    // Enable Clocking to the USB controller.
    //
     USB0ModuleClkConfig();

    //
    // Register the host class drivers.
    //
    USBHCDRegisterDrivers(USB_INSTANCE_Host, g_ppHostClassDrivers, NUM_CLASS_DRIVERS);

    //
    // Open an instance of the mass storage class driver.
    //
    g_ulMSCInstance = USBHMSCDriveOpen(USB_INSTANCE_Host, FatFS_Drive_Index, MSCCallback);

    //
    // Initialize the power configuration.  This sets the power enable signal
    // to be active high and does not enable the power fault.
    //
    USBHCDPowerConfigInit(USB_INSTANCE_Host, USBHCD_VBUS_AUTO_HIGH);

#ifdef DMA_MODE
    Cppi41DmaInit(USB_INSTANCE_Host, epInfoHost, NUMBER_OF_HOST_ENDPOINTS);
#endif


    // Initialize the host controller.
    //
    USBHCDInit(USB_INSTANCE_Host, g_pHCDPool, HCD_MEMORY_SIZE);


    //
    // Initialize the file system.
    //
    f_mount(FatFS_Drive_Index, &g_sFatFs);

    //
    // Initialize the idle timeout and reset all flags.
    //
    //g_ulIdleTimeout = 0;
    g_ulFlags = 0;

    //
    // Initialize the state to idle.
    //
    g_eMSCState = MSC_DEV_IDLE;

    USBDMSCInit(USB_INSTANCE_Device, (tUSBDMSCDevice *)&g_sMSCDevice);

#ifdef DMA_MODE
    Cppi41DmaInit(USB_INSTANCE_Device, epInfo, NUMBER_OF_ENDPOINTS);

    for(;g_bufferIndex < NUM_OF_RX_BDs; g_bufferIndex++)
    {
        dataBuffer = (unsigned char *)cppiDmaAllocBuffer();
        doDmaRxTransfer(USB_INSTANCE_Device, USB_MSC_BUFER_SIZE, dataBuffer,
                            g_sMSCDevice.psPrivateData->ucOUTEndpoint);
    }
#endif


    //
    // Enter an infinite loop for reading and processing commands from the
    // user.
    //
    while(1)
    {
        //
        // Get a line of text from the user.
        //
        ReadLine();
        if(g_cCmdBuf[0] == '\0')
        {
            delay(100);
            continue;
        }

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        iStatus = CmdLineProcess(g_cCmdBuf);

        //
        // Handle the case of bad command.
        //
        if(iStatus == CMDLINE_BAD_CMD)
        {
            ConsoleUtilsPrintf("Bad command!\n");
        }

        //
        // Handle the case of too many arguments.
        //
        else if(iStatus == CMDLINE_TOO_MANY_ARGS)
        {
            ConsoleUtilsPrintf("Too many arguments for command processor!\n");
        }

        //
        // Otherwise the command was executed.  Print the error
        // code if one was returned.
        //
        else if(iStatus != 0)
        {
            ConsoleUtilsPrintf("Command returned error code %s\n",
                        StringFromFresult((FRESULT)iStatus));
        }
    }
}
