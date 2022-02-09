
#include "LPC51U68.h"
#include "string.h"
#include "cortex_m.h"
#include "stdio.h"


unsigned long CCLK = 12000;            // CCLK in kHz

struct sIAP
{                  // IAP Structure
    unsigned long cmd;           // Command
    unsigned long par[4];        // Parameters
    unsigned long stat;          // Status
    unsigned long res[2];        // Result
} IAP;

/* IAP Call */
typedef void (*IAP_Entry) (unsigned long *cmd, unsigned long *stat);

#define IAP_Call ((IAP_Entry) 0x03000205)
#define PHY_PAGE_SIZE       (256)       /* actual flash page size */
#define SECTOR_SIZE         (32*1024)


unsigned long GetSecNum (unsigned long adr)
{
    unsigned long n;

    n = adr / SECTOR_SIZE;
    return (n);
}


uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    printf("%s\r\n", __FUNCTION__);
    CCLK = SystemCoreClock/1000;
    return 0;
}


uint32_t UnInit(uint32_t fnc)
{
    //printf("%s\r\n", __FUNCTION__);
    return (0);
}




/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseChip(void)
{
    //printf("%s\r\n", __FUNCTION__);
    return 0;
}

uint8_t FLASH_EraseSector(uint32_t addr)
{
    unsigned long n;

    n = GetSecNum(addr);                          // Get Sector Number

    IAP.cmd    = 50;                             // Prepare Sector for Erase
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 52;                             // Erase Sector
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    IAP.par[2] = CCLK;                           // CCLK in kHz
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    return (0);                                  // Finished without Errors
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t EraseSector(uint32_t adr)
{
    printf("%s 0x%08X\r\n", __FUNCTION__, adr);
    FLASH_EraseSector(adr);
    return 0;
}



uint8_t FLASH_WritePage(uint32_t addr, const uint8_t *buf)
{
    unsigned long n;

    n = GetSecNum(addr);                          // Get Sector Number
  
    IAP.cmd    = 50;                             // Prepare Sector for Write
    IAP.par[0] = n;                              // Start Sector
    IAP.par[1] = n;                              // End Sector
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    IAP.cmd    = 51;                             // Copy RAM to Flash
    IAP.par[0] = addr;                            // Destination Flash Address
    IAP.par[1] = (unsigned long)buf;             // Source RAM Address
    IAP.par[2] = PHY_PAGE_SIZE;                 // Fixed Page Size
    IAP.par[3] = CCLK;                           // CCLK in kHz
    __disable_irq();
    IAP_Call (&IAP.cmd, &IAP.stat);              // Call IAP Command
    __enable_irq();
    if (IAP.stat) return (1);                    // Command Failed

    return 0;
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    int len = 0;
    uint8_t *p;
    printf("write page:0x%08X sz:%d\r\n", adr, sz);
    while(len < sz)
    {
        p = (uint8_t*)buf + len;
        FLASH_WritePage(adr+len, p);
        len += PHY_PAGE_SIZE;
        printf("len:%d buf[1]:0x%X\r\n", len, p[1]);
    }
    return 0;
}

