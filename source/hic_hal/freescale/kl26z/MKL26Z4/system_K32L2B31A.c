

#include <stdint.h>
#include "K32L2B31A.h"



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {

#if (DISABLE_WDOG)
  /* Disable the COP module */
  /* SIM_COPC: COPCLKSEL=0,COPDBGEN=0,COPSTPEN=0,COPT=0,COPCLKS=0,COPW=0 */
  SIM->COPC = (uint32_t)0x00u;
#endif /* (DISABLE_WDOG) */
#if (CLOCK_SETUP == 0)
  /* SIM->SOPT2: USBSRC=0 */
  SIM->SOPT2 &= ~SIM_SOPT2_USBSRC_MASK; /* USB_CLKIN is clock source for USB FS  (applicable only for derivatived with USB)*/
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x01); /* Update system prescalers */
  /* MCG->SC: FCRDIV=1 */
  MCG->SC = MCG_SC_FCRDIV(0x01);       /* Set the LIRC1 divider*/
  /* MCG->MC: HIRC=0,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_LIRC_DIV2(0x00);    /* Set the LIRC2 divider*/
  /* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint32_t)0x00u;         /* Disable External reference */
  /* MCG->C2: RANGE0=0,HGO0=0,EREFS0=0,IRCS=1 */
  MCG->C2 = MCG_C2_IRCS_MASK;         /* Enable LIRC 8MHz */
  /* Switch to LIRC 8MHz Mode */
  /* MCG->C1: CLKS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x01) |
            MCG_C1_IRCLKEN_MASK;       /* Enable LIRC and select LIRC as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x04u) {} /* Check that the clock source is the LIRC clock. */
#elif (CLOCK_SETUP == 1)
  /* SIM->SOPT2: USBSRC=0 */
  SIM->SOPT2 &= ~SIM_SOPT2_USBSRC_MASK; /* USB_CLKIN is clock source for USB FS  (applicable only for derivatived with USB)*/
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers */
  /* MCG->SC: FCRDIV=1 */
  MCG->SC = MCG_SC_FCRDIV(0x01);       /* LIRC1 divider not used - leave the default value*/
  /* MCG->MC: HIRC=1,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_HIRCEN_MASK;    /* Enable HIRC clock source*/
  /* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint32_t)0x00u;         /* Disable External reference */
  /* MCG->C2: RANGE0=0,HGO0=0,EREFS0=0,IRCS=1 */
  MCG->C2 = MCG_C2_IRCS_MASK;         /* Not used - leave default value */
  /* Switch to HIRC Mode */
  /* MCG->C1: CLKS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x00) |
            MCG_C1_IRCLKEN_MASK;       /* Leave LIRC enabled and select HIRC as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) {} /* Check that the clock source is the HIRC clock. */
#elif (CLOCK_SETUP == 2)
  /* SIM->SOPT2: USBSRC=0 */
  SIM->SOPT2 &= ~SIM_SOPT2_USBSRC_MASK; /* USB_CLKIN is clock source for USB FS  (applicable only for derivatived with USB)*/
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; /* Enable clock gate for port to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x00); /* Set system prescalers */
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~(uint32_t)(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~(uint32_t)(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
  /* MCG->SC: FCRDIV=1 */
  MCG->SC = MCG_SC_FCRDIV(0x01);       /* LIRC1 divider not used - leave the default value*/
  /* MCG->MC: HIRC=0,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_LIRC_DIV2(0x00);    /* Not used - leave the default value */
  /* MCG->C2: RANGE0=0,HGO0=0,EREFS0=1,IRCS=1 */
  MCG->C2 = MCG_C2_EREFS0_MASK | MCG_C2_IRCS_MASK;   /* Select external crystal, low range, low power, for LIRC - leave default value */
  /* OSC0->CR: ERCLKEN=1,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;         /* Enable External reference */
  /* Switch to EXT Mode */
  /* MCG->C1: CLKS=0,IRCLKEN=0,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x02);       /* Disable LIRC and select EXT as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x08u) {} /* Check that the clock source is the EXT clock. */
#elif (CLOCK_SETUP == 3)
  /* SIM->SOPT2: USBSRC=0 */
  SIM->SOPT2 &= ~SIM_SOPT2_USBSRC_MASK; /* USB_CLKIN is clock source for USB FS  (applicable only for derivatived with USB)*/
  /* MCG->MC: HIRC=1 */
  MCG->MC |= MCG_MC_HIRCEN_MASK;    /* Enable HIRC clock source*/
  /* MCG->C1: CLKS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x00) |
            MCG_C1_IRCLKEN_MASK;       /* Leave LIRC enabled and select HIRC as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) {} /* Check that the clock source is the HIRC clock. */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers */
  /* MCG->SC: FCRDIV=0 */
  MCG->SC = MCG_SC_FCRDIV(0x00);       /* Set the LIRC1 divider to 1*/
  /* MCG->MC: HIRC=0,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_LIRC_DIV2(0x00);    /* Set the LIRC2 divider to 1 */
  /* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint32_t)0x00u;         /* Disable External reference */
  /* MCG->C2: RANGE0=0,HGO0=0,EREFS0=0,IRCS=0 */
  MCG->C2 = (uint32_t)0x00u;         /* Enable LIRC 2MHz */
  /* Switch to LIRC 2MHz Mode */
  /* MCG->C1: CLKS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x01) |
            MCG_C1_IRCLKEN_MASK;       /* Enable LIRC and select LIRC as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x04u) {} /* Check that the clock source is the LIRC clock. */
#elif (CLOCK_SETUP == 4)
  /* SIM->SOPT2: USBSRC=1 */
  SIM->SOPT2 |= SIM_SOPT2_USBSRC_MASK; /* Internal 48 MHz oscillator (IRC48) is clock source for USB FS */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers */
  /* MCG->SC: FCRDIV=1 */
  MCG->SC = MCG_SC_FCRDIV(0x01);       /* LIRC1 divider not used - leave the default value*/
  /* MCG->MC: HIRC=1,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_HIRCEN_MASK;    /* Enable HIRC clock source*/
  /* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = (uint32_t)0x00u;         /* Disable External reference */
  /* MCG->C2: RANGE0=0,HGO0=0,EREFS0=0,IRCS=1 */
  MCG->C2 = MCG_C2_IRCS_MASK;         /* Not used - leave default value */
  /* Switch to HIRC Mode */
  /* MCG->C1: CLKS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x00) |
            MCG_C1_IRCLKEN_MASK;       /* Leave LIRC enabled and select HIRC as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) {} /* Check that the clock source is the HIRC clock. */
#elif (CLOCK_SETUP == 5)
  /* SIM->SOPT2: USBSRC=0 */
  SIM->SOPT2 &= ~SIM_SOPT2_USBSRC_MASK; /* USB_CLKIN is clock source for USB FS  (applicable only for derivatived with USB)*/
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; /* Enable clock gate for port to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                 SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers */
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~(uint32_t)(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA->PCR[19] &= (uint32_t)~(uint32_t)(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
  /* MCG->SC: FCRDIV=1 */
  MCG->SC = MCG_SC_FCRDIV(0x01);       /* LIRC1 divider not used - leave the default value*/
  /* MCG->MC: HIRC=0,LIRC_DIV2=0 */
  MCG->MC = MCG_MC_LIRC_DIV2(0x00);    /* Not used - leave the default value */
  /* MCG->C2: RANGE0=1,HGO0=0,EREFS0=1,IRCS=1 */
  MCG->C2 = MCG_C2_EREFS0_MASK | MCG_C2_IRCS_MASK | MCG_C2_RANGE0(1);   /* Select external crystal, high range, low power, for LIRC - leave default value */
  /* OSC0->CR: ERCLKEN=1,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;         /* Enable External reference */
  /* Switch to EXT Mode */
  /* MCG->C1: CLKS=0,IRCLKEN=0,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x02);       /* Disable LIRC and select EXT as a clock source */
  while((MCG->S & MCG_S_CLKST_MASK) != 0x08u) {} /* Check that the clock source is the EXT clock. */
#endif

}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t ICSOUTClock;                                                        /* Variable to store output clock frequency of the ICS module */
  uint8_t Divider;

  if ((MCG->S & MCG_S_CLKST_MASK) == 0x04u) {
    /* LIRC reference clock is selected */
    ICSOUTClock = CPU_INT_SLOW_CLK_HZ;
    Divider = (uint8_t)(1u << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));
    ICSOUTClock = (ICSOUTClock / Divider);  /* Calculate the divided LIRC clock */
  } else if ((MCG->S & MCG_S_CLKST_MASK) == 0x0u) {
    /* HIRC reference clock is selected */
    ICSOUTClock = CPU_INT_FAST_CLK_HZ;
  } else if ((MCG->S & MCG_S_CLKST_MASK) == 0x80u) {
    /* External reference clock is selected */
    ICSOUTClock = CPU_XTAL_CLK_HZ;
  } else {
    /* Reserved value */
    return;
  }
  SystemCoreClock = (ICSOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));

}
