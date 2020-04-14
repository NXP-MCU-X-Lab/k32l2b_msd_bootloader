
#ifndef SYSTEM_K32L2B_H_
#define SYSTEM_K32L2B_H_                        /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define DISABLE_WDOG    1

#ifndef CLOCK_SETUP
  #define CLOCK_SETUP   1
#endif
/* Predefined clock setups
   0 ... Multipurpose Clock Generator Lite (MCG_Lite) in Low-frequency Internal Reference Clock 8 MHz (LIRC 8 MHz) mode
         Default part configuration.
         Core clock/Bus clock derived from the internal clock source 8 MHz
         Core clock = 4MHz, BusClock = 2MHz, USB FS clock derived from external clock USB_CLKIN (applicable only for derivatived with USB)
   1 ... Multipurpose Clock Generator Lite (MCG_Lite) in High-frequency Internal Reference Clock (HIRC) mode
         Maximum achievable clock frequency configuration using internal clock.
         Core clock/Bus clock derived from the internal clock source 48MHz
         Core clock = 48MHz, BusClock = 24MHz, USB FS clock derived from external clock USB_CLKIN (applicable only for derivatived with USB)
   2 ... Multipurpose Clock Generator Lite (MCG_Lite) in External Oscillator (EXT) mode
         Core clock/Bus clock derived directly from the external crystal 32.768kHz
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 32.768kHz, BusClock = 32.768kHz, USB FS clock derived from external clock USB_CLKIN (applicable only for derivatived with USB)
   3 ... Multipurpose Clock Generator Lite (MCG_Lite) in Low-frequency Internal Reference Clock 2 MHz (LIRC 2 MHz) mode
         Core clock/Bus clock derived from the internal clock source 2 MHz
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 2MHz, BusClock = 1MHz, USB FS clock derived from external clock USB_CLKIN (applicable only for derivatived with USB)
   4 ... Multipurpose Clock Generator Lite (MCG_Lite) in High-frequency Internal Reference Clock (HIRC) mode
         USB clock setup - for USB to receive internal 48MHz clock derived from HIRC.
         Core clock/Bus clock derived from the internal clock source 48MHz
         Core clock = 48MHz, BusClock = 24MHz, USB FS clock derived from HIRC (MCGPCLK)
   5 ... Multipurpose Clock Generator Lite (MCG_Lite) in External Oscillator (EXT) mode
         Core clock/Bus clock derived directly from the external crystal 8 MHz
         Core clock = 8MHz, BusClock = 4MHz, USB FS clock derived from external clock USB_CLKIN (applicable only for derivatived with USB)
*/

/*----------------------------------------------------------------------------
  Define clock source values
 *----------------------------------------------------------------------------*/
#if (CLOCK_SETUP == 0)
    #define CPU_XTAL_CLK_HZ                 32768u    /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             8000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            4000000u  /* Default System clock value */
#elif (CLOCK_SETUP == 1)
    #define CPU_XTAL_CLK_HZ                 32768u    /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             8000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            48000000u /* Default System clock value */
#elif (CLOCK_SETUP == 2)
    #define CPU_XTAL_CLK_HZ                 32768u    /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             8000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            32768u    /* Default System clock value */
#elif (CLOCK_SETUP == 3)
    #define CPU_XTAL_CLK_HZ                 32768u    /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             2000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            2000000u  /* Default System clock value */
#elif (CLOCK_SETUP == 4)
    #define CPU_XTAL_CLK_HZ                 32768u    /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             8000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            48000000u /* Default System clock value */
#elif (CLOCK_SETUP == 5)
    #define CPU_XTAL_CLK_HZ                 8000000u  /* Value of the external crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             8000000u  /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             48000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            8000000u    /* Default System clock value */
#endif /* (CLOCK_SETUP == 5) */


/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (void);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif
