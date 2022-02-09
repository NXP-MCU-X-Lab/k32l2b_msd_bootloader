# LPC51U68MSD bootloader



The project is derivate from ARM daplink open source project, The original ARMmbed bootloader code is well tested on serval hardware and proven to be very stable.

https://github.com/ARMmbed/DAPLink



| Folder       | Description                         |
| ------------ | ----------------------------------- |
| binary       | Example application start at 0x8000 |
| projectfiles | bootloader Keil project             |
| source       | bootloader source code              |

#define DAPLINK_ROM_START               0x00000000
#define DAPLINK_ROM_SIZE                0x00040000

/* ROM sizes */

#define DAPLINK_ROM_IF_START            0x00008000
#define DAPLINK_ROM_IF_SIZE             0x00020000

#define DAPLINK_ROM_CONFIG_USER_START   0x00028000
#define DAPLINK_ROM_CONFIG_USER_SIZE    0x00018000



# How To Use

1. Open **\k32l2b_msd_bootloader\projectfiles\uvision\k32l2b_bl**  Keil project, compile and download to target board.
2. Plug USB cable to board
3. Hold PIO1_0 and press reset
4. Drag frdm_**lpc51u68_fast_0x8000.bin** or **lpc51u68_slow_0x8000.bin** under example_app folder to update image.





**DATA PROGRAMMING:**

* drag any valid K32l2b image(any file name) will programming at 0x8000.

* drag config data will programming at 0x00028000

config data:

config data is defined as the first 3 byte of file must be 'C', 'F', 'G'. as in the example_app folder shows:

![](./img/cfgdata.png)





