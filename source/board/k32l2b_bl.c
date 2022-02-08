

#include "target_config.h"
#include "daplink_addr.h"
#include "compiler.h"

const char *board_id = "0000";



// kl26z128 target information
target_cfg_t target_device = {
    .sector_size    = 1024,
    // Assume memory is regions are same size. Flash algo should ignore requests
    //  when variable sized sectors exist
    // .sector_cnt = ((.flash_end - .flash_start) / .sector_size);
    .sector_cnt     = (DAPLINK_ROM_IF_SIZE / 1024),
    .flash_start    = DAPLINK_ROM_IF_START,
    .flash_end      = DAPLINK_ROM_IF_START + DAPLINK_ROM_IF_SIZE,
    .ram_start      = 0x20000000,
    .ram_end        = 0x20006000,
    /* .flash_algo not needed for bootloader */
};
