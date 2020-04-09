/**
 * @file    target.c
 * @brief   Target information for the i.MXRT1064
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_config.h"

// The file flash_blob.c must only be included in target.c
#include "flash_blob.c"

uint8_t validate_bin_nvic(const uint8_t *buf)
{
    if(buf[0] == 'F' && buf[1] == 'C' && buf[2] == 'F' && buf[3] == 'B')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// target information
target_cfg_t target_device = {
    .sector_size    = KB(4),
    .sector_cnt     = 2048,
    .flash_start    = 0x60000000,
    .flash_end      = 0x60000000 + MB(4),
    .ram_start      = 0x20000000,
    .ram_end        = 0x20000000 + MB(4),
    .flash_algo     = (program_target_t *) &flash,
};
