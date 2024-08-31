 /* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef __GC02M1_MACRO_OFILM_MIPI_RAW_EEPROM_H__
#define __GC02M1_MACRO_OFILM_MIPI_RAW_EEPROM_H__

#include "kd_camera_typedef.h"

/*
 * LRC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_gc02m1_ofilm_mipi_raw_LRC(BYTE *data);

/*
 * DCC
 *
 * @param data Buffer
 * @return size of data
 */
unsigned int read_gc02m1_ofilm_mipi_raw_DCC(BYTE *data);

#endif

