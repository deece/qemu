/*
 * Epson RX8900SA/CE Realtime Clock Module
 *
 * Copyright (c) 2016 IBM Corporation
 * Authors:
 *  Alastair D'Silva <alastair@d-silva.org>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 * Datasheet available at:
 *  https://support.epson.biz/td/api/doc_check.php?dl=app_RX8900CE&lang=en
 *
 */

#ifndef RX8900_REGS_H
#define RX8900_REGS_H

#define RX8900_NVRAM_SIZE 0x20

typedef enum RX8900Addresses {
    SECONDS = 0x00,
    MINUTES = 0x01,
    HOURS = 0x02,
    WEEKDAY = 0x03,
    DAY = 0x04,
    MONTH = 0x05,
    YEAR = 0x06,
    RAM = 0x07,
    ALARM_MINUTE = 0x08,
    ALARM_HOUR = 0x09,
    ALARM_WEEK_DAY = 0x0A,
    TIMER_COUNTER_0 = 0x0B,
    TIMER_COUNTER_1 = 0x0C,
    EXTENSION_REGISTER = 0x0D,
    FLAG_REGISTER = 0X0E,
    CONTROL_REGISTER = 0X0F,
    EXT_SECONDS = 0x010, /* Alias of SECONDS */
    EXT_MINUTES = 0x11, /* Alias of MINUTES */
    EXT_HOURS = 0x12, /* Alias of HOURS */
    EXT_WEEKDAY = 0x13, /* Alias of WEEKDAY */
    EXT_DAY = 0x14, /* Alias of DAY */
    EXT_MONTH = 0x15, /* Alias of MONTH */
    EXT_YEAR = 0x16, /* Alias of YEAR */
    TEMPERATURE = 0x17,
    BACKUP_FUNCTION = 0x18,
    NO_USE_1 = 0x19,
    NO_USE_2 = 0x1A,
    EXT_TIMER_COUNTER_0 = 0x1B, /* Alias of TIMER_COUNTER_0 */
    EXT_TIMER_COUNTER_1 = 0x1C, /* Alias of TIMER_COUNTER_1 */
    EXT_EXTENSION_REGISTER = 0x1D, /* Alias of EXTENSION_REGISTER */
    EXT_FLAG_REGISTER = 0X1E, /* Alias of FLAG_REGISTER */
    EXT_CONTROL_REGISTER = 0X1F /* Alias of CONTROL_REGISTER */
} RX8900Addresses;

typedef enum ExtRegBits {
    EXT_REG_TSEL0 = 0,
    EXT_REG_TSEL1 = 1,
    EXT_REG_FSEL0 = 2,
    EXT_REG_FSEL1 = 3,
    EXT_REG_TE = 4,
    EXT_REG_USEL = 5,
    EXT_REG_WADA = 6,
    EXT_REG_TEST = 7
} ExtRegBits;

typedef enum ExtRegMasks {
    EXT_MASK_TSEL0 = 1UL << 0,
    EXT_MASK_TSEL1 = 1UL << 1,
    EXT_MASK_FSEL0 = 1UL << 2,
    EXT_MASK_FSEL1 = 1UL << 3,
    EXT_MASK_TE = 1UL << 4,
    EXT_MASK_USEL = 1UL << 5,
    EXT_MASK_WADA = 1UL << 6,
    EXT_MASK_TEST = 1UL << 7
} ExtRegMasks;

typedef enum CtrlRegBits {
    CTRL_REG_RESET = 0,
    CTRL_REG_WP0 = 1,
    CTRL_REG_WP1 = 2,
    CTRL_REG_AIE = 3,
    CTRL_REG_TIE = 4,
    CTRL_REG_UIE = 5,
    CTRL_REG_CSEL0 = 6,
    CTRL_REG_CSEL1 = 7
} CtrlRegBits;

typedef enum CtrlRegMask {
    CTRL_MASK_RESET = 0,
    CTRL_MASK_WP0 = 1,
    CTRL_MASK_WP1 = 2,
    CTRL_MASK_AIE = 3,
    CTRL_MASK_TIE = 4,
    CTRL_MASK_UIE = 5,
    CTRL_MASK_CSEL0 = 6,
    CTRL_MASK_CSEL1 = 7
} CtrlRegMask;

typedef enum FlagRegBits {
    FLAG_REG_VDET = 0,
    FLAG_REG_VLF = 1,
    /* No bit 2 */
    FLAG_REG_AF = 3,
    FLAG_REG_TF = 4,
    FLAG_REG_UF = 5
    /* No bit 6 */
    /* No bit 7 */
} FlagRegBits;

typedef enum FlagRegMask {
    FLAG_MASK_VDET = 0,
    FLAG_MASK_VLF = 1,
    /* No bit 2 */
    FLAG_MASK_AF = 3,
    FLAG_MASK_TF = 4,
    FLAG_MASK_UF = 5
    /* No bit 6 */
    /* No bit 7 */
} FlagRegMask;

#endif
