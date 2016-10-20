/*
 * Epson RX8900SA/CE Realtime Clock Module
 *
 * Copyright (c) 2016 IBM Corporation
 * Authors:
 * 	 Alastair D'Silva <alastair@d-silva.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 *
 * Datasheet available at:
 * 	https://support.epson.biz/td/api/doc_check.php?dl=app_RX8900CE&lang=en
 *
 * Not implemented:
 * 	Implement Alarm functions
 * 	Implement Timer Counters
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/i2c/i2c.h"
#include "qemu/bcd.h"
#include "qemu/error-report.h"

#define TYPE_RX8900 "rx8900"
#define RX8900(obj) OBJECT_CHECK(RX8900State, (obj), TYPE_RX8900)

#define NVRAM_SIZE 0x20

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

typedef struct RX8900State {
    I2CSlave parent_obj;

    int64_t offset;
    uint8_t wday_offset;
    uint8_t nvram[NVRAM_SIZE];
    int32_t ptr;
    bool addr_byte;
} RX8900State;

static const VMStateDescription vmstate_rx8900 = {
    .name = "rx8900",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[] ) {
        VMSTATE_I2C_SLAVE(parent_obj, RX8900State),
        VMSTATE_INT64(offset, RX8900State),
        VMSTATE_UINT8_V(wday_offset, RX8900State, 2),
        VMSTATE_UINT8_ARRAY(nvram, RX8900State, NVRAM_SIZE),
        VMSTATE_INT32(ptr, RX8900State),
        VMSTATE_BOOL(addr_byte, RX8900State),
        VMSTATE_END_OF_LIST()
    }
};

static void capture_current_time(RX8900State *s)
{
    /* Capture the current time into the secondary registers
     * which will be actually read by the data transfer operation.
     */
    struct tm now;
    qemu_get_timedate(&now, s->offset);
    s->nvram[SECONDS] = to_bcd(now.tm_sec);
    s->nvram[MINUTES] = to_bcd(now.tm_min);
    s->nvram[HOURS] = to_bcd(now.tm_hour);

    s->nvram[WEEKDAY] = 0x01 << ((now.tm_wday + 1) % 7);
    s->nvram[DAY] = to_bcd(now.tm_mday);
    s->nvram[MONTH] = to_bcd(now.tm_mon + 1);
    s->nvram[YEAR] = to_bcd(now.tm_year % 100);

    s->nvram[EXT_SECONDS] = s->nvram[SECONDS];
    s->nvram[EXT_MINUTES] = s->nvram[MINUTES];
    s->nvram[EXT_HOURS] = s->nvram[HOURS];
    s->nvram[EXT_WEEKDAY] = s->nvram[WEEKDAY];
    s->nvram[EXT_DAY] = s->nvram[DAY];
    s->nvram[EXT_MONTH] = s->nvram[MONTH];
    s->nvram[EXT_YEAR] = s->nvram[YEAR];
}

static void inc_regptr(RX8900State *s)
{
    /* The register pointer wraps around after 0x1F
     */
    s->ptr = (s->ptr + 1) & (NVRAM_SIZE - 1);
    if (s->ptr == 0x00) {
        capture_current_time(s);
    }
}

static void rx8900_event(I2CSlave *i2c, enum i2c_event event)
{
    RX8900State *s = RX8900(i2c);

    switch (event) {
    case I2C_START_RECV:
        /* In h/w, capture happens on any START condition, not just a
         * START_RECV, but there is no need to actually capture on
         * START_SEND, because the guest can't get at that data
         * without going through a START_RECV which would overwrite it.
         */
        capture_current_time(s);
        break;
    case I2C_START_SEND:
        s->addr_byte = true;
        break;
    default:
        break;
    }
}

static int rx8900_recv(I2CSlave *i2c)
{
    RX8900State *s = RX8900(i2c);
    uint8_t res = s->nvram[s->ptr];
    inc_regptr(s);
    return res;
}

static void validate_extension_register(RX8900State *s, uint8_t data)
{
    uint8_t diffmask = data ^ s->nvram[EXTENSION_REGISTER];

    if ((diffmask & 1 << EXT_REG_TSEL0) || (diffmask & 1 << EXT_REG_TSEL1)) {
        error_report("WARNING: RX8900 - "
            "Timer select modified but is unimplemented");
    }

    if ((diffmask & 1 << EXT_REG_FSEL0) || (diffmask & 1 << EXT_REG_FSEL1)) {
        error_report("WARNING: RX8900 - "
            "FOut Frequency modified but is unimplemented");
    }

    if (diffmask & 1 << EXT_REG_TE) {
        error_report("WARNING: RX8900 - "
            "Timer enable modified but is unimplemented");
    }

    if (diffmask & 1 << EXT_REG_USEL) {
        error_report("WARNING: RX8900 - "
            "Update interrupt modified but is unimplemented");
    }

    if (diffmask & 1 << EXT_REG_WADA) {
        error_report("WARNING: RX8900 - "
            "Week/day alarm modified but is unimplemented");
    }

    if (data & 1 << EXT_REG_TEST) {
        error_report("WARNING: RX8900 - "
            "Test bit is enabled but is forbidden by the manufacturer");
    }
}

static void validate_control_register(RX8900State *s, uint8_t data)
{
    uint8_t diffmask = data ^ s->nvram[CONTROL_REGISTER];

    if (diffmask & 1 << CTRL_REG_RESET) {
        error_report("WARNING: RX8900 - "
            "Reset requested but is unimplemented");
    }

    if (diffmask & 1 << CTRL_REG_WP0) {
        error_report("WARNING: RX8900 - "
            "Attempt to write to write protected bit %d in control register",
            CTRL_REG_WP0);
    }

    if (diffmask & 1 << CTRL_REG_WP1) {
        error_report("WARNING: RX8900 - "
            "Attempt to write to write protected bit %d in control register",
            CTRL_REG_WP1);
    }

    if (diffmask & 1 << CTRL_REG_AIE) {
        error_report("WARNING: RX8900 - "
            "Alarm interrupt requested but is unimplemented");
    }

    if (diffmask & 1 << CTRL_REG_TIE) {
        error_report("WARNING: RX8900 - "
            "Timer interrupt requested but is unimplemented");
    }

    if (diffmask & 1 << CTRL_REG_UIE) {
        error_report("WARNING: RX8900 - "
            "Update interrupt requested but is unimplemented");
    }

}

static int rx8900_send(I2CSlave *i2c, uint8_t data)
{
    RX8900State *s = RX8900(i2c);
    struct tm now;

    if (s->addr_byte) {
        s->ptr = data & (NVRAM_SIZE - 1);
        s->addr_byte = false;
        return 0;
    }

    qemu_get_timedate(&now, s->offset);
    switch (s->ptr) {
    case SECONDS:
    case EXT_SECONDS:
        now.tm_sec = from_bcd(data & 0x7f);
        s->nvram[SECONDS] = data;
        s->nvram[EXT_SECONDS] = data;
        break;

    case MINUTES:
    case EXT_MINUTES:
        now.tm_min = from_bcd(data & 0x7f);
        s->nvram[MINUTES] = data;
        s->nvram[EXT_MINUTES] = data;
        break;

    case HOURS:
    case EXT_HOURS:
        now.tm_hour = from_bcd(data & 0x3f);
        s->nvram[HOURS] = data;
        s->nvram[EXT_HOURS] = data;
        break;

    case WEEKDAY:
    case EXT_WEEKDAY: {
        int user_wday = 0;
        /* The day field is supposed to contain a value in
         * the range 0-6. Otherwise behavior is undefined.
         */
        switch (data) {
        case 0x01:
            user_wday = 0;
            break;
        case 0x02:
            user_wday = 1;
            break;
        case 0x04:
            user_wday = 2;
            break;
        case 0x08:
            user_wday = 3;
            break;
        case 0x10:
            user_wday = 4;
            break;
        case 0x20:
            user_wday = 5;
            break;
        case 0x40:
            user_wday = 6;
            break;
        default:
            error_report("WARNING: RX8900 - weekday data '%x' is out of range,"
                    " undefined behavior will result", data);
            break;
        }
        s->wday_offset = (user_wday - now.tm_wday + 7) % 7;
        s->nvram[WEEKDAY] = data;
        s->nvram[EXT_WEEKDAY] = data;
        break;
    }

    case DAY:
    case EXT_DAY:
        now.tm_mday = from_bcd(data & 0x3f);
        s->nvram[DAY] = data;
        s->nvram[EXT_DAY] = data;
        break;

    case MONTH:
    case EXT_MONTH:
        now.tm_mon = from_bcd(data & 0x1f) - 1;
        s->nvram[MONTH] = data;
        s->nvram[EXT_MONTH] = data;
        break;

    case YEAR:
    case EXT_YEAR:
        now.tm_year = from_bcd(data) + 100;
        s->nvram[YEAR] = data;
        s->nvram[EXT_YEAR] = data;
        break;

    case EXTENSION_REGISTER:
    case EXT_EXTENSION_REGISTER:
        validate_extension_register(s, data);
        s->nvram[EXTENSION_REGISTER] = data;
        s->nvram[EXT_EXTENSION_REGISTER] = data;
        break;

    case CONTROL_REGISTER:
    case EXT_CONTROL_REGISTER:
        validate_control_register(s, data);
        s->nvram[EXTENSION_REGISTER] = data;
        s->nvram[EXT_EXTENSION_REGISTER] = data;
        break;

    default:
        s->nvram[s->ptr] = data;
    }

    inc_regptr(s);
    return 0;
}

static int rx8900_init(I2CSlave *i2c)
{
    return 0;
}

static void rx8900_reset(DeviceState *dev)
{
    RX8900State *s = RX8900(dev);

    /* The clock is running and synchronized with the host */
    s->offset = 0;
    memset(s->nvram, 0, NVRAM_SIZE);

    /* Temperature formulation from the datasheet
     * ( TEMP[ 7:0 ] * 2 – 187.19) / 3.218
     *
     * Hardcode it to 25 degrees Celcius
     */
    s->nvram[TEMPERATURE] = 135; /* (25 * 3.218 + 187.19) / 2 */

    s->nvram[CONTROL_REGISTER] = 1 << CTRL_REG_CSEL0;

    s->ptr = 0;
    s->addr_byte = false;
}

static void rx8900_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = rx8900_init;
    k->event = rx8900_event;
    k->recv = rx8900_recv;
    k->send = rx8900_send;
    dc->reset = rx8900_reset;
    dc->vmsd = &vmstate_rx8900;
}

static const TypeInfo rx8900_info = {
    .name = TYPE_RX8900,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(RX8900State),
    .class_init = rx8900_class_init,
};

static void rx8900_register_types(void)
{
    type_register_static(&rx8900_info);
}

type_init(rx8900_register_types)
