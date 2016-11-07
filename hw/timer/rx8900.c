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
 * Not implemented:
 *  Implement Alarm functions
 *  Implement Timer Counters
 *  Implement i2c timeout
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/i2c/i2c.h"
#include "hw/timer/rx8900_regs.h"
#include "hw/ptimer.h"
#include "qemu/main-loop.h"
#include "qemu/bcd.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"

#define TYPE_RX8900 "rx8900"
#define RX8900(obj) OBJECT_CHECK(RX8900State, (obj), TYPE_RX8900)

static bool log;

typedef struct RX8900State {
    I2CSlave parent_obj;

    ptimer_state *update_timer;
    int64_t offset;
    uint8_t weekday; /* Saved for deferred offset calculation, 0-6 */
    uint8_t wday_offset;
    uint8_t nvram[RX8900_NVRAM_SIZE];
    int32_t ptr; /* Wrapped to stay within RX8900_NVRAM_SIZE */
    bool addr_byte;
} RX8900State;

static const VMStateDescription vmstate_rx8900 = {
    .name = "rx8900",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(update_timer, RX8900State),
        VMSTATE_I2C_SLAVE(parent_obj, RX8900State),
        VMSTATE_INT64(offset, RX8900State),
        VMSTATE_UINT8_V(weekday, RX8900State, 2),
        VMSTATE_UINT8_V(wday_offset, RX8900State, 2),
        VMSTATE_UINT8_ARRAY(nvram, RX8900State, RX8900_NVRAM_SIZE),
        VMSTATE_INT32(ptr, RX8900State),
        VMSTATE_BOOL(addr_byte, RX8900State),
        VMSTATE_END_OF_LIST()
    }
};

#define RX8900_TRACE_BUF_SIZE 256
/**
 * Emit a trace message
 * @param file the source filename
 * @param line the line number the message was emitted from
 * @param dev the RX8900 device
 * @param fmt a printf style format
 */
static void trace(const char *file, int line, const char *func,
        I2CSlave *dev, const char *fmt, ...)
{
    va_list ap;
    char buf[RX8900_TRACE_BUF_SIZE];
    int len;

    len = snprintf(buf, sizeof(buf), "\n\t%s:%s:%d: RX8900 %s %s@0x%x: %s",
            file, func, line, dev->qdev.id, dev->qdev.parent_bus->name,
            dev->address, fmt);
    if (len >= RX8900_TRACE_BUF_SIZE) {
        error_report("%s:%d: Trace buffer overflow", file, line);
    }

    va_start(ap, fmt);
    error_vreport(buf, ap);
    va_end(ap);
}

/**
 * Emit a trace message
 * @param dev the RX8900 device
 * @param fmt a printf format
 */
#define TRACE(dev, fmt, ...) \
    do { \
        if (log) { \
            trace(__FILE__, __LINE__, __func__, &dev, fmt, ## __VA_ARGS__); \
        } \
    } while (0)


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

    s->nvram[WEEKDAY] = 0x01 << ((now.tm_wday + s->wday_offset) % 7);
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

    TRACE(s->parent_obj, "Update current time to %02d:%02d:%02d %d %d/%d/%d "
            "(0x%02x%02x%02x%02x%02x%02x%02x)",
            now.tm_hour, now.tm_min, now.tm_sec,
            (now.tm_wday + s->wday_offset) % 7,
            now.tm_mday, now.tm_mon, now.tm_year + 1900,
            s->nvram[HOURS], s->nvram[MINUTES], s->nvram[SECONDS],
            s->nvram[DAY], s->nvram[MONTH], s->nvram[YEAR]);
}

static void inc_regptr(RX8900State *s)
{
    /* The register pointer wraps around after 0x1F
     */
    s->ptr = (s->ptr + 1) & (RX8900_NVRAM_SIZE - 1);
    TRACE(s->parent_obj, "Operating on register 0x%02x", s->ptr);

    if (s->ptr == 0x00) {
        TRACE(s->parent_obj, "Register pointer has overflowed, wrapping to 0");
        capture_current_time(s);
    }
}

static void rx8900_event(I2CSlave *i2c, enum i2c_event event)
{
    RX8900State *s = RX8900(i2c);

    switch (event) {
    case I2C_START_RECV:
        /* In h/w, time capture happens on any START condition, not just a
         * START_RECV. For the emulation, it doesn't actually matter,
         * since a START_RECV has to occur before the data can be read.
         */
        capture_current_time(s);
        break;
    case I2C_START_SEND:
        s->addr_byte = true;
        break;
    case I2C_FINISH:
        if (s->weekday < 7) {
            /* We defer the weekday calculation as it is handed to us before
             * the date has been updated. If we calculate the weekday offset
             * when it is passed to us, we will incorrectly determine it
             * based on the current emulated date, rather than the date that
             * has been written.
             */
            struct tm now;
            qemu_get_timedate(&now, s->offset);

            s->wday_offset = (s->weekday - now.tm_wday + 7) % 7;

            TRACE(s->parent_obj, "Set weekday to 0x%02x, wday_offset=%d",
                    s->weekday, s->wday_offset);

            s->weekday = 7;
        }
        break;

    default:
        break;
    }
}

static int rx8900_recv(I2CSlave *i2c)
{
    RX8900State *s = RX8900(i2c);
    uint8_t res = s->nvram[s->ptr];
    TRACE(s->parent_obj, "Read register 0x%x = 0x%x", s->ptr, res);
    inc_regptr(s);
    return res;
}

static void validate_extension_register(RX8900State *s, uint8_t data)
{
    uint8_t diffmask = data ^ s->nvram[EXTENSION_REGISTER];

    if ((diffmask & EXT_MASK_TSEL0) || (diffmask & EXT_MASK_TSEL1)) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Timer select modified but is unimplemented");
    }

    if ((diffmask & EXT_MASK_FSEL0) || (diffmask & EXT_MASK_FSEL1)) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "FOut Frequency modified but is unimplemented");
    }

    if (diffmask & EXT_MASK_TE) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Timer enable modified but is unimplemented");
    }

    if (diffmask & EXT_MASK_USEL) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Update interrupt modified but is unimplemented");
    }

    if (diffmask & EXT_MASK_WADA) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Week/day alarm modified but is unimplemented");
    }

    if (data & EXT_MASK_TEST) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Test bit is enabled but is forbidden by the manufacturer");
    }
}

static void validate_control_register(RX8900State *s, uint8_t data)
{
    uint8_t diffmask = data ^ s->nvram[CONTROL_REGISTER];

    if (diffmask & CTRL_MASK_RESET) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Reset requested but is unimplemented");
    }

    if (diffmask & CTRL_MASK_WP0) {
        error_report("WARNING: RX8900 - "
            "Attempt to write to write protected bit %d in control register",
            CTRL_REG_WP0);
    }

    if (diffmask & CTRL_MASK_WP1) {
        error_report("WARNING: RX8900 - "
            "Attempt to write to write protected bit %d in control register",
            CTRL_REG_WP1);
    }

    if (diffmask & CTRL_MASK_AIE) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Alarm interrupt requested but is unimplemented");
    }

    if (diffmask & CTRL_MASK_TIE) {
        qemu_log_mask(LOG_UNIMP, "WARNING: RX8900 - "
            "Timer interrupt requested but is unimplemented");
    }
}

static void disable_update_timer(RX8900State *s)
{
    /* Fill this in to disable the timer */
    return;
}

static void enable_update_timer(RX8900State *s)
{
    /* Update once per second */
    ptimer_set_freq(s->update_timer, 1);
    ptimer_run(s->update_timer, 0);
}

static int rx8900_send(I2CSlave *i2c, uint8_t data)
{
    RX8900State *s = RX8900(i2c);
    struct tm now;

    TRACE(s->parent_obj, "Received I2C data 0x%02x", data);

    if (s->addr_byte) {
        s->ptr = data & (RX8900_NVRAM_SIZE - 1);
        TRACE(s->parent_obj, "Operating on register 0x%02x", s->ptr);
        s->addr_byte = false;
        return 0;
    }

    TRACE(s->parent_obj, "Set data 0x%02x=0x%02x", s->ptr, data);

    qemu_get_timedate(&now, s->offset);
    switch (s->ptr) {
    case SECONDS:
    case EXT_SECONDS:
        now.tm_sec = from_bcd(data & 0x7f);
        s->offset = qemu_timedate_diff(&now);
        break;

    case MINUTES:
    case EXT_MINUTES:
        now.tm_min = from_bcd(data & 0x7f);
        s->offset = qemu_timedate_diff(&now);
        break;

    case HOURS:
    case EXT_HOURS:
        now.tm_hour = from_bcd(data & 0x3f);
        s->offset = qemu_timedate_diff(&now);
        break;

    case WEEKDAY:
    case EXT_WEEKDAY: {
        int user_wday = ctz32(data);
        /* The day field is supposed to contain a value in
         * the range 0-6. Otherwise behavior is undefined.
         */
        switch (data) {
        case 0x01:
        case 0x02:
        case 0x04:
        case 0x08:
        case 0x10:
        case 0x20:
        case 0x40:
            break;
        default:
            error_report("WARNING: RX8900 - weekday data '%x' is out of range,"
                    " undefined behavior will result", data);
            break;
        }
        s->weekday = user_wday;
        break;
    }

    case DAY:
    case EXT_DAY:
        now.tm_mday = from_bcd(data & 0x3f);
        s->offset = qemu_timedate_diff(&now);
        break;

    case MONTH:
    case EXT_MONTH:
        now.tm_mon = from_bcd(data & 0x1f) - 1;
        s->offset = qemu_timedate_diff(&now);
        break;

    case YEAR:
    case EXT_YEAR:
        now.tm_year = from_bcd(data) + 100;
        s->offset = qemu_timedate_diff(&now);
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
        if (s->nvram[CONTROL_REGISTER] == CTRL_MASK_UIE) {
            enable_update_timer(s);
        } else {
            disable_update_timer(s);
        }
        s->nvram[CONTROL_REGISTER] = data;
        s->nvram[EXT_CONTROL_REGISTER] = data;
        break;

    default:
        s->nvram[s->ptr] = data;
    }

    inc_regptr(s);
    return 0;
}

/**
 * Calculate the device temperature in Celcius
 */
static void rx8900_get_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    RX8900State *s = RX8900(obj);
    double value = (s->nvram[TEMPERATURE] * 2.0f - 187.1f) / 3.218f;

    TRACE(s->parent_obj, "Read temperature property, 0x%x = %f°C",
            s->nvram[TEMPERATURE], value);

    visit_type_number(v, name, &value, errp);
}

/**
 * Set the device temperature in Celcius
 */
static void rx8900_set_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    RX8900State *s = RX8900(obj);
    Error *local_err = NULL;
    double temp; /* degrees Celcius */
    visit_type_number(v, name, &temp, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    if (temp >= 100 || temp < -58) {
        error_setg(errp, "value %f°C is out of range", temp);
        return;
    }

    s->nvram[TEMPERATURE] = (uint8_t) ((temp * 3.218f + 187.19f) / 2);

    TRACE(s->parent_obj, "Set temperature property, 0x%x = %f°C",
            s->nvram[TEMPERATURE], temp);
}


static int rx8900_init(I2CSlave *i2c)
{
    TRACE(*i2c, "Initialized");

    return 0;
}

static void rx8900_initfn(Object *obj)
{
    object_property_add(obj, "temperature", "number",
                        rx8900_get_temperature,
                        rx8900_set_temperature, NULL, NULL, NULL);
}

static void rx8900_reset(DeviceState *dev)
{
    RX8900State *s = RX8900(dev);

    TRACE(s->parent_obj, "Reset");

    /* The clock is running and synchronized with the host */
    s->offset = 0;
    s->weekday = 7; /* Set to an invalid value */
    memset(s->nvram, 0, RX8900_NVRAM_SIZE);

    /* Temperature formulation from the datasheet
     * ( TEMP[ 7:0 ] * 2 - 187.19) / 3.218
     *
     * Set the initial state to 25 degrees Celcius
     */
    s->nvram[TEMPERATURE] = 135; /* (25 * 3.218 + 187.19) / 2 */

    s->nvram[CONTROL_REGISTER] = BIT(CTRL_REG_CSEL0);

    s->ptr = 0;
    TRACE(s->parent_obj, "Operating on register 0x%02x", s->ptr);

    s->addr_byte = false;
}

static void rx8900_raise_interrupt(void)
{
}

static void rx8900_update_timer_tick(void *opaque)
{
    RX8900State *s = (RX8900State *)opaque;

    if (s->nvram[EXTENSION_REGISTER] & EXT_MASK_USEL) {
        /* Update once per minute */
        capture_current_time(s);
        if (s->nvram[HOURS] != 0x0) {
            return;
        }
    }
    s->nvram[FLAG_REGISTER] |= FLAG_MASK_UF;
    rx8900_raise_interrupt();
}

static void rx8900_realize(DeviceState *dev, Error **errp)
{
    RX8900State *s = RX8900(dev);
    QEMUBH *bh;

    bh = qemu_bh_new(rx8900_update_timer_tick, s);
    s->update_timer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);
}

static void rx8900_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = rx8900_init;
    k->event = rx8900_event;
    k->recv = rx8900_recv;
    k->send = rx8900_send;
    dc->realize = rx8900_realize;
    dc->reset = rx8900_reset;
    dc->vmsd = &vmstate_rx8900;
}

static const TypeInfo rx8900_info = {
    .name = TYPE_RX8900,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(RX8900State),
    .instance_init = rx8900_initfn,
    .class_init = rx8900_class_init,
};

static void rx8900_register_types(void)
{
    log = getenv("RX8900_TRACE") != NULL;
    type_register_static(&rx8900_info);
}

type_init(rx8900_register_types)
