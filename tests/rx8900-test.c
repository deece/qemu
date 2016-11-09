/*
 * QTest testcase for the Enpes RX8900SA/CE RTC
 *
 * Copyright (c) 2016 IBM Corporation
 * Authors:
 *  Alastair D'Silva <alastair@d-silva.org>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/timer/rx8900_regs.h"
#include "libqtest.h"
#include "libqos/i2c.h"
#include "qemu/timer.h"

#define IMX25_I2C_0_BASE 0x43F80000
#define RX8900_TEST_ID "rx8900-test"
#define RX8900_ADDR 0x32

static I2CAdapter *i2c;
static uint8_t addr;

static inline uint8_t bcd2bin(uint8_t x)
{
    return (x & 0x0f) + (x >> 4) * 10;
}

static inline uint8_t bin2bcd(uint8_t x)
{
    return (x / 10 << 4) | (x % 10);
}

static void qmp_rx8900_set_temperature(const char *id, double value)
{
    QDict *response;

    response = qmp("{ 'execute': 'qom-set', 'arguments': { 'path': %s, "
                   "'property': 'temperature', 'value': %f } }", id, value);
    g_assert(qdict_haskey(response, "return"));
    QDECREF(response);
}

/**
 * Read an RX8900 register
 * @param reg the address of the register
 * @return the value of the register
 */
static uint8_t read_register(RX8900Addresses reg)
{
    uint8_t val;
    uint8_t reg_address = (uint8_t)reg;

    i2c_send(i2c, addr, &reg_address, 1);
    i2c_recv(i2c, addr, &val, 1);

    fprintf(stderr, "Read register 0x%02x, value=0x%02x\n", reg, val);

    return val;
}

/**
 * Write to an RX8900 register
 * @param reg the address of the register
 * @param val the value to write
 */
static uint8_t write_register(RX8900Addresses reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;

    fprintf(stderr, "Writing register 0x%02x, value=0x%02x\n", reg, val);

    i2c_send(i2c, addr, buf, 2);

    return val;
}

/**
 * Set bits in a register
 * @param reg the address of the register
 * @param mask a mask of the bits to set
 */
static void set_bits_in_register(RX8900Addresses reg, uint8_t mask)
{
    uint8_t value = read_register(reg);
    fprintf(stderr, "set bits, register=%02x old=%02x mask=%02x ", reg, value, mask);
    value |= mask;
    fprintf(stderr, "new=%02x\n", value);
    write_register(reg, value);
}

/**
 * Clear bits in a register
 * @param reg the address of the register
 * @param mask a mask of the bits to set
 */
static void clear_bits_in_register(RX8900Addresses reg, uint8_t mask)
{
    uint8_t value = read_register(reg);
    fprintf(stderr, "clear bits, register=%02x old=%02x mask=%02x ", reg, value, mask);
    value &= ~mask;
    fprintf(stderr, "new=%02x\n", value);
    write_register(reg, value);
}

/**
 * Read a number of sequential RX8900 registers
 * @param reg the address of the first register
 * @param buf (out) an output buffer to stash the register values
 * @param count the number of registers to read
 */
static void read_registers(RX8900Addresses reg, uint8_t *buf, uint8_t count)
{
    uint8_t val;
    uint8_t reg_address = (uint8_t)reg;

    i2c_send(i2c, addr, &reg_address, 1);
    i2c_recv(i2c, addr, buf, count);
}

/**
 * Write to a sequential number of RX8900 registers
 * @param reg the address of the first register
 * @param buffer a buffer of values to write
 * @param count the sumber of registers to write
 */
static void write_registers(RX8900Addresses reg, uint8_t *buffer, uint8_t count)
{
    uint8_t reg_address = (uint8_t)reg;
    uint8_t buf[RX8900_NVRAM_SIZE + 1];

    buf[0] = (uint8_t)reg;
    memcpy(buf + 1, buffer, count);

    i2c_send(i2c, addr, buf, count + 1);
}

/**
 * Set the time on the RX8900
 * @param secs the seconds to set
 * @param mins the minutes to set
 * @param hours the hours to set
 * @param weekday the day of the week to set (0 = Sunday)
 * @param day the day of the month to set
 * @param month the month to set
 * @param year the year to set
 */
static void set_time(uint8_t secs, uint8_t mins, uint8_t hours,
        uint8_t weekday, uint8_t day, uint8_t month, uint8_t year)
{
    uint8_t buf[7];

    buf[0] = bin2bcd(secs);
    buf[1] = bin2bcd(mins);
    buf[2] = bin2bcd(hours);
    buf[3] = BIT(weekday);
    buf[4] = bin2bcd(day);
    buf[5] = bin2bcd(month);
    buf[6] = bin2bcd(year);

    write_registers(SECONDS, buf, 7);
}


/**
 * Check basic communication
 */
static void send_and_receive(void)
{
    uint8_t rx8900_register;
    uint8_t buf[7];
    time_t now = time(NULL);
    struct tm *tm_ptr;

    /* retrieve the date */
    read_registers(SECONDS, buf, 7);

    tm_ptr = gmtime(&now);

    /* check retrieved time against local time */
    g_assert_cmpuint(bcd2bin(buf[0]), == , tm_ptr->tm_sec);
    g_assert_cmpuint(bcd2bin(buf[1]), == , tm_ptr->tm_min);
    g_assert_cmpuint(bcd2bin(buf[2]), == , tm_ptr->tm_hour);
    g_assert_cmpuint(bcd2bin(buf[4]), == , tm_ptr->tm_mday);
    g_assert_cmpuint(bcd2bin(buf[5]), == , 1 + tm_ptr->tm_mon);
    g_assert_cmpuint(2000 + bcd2bin(buf[6]), == , 1900 + tm_ptr->tm_year);
}

/**
 * Check that the temperature can be altered via properties
 */
static void check_temperature(void)
{
   /* Check the initial temperature is 25C */
    uint8_t temperature;

    temperature = read_register(TEMPERATURE);
    g_assert_cmpuint(temperature, == , 135);

    /* Set the temperature to 40C and check the temperature again */
    qmp_rx8900_set_temperature(RX8900_TEST_ID, 40.0f);
    temperature = read_register(TEMPERATURE);
    g_assert_cmpuint(temperature, == , 157);
}

/**
 * Check that the time rolls over correctly
 */
static void check_rollover(void)
{
    uint8_t buf[7];


    set_time(59, 59, 23, 1, 29, 2, 16);

    /* Wait for the clock to rollover */
    sleep(2);

    memset(buf, 0, sizeof(buf));

    /* Check that the clock rolled over */
    /* Read from registers starting at 0x00 */
    buf[0] = 0x00;

    read_registers(SECONDS, buf, 7);

    /* Ignore seconds as there may be some noise,
     * we expect 00:00:xx Tuesday 1/3/2016
     */
    g_assert_cmpuint(bcd2bin(buf[1]), == , 0);
    g_assert_cmpuint(bcd2bin(buf[2]), == , 0);
    g_assert_cmpuint(bcd2bin(buf[3]), == , 0x04);
    g_assert_cmpuint(bcd2bin(buf[4]), == , 1);
    g_assert_cmpuint(bcd2bin(buf[5]), == , 3);
    g_assert_cmpuint(bcd2bin(buf[6]), == , 16);
}

uint32_t interrupt_counts[RX8900_INTERRUPT_SOURCES];

/**
 * Reset the interrupt counts
 */
static void count_reset(void)
{
    for (int source = 0; source < RX8900_INTERRUPT_SOURCES; source++) {
        interrupt_counts[source] = 0;
    }
}

/**
 * Handle an RX8900 interrupt (update the counts for that interrupt type)
 */
static void handle_interrupt(void)
{
    uint8_t flags = read_register(FLAG_REGISTER);

    for (int flag = 0; flag < 8; flag++) {
        if (flags & BIT(flag)) {
            interrupt_counts[flag]++;
        }
    }

    write_register(FLAG_REGISTER, 0x00);
}

/**
 * Sleep for some time while counting interrupts
 * @param delay the delay in microseconds
 */
static void wait_for(uint64_t delay)
{
    struct timeval end, now;
    bool last_interrupt_level = 0;

    gettimeofday(&end, NULL);
    delay += end.tv_usec;
    end.tv_sec += delay / 1000000;
    end.tv_usec = delay % 1000000;

    while (gettimeofday(&now, NULL),
            now.tv_sec < end.tv_sec || now.tv_usec < end.tv_usec) {
        QDict *resp = qmp("{'execute':'query-status'}");
        bool interrupt_level = get_irq(0);

        if (interrupt_level && !last_interrupt_level) {
            fprintf(stderr, "Received interrupt\n");
            /* Interrupt has been raised */
            handle_interrupt();
        }

        last_interrupt_level = interrupt_level;
        clock_step(1000000); /* 1ms */
    }
}

/**
 * Check that when the update timer interrupt is disabled, that no interrupts
 * occur
 */
static void check_update_interrupt_disabled(void)
{
    /* Disable the update interrupt */
    clear_bits_in_register(CONTROL_REGISTER, CTRL_MASK_UIE);

    /* Wait for the clock to rollover, this will cover both seconds & minutes
     */
    set_time(59, 59, 23, 1, 29, 2, 16);

    count_reset();
    wait_for(2 * 1000000);

    g_assert_cmpuint(interrupt_counts[FLAG_REG_UF], ==, 0);
}

/**
 * Check that when the update timer interrupt is enabled and configured for
 * per second updates, that we get the appropriate number of interrupts
 * occur
 */
static void check_update_interrupt_seconds(void)
{
    set_time(59, 59, 23, 1, 29, 2, 16);

    /* Enable the update interrupt for per second updates */
    clear_bits_in_register(EXTENSION_REGISTER, EXT_MASK_USEL);
    set_bits_in_register(CONTROL_REGISTER, CTRL_MASK_UIE);

    count_reset();
    wait_for(5 * 1000000);

    g_assert_cmpuint(interrupt_counts[FLAG_REG_UF], ==, 5);
}


int main(int argc, char **argv)
{
    QTestState *s = NULL;
    int ret;
    char args[255];
    snprintf(args, sizeof(args), "-display none -machine imx25-pdk "
            "-device rx8900,bus=i2c.0,address=0x%x,id=%s",
            RX8900_ADDR, RX8900_TEST_ID);

    g_test_init(&argc, &argv, NULL);

    s = qtest_start(args);
    i2c = imx_i2c_create(IMX25_I2C_0_BASE);
    addr = RX8900_ADDR;

    qtest_irq_intercept_in(global_qtest, RX8900_TEST_ID);

#if 000
    qtest_add_func("/rx8900/tx-rx", send_and_receive);
    qtest_add_func("/rx8900/temperature", check_temperature);
    qtest_add_func("/rx8900/rollover", check_rollover);
    qtest_add_func("/rx8900/update-interrupt-disabled",
            check_update_interrupt_disabled);
#endif
    qtest_add_func("/rx8900/update-interrupt-seconds",
            check_update_interrupt_seconds);

    ret = g_test_run();

    if (s) {
        qtest_quit(s);
    }
    g_free(i2c);

    return ret;
}
