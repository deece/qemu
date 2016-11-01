/*
 * QTest testcase for the DS1338 RTC
 *
 * Copyright (c) 2013 Jean-Christophe Dubois
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/timer/rx8900_regs.h"
#include "libqtest.h"
#include "libqos/i2c.h"

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

static void send_and_receive(void)
{
    uint8_t rx8900_register;
    uint8_t buf[7];
    time_t now = time(NULL);
    struct tm *tm_ptr;

    /* reset the index in the RTC memory */
    rx8900_register = SECONDS;
    i2c_send(i2c, addr, &rx8900_register, 1);

    tm_ptr = gmtime(&now);
    /* retrieve the date */
    i2c_recv(i2c, addr, buf, 7);

    /* check retrieved time against local time */
    g_assert_cmpuint(bcd2bin(buf[0]), == , tm_ptr->tm_sec);
    g_assert_cmpuint(bcd2bin(buf[1]), == , tm_ptr->tm_min);
    g_assert_cmpuint(bcd2bin(buf[2]), == , tm_ptr->tm_hour);
    g_assert_cmpuint(bcd2bin(buf[4]), == , tm_ptr->tm_mday);
    g_assert_cmpuint(bcd2bin(buf[5]), == , 1 + tm_ptr->tm_mon);
    g_assert_cmpuint(2000 + bcd2bin(buf[6]), == , 1900 + tm_ptr->tm_year);
}

static void check_temperature(void)
{
   /* Check the initial temperature is 25C */
    uint8_t buf[1];
    uint8_t rx8900_register = TEMPERATURE;

    i2c_send(i2c, addr, &rx8900_register, 1);
    i2c_recv(i2c, addr, buf, 1);
    g_assert_cmpuint(buf[0], == , 135);

    /* Set the temperature to 40C and check the temperature again */
    qmp_rx8900_set_temperature(RX8900_TEST_ID, 40.0f);
    i2c_send(i2c, addr, &rx8900_register, 1);
    i2c_recv(i2c, addr, buf, 1);
    g_assert_cmpuint(buf[0], == , 157);
}

static void check_rollover(void)
{
    uint8_t buf[8];

    /* Write to registers starting at 0x00 */
    buf[0] = 0x00;

    /* Set the time to 23:59:59 29/2/2016 */
    buf[1] = bin2bcd(59); /* secs */
    buf[2] = bin2bcd(59); /* mins */
    buf[3] = bin2bcd(23); /* hours */
    buf[4] = 0x02; /* Monday */
    buf[5] = bin2bcd(29); /* day */
    buf[6] = bin2bcd(2); /* month */
    buf[7] = bin2bcd(16); /* year */

    i2c_send(i2c, addr, buf, 8);

    /* Wait for the clock to rollover */
    sleep(2);

    memset(buf, 0, sizeof(buf));

    /* Check that the clock rolled over */
    /* Read from registers starting at 0x00 */
    buf[0] = 0x00;

    i2c_send(i2c, addr, buf, 1);
    i2c_recv(i2c, addr, buf + 1, 7);

    /* Ignore seconds as there may be some noise,
     * we expect 00:00:xx Tuesday 1/3/2016 */
    g_assert_cmpuint(bcd2bin(buf[2]), == , 0);
    g_assert_cmpuint(bcd2bin(buf[3]), == , 0);
    g_assert_cmpuint(bcd2bin(buf[4]), == , 0x04);
    g_assert_cmpuint(bcd2bin(buf[5]), == , 1);
    g_assert_cmpuint(bcd2bin(buf[6]), == , 3);
    g_assert_cmpuint(bcd2bin(buf[7]), == , 16);
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

    qtest_add_func("/rx8900/tx-rx", send_and_receive);
    qtest_add_func("/rx8900/temperature", check_temperature);
    qtest_add_func("/rx8900/rollover", check_rollover);

    ret = g_test_run();

    if (s) {
        qtest_quit(s);
    }
    g_free(i2c);

    return ret;
}
