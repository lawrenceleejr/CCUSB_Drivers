/*
 * Byte-exact checks of the USB packets the library sends, using a mock
 * transport.  Expected encodings follow W-IE-NE-R's libxxusb and the
 * CC-USB manual.
 */
#include "ccusb/ccusb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,         \
                    __LINE__, #cond);                                      \
            failures++;                                                    \
        }                                                                  \
    } while (0)

typedef struct mock {
    uint8_t out[4096];
    size_t  out_len;
    uint8_t reply[256];
    size_t  reply_len;
    int     reads;
} mock;

static int m_write(void *ctx, const uint8_t *buf, size_t len, size_t *sent,
                   unsigned int t)
{
    mock *m = (mock *)ctx;
    (void)t;
    memcpy(m->out, buf, len);
    m->out_len = len;
    *sent = len;
    return CCUSB_OK;
}

static int m_read(void *ctx, uint8_t *buf, size_t len, size_t *got,
                  unsigned int t)
{
    mock *m = (mock *)ctx;
    (void)t;
    (void)len;
    m->reads++;
    if (!m->reply_len) {
        *got = 0;
        return CCUSB_ERR_TIMEOUT;
    }
    memcpy(buf, m->reply, m->reply_len);
    *got = m->reply_len;
    return CCUSB_OK;
}

static void set_reply(mock *m, const uint16_t *w, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) {
        m->reply[2 * i] = (uint8_t)(w[i] & 0xFF);
        m->reply[2 * i + 1] = (uint8_t)(w[i] >> 8);
    }
    m->reply_len = n * 2;
}

static int out_is(const mock *m, const uint16_t *w, size_t n)
{
    size_t i;
    if (m->out_len != n * 2) return 0;
    for (i = 0; i < n; i++)
        if ((m->out[2 * i] | (m->out[2 * i + 1] << 8)) != w[i]) return 0;
    return 1;
}

static int count_events(const ccusb_event *ev, void *user)
{
    int *n = (int *)user;
    (*n)++;
    (void)ev;
    return 0;
}

int main(void)
{
    static const ccusb_transport ops = {m_write, m_read, NULL};
    mock m;
    ccusb_device *dev;
    uint32_t d;
    int q, x, rc;

    memset(&m, 0, sizeof(m));
    CHECK(ccusb_open_transport(&dev, &ops, &m, "MOCK") == CCUSB_OK);
    CHECK(strcmp(ccusb_serial(dev), "MOCK") == 0);

    /* NAF encoding identical to libxxusb: F + A*32 + N*512 + 0x4000 */
    CHECK(ccusb_naf_word(5, 2, 0, 1) == 0 + 2 * 32 + 5 * 512 + 0x4000);
    CHECK(ccusb_naf_word(28, 8, 29, 1) == 0x791D);

    /* 24-bit read: [000C][0001][NAF] ; reply lo, hi|Q<<8|X<<9 */
    {
        uint16_t exp[] = {0x000C, 0x0001, 0x4A40};
        uint16_t rep[] = {0x1234, 0x0356};
        set_reply(&m, rep, 2);
        rc = ccusb_read(dev, 5, 2, 0, &d, &q, &x);
        CHECK(rc == CCUSB_OK);
        CHECK(out_is(&m, exp, 3));
        CHECK(d == 0x561234);
        CHECK(q == 1 && x == 1);
        rep[1] = 0x0200; /* X only */
        set_reply(&m, rep, 2);
        ccusb_read(dev, 5, 2, 0, &d, &q, &x);
        CHECK(q == 0 && x == 1);
    }

    /* 24-bit write: [000C][0003][NAF][lo][hi] ; reply Q|X<<1 */
    {
        uint16_t exp[] = {0x000C, 0x0003, 0x4630, 0xCDEF, 0x00AB};
        uint16_t rep[] = {0x0003};
        set_reply(&m, rep, 1);
        rc = ccusb_write(dev, 3, 1, 16, 0xABCDEF, &q, &x);
        CHECK(rc == CCUSB_OK);
        CHECK(out_is(&m, exp, 5));
        CHECK(q == 1 && x == 1);
        rep[0] = 0x0002;
        set_reply(&m, rep, 1);
        ccusb_write(dev, 3, 1, 16, 0, &q, &x);
        CHECK(q == 0 && x == 1);
    }

    /* invalid function classes are rejected without touching the bus */
    m.out_len = 0;
    CHECK(ccusb_write(dev, 3, 1, 0, 1, &q, &x) == CCUSB_ERR_INVALID_PARAM);
    CHECK(ccusb_read(dev, 3, 1, 16, &d, &q, &x) == CCUSB_ERR_INVALID_PARAM);
    CHECK(ccusb_read(dev, 32, 0, 0, &d, &q, &x) == CCUSB_ERR_INVALID_PARAM);
    CHECK(ccusb_read(dev, 1, 16, 0, &d, &q, &x) == CCUSB_ERR_INVALID_PARAM);
    CHECK(m.out_len == 0);

    /* Z, C, I as in libxxusb */
    {
        uint16_t rep[] = {0x0003};
        uint16_t z[] = {0x000C, 1, 29 + 8 * 32 + 28 * 512 + 0x4000};
        uint16_t c[] = {0x000C, 1, 29 + 9 * 32 + 28 * 512 + 0x4000};
        uint16_t i1[] = {0x000C, 1, 24 + 9 * 32 + 29 * 512 + 0x4000};
        uint16_t i0[] = {0x000C, 1, 26 + 9 * 32 + 29 * 512 + 0x4000};
        set_reply(&m, rep, 1);
        CHECK(ccusb_z(dev) == CCUSB_OK && out_is(&m, z, 3));
        CHECK(ccusb_c(dev) == CCUSB_OK && out_is(&m, c, 3));
        CHECK(ccusb_inhibit(dev, 1) == CCUSB_OK && out_is(&m, i1, 3));
        CHECK(ccusb_inhibit(dev, 0) == CCUSB_OK && out_is(&m, i0, 3));
    }

    /* registers: N25, full 32-bit data in two words */
    {
        uint16_t exp_w[] = {0x000C, 3, 0x4000 | 25 << 9 | 7 << 5 | 16, 0x5678,
                            0x1234};
        uint16_t rep[] = {0xBEEF, 0xDEAD};
        set_reply(&m, rep, 1);
        CHECK(ccusb_reg_write(dev, CCUSB_REG_DGG_A, 0x12345678) == CCUSB_OK);
        CHECK(out_is(&m, exp_w, 5));
        set_reply(&m, rep, 2);
        CHECK(ccusb_reg_read(dev, CCUSB_REG_FIRMWARE, &d) == CCUSB_OK);
        CHECK(d == 0xDEADBEEF);
        CHECK(ccusb_reg_write(dev, CCUSB_REG_FIRMWARE, 1) ==
              CCUSB_ERR_INVALID_PARAM);
        rep[1] = 0x0100; /* bit 24 = inhibit */
        set_reply(&m, rep, 2);
        CHECK(ccusb_is_inhibited(dev, &q) == CCUSB_OK && q == 1);
    }

    /* action register: [0005][000A][value], no reply expected */
    {
        uint16_t exp[] = {0x0005, 0x000A, 0x0001};
        int reads = m.reads;
        CHECK(ccusb_daq_start(dev) == CCUSB_OK);
        CHECK(out_is(&m, exp, 3));
        CHECK(m.reads == reads);
        CHECK(ccusb_daq_running(dev));
        /* immediate operations are refused while DAQ runs */
        CHECK(ccusb_read(dev, 1, 0, 0, &d, &q, &x) == CCUSB_ERR_BUSY);
        exp[2] = 0;
        CHECK(ccusb_daq_stop(dev) == CCUSB_OK);
        CHECK(out_is(&m, exp, 3));
        CHECK(!ccusb_daq_running(dev));
    }

    /* stacks */
    {
        ccusb_stack s;
        uint16_t exp_data[] = {0x0006, 6,
                               (uint16_t)(7 << 9 | 0 << 5 | 0),
                               (uint16_t)(0x8000 | 0x4000 | 7 << 9 | 2),
                               0x8000 | 0x0010, 8,
                               (uint16_t)(0 << 9 | 16), 0xCAFE};
        uint16_t exp_scaler[] = {0x0007, 1, (uint16_t)(5 << 9 | 9)};
        uint16_t exp_read[] = {0x0002};
        uint16_t rep[] = {2, 0x0E00, 0x8E02};
        uint16_t got[16];
        size_t n;
        ccusb_stack_init(&s);
        CHECK(ccusb_stack_add_read(&s, 7, 0, 0, 0, 0) == CCUSB_OK);
        CHECK(ccusb_stack_add_qstop(&s, 7, 0, 2, 8, 1, 0) == CCUSB_OK);
        CHECK(ccusb_stack_add_marker(&s, 0xCAFE) == CCUSB_OK);
        CHECK(s.count == 6);
        CHECK(ccusb_stack_load(dev, CCUSB_STACK_DATA, s.words, s.count) ==
              CCUSB_OK);
        CHECK(out_is(&m, exp_data, 8));
        ccusb_stack_clear(&s);
        CHECK(ccusb_stack_add_control(&s, 5, 0, 9) == CCUSB_OK);
        CHECK(ccusb_stack_load(dev, CCUSB_STACK_SCALER, s.words, s.count) ==
              CCUSB_OK);
        CHECK(out_is(&m, exp_scaler, 3));
        /* bad entries are remembered */
        CHECK(ccusb_stack_add_write(&s, 5, 0, 0, 1, 0) ==
              CCUSB_ERR_INVALID_PARAM);
        CHECK(s.error == CCUSB_ERR_INVALID_PARAM);
        CHECK(ccusb_stack_execute(dev, &s, got, 16, &n) ==
              CCUSB_ERR_INVALID_PARAM);
        ccusb_stack_free(&s);

        set_reply(&m, rep, 3);
        CHECK(ccusb_stack_read(dev, CCUSB_STACK_DATA, got, 16, &n) == CCUSB_OK);
        CHECK(out_is(&m, exp_read, 1));
        CHECK(n == 3 && got[0] == 2);

        {
            uint16_t too_long[CCUSB_SCALER_STACK_MAX + 1];
            memset(too_long, 0, sizeof(too_long));
            CHECK(ccusb_stack_load(dev, CCUSB_STACK_SCALER, too_long,
                                   CCUSB_SCALER_STACK_MAX + 1) ==
                  CCUSB_ERR_INVALID_PARAM);
        }
    }

    /* reply larger than caller's buffer */
    {
        uint16_t rep[8] = {0};
        uint16_t naf = ccusb_naf_word(1, 0, 0, 1), small[2];
        size_t n;
        set_reply(&m, rep, 8);
        CHECK(ccusb_execute(dev, &naf, 1, small, 2, &n) == CCUSB_ERR_OVERFLOW);
    }

    /* timeouts propagate */
    m.reply_len = 0;
    CHECK(ccusb_read(dev, 1, 0, 0, &d, &q, &x) == CCUSB_ERR_TIMEOUT);

    /* buffer decoding */
    {
        /* 2 events + terminator: [hdr=2][len=2][a][b][len=1|stack1][c][FFFF] */
        uint16_t buf[] = {0x8002, 2, 0x11, 0x22, 0x2001, 0x33, 0xFFFF};
        uint16_t buf2[] = {0x0001, 9, 0x11}; /* truncated */
        int n = 0;
        CHECK(ccusb_parse_buffer(buf, 7, 0, count_events, &n) == 2);
        CHECK(n == 2);
        CHECK(ccusb_parse_buffer(buf2, 3, 0, NULL, NULL) == CCUSB_ERR_PROTOCOL);
    }

    CHECK(strcmp(ccusb_reg_name(CCUSB_REG_DGG_EXT), "dgg_ext") == 0);
    CHECK(ccusb_strerror(CCUSB_ERR_ACCESS) != NULL);

    ccusb_close(dev);
    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("protocol: all checks passed\n");
    return 0;
}
