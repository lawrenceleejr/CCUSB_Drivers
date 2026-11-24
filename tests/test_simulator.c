/* End-to-end tests against the built-in simulator. */
#include "ccusb/ccusb.h"

#include <stdio.h>
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

typedef struct stats {
    int events, bad;
} stats;

static int on_event(const ccusb_event *ev, void *user)
{
    stats *s = (stats *)user;
    s->events++;
    /* stack: 8 x 16-bit ADC reads + marker */
    if (ev->nwords != 9 || ev->data[8] != 0xEEEE) s->bad++;
    return 0;
}

int main(void)
{
    ccusb_device *dev;
    uint32_t d, a1, a2;
    int q, x, on, i;

    CHECK(ccusb_open(&dev, "SIM") == CCUSB_OK);
    if (!dev) return 1;
    CHECK(ccusb_is_simulator(dev));

    CHECK(ccusb_firmware(dev, &d) == CCUSB_OK && d == 0x8E000601u);

    /* memory module in N3 */
    CHECK(ccusb_write(dev, 3, 4, 16, 0xABCDEF, &q, &x) == CCUSB_OK);
    CHECK(q == 1 && x == 1);
    CHECK(ccusb_read(dev, 3, 4, 0, &d, &q, &x) == CCUSB_OK);
    CHECK(d == 0xABCDEF && q == 1 && x == 1);
    d = 0x42;
    CHECK(ccusb_naf(dev, 3, 5, 16, &d, &q, &x) == CCUSB_OK);
    CHECK(ccusb_naf(dev, 3, 5, 0, &d, &q, &x) == CCUSB_OK && d == 0x42);

    /* empty station: no X, no Q */
    CHECK(ccusb_read(dev, 10, 0, 0, &d, &q, &x) == CCUSB_OK);
    CHECK(q == 0 && x == 0);

    /* C clears module data */
    CHECK(ccusb_c(dev) == CCUSB_OK);
    CHECK(ccusb_read(dev, 3, 4, 0, &d, &q, &x) == CCUSB_OK && d == 0);

    /* inhibit */
    CHECK(ccusb_inhibit(dev, 1) == CCUSB_OK);
    CHECK(ccusb_is_inhibited(dev, &on) == CCUSB_OK && on == 1);
    CHECK(ccusb_inhibit(dev, 0) == CCUSB_OK);
    CHECK(ccusb_is_inhibited(dev, &on) == CCUSB_OK && on == 0);

    /* registers round trip */
    CHECK(ccusb_reg_write(dev, CCUSB_REG_DGG_A, 0x00640010) == CCUSB_OK);
    CHECK(ccusb_reg_read(dev, CCUSB_REG_DGG_A, &d) == CCUSB_OK &&
          d == 0x00640010);

    /* scaler module in N5 counts */
    CHECK(ccusb_control(dev, 5, 0, 9, &q, &x) == CCUSB_OK && q == 1);
    CHECK(ccusb_read(dev, 5, 0, 0, &a1, &q, &x) == CCUSB_OK);
    for (i = 0; i < 100; i++) ccusb_read(dev, 3, 0, 0, &d, &q, &x);
    {
        /* wait ~50 ms so the 1 kHz channel advances */
        uint32_t t;
        do {
            ccusb_read(dev, 5, 0, 0, &a2, &q, &x);
            t = a2;
        } while (t <= a1 + 10 && ++i < 100000);
    }
    CHECK(a2 > a1);

    /* LAMs: ADC in N7 after a software conversion */
    CHECK(ccusb_control(dev, 7, 0, 25, &q, &x) == CCUSB_OK);
    CHECK(ccusb_read_lams(dev, &d) == CCUSB_OK && (d & (1u << 6)));
    CHECK(ccusb_read(dev, 7, 2, 0, &d, &q, &x) == CCUSB_OK && q == 1);
    CHECK(d < 4096);

    /* immediate stack with a Q-stop read */
    {
        ccusb_stack s;
        uint16_t reply[64];
        size_t n;
        ccusb_stack_init(&s);
        ccusb_stack_add_qstop(&s, 7, 0, 0, 20, 0, 0); /* Q while valid */
        ccusb_stack_add_marker(&s, 0x1234);
        CHECK(ccusb_stack_execute(dev, &s, reply, 64, &n) == CCUSB_OK);
        CHECK(n == 21 && reply[20] == 0x1234);
        ccusb_stack_free(&s);
    }

    /* DAQ mode */
    {
        ccusb_stack s;
        uint16_t buf[8192], back[1024];
        size_t n;
        stats st = {0, 0};
        int buffers = 0, last_seen = 0, k;
        ccusb_stack_init(&s);
        for (k = 0; k < 7; k++) ccusb_stack_add_read(&s, 7, k, 0, 0, 0);
        ccusb_stack_add_read(&s, 7, 7, 2, 0, 0);
        ccusb_stack_add_marker(&s, 0xEEEE);
        CHECK(ccusb_stack_load(dev, CCUSB_STACK_DATA, s.words, s.count) ==
              CCUSB_OK);
        CHECK(ccusb_stack_read(dev, CCUSB_STACK_DATA, back, 1024, &n) ==
              CCUSB_OK);
        CHECK(n == s.count + 1 && back[0] == s.count &&
              memcmp(back + 1, s.words, s.count * 2) == 0);
        CHECK(ccusb_reg_write(dev, CCUSB_REG_GLOBAL_MODE,
                              CCUSB_GMODE_BUFLEN_512) == CCUSB_OK);
        CHECK(ccusb_daq_start(dev) == CCUSB_OK);
        for (k = 0; k < 6; k++) {
            if (ccusb_read_buffer(dev, buf, 8192, &n, 500) == CCUSB_OK) {
                buffers++;
                CHECK(n <= 512);
                CHECK(buf[n - 1] == 0xFFFF);
                CHECK(ccusb_parse_buffer(buf, n, 0, on_event, &st) > 0);
            }
        }
        CHECK(ccusb_daq_stop(dev) == CCUSB_OK);
        while (ccusb_read_buffer(dev, buf, 8192, &n, 200) == CCUSB_OK) {
            if (buf[0] & CCUSB_BUF_LAST) last_seen = 1;
            ccusb_parse_buffer(buf, n, 0, on_event, &st);
        }
        CHECK(buffers >= 3);
        CHECK(st.events > 20);
        CHECK(st.bad == 0);
        CHECK(last_seen);
        printf("daq: %d buffers, %d events\n", buffers, st.events);
        ccusb_stack_free(&s);
    }

    ccusb_close(dev);
    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("simulator: all checks passed\n");
    return 0;
}
