/*
 * ccusb.c - CC-USB protocol layer (transport independent).
 *
 * Packet formats (all 16-bit words little endian):
 *   execute immediately : [0x000C][n][w1 .. wn]      -> reply words
 *   load data stack     : [0x0006][n][w1 .. wn]      (no reply)
 *   load scaler stack   : [0x0007][n][w1 .. wn]      (no reply)
 *   read stack          : [0x0002 | id]              -> reply words
 *   action register     : [0x0005][0x000A][value]    (no reply)
 *
 * A stack NAF word is  F | A<<5 | N<<9 | LONG(0x4000) | CONT(0x8000).
 * Reply of a 24-bit read: [d15..d0][X<<9 | Q<<8 | d23..d16].
 * Reply of a write/control: [X<<1 | Q].
 */
#include "ccusb_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STR2(x) #x
#define STR(x) STR2(x)

const char *ccusb_version(void)
{
    return STR(CCUSB_VERSION_MAJOR) "." STR(CCUSB_VERSION_MINOR) "." STR(
        CCUSB_VERSION_PATCH);
}

const char *ccusb_strerror(int err)
{
    switch (err) {
    case CCUSB_OK: return "success";
    case CCUSB_ERR_IO: return "USB input/output error";
    case CCUSB_ERR_INVALID_PARAM: return "invalid parameter";
    case CCUSB_ERR_ACCESS:
        return "access denied (on Linux install the udev rule; on Windows "
               "install the WinUSB driver with Zadig)";
    case CCUSB_ERR_NO_DEVICE: return "device disconnected";
    case CCUSB_ERR_NOT_FOUND: return "no CC-USB found";
    case CCUSB_ERR_BUSY:
        return "device busy (claimed by another program, or DAQ mode is "
               "running)";
    case CCUSB_ERR_TIMEOUT: return "timeout";
    case CCUSB_ERR_OVERFLOW: return "reply larger than buffer";
    case CCUSB_ERR_PIPE: return "USB endpoint stalled";
    case CCUSB_ERR_NO_MEM: return "out of memory";
    case CCUSB_ERR_PROTOCOL: return "unexpected reply from CC-USB";
    case CCUSB_ERR_NOT_SUPPORTED: return "operation not supported";
    default: return "unknown error";
    }
}

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

static int valid_naf(int n, int a, int f)
{
    return n >= 0 && n <= 31 && a >= 0 && a <= 15 && f >= 0 && f <= 31;
}

static int is_write_f(int f) { return f >= 16 && f <= 23; }
static int is_control_f(int f) { return (f >= 8 && f <= 15) || f >= 24; }

uint16_t ccusb_naf_word(int n, int a, int f, int long_mode)
{
    uint16_t w = (uint16_t)(((n & 31) << 9) | ((a & 15) << 5) | (f & 31));
    if (long_mode) w |= CCUSB_NAF_LONG;
    return w;
}

static void trace_dump(const ccusb_device *dev, const char *dir,
                       const uint8_t *buf, size_t len)
{
    size_t i;
    if (!dev->trace) return;
    fprintf(stderr, "ccusb %s %3u bytes:", dir, (unsigned)len);
    for (i = 0; i + 1 < len && i < 64; i += 2)
        fprintf(stderr, " %04x", buf[i] | (buf[i + 1] << 8));
    if (len > 64) fprintf(stderr, " ...");
    fprintf(stderr, "\n");
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

int ccusb_bulk_write(ccusb_device *dev, const void *buf, size_t len,
                     size_t *sent, unsigned int timeout_ms)
{
    size_t s = 0;
    int rc;
    if (!dev || (!buf && len)) return CCUSB_ERR_INVALID_PARAM;
    trace_dump(dev, "OUT", (const uint8_t *)buf, len);
    rc = dev->ops.write(dev->ctx, (const uint8_t *)buf, len, &s, timeout_ms);
    if (sent) *sent = s;
    if (rc == CCUSB_OK && s != len) rc = CCUSB_ERR_IO;
    return rc;
}

int ccusb_bulk_read(ccusb_device *dev, void *buf, size_t len, size_t *got,
                    unsigned int timeout_ms)
{
    size_t g = 0;
    int rc;
    if (!dev || !buf) return CCUSB_ERR_INVALID_PARAM;
    rc = dev->ops.read(dev->ctx, (uint8_t *)buf, len, &g, timeout_ms);
    /* A timeout after some data arrived means the reply simply ended on a
     * packet boundary without a zero-length packet: treat as success. */
    if (rc == CCUSB_ERR_TIMEOUT && g > 0) rc = CCUSB_OK;
    if (got) *got = g;
    if (g) trace_dump(dev, "IN ", (const uint8_t *)buf, g);
    return rc;
}

/* Send an OUT packet made of 16-bit words. */
static int send_words(ccusb_device *dev, const uint16_t *w, size_t n)
{
    uint8_t stackbuf[64];
    uint8_t *pkt = stackbuf;
    size_t i;
    int rc;
    if (n * 2 > sizeof(stackbuf)) {
        pkt = (uint8_t *)malloc(n * 2);
        if (!pkt) return CCUSB_ERR_NO_MEM;
    }
    for (i = 0; i < n; i++) put16(pkt + 2 * i, w[i]);
    rc = ccusb_bulk_write(dev, pkt, n * 2, NULL, dev->timeout_ms);
    if (pkt != stackbuf) free(pkt);
    return rc;
}

/* Read a command reply into words. */
static int recv_words(ccusb_device *dev, uint16_t *w, size_t max,
                      size_t *nwords)
{
    size_t got = 0, i;
    int rc = ccusb_bulk_read(dev, dev->rx, CCUSB_RX_BUFFER_BYTES, &got,
                             dev->timeout_ms);
    if (nwords) *nwords = 0;
    if (rc != CCUSB_OK) return rc;
    if (got / 2 > max) return CCUSB_ERR_OVERFLOW;
    for (i = 0; i < got / 2; i++) w[i] = get16(dev->rx + 2 * i);
    if (nwords) *nwords = got / 2;
    return CCUSB_OK;
}

static int check_idle(const ccusb_device *dev)
{
    if (!dev) return CCUSB_ERR_INVALID_PARAM;
    if (dev->daq_running) return CCUSB_ERR_BUSY;
    return CCUSB_OK;
}

/* Packet [TA][n][words...] */
static int send_list(ccusb_device *dev, uint16_t ta, const uint16_t *words,
                     size_t nwords)
{
    uint16_t small[32];
    uint16_t *pkt = small;
    int rc;
    if (nwords > 0xFFF) return CCUSB_ERR_INVALID_PARAM;
    if (nwords + 2 > sizeof(small) / sizeof(small[0])) {
        pkt = (uint16_t *)malloc((nwords + 2) * sizeof(uint16_t));
        if (!pkt) return CCUSB_ERR_NO_MEM;
    }
    pkt[0] = ta;
    pkt[1] = (uint16_t)nwords;
    if (nwords) memcpy(pkt + 2, words, nwords * sizeof(uint16_t));
    rc = send_words(dev, pkt, nwords + 2);
    if (pkt != small) free(pkt);
    return rc;
}

int ccusb_execute(ccusb_device *dev, const uint16_t *words, size_t nwords,
                  uint16_t *reply, size_t reply_max, size_t *reply_words)
{
    int rc = check_idle(dev);
    if (reply_words) *reply_words = 0;
    if (rc) return rc;
    if (!words || nwords == 0 || (!reply && reply_max))
        return CCUSB_ERR_INVALID_PARAM;
    rc = send_list(dev, TA_EXECUTE, words, nwords);
    if (rc) return rc;
    if (!reply) { /* caller does not want the reply: read and discard it */
        size_t got = 0;
        rc = ccusb_bulk_read(dev, dev->rx, CCUSB_RX_BUFFER_BYTES, &got,
                             dev->timeout_ms);
        if (reply_words) *reply_words = got / 2;
        return rc;
    }
    return recv_words(dev, reply, reply_max, reply_words);
}

/* ------------------------------------------------------------------ */
/* single CAMAC cycles                                                 */
/* ------------------------------------------------------------------ */

static void qx_from_status(uint16_t w, int *q, int *x)
{
    if (q) *q = w & 1;
    if (x) *x = (w >> 1) & 1;
}

static void qx_from_hi(uint16_t hi, int *q, int *x)
{
    if (q) *q = (hi >> 8) & 1;
    if (x) *x = (hi >> 9) & 1;
}

int ccusb_read(ccusb_device *dev, int n, int a, int f, uint32_t *data,
               int *q, int *x)
{
    uint16_t naf, r[4];
    size_t nr;
    int rc;
    if (!valid_naf(n, a, f) || is_write_f(f)) return CCUSB_ERR_INVALID_PARAM;
    naf = ccusb_naf_word(n, a, f, 1);
    rc = ccusb_execute(dev, &naf, 1, r, 4, &nr);
    if (rc) return rc;
    if (nr >= 2) {
        if (data) *data = r[0] | ((uint32_t)(r[1] & 0xFF) << 16);
        qx_from_hi(r[1], q, x);
    } else if (nr == 1) {
        if (data) *data = 0;
        qx_from_status(r[0], q, x);
    } else {
        return CCUSB_ERR_PROTOCOL;
    }
    return CCUSB_OK;
}

int ccusb_write(ccusb_device *dev, int n, int a, int f, uint32_t data,
                int *q, int *x)
{
    uint16_t w[3], r[4];
    size_t nr;
    int rc;
    if (!valid_naf(n, a, f) || !is_write_f(f)) return CCUSB_ERR_INVALID_PARAM;
    w[0] = ccusb_naf_word(n, a, f, 1);
    w[1] = (uint16_t)(data & 0xFFFF);
    w[2] = (uint16_t)((data >> 16) & 0xFF);
    rc = ccusb_execute(dev, w, 3, r, 4, &nr);
    if (rc) return rc;
    if (nr < 1) return CCUSB_ERR_PROTOCOL;
    qx_from_status(r[0], q, x);
    return CCUSB_OK;
}

int ccusb_control(ccusb_device *dev, int n, int a, int f, int *q, int *x)
{
    uint16_t naf, r[4];
    size_t nr;
    int rc;
    if (!valid_naf(n, a, f) || is_write_f(f)) return CCUSB_ERR_INVALID_PARAM;
    naf = ccusb_naf_word(n, a, f, 1);
    rc = ccusb_execute(dev, &naf, 1, r, 4, &nr);
    if (rc) return rc;
    if (nr == 1)
        qx_from_status(r[0], q, x);
    else if (nr >= 2)
        qx_from_hi(r[1], q, x);
    else
        return CCUSB_ERR_PROTOCOL;
    return CCUSB_OK;
}

int ccusb_naf(ccusb_device *dev, int n, int a, int f, uint32_t *data, int *q,
              int *x)
{
    if (!valid_naf(n, a, f)) return CCUSB_ERR_INVALID_PARAM;
    if (is_write_f(f))
        return ccusb_write(dev, n, a, f, data ? *data : 0, q, x);
    if (is_control_f(f)) {
        if (data) *data = 0;
        return ccusb_control(dev, n, a, f, q, x);
    }
    return ccusb_read(dev, n, a, f, data, q, x);
}

int ccusb_z(ccusb_device *dev) { return ccusb_control(dev, 28, 8, 29, NULL, NULL); }
int ccusb_c(ccusb_device *dev) { return ccusb_control(dev, 28, 9, 29, NULL, NULL); }

int ccusb_inhibit(ccusb_device *dev, int on)
{
    return ccusb_control(dev, 29, 9, on ? 24 : 26, NULL, NULL);
}

/* ------------------------------------------------------------------ */
/* registers                                                           */
/* ------------------------------------------------------------------ */

static const char *const reg_names[16] = {
    "firmware",   "global_mode", "delays",    "scaler_ctrl",
    "led_src",    "output_src",  "device_src","dgg_a",
    "dgg_b",      "lam_mask",    "lam",       "scaler_a",
    "scaler_b",   "dgg_ext",     "usb_setup", NULL};

const char *ccusb_reg_name(int reg)
{
    if (reg < 0 || reg > 15) return NULL;
    return reg_names[reg];
}

int ccusb_reg_read(ccusb_device *dev, int reg, uint32_t *value)
{
    uint16_t naf, r[4];
    size_t nr;
    int rc;
    if (reg < 0 || reg > 15 || !value) return CCUSB_ERR_INVALID_PARAM;
    naf = ccusb_naf_word(CCUSB_REGISTER_STATION, reg, 0, 1);
    rc = ccusb_execute(dev, &naf, 1, r, 4, &nr);
    if (rc) return rc;
    if (nr < 2) return CCUSB_ERR_PROTOCOL;
    *value = r[0] | ((uint32_t)r[1] << 16);
    return CCUSB_OK;
}

int ccusb_reg_write(ccusb_device *dev, int reg, uint32_t value)
{
    uint16_t w[3], r[4];
    size_t nr;
    if (reg < 0 || reg > 15) return CCUSB_ERR_INVALID_PARAM;
    if (reg == CCUSB_REG_FIRMWARE || reg == CCUSB_REG_LAM ||
        reg == CCUSB_REG_SCALER_A || reg == CCUSB_REG_SCALER_B)
        return CCUSB_ERR_INVALID_PARAM; /* read-only */
    w[0] = ccusb_naf_word(CCUSB_REGISTER_STATION, reg, 16, 1);
    w[1] = (uint16_t)(value & 0xFFFF);
    w[2] = (uint16_t)(value >> 16);
    return ccusb_execute(dev, w, 3, r, 4, &nr);
}

int ccusb_firmware(ccusb_device *dev, uint32_t *id)
{
    return ccusb_reg_read(dev, CCUSB_REG_FIRMWARE, id);
}

int ccusb_read_lams(ccusb_device *dev, uint32_t *lams)
{
    uint32_t v;
    int rc;
    if (!lams) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_reg_read(dev, CCUSB_REG_LAM, &v);
    if (rc == CCUSB_OK) *lams = v & 0xFFFFFF;
    return rc;
}

int ccusb_is_inhibited(ccusb_device *dev, int *on)
{
    uint32_t v;
    int rc;
    if (!on) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_reg_read(dev, CCUSB_REG_LAM, &v);
    if (rc == CCUSB_OK) *on = (int)((v >> 24) & 1);
    return rc;
}

int ccusb_read_scalers(ccusb_device *dev, uint32_t *a, uint32_t *b)
{
    uint32_t v;
    int rc;
    if (a) {
        rc = ccusb_reg_read(dev, CCUSB_REG_SCALER_A, &v);
        if (rc) return rc;
        *a = v & 0xFFFFFF;
    }
    if (b) {
        rc = ccusb_reg_read(dev, CCUSB_REG_SCALER_B, &v);
        if (rc) return rc;
        *b = v & 0xFFFFFF;
    }
    return CCUSB_OK;
}

int ccusb_write_action(ccusb_device *dev, uint16_t value)
{
    uint16_t pkt[3];
    if (!dev) return CCUSB_ERR_INVALID_PARAM;
    pkt[0] = TA_REGISTER;
    pkt[1] = ACTION_REG_ADDR;
    pkt[2] = value;
    return send_words(dev, pkt, 3);
}

/* ------------------------------------------------------------------ */
/* stacks                                                              */
/* ------------------------------------------------------------------ */

void ccusb_stack_init(ccusb_stack *s)
{
    if (!s) return;
    s->words = NULL;
    s->count = s->capacity = 0;
    s->error = 0;
}

void ccusb_stack_free(ccusb_stack *s)
{
    if (!s) return;
    free(s->words);
    ccusb_stack_init(s);
}

void ccusb_stack_clear(ccusb_stack *s)
{
    if (!s) return;
    s->count = 0;
    s->error = 0;
}

static int stack_fail(ccusb_stack *s, int err)
{
    if (s && !s->error) s->error = err;
    return err;
}

int ccusb_stack_add_word(ccusb_stack *s, uint16_t w)
{
    if (!s) return CCUSB_ERR_INVALID_PARAM;
    if (s->count == s->capacity) {
        size_t cap = s->capacity ? s->capacity * 2 : 64;
        uint16_t *nw = (uint16_t *)realloc(s->words, cap * sizeof(uint16_t));
        if (!nw) return stack_fail(s, CCUSB_ERR_NO_MEM);
        s->words = nw;
        s->capacity = cap;
    }
    s->words[s->count++] = w;
    return CCUSB_OK;
}

static int add_words(ccusb_stack *s, const uint16_t *w, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) {
        int rc = ccusb_stack_add_word(s, w[i]);
        if (rc) return rc;
    }
    return CCUSB_OK;
}

int ccusb_stack_add_read(ccusb_stack *s, int n, int a, int f, int long_mode,
                         int lam_wait)
{
    uint16_t w[2];
    if (!valid_naf(n, a, f) || is_write_f(f))
        return stack_fail(s, CCUSB_ERR_INVALID_PARAM);
    w[0] = ccusb_naf_word(n, a, f, long_mode);
    if (lam_wait) {
        w[0] |= CCUSB_NAF_CONTINUATION;
        w[1] = CCUSB_MODE_LAMWAIT;
        return add_words(s, w, 2);
    }
    return add_words(s, w, 1);
}

int ccusb_stack_add_write(ccusb_stack *s, int n, int a, int f, uint32_t data,
                          int long_mode)
{
    uint16_t w[3];
    if (!valid_naf(n, a, f) || !is_write_f(f))
        return stack_fail(s, CCUSB_ERR_INVALID_PARAM);
    w[0] = ccusb_naf_word(n, a, f, long_mode);
    w[1] = (uint16_t)(data & 0xFFFF);
    if (long_mode) {
        w[2] = (uint16_t)((data >> 16) & (n == CCUSB_REGISTER_STATION ? 0xFFFF : 0xFF));
        return add_words(s, w, 3);
    }
    return add_words(s, w, 2);
}

int ccusb_stack_add_control(ccusb_stack *s, int n, int a, int f)
{
    if (!valid_naf(n, a, f) || !is_control_f(f))
        return stack_fail(s, CCUSB_ERR_INVALID_PARAM);
    return ccusb_stack_add_word(s, ccusb_naf_word(n, a, f, 0));
}

static int add_qop(ccusb_stack *s, uint16_t mode, int n, int a, int f,
                   uint16_t max, int long_mode, int lam_wait)
{
    uint16_t w[3];
    if (!valid_naf(n, a, f) || is_write_f(f) || is_control_f(f))
        return stack_fail(s, CCUSB_ERR_INVALID_PARAM);
    w[0] = (uint16_t)(ccusb_naf_word(n, a, f, long_mode) | CCUSB_NAF_CONTINUATION);
    w[1] = (uint16_t)(mode | CCUSB_NAF_CONTINUATION | (lam_wait ? CCUSB_MODE_LAMWAIT : 0));
    w[2] = max;
    return add_words(s, w, 3);
}

int ccusb_stack_add_qstop(ccusb_stack *s, int n, int a, int f, uint16_t max,
                          int long_mode, int lam_wait)
{
    return add_qop(s, CCUSB_MODE_QSTOP, n, a, f, max, long_mode, lam_wait);
}

int ccusb_stack_add_qscan(ccusb_stack *s, int n, int a, int f, uint16_t max,
                          int long_mode, int lam_wait)
{
    return add_qop(s, CCUSB_MODE_QSCAN, n, a, f, max, long_mode, lam_wait);
}

int ccusb_stack_add_repeat(ccusb_stack *s, int n, int a, int f,
                           uint16_t count, int long_mode, int lam_wait)
{
    return add_qop(s, CCUSB_MODE_REPEAT, n, a, f, count, long_mode, lam_wait);
}

int ccusb_stack_add_marker(ccusb_stack *s, uint16_t value)
{
    uint16_t w[2];
    w[0] = ccusb_naf_word(0, 0, 16, 0);
    w[1] = value;
    return add_words(s, w, 2);
}

int ccusb_stack_execute(ccusb_device *dev, const ccusb_stack *s,
                        uint16_t *reply, size_t reply_max, size_t *reply_words)
{
    if (!s || s->error) return s ? s->error : CCUSB_ERR_INVALID_PARAM;
    return ccusb_execute(dev, s->words, s->count, reply, reply_max,
                         reply_words);
}

int ccusb_stack_load(ccusb_device *dev, int which, const uint16_t *words,
                     size_t nwords)
{
    size_t limit;
    uint16_t ta = TA_WRITE | TA_STACK_SEL;
    int rc = check_idle(dev);
    if (rc) return rc;
    if (which == CCUSB_STACK_DATA)
        limit = CCUSB_DATA_STACK_MAX;
    else if (which == CCUSB_STACK_SCALER) {
        limit = CCUSB_SCALER_STACK_MAX;
        ta |= TA_STACK_ID0;
    } else
        return CCUSB_ERR_INVALID_PARAM;
    if ((!words && nwords) || nwords > limit) return CCUSB_ERR_INVALID_PARAM;
    return send_list(dev, ta, words, nwords);
}

int ccusb_stack_read(ccusb_device *dev, int which, uint16_t *words, size_t max,
                     size_t *nwords)
{
    uint16_t ta = TA_STACK_SEL;
    int rc = check_idle(dev);
    if (nwords) *nwords = 0;
    if (rc) return rc;
    if (which == CCUSB_STACK_SCALER)
        ta |= TA_STACK_ID0;
    else if (which != CCUSB_STACK_DATA)
        return CCUSB_ERR_INVALID_PARAM;
    if (!words) return CCUSB_ERR_INVALID_PARAM;
    rc = send_words(dev, &ta, 1);
    if (rc) return rc;
    return recv_words(dev, words, max, nwords);
}

/* ------------------------------------------------------------------ */
/* data acquisition                                                    */
/* ------------------------------------------------------------------ */

int ccusb_daq_start(ccusb_device *dev)
{
    int rc = check_idle(dev);
    if (rc) return rc;
    rc = ccusb_write_action(dev, CCUSB_ACTION_START_DAQ);
    if (rc == CCUSB_OK) dev->daq_running = 1;
    return rc;
}

int ccusb_daq_stop(ccusb_device *dev)
{
    int rc;
    if (!dev) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_write_action(dev, 0);
    if (rc == CCUSB_OK) dev->daq_running = 0;
    return rc;
}

int ccusb_daq_running(const ccusb_device *dev)
{
    return dev ? dev->daq_running : 0;
}

int ccusb_read_buffer(ccusb_device *dev, uint16_t *buf, size_t max_words,
                      size_t *nwords, unsigned int timeout_ms)
{
    size_t got = 0, i;
    int rc;
    if (nwords) *nwords = 0;
    if (!dev || !buf) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_bulk_read(dev, dev->rx, CCUSB_RX_BUFFER_BYTES, &got, timeout_ms);
    if (rc) return rc;
    if (got / 2 > max_words) return CCUSB_ERR_OVERFLOW;
    for (i = 0; i < got / 2; i++) buf[i] = get16(dev->rx + 2 * i);
    if (nwords) *nwords = got / 2;
    return CCUSB_OK;
}

int ccusb_flush(ccusb_device *dev)
{
    int total = 0, i;
    if (!dev) return CCUSB_ERR_INVALID_PARAM;
    for (i = 0; i < 1000; i++) {
        size_t got = 0;
        int rc = dev->ops.read(dev->ctx, dev->rx, CCUSB_RX_BUFFER_BYTES, &got,
                               20);
        total += (int)got;
        if (got == 0 || (rc != CCUSB_OK && rc != CCUSB_ERR_TIMEOUT)) break;
    }
    return total;
}

int ccusb_parse_buffer(const uint16_t *buf, size_t nwords, int double_header,
                       ccusb_event_cb cb, void *user)
{
    size_t pos;
    int nevents, done = 0;
    if (!buf || nwords == 0) return CCUSB_ERR_INVALID_PARAM;
    nevents = buf[0] & CCUSB_BUF_NEVENTS;
    pos = double_header ? 2 : 1;
    while (done < nevents) {
        ccusb_event ev;
        uint16_t hdr;
        size_t len;
        if (pos >= nwords) return CCUSB_ERR_PROTOCOL;
        hdr = buf[pos];
        if (hdr == CCUSB_BUFFER_TERMINATOR) break;
        len = hdr & CCUSB_EVT_LENGTH;
        if (pos + 1 + len > nwords) return CCUSB_ERR_PROTOCOL;
        ev.data = buf + pos + 1;
        ev.nwords = len;
        ev.stack_id = (hdr & CCUSB_EVT_STACK_ID) ? 1 : 0;
        ev.continued = (hdr & CCUSB_EVT_CONTINUED) ? 1 : 0;
        done++;
        pos += 1 + len;
        if (cb && cb(&ev, user)) break;
    }
    return done;
}

/* ------------------------------------------------------------------ */
/* open / close                                                        */
/* ------------------------------------------------------------------ */

int ccusb_open_transport(ccusb_device **out, const ccusb_transport *ops,
                         void *ctx, const char *name)
{
    ccusb_device *dev;
    if (!out || !ops || !ops->write || !ops->read)
        return CCUSB_ERR_INVALID_PARAM;
    *out = NULL;
    dev = (ccusb_device *)calloc(1, sizeof(*dev));
    if (!dev) return CCUSB_ERR_NO_MEM;
    dev->rx = (uint8_t *)malloc(CCUSB_RX_BUFFER_BYTES);
    if (!dev->rx) {
        free(dev);
        return CCUSB_ERR_NO_MEM;
    }
    dev->ops = *ops;
    dev->ctx = ctx;
    dev->timeout_ms = 2000;
    if (name) {
        strncpy(dev->serial, name, sizeof(dev->serial) - 1);
        dev->serial[sizeof(dev->serial) - 1] = 0;
    }
    *out = dev;
    return CCUSB_OK;
}

/* Bring the controller into a known state: stop any DAQ run left behind
 * by a previous program, drop stale data and check that it answers. */
static int handshake(ccusb_device *dev)
{
    uint32_t fw;
    int rc = CCUSB_OK, attempt;
    ccusb_write_action(dev, 0);
    ccusb_flush(dev);
    for (attempt = 0; attempt < 3; attempt++) {
        rc = ccusb_firmware(dev, &fw);
        if (rc == CCUSB_OK) return CCUSB_OK;
        ccusb_flush(dev);
    }
    return rc;
}

int ccusb_open_simulator(ccusb_device **out)
{
    ccusb_transport ops;
    void *ctx = NULL;
    int rc;
    if (!out) return CCUSB_ERR_INVALID_PARAM;
    *out = NULL;
    rc = ccusb__sim_open(&ops, &ctx);
    if (rc) return rc;
    rc = ccusb_open_transport(out, &ops, ctx, "SIM");
    if (rc) {
        ops.close(ctx);
        return rc;
    }
    (*out)->is_sim = 1;
    return CCUSB_OK;
}

int ccusb_open(ccusb_device **out, const char *serial)
{
    ccusb_transport ops;
    void *ctx = NULL;
    char name[32] = {0};
    int rc;
    if (!out) return CCUSB_ERR_INVALID_PARAM;
    *out = NULL;
    if (serial && (strcmp(serial, "SIM") == 0 || strcmp(serial, "sim") == 0))
        return ccusb_open_simulator(out);
    rc = ccusb__usb_open(serial, &ops, &ctx, name, sizeof(name));
    if (rc) return rc;
    rc = ccusb_open_transport(out, &ops, ctx, name);
    if (rc) {
        ops.close(ctx);
        return rc;
    }
    rc = handshake(*out);
    if (rc) {
        ccusb_close(*out);
        *out = NULL;
    }
    return rc;
}

void ccusb_close(ccusb_device *dev)
{
    if (!dev) return;
    if (dev->daq_running) ccusb_daq_stop(dev);
    if (dev->ops.close) dev->ops.close(dev->ctx);
    free(dev->rx);
    free(dev);
}

const char *ccusb_serial(const ccusb_device *dev)
{
    return dev ? dev->serial : "";
}

int ccusb_is_simulator(const ccusb_device *dev)
{
    return dev ? dev->is_sim : 0;
}

void ccusb_set_timeout(ccusb_device *dev, unsigned int ms)
{
    if (dev) dev->timeout_ms = ms ? ms : 1;
}

void ccusb_set_trace(ccusb_device *dev, int on)
{
    if (dev) dev->trace = on;
}

int ccusb_list(ccusb_device_info *out, int max)
{
    return ccusb__usb_list(out, max);
}
