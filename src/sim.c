/*
 * sim.c - a software CC-USB with a small simulated CAMAC crate.
 *
 * It speaks the same USB packet protocol as the real controller, so
 * everything above the transport (library, CLI, Python, GUI) can be
 * exercised without hardware:  ccusb_open(&dev, "SIM").
 *
 * Simulated crate:
 *   N3  SIM-MEM16   16 read/write registers   F0 An read, F16 An write,
 *                   F9 clear, F8 test LAM, F10 clear LAM, F25 set LAM
 *   N5  SIM-SCALER  12 counters with different (some modulated) rates
 *                   F0 An read, F2 An read+clear, F9 clear all,
 *                   F24 disable, F26 enable
 *   N7  SIM-ADC8    8-channel peak ADC, one conversion per trigger
 *                   F0 An read, F2 An read (A7 clears), F8 test LAM,
 *                   F9 clear, F10 clear LAM, F24/F26 LAM disable/enable,
 *                   F25 software conversion (trigger)
 *   N25 controller registers, N28/N29 Z, C, inhibit.
 * In DAQ mode it triggers at ~200 Hz, executes the loaded data stack per
 * event and returns buffers in the CC-USB buffer format.
 */
#include "ccusb_internal.h"

#include <stdlib.h>
#include <string.h>

#define SIM_FIRMWARE 0x8E000601u
#define SIM_TRIGGER_HZ 200
#define SIM_FLUSH_MS 100

enum { MOD_EMPTY = 0, MOD_MEM, MOD_SCALER, MOD_ADC };

typedef struct sim_module {
    int      type;
    uint32_t reg[16];
    int      lam, lam_enable, enabled, valid;
    uint64_t t_clear[16]; /* scaler: time the channel was last cleared */
    double   base[16];    /* scaler: integral at last clear            */
} sim_module;

typedef struct msg {
    struct msg *next;
    size_t      len;
    uint8_t     data[1];
} msg;

typedef struct sim {
    sim_module slot[24];
    uint32_t   regs[16];
    int        inhibit;
    uint16_t   data_stack[CCUSB_DATA_STACK_MAX];
    size_t     data_n;
    uint16_t   scaler_stack[CCUSB_SCALER_STACK_MAX];
    size_t     scaler_n;
    uint64_t   t0;
    uint32_t   rng;
    /* DAQ */
    int        daq;
    uint64_t   next_trigger, buf_open_ms, next_scaler;
    uint16_t  *ev;            /* events of the buffer being filled */
    size_t     ev_n, ev_cap, ev_count;
    msg       *head, *tail;   /* queued IN packets */
} sim;

/* ---------------------------------------------------------------- */

static uint32_t rnd(sim *s)
{
    uint32_t x = s->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return s->rng = x;
}

/* approximately normal(0,1): Irwin-Hall with 12 uniforms */
static double gauss(sim *s)
{
    double t = 0;
    int i;
    for (i = 0; i < 12; i++) t += (rnd(s) & 0xFFFF) / 65536.0;
    return t - 6.0;
}

static double now_s(sim *s)
{
    return (double)(ccusb__now_ms() - s->t0) / 1000.0;
}

/* cos(x) without libm: range-reduce to [-pi, pi], then Taylor to x^12
 * (error < 2e-4, plenty for a simulator). */
static double sim_cos(double x)
{
    const double two_pi = 6.283185307179586, pi = 3.141592653589793;
    double z;
    x -= two_pi * (double)(long)(x / two_pi);
    if (x > pi) x -= two_pi;
    if (x < -pi) x += two_pi;
    z = x * x;
    return 1 - z / 2 * (1 - z / 12 * (1 - z / 30 * (1 - z / 56 *
           (1 - z / 90 * (1 - z / 132)))));
}

/* Integral from 0 to t of the counting rate of scaler channel ch.
 * Channels 0-3 are modulated, r(t) = R (1 + k sin(2 pi t / P)), so that
 * rate plots have something to show; the rest count at a constant rate. */
static double scaler_integral(int ch, double t)
{
    static const double rate[12] = {1000, 800, 500, 300, 200, 100,
                                    50,   20,  10,  5,   2,   1};
    const double two_pi = 6.283185307179586, k = 0.6;
    double r = rate[ch];
    if (ch < 4) {
        double period = 8.0 + 6.0 * ch;
        return r * (t - k * period / two_pi * (sim_cos(two_pi * t / period) - 1.0));
    }
    return r * t;
}

static void module_reset(sim *s, int n)
{
    sim_module *m = &s->slot[n];
    int type = m->type, i;
    memset(m, 0, sizeof(*m));
    m->type = type;
    m->enabled = 1;
    for (i = 0; i < 16; i++) {
        m->t_clear[i] = ccusb__now_ms();
        m->base[i] = scaler_integral(i < 12 ? i : 0, now_s(s));
    }
}

static uint32_t scaler_value(sim *s, sim_module *m, int ch)
{
    double t = now_s(s), v;
    if (ch >= 12) return 0;
    v = scaler_integral(ch, t) - m->base[ch];
    if (!m->enabled) v = m->reg[ch];
    return (uint32_t)v & 0xFFFFFF;
}

static void scaler_clear(sim *s, sim_module *m, int ch)
{
    m->base[ch] = scaler_integral(ch, now_s(s));
    m->reg[ch] = 0;
}

static void adc_convert(sim *s, sim_module *m)
{
    int ch;
    for (ch = 0; ch < 8; ch++) {
        double v = 400.0 + 420.0 * ch + 35.0 * (1 + ch * 0.3) * gauss(s);
        if ((rnd(s) % 10) == 0) v = (rnd(s) % 4096); /* flat background */
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        m->reg[ch] = (uint32_t)v;
    }
    m->valid = 1;
    m->lam = 1;
}

/* One dataway cycle. */
static void cycle(sim *s, int n, int a, int f, uint32_t *data, int *q, int *x)
{
    uint32_t d = data ? *data : 0;
    *q = 0;
    *x = 0;
    if (n == 25) { /* controller registers */
        *q = *x = 1;
        if (f == 0) {
            uint32_t v = s->regs[a];
            if (a == CCUSB_REG_FIRMWARE) v = SIM_FIRMWARE;
            if (a == CCUSB_REG_LAM) {
                int k;
                v = 0;
                for (k = 1; k <= 23; k++)
                    if (s->slot[k].lam) v |= 1u << (k - 1);
                if (s->inhibit) v |= 1u << 24;
            }
            if (a == CCUSB_REG_SCALER_A) v = (uint32_t)(now_s(s) * 1000.0) & 0xFFFFFF;
            if (a == CCUSB_REG_SCALER_B) v = (uint32_t)(now_s(s) * 10.0) & 0xFFFFFF;
            d = v;
        } else if (f == 16) {
            if (a != 0 && a != 10 && a != 11 && a != 12) s->regs[a] = d;
        }
        if (data) *data = d;
        return;
    }
    if (n == 28 && f == 29 && (a == 8 || a == 9)) { /* Z / C */
        int k;
        for (k = 1; k <= 23; k++)
            if (s->slot[k].type) module_reset(s, k);
        if (a == 8) s->inhibit = 0;
        *q = *x = 1;
        return;
    }
    if (n == 29 && a == 9 && (f == 24 || f == 26 || f == 27)) {
        if (f == 24) s->inhibit = 1;
        if (f == 26) s->inhibit = 0;
        *q = (f == 27) ? s->inhibit : 1;
        *x = 1;
        return;
    }
    if (n < 1 || n > 23 || s->slot[n].type == MOD_EMPTY) {
        if (data && f < 8) *data = 0;
        return; /* no module: no X, no Q */
    }
    {
        sim_module *m = &s->slot[n];
        *x = 1;
        switch (m->type) {
        case MOD_MEM:
            if (f == 0) { d = m->reg[a]; *q = 1; }
            else if (f == 16) { m->reg[a] = d & 0xFFFFFF; *q = 1; }
            else if (f == 9) { memset(m->reg, 0, sizeof(m->reg)); *q = 1; }
            else if (f == 8) { *q = m->lam; }
            else if (f == 10) { m->lam = 0; *q = 1; }
            else if (f == 25) { m->lam = 1; *q = 1; }
            else *x = 0;
            break;
        case MOD_SCALER:
            if ((f == 0 || f == 2) && a < 12) {
                d = scaler_value(s, m, a);
                if (f == 2) scaler_clear(s, m, a);
                *q = 1;
            } else if (f == 9) {
                int k;
                for (k = 0; k < 12; k++) scaler_clear(s, m, k);
                *q = 1;
            } else if (f == 24 || f == 26) {
                int k;
                if (f == 24 && m->enabled)
                    for (k = 0; k < 12; k++) m->reg[k] = scaler_value(s, m, k);
                if (f == 26 && !m->enabled)
                    for (k = 0; k < 12; k++)
                        m->base[k] = scaler_integral(k, now_s(s)) - m->reg[k];
                m->enabled = (f == 26);
                *q = 1;
            } else if (f == 0 || f == 2) {
                d = 0; /* subaddress out of range: X but no Q */
            } else *x = 0;
            break;
        case MOD_ADC:
            if ((f == 0 || f == 2) && a < 8) {
                d = m->reg[a];
                *q = m->valid;
                if (f == 2 && a == 7) { m->valid = 0; m->lam = 0; }
            } else if (f == 0 || f == 2) {
                d = 0;
            } else if (f == 8) { *q = m->lam && m->lam_enable; }
            else if (f == 9) { memset(m->reg, 0, sizeof(m->reg)); m->valid = 0; m->lam = 0; *q = 1; }
            else if (f == 10) { m->lam = 0; *q = 1; }
            else if (f == 24 || f == 26) { m->lam_enable = (f == 26); *q = 1; }
            else if (f == 25) { adc_convert(s, m); *q = 1; }
            else *x = 0;
            break;
        default: break;
        }
    }
    if (data) *data = d;
}

/* ---------------------------------------------------------------- */
/* growable word output                                              */

typedef struct wbuf {
    uint16_t *w;
    size_t    n, cap;
} wbuf;

static void wput(wbuf *b, uint16_t v)
{
    if (b->n == b->cap) {
        size_t cap = b->cap ? b->cap * 2 : 256;
        uint16_t *nw = (uint16_t *)realloc(b->w, cap * sizeof(uint16_t));
        if (!nw) return;
        b->w = nw;
        b->cap = cap;
    }
    b->w[b->n++] = v;
}

static void put_read(wbuf *b, int n, uint32_t d, int q, int x, int lng)
{
    wput(b, (uint16_t)(d & 0xFFFF));
    if (lng) {
        if (n == 25)
            wput(b, (uint16_t)(d >> 16));
        else
            wput(b, (uint16_t)(((d >> 16) & 0xFF) | (q << 8) | (x << 9)));
    }
}

static int is_write_f(int f) { return f >= 16 && f <= 23; }
static int is_ctl_f(int f) { return (f >= 8 && f <= 15) || f >= 24; }

/* Interpret a stack.  immediate=1 also returns Q/X words for writes and
 * controls (as the controller does for CNAF execution). */
static void run_stack(sim *s, const uint16_t *w, size_t n, wbuf *out,
                      int immediate)
{
    size_t i = 0;
    while (i < n) {
        uint16_t naf = w[i++];
        int f = naf & 31, a = (naf >> 5) & 15, nn = (naf >> 9) & 31;
        int lng = (naf & CCUSB_NAF_LONG) != 0;
        uint16_t mode = 0, max = 0;
        int q, x;
        if (naf & CCUSB_NAF_CONTINUATION) {
            if (i >= n) break;
            mode = w[i++];
            if (mode & (CCUSB_MODE_QSTOP | CCUSB_MODE_QSCAN | CCUSB_MODE_REPEAT)) {
                if (i >= n) break;
                max = w[i++];
            }
        }
        if (is_write_f(f)) {
            uint32_t d;
            if (i >= n) break;
            d = w[i++];
            if (lng && i < n) d |= (uint32_t)w[i++] << 16;
            if (nn == 0) { wput(out, (uint16_t)d); continue; } /* marker */
            cycle(s, nn, a, f, &d, &q, &x);
            if (immediate) wput(out, (uint16_t)(q | (x << 1)));
        } else if (is_ctl_f(f)) {
            cycle(s, nn, a, f, NULL, &q, &x);
            if (immediate) wput(out, (uint16_t)(q | (x << 1)));
        } else if (mode & CCUSB_MODE_REPEAT) {
            uint16_t k;
            for (k = 0; k < max; k++) {
                uint32_t d = 0;
                cycle(s, nn, a, f, &d, &q, &x);
                put_read(out, nn, d, q, x, lng);
            }
        } else if (mode & CCUSB_MODE_QSTOP) {
            uint16_t k;
            for (k = 0; k < max; k++) {
                uint32_t d = 0;
                cycle(s, nn, a, f, &d, &q, &x);
                if (!q) break;
                put_read(out, nn, d, q, x, lng);
            }
        } else if (mode & CCUSB_MODE_QSCAN) {
            uint16_t k;
            int cn = nn, ca = a;
            for (k = 0; k < max && cn <= 23; k++) {
                uint32_t d = 0;
                cycle(s, cn, ca, f, &d, &q, &x);
                if (!x) break;
                if (q) {
                    put_read(out, cn, d, q, x, lng);
                    if (++ca > 15) { ca = 0; cn++; }
                } else {
                    ca = 0;
                    cn++;
                }
            }
        } else {
            uint32_t d = 0;
            cycle(s, nn, a, f, &d, &q, &x);
            put_read(out, nn, d, q, x, lng);
        }
    }
}

/* ---------------------------------------------------------------- */
/* IN packet queue                                                   */

static void queue_words(sim *s, const uint16_t *w, size_t n)
{
    msg *m = (msg *)malloc(sizeof(msg) + n * 2);
    size_t i;
    if (!m) return;
    m->next = NULL;
    m->len = n * 2;
    for (i = 0; i < n; i++) {
        m->data[2 * i] = (uint8_t)(w[i] & 0xFF);
        m->data[2 * i + 1] = (uint8_t)(w[i] >> 8);
    }
    if (s->tail) s->tail->next = m; else s->head = m;
    s->tail = m;
}

/* ---------------------------------------------------------------- */
/* DAQ mode                                                          */

static size_t buffer_words(sim *s)
{
    static const size_t len[8] = {4096, 2048, 1024, 512, 256, 128, 64, 0};
    return len[s->regs[CCUSB_REG_GLOBAL_MODE] & 7];
}

static void emit_buffer(sim *s, int last)
{
    wbuf b = {0, 0, 0};
    size_t i;
    if (s->ev_count == 0 && !last) return;
    wput(&b, (uint16_t)((s->ev_count & 0xFFF) | (last ? CCUSB_BUF_LAST : 0)));
    if (s->regs[CCUSB_REG_GLOBAL_MODE] & CCUSB_GMODE_DOUBLE_HEADER)
        wput(&b, (uint16_t)(s->ev_n + 1));
    for (i = 0; i < s->ev_n; i++) wput(&b, s->ev[i]);
    wput(&b, CCUSB_BUFFER_TERMINATOR);
    queue_words(s, b.w, b.n);
    free(b.w);
    s->ev_n = 0;
    s->ev_count = 0;
}

static void add_event(sim *s, const uint16_t *body, size_t n, int stack_id)
{
    size_t need = s->ev_n + n + 1, i;
    if (need > s->ev_cap) {
        size_t cap = need * 2;
        uint16_t *nw = (uint16_t *)realloc(s->ev, cap * sizeof(uint16_t));
        if (!nw) return;
        s->ev = nw;
        s->ev_cap = cap;
    }
    s->ev[s->ev_n++] = (uint16_t)((n & CCUSB_EVT_LENGTH) |
                                  (stack_id ? CCUSB_EVT_STACK_ID : 0));
    for (i = 0; i < n; i++) s->ev[s->ev_n++] = body[i];
    s->ev_count++;
}

static void daq_trigger(sim *s)
{
    wbuf out = {0, 0, 0};
    int k;
    size_t blen = buffer_words(s);
    for (k = 1; k <= 23; k++)
        if (s->slot[k].type == MOD_ADC) adc_convert(s, &s->slot[k]);
    run_stack(s, s->data_stack, s->data_n, &out, 0);
    /* buffer = header + events + terminator must fit the buffer length */
    if (blen && s->ev_count && s->ev_n + out.n + 1 + 2 > blen)
        emit_buffer(s, 0);
    if (s->ev_count == 0) s->buf_open_ms = ccusb__now_ms();
    add_event(s, out.w, out.n, 0);
    free(out.w);
    if (blen == 0 || s->ev_count >= 0xFFF) emit_buffer(s, 0);
}

static void daq_scaler(sim *s)
{
    wbuf out = {0, 0, 0};
    uint16_t *save = s->ev;
    size_t sn = s->ev_n, sc = s->ev_count, scap = s->ev_cap;
    run_stack(s, s->scaler_stack, s->scaler_n, &out, 0);
    /* scaler data travel in their own buffer */
    s->ev = NULL; s->ev_n = 0; s->ev_count = 0; s->ev_cap = 0;
    add_event(s, out.w, out.n, 1);
    {
        wbuf b = {0, 0, 0};
        size_t i;
        wput(&b, (uint16_t)(1 | CCUSB_BUF_SCALER));
        for (i = 0; i < s->ev_n; i++) wput(&b, s->ev[i]);
        wput(&b, CCUSB_BUFFER_TERMINATOR);
        queue_words(s, b.w, b.n);
        free(b.w);
    }
    free(s->ev);
    s->ev = save; s->ev_n = sn; s->ev_count = sc; s->ev_cap = scap;
    free(out.w);
}

static void daq_advance(sim *s)
{
    uint64_t now = ccusb__now_ms();
    uint32_t period = (s->regs[CCUSB_REG_SCALER_CTRL] >> 16) & 0xFF;
    int budget = 2000;
    while (s->next_trigger <= now && budget-- > 0) {
        daq_trigger(s);
        /* exponential-ish spacing around the mean trigger rate */
        s->next_trigger += 1 + (rnd(s) % (2000 / SIM_TRIGGER_HZ));
    }
    if (s->next_trigger < now) s->next_trigger = now;
    if (period && s->scaler_n && now >= s->next_scaler) {
        daq_scaler(s);
        s->next_scaler = now + period * 500u;
    }
    if (s->ev_count && now - s->buf_open_ms >= SIM_FLUSH_MS) emit_buffer(s, 0);
}

static void action(sim *s, uint16_t v)
{
    if (v & CCUSB_ACTION_START_DAQ) {
        if (!s->daq) {
            s->daq = 1;
            s->next_trigger = ccusb__now_ms();
            s->next_scaler = ccusb__now_ms() +
                ((s->regs[CCUSB_REG_SCALER_CTRL] >> 16) & 0xFF) * 500u;
            s->ev_n = s->ev_count = 0;
        }
    } else if (s->daq) {
        s->daq = 0;
        emit_buffer(s, 1);
    }
    if ((v & CCUSB_ACTION_SCALER_DUMP) && s->daq && s->scaler_n) daq_scaler(s);
    if ((v & CCUSB_ACTION_USB_TRIGGER) && s->daq) daq_trigger(s);
}

/* ---------------------------------------------------------------- */
/* transport                                                         */

static int sim_write(void *vctx, const uint8_t *buf, size_t len, size_t *sent,
                     unsigned int timeout_ms)
{
    sim *s = (sim *)vctx;
    size_t nw = len / 2, i;
    uint16_t *w;
    (void)timeout_ms;
    *sent = len;
    if (nw < 1) return CCUSB_OK;
    w = (uint16_t *)malloc(nw * sizeof(uint16_t));
    if (!w) return CCUSB_ERR_NO_MEM;
    for (i = 0; i < nw; i++) w[i] = (uint16_t)(buf[2 * i] | (buf[2 * i + 1] << 8));

    if (w[0] == 0xFFFF) {
        /* reset toggle / flash programming: ignored */
    } else if (w[0] == 5) {
        if (nw >= 3 && w[1] == 10) action(s, w[2]);
    } else if (w[0] & 0x8) { /* execute immediately */
        if (!s->daq && nw >= 2) {
            size_t cnt = w[1];
            wbuf out = {0, 0, 0};
            if (cnt > nw - 2) cnt = nw - 2;
            run_stack(s, w + 2, cnt, &out, 1);
            queue_words(s, out.w, out.n);
            free(out.w);
        }
    } else if ((w[0] & 0x6) == 0x6) { /* load stack */
        size_t cnt = nw >= 2 ? w[1] : 0;
        if (cnt > nw - 2) cnt = nw - 2;
        if (w[0] & 1) {
            if (cnt > CCUSB_SCALER_STACK_MAX) cnt = CCUSB_SCALER_STACK_MAX;
            memcpy(s->scaler_stack, w + 2, cnt * 2);
            s->scaler_n = cnt;
        } else {
            if (cnt > CCUSB_DATA_STACK_MAX) cnt = CCUSB_DATA_STACK_MAX;
            memcpy(s->data_stack, w + 2, cnt * 2);
            s->data_n = cnt;
        }
    } else if (w[0] & 0x2) { /* read stack */
        wbuf out = {0, 0, 0};
        const uint16_t *st = (w[0] & 1) ? s->scaler_stack : s->data_stack;
        size_t cnt = (w[0] & 1) ? s->scaler_n : s->data_n;
        wput(&out, (uint16_t)cnt);
        for (i = 0; i < cnt; i++) wput(&out, st[i]);
        queue_words(s, out.w, out.n);
        free(out.w);
    }
    free(w);
    return CCUSB_OK;
}

static int sim_read(void *vctx, uint8_t *buf, size_t len, size_t *got,
                    unsigned int timeout_ms)
{
    sim *s = (sim *)vctx;
    uint64_t deadline = ccusb__now_ms() + timeout_ms;
    *got = 0;
    for (;;) {
        if (s->daq) daq_advance(s);
        if (s->head) {
            msg *m = s->head;
            if (m->len > len) return CCUSB_ERR_OVERFLOW;
            memcpy(buf, m->data, m->len);
            *got = m->len;
            s->head = m->next;
            if (!s->head) s->tail = NULL;
            free(m);
            return CCUSB_OK;
        }
        if (!s->daq) {
            /* nothing will ever arrive: emulate a (short) timeout */
            ccusb__sleep_ms(timeout_ms < 5 ? timeout_ms : 5);
            return CCUSB_ERR_TIMEOUT;
        }
        if (ccusb__now_ms() >= deadline) return CCUSB_ERR_TIMEOUT;
        ccusb__sleep_ms(2);
    }
}

static void sim_close(void *vctx)
{
    sim *s = (sim *)vctx;
    while (s->head) {
        msg *m = s->head;
        s->head = m->next;
        free(m);
    }
    free(s->ev);
    free(s);
}

int ccusb__sim_open(ccusb_transport *ops, void **ctx)
{
    sim *s = (sim *)calloc(1, sizeof(sim));
    if (!s) return CCUSB_ERR_NO_MEM;
    s->t0 = ccusb__now_ms();
    s->rng = 0x12345678u;
    s->slot[3].type = MOD_MEM;
    s->slot[5].type = MOD_SCALER;
    s->slot[7].type = MOD_ADC;
    module_reset(s, 3);
    module_reset(s, 5);
    module_reset(s, 7);
    s->regs[CCUSB_REG_USB_SETUP] = 0x0100;
    ops->write = sim_write;
    ops->read = sim_read;
    ops->close = sim_close;
    *ctx = s;
    return CCUSB_OK;
}
