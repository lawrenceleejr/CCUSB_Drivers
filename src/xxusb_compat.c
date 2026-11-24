/*
 * xxusb_compat.c - the classic libxxusb API (CC-USB subset) on top of
 * libccusb.  Behaviour and return values mirror W-IE-NE-R's libxxusb 3.x so
 * existing programs keep working; bugs that made functions unusable are
 * fixed (fixed-size stack buffers that could overflow, enumeration
 * aborting on the first inaccessible device, libusb-0.1 dependency).
 *
 * Setting the environment variable CCUSB_SIM=1 makes xxusb_devices_find()
 * also report the software simulator (serial "SIM"), which lets legacy
 * programs run without hardware.
 */
#include "libxxusb.h"
#include "ccusb_internal.h"

#include <stdlib.h>
#include <string.h>

struct usb_device {
    char serial[32];
};

static struct usb_device g_devs[XXUSB_MAX_DEVICES];

static int sim_enabled(void)
{
    const char *e = getenv("CCUSB_SIM");
    return e && e[0] && strcmp(e, "0") != 0;
}

/* ---------------- raw transfers ---------------------------------- */

int xxusb_bulk_read(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout)
{
    size_t got = 0;
    int rc;
    if (lDataLen <= 0) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_bulk_read(hDev, DataBuffer, (size_t)lDataLen, &got,
                         (unsigned)timeout);
    return rc ? rc : (int)got;
}

int xxusb_bulk_write(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout)
{
    size_t sent = 0;
    int rc;
    if (lDataLen < 0) return CCUSB_ERR_INVALID_PARAM;
    rc = ccusb_bulk_write(hDev, DataBuffer, (size_t)lDataLen, &sent,
                          (unsigned)timeout);
    return rc ? rc : (int)sent;
}

int xxusb_usbfifo_read(usb_dev_handle *hDev, int *DataBuffer, int lDataLen, int timeout)
{
    unsigned short *us = (unsigned short *)DataBuffer;
    int ret = xxusb_bulk_read(hDev, DataBuffer, lDataLen, timeout), i;
    if (ret > 0)
        for (i = ret / 2 - 1; i >= 0; i--) {
            unsigned short v = us[i];
            DataBuffer[i] = v;
        }
    return ret;
}

int xxusb_longstack_execute(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout)
{
    unsigned char *c = (unsigned char *)DataBuffer;
    unsigned short *us = (unsigned short *)DataBuffer;
    int size, ret;
    c[0] = 12;
    c[1] = 0;
    size = 2 * (us[1] + 0x10000 * us[2]) + 4;
    ret = xxusb_bulk_write(hDev, DataBuffer, size, timeout);
    if (ret > 0) ret = xxusb_bulk_read(hDev, DataBuffer, lDataLen, timeout);
    return ret;
}

/* ---------------- registers / stacks ------------------------------ */

short xxusb_register_write(usb_dev_handle *hDev, short RegAddr, long RegData)
{
    unsigned char buf[8] = {5, 0, 0, 0, 0, 0, 0, 0};
    int len = 6;
    if (RegAddr == 0 || RegAddr == 12 || RegAddr == 15) return 0;
    buf[2] = (unsigned char)(RegAddr & 15);
    buf[4] = (unsigned char)(RegData & 255);
    buf[5] = (unsigned char)((RegData >> 8) & 255);
    if (RegAddr == 8) {
        buf[6] = (unsigned char)((RegData >> 16) & 255);
        len = 8;
    }
    return (short)xxusb_bulk_write(hDev, buf, len, 100);
}

short xxusb_register_read(usb_dev_handle *hDev, short RegAddr, long *RegData)
{
    uint32_t v = 0;
    int rc = ccusb_reg_read(hDev, RegAddr, &v);
    if (rc) return (short)rc;
    if (RegData) *RegData = (long)v;
    return 4;
}

short xxusb_stack_write(usb_dev_handle *hDev, short StackAddr, long *intbuf)
{
    uint16_t *w;
    long n, i;
    int rc, which = (StackAddr & 1) ? CCUSB_STACK_SCALER : CCUSB_STACK_DATA;
    if (!intbuf) return CCUSB_ERR_INVALID_PARAM;
    n = intbuf[0] & 0xFFF;
    if (n == 0) return 0;
    w = (uint16_t *)malloc((size_t)n * sizeof(uint16_t));
    if (!w) return CCUSB_ERR_NO_MEM;
    for (i = 0; i < n; i++) w[i] = (uint16_t)(intbuf[i + 1] & 0xFFFF);
    rc = ccusb_stack_load(hDev, which, w, (size_t)n);
    free(w);
    return rc ? (short)rc : (short)(n * 2 + 4);
}

short xxusb_stack_read(usb_dev_handle *hDev, short StackAddr, long *intbuf)
{
    uint16_t w[1024];
    size_t n = 0, i;
    int rc = ccusb_stack_read(hDev, (StackAddr & 1) ? CCUSB_STACK_SCALER
                                                    : CCUSB_STACK_DATA,
                              w, 1024, &n);
    if (rc) return (short)rc;
    for (i = 0; i < n; i++) intbuf[i] = w[i];
    return (short)(n * 2);
}

short xxusb_stack_execute(usb_dev_handle *hDev, long *intbuf)
{
    uint16_t *w, *r;
    long n, i;
    size_t nr = 0;
    int rc;
    const size_t rmax = 26700 / 2;
    if (!intbuf) return CCUSB_ERR_INVALID_PARAM;
    n = intbuf[0] & 0xFFF;
    if (n == 0) return 0;
    w = (uint16_t *)malloc((size_t)n * sizeof(uint16_t));
    r = (uint16_t *)malloc(rmax * sizeof(uint16_t));
    if (!w || !r) {
        free(w);
        free(r);
        return CCUSB_ERR_NO_MEM;
    }
    for (i = 0; i < n; i++) w[i] = (uint16_t)(intbuf[i + 1] & 0xFFFF);
    rc = ccusb_execute(hDev, w, (size_t)n, r, rmax, &nr);
    if (rc == CCUSB_OK)
        for (i = 0; i < (long)nr; i++) intbuf[i] = r[i];
    free(w);
    free(r);
    return rc ? (short)rc : (short)(nr * 2);
}

short xxusb_reset_toggle(usb_dev_handle *hDev)
{
    unsigned char buf[2] = {0xFF, 0xFF};
    return (short)xxusb_bulk_write(hDev, buf, 2, 1000);
}

/* Ported verbatim from libxxusb (where it is marked untested). */
short xxusb_flash_program(usb_dev_handle *hDev, char *config, short nsect)
{
    unsigned char buf[518] = {0xAA, 0xAA, 0x55, 0x55, 0xA0, 0xA0};
    char *p = config;
    int i, k, ret = 0;
    while (*p++ != -1) {
    }
    for (i = 0; i < nsect; i++) {
        unsigned char *b = buf + 6;
        for (k = 0; k < 256; k++) {
            *(b++) = (unsigned char)*p;
            *(b++) = (unsigned char)*(p++);
        }
        ret = xxusb_bulk_write(hDev, buf, 518, 2000);
        if (ret < 0) return (short)ret;
        ccusb__sleep_ms(30);
    }
    return (short)ret;
}

short xxusb_flashblock_program(usb_dev_handle *hDev, UCHAR *config)
{
    unsigned char buf[518] = {0xAA, 0xAA, 0x55, 0x55, 0xA0, 0xA0};
    unsigned char *b = buf + 6;
    int k;
    for (k = 0; k < 256; k++) {
        *(b++) = config[k];
        *(b++) = config[k];
    }
    return (short)xxusb_bulk_write(hDev, buf, 518, 2000);
}

/* ---------------- enumeration / open ------------------------------ */

short xxusb_devices_find(xxusb_device_type *xxdev)
{
    ccusb_device_info info[XXUSB_MAX_DEVICES];
    int n = ccusb_list(info, XXUSB_MAX_DEVICES), i, found = 0;
    if (n < 0) n = 0;
    if (n > XXUSB_MAX_DEVICES) n = XXUSB_MAX_DEVICES;
    for (i = 0; i < n; i++) {
        if (!info[i].accessible) continue;
        strncpy(g_devs[found].serial, info[i].serial, sizeof(g_devs[0].serial) - 1);
        xxdev[found].usbdev = &g_devs[found];
        strncpy(xxdev[found].SerialString, info[i].serial, 6);
        xxdev[found].SerialString[6] = 0;
        found++;
    }
    if (sim_enabled() && found < XXUSB_MAX_DEVICES) {
        strcpy(g_devs[found].serial, "SIM");
        xxdev[found].usbdev = &g_devs[found];
        strcpy(xxdev[found].SerialString, "SIM");
        found++;
    }
    if (found == 0 && n > 0) return -1; /* present but not accessible */
    return (short)found;
}

usb_dev_handle *xxusb_device_open(struct usb_device *dev)
{
    ccusb_device *h = NULL;
    if (ccusb_open(&h, dev ? dev->serial : NULL) != CCUSB_OK) return NULL;
    return h;
}

usb_dev_handle *xxusb_serial_open(char *SerialString)
{
    ccusb_device *h = NULL;
    if (!SerialString || !SerialString[0]) return NULL;
    if (ccusb_open(&h, SerialString) != CCUSB_OK) return NULL;
    return h;
}

short xxusb_device_close(usb_dev_handle *hDev)
{
    ccusb_close(hDev);
    return 1;
}

/* ---------------- CAMAC ------------------------------------------- */

/* Execute one NAF (+ data words) and return the raw reply. */
static int exec_raw(usb_dev_handle *h, const uint16_t *w, size_t n,
                    uint16_t *r, size_t *nr)
{
    int rc = ccusb_execute(h, w, n, r, 4, nr);
    return rc ? rc : (int)(*nr * 2);
}

short CAMAC_read(usb_dev_handle *hdev, int N, int A, int F, long *Data, int *Q, int *X)
{
    uint16_t w, r[4];
    size_t nr = 0;
    int ret;
    if (F >= 16 && F <= 23) return -1;
    w = ccusb_naf_word(N, A, F, 1);
    ret = exec_raw(hdev, &w, 1, r, &nr);
    if (ret == 2) {
        if (Q) *Q = r[0] & 1;
        if (X) *X = (r[0] >> 1) & 1;
    }
    if (ret == 4) {
        if (Data) *Data = r[0] + (long)(r[1] & 255) * 0x10000;
        if (Q) *Q = (r[1] >> 8) & 1;
        if (X) *X = (r[1] >> 9) & 1;
    }
    return (short)ret;
}

short CAMAC_write(usb_dev_handle *hdev, int N, int A, int F, long Data, int *Q, int *X)
{
    uint16_t w[3], r[4];
    size_t nr = 0;
    int ret;
    if (F < 16 || F > 23) return -1;
    w[0] = ccusb_naf_word(N, A, F, 1);
    w[1] = (uint16_t)(Data & 0xFFFF);
    w[2] = (uint16_t)((Data >> 16) & 255);
    ret = exec_raw(hdev, w, 3, r, &nr);
    if (ret > 0) {
        if (Q) *Q = r[0] & 1;
        if (X) *X = (r[0] >> 1) & 1;
    }
    return (short)ret;
}

static short control(usb_dev_handle *hdev, int n, int a, int f)
{
    uint16_t w = ccusb_naf_word(n, a, f, 1), r[4];
    size_t nr = 0;
    return (short)exec_raw(hdev, &w, 1, r, &nr);
}

short CAMAC_Z(usb_dev_handle *hdev) { return control(hdev, 28, 8, 29); }
short CAMAC_C(usb_dev_handle *hdev) { return control(hdev, 28, 9, 29); }
short CAMAC_I(usb_dev_handle *hdev, int inhibit)
{
    return control(hdev, 29, 9, inhibit ? 24 : 26);
}

short CAMAC_register_read(usb_dev_handle *hdev, int A, long *Data)
{
    uint32_t v = 0;
    int rc = ccusb_reg_read(hdev, A, &v);
    if (rc) return (short)rc;
    if (Data) *Data = (long)v;
    return 4;
}

short CAMAC_register_write(usb_dev_handle *hdev, int A, long Data)
{
    uint16_t w[3], r[4];
    size_t nr = 0;
    if (A < 0 || A > 15) return -1;
    w[0] = ccusb_naf_word(25, A, 16, 1);
    w[1] = (uint16_t)(Data & 0xFFFF);
    w[2] = (uint16_t)((Data >> 16) & 0xFFFF);
    return (short)exec_raw(hdev, w, 3, r, &nr);
}

short CAMAC_read_LAM_mask(usb_dev_handle *hdev, long *Data)
{
    long v = 0;
    short ret = CAMAC_register_read(hdev, CCUSB_REG_LAM_MASK, &v);
    if (ret > 0 && Data) *Data = v & 0xFFFFFF;
    return ret;
}

short CAMAC_write_LAM_mask(usb_dev_handle *hdev, long Data)
{
    return CAMAC_register_write(hdev, CCUSB_REG_LAM_MASK, Data);
}

/* LED / NIM output selector: three byte-wide fields (code, invert 0x10,
 * latch 0x20) in register 4 (LEDs 1..3) or 5 (outputs 1..3). */
static short selector(usb_dev_handle *hdev, int reg, int which, int code,
                      int invert, int latch)
{
    long data = 0;
    int shift;
    short ret;
    if (which < 1 || which > 3 || code < 0 || code > 7) return -1;
    ret = CAMAC_register_read(hdev, reg, &data);
    if (ret < 0) return ret;
    shift = (which - 1) * 8;
    data &= ~(0xFFL << shift) & 0xFFFFFF;
    data |= (long)(code | (invert ? 0x10 : 0) | (latch ? 0x20 : 0)) << shift;
    return CAMAC_register_write(hdev, reg, data);
}

short CAMAC_LED_settings(usb_dev_handle *hdev, int LED, int code, int invert, int latch)
{
    return selector(hdev, CCUSB_REG_LED_SRC, LED, code, invert, latch);
}

short CAMAC_Output_settings(usb_dev_handle *hdev, int Channel, int code, int invert, int latch)
{
    return selector(hdev, CCUSB_REG_OUTPUT_SRC, Channel, code, invert, latch);
}

short CAMAC_DGG(usb_dev_handle *hdev, short channel, short trigger, short output,
                int delay, int gate, short invert, short latch)
{
    long data = 0, sel = 0, internal;
    short ret, code;
    if (output < 1 || output > 3 || channel < 0 || channel > 1) return -1;
    /* Firmware >= 5.0 encodes the output source differently. */
    ret = CAMAC_register_read(hdev, CCUSB_REG_FIRMWARE, &data);
    if (ret < 0) return ret;
    if ((data & 0xFFF) > 0x499)
        code = (short)(2 + channel);
    else
        code = (short)(2 + output + channel);

    ret = CAMAC_register_read(hdev, CCUSB_REG_OUTPUT_SRC, &data);
    if (ret < 0) return ret;
    internal = 0xFFFFFF ^ (0xFFL << ((output - 1) * 8));
    data &= internal;
    sel = 0xFFFFFFF & ((long)(code + invert * 8 + latch * 16) << ((output - 1) * 8));
    ret = CAMAC_register_write(hdev, CCUSB_REG_OUTPUT_SRC, data | sel);
    if (ret < 0) return ret;

    ret = CAMAC_register_read(hdev, CCUSB_REG_DEVICE_SRC, &sel);
    if (ret < 0) return ret;
    data = ((long)gate << 16) + (delay & 0xFFFF);
    if (channel == 0) {
        ret = CAMAC_register_write(hdev, CCUSB_REG_DGG_A, data);
        if (ret < 0) return ret;
        ret = CAMAC_register_write(hdev, CCUSB_REG_DGG_EXT, (delay / 0x10000) & 0xFFFF);
        if (ret < 0) return ret;
        sel = (sel & 0xFF00FFFF) | ((long)trigger << 16);
    } else {
        ret = CAMAC_register_write(hdev, CCUSB_REG_DGG_B, data);
        if (ret < 0) return ret;
        sel = (sel & 0x00FFFFFF) | ((long)trigger << 24);
    }
    return CAMAC_register_write(hdev, CCUSB_REG_DEVICE_SRC, sel);
}
