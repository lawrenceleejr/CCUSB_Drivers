/*
 * usb.c - libusb-1.0 transport for the CC-USB.
 *
 * Works on Linux (usbfs, needs read/write permission on the device node:
 * see udev/99-wiener-ccusb.rules), Windows (WinUSB / libusbK driver, e.g.
 * installed with Zadig) and macOS.
 */
#include "ccusb_internal.h"

#include <libusb.h>
#include <stdlib.h>
#include <string.h>

typedef struct usb_ctx {
    libusb_context       *ctx;
    libusb_device_handle *h;
} usb_ctx;

static int map_err(int e)
{
    switch (e) {
    case LIBUSB_SUCCESS: return CCUSB_OK;
    case LIBUSB_ERROR_TIMEOUT: return CCUSB_ERR_TIMEOUT;
    case LIBUSB_ERROR_ACCESS: return CCUSB_ERR_ACCESS;
    case LIBUSB_ERROR_NO_DEVICE: return CCUSB_ERR_NO_DEVICE;
    case LIBUSB_ERROR_NOT_FOUND: return CCUSB_ERR_NOT_FOUND;
    case LIBUSB_ERROR_BUSY: return CCUSB_ERR_BUSY;
    case LIBUSB_ERROR_OVERFLOW: return CCUSB_ERR_OVERFLOW;
    case LIBUSB_ERROR_PIPE: return CCUSB_ERR_PIPE;
    case LIBUSB_ERROR_NO_MEM: return CCUSB_ERR_NO_MEM;
    case LIBUSB_ERROR_NOT_SUPPORTED: return CCUSB_ERR_NOT_SUPPORTED;
    case LIBUSB_ERROR_INVALID_PARAM: return CCUSB_ERR_INVALID_PARAM;
    case LIBUSB_ERROR_IO: return CCUSB_ERR_IO;
    default: return CCUSB_ERR_OTHER;
    }
}

static int usb_write(void *vctx, const uint8_t *buf, size_t len, size_t *sent,
                     unsigned int timeout_ms)
{
    usb_ctx *c = (usb_ctx *)vctx;
    int done = 0;
    int rc = libusb_bulk_transfer(c->h, CCUSB_EP_OUT, (unsigned char *)buf,
                                  (int)len, &done, timeout_ms);
    if (rc == LIBUSB_ERROR_PIPE) {
        libusb_clear_halt(c->h, CCUSB_EP_OUT);
        rc = libusb_bulk_transfer(c->h, CCUSB_EP_OUT, (unsigned char *)buf,
                                  (int)len, &done, timeout_ms);
    }
    *sent = (size_t)(done > 0 ? done : 0);
    return map_err(rc);
}

static int usb_read(void *vctx, uint8_t *buf, size_t len, size_t *got,
                    unsigned int timeout_ms)
{
    usb_ctx *c = (usb_ctx *)vctx;
    int done = 0;
    int rc = libusb_bulk_transfer(c->h, CCUSB_EP_IN, buf, (int)len, &done,
                                  timeout_ms);
    if (rc == LIBUSB_ERROR_PIPE) libusb_clear_halt(c->h, CCUSB_EP_IN);
    *got = (size_t)(done > 0 ? done : 0);
    return map_err(rc);
}

static void usb_close(void *vctx)
{
    usb_ctx *c = (usb_ctx *)vctx;
    if (!c) return;
    if (c->h) {
        libusb_release_interface(c->h, 0);
        libusb_close(c->h);
    }
    if (c->ctx) libusb_exit(c->ctx);
    free(c);
}

static int is_ccusb(const struct libusb_device_descriptor *d)
{
    return d->idVendor == CCUSB_VENDOR_ID && d->idProduct == CCUSB_PRODUCT_ID;
}

static void read_serial(libusb_device_handle *h,
                        const struct libusb_device_descriptor *d, char *out,
                        size_t outlen)
{
    unsigned char s[64];
    int n;
    out[0] = 0;
    if (!d->iSerialNumber) return;
    n = libusb_get_string_descriptor_ascii(h, d->iSerialNumber, s, sizeof(s));
    if (n > 0) {
        size_t k = (size_t)n < outlen - 1 ? (size_t)n : outlen - 1;
        memcpy(out, s, k);
        out[k] = 0;
    }
}

int ccusb__usb_list(ccusb_device_info *out, int max)
{
    libusb_context *ctx = NULL;
    libusb_device **list = NULL;
    ssize_t n, i;
    int found = 0, rc;

    rc = libusb_init(&ctx);
    if (rc) return map_err(rc);
    n = libusb_get_device_list(ctx, &list);
    if (n < 0) {
        libusb_exit(ctx);
        return map_err((int)n);
    }
    for (i = 0; i < n; i++) {
        struct libusb_device_descriptor d;
        if (libusb_get_device_descriptor(list[i], &d) || !is_ccusb(&d))
            continue;
        if (out && found < max) {
            ccusb_device_info *info = &out[found];
            libusb_device_handle *h = NULL;
            memset(info, 0, sizeof(*info));
            info->vendor_id = d.idVendor;
            info->product_id = d.idProduct;
            info->bus = libusb_get_bus_number(list[i]);
            info->address = libusb_get_device_address(list[i]);
            if (libusb_open(list[i], &h) == 0) {
                info->accessible = 1;
                read_serial(h, &d, info->serial, sizeof(info->serial));
                libusb_close(h);
            }
        }
        found++;
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return found;
}

static int claim(libusb_device_handle *h)
{
    int cfg = 0, rc;
    libusb_set_auto_detach_kernel_driver(h, 1); /* no-op where unsupported */
    if (libusb_get_configuration(h, &cfg) == 0 && cfg != 1)
        libusb_set_configuration(h, 1); /* may fail on Windows: harmless */
    rc = libusb_claim_interface(h, 0);
    return map_err(rc);
}

int ccusb__usb_open(const char *serial, ccusb_transport *ops, void **vctx,
                    char *serial_out, size_t serial_len)
{
    libusb_context *ctx = NULL;
    libusb_device **list = NULL;
    libusb_device_handle *found = NULL;
    ssize_t n, i;
    int rc, last_err = CCUSB_ERR_NOT_FOUND;
    int want_any = (serial == NULL || serial[0] == 0);
    usb_ctx *c;

    rc = libusb_init(&ctx);
    if (rc) return map_err(rc);
    n = libusb_get_device_list(ctx, &list);
    if (n < 0) {
        libusb_exit(ctx);
        return map_err((int)n);
    }
    for (i = 0; i < n && !found; i++) {
        struct libusb_device_descriptor d;
        libusb_device_handle *h = NULL;
        char sn[32];
        if (libusb_get_device_descriptor(list[i], &d) || !is_ccusb(&d))
            continue;
        rc = libusb_open(list[i], &h);
        if (rc) {
            last_err = map_err(rc);
            continue;
        }
        read_serial(h, &d, sn, sizeof(sn));
        if (!want_any && strcmp(sn, serial) != 0) {
            libusb_close(h);
            continue;
        }
        rc = claim(h);
        if (rc) {
            last_err = rc;
            libusb_close(h);
            continue;
        }
        found = h;
        if (serial_out && serial_len) {
            strncpy(serial_out, sn, serial_len - 1);
            serial_out[serial_len - 1] = 0;
        }
    }
    libusb_free_device_list(list, 1);
    if (!found) {
        libusb_exit(ctx);
        return last_err;
    }
    c = (usb_ctx *)calloc(1, sizeof(*c));
    if (!c) {
        libusb_release_interface(found, 0);
        libusb_close(found);
        libusb_exit(ctx);
        return CCUSB_ERR_NO_MEM;
    }
    c->ctx = ctx;
    c->h = found;
    ops->write = usb_write;
    ops->read = usb_read;
    ops->close = usb_close;
    *vctx = c;
    return CCUSB_OK;
}
