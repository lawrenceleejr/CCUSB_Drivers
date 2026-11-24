/* The classic libxxusb API, exercised against the simulator. */
#include "libxxusb.h"

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

int main(void)
{
    xxusb_device_type devs[XXUSB_MAX_DEVICES];
    usb_dev_handle *udev = NULL;
    long data = 0;
    int q = -1, x = -1, i, n;
    long stack[64];

#ifdef _WIN32
    _putenv("CCUSB_SIM=1");
#else
    setenv("CCUSB_SIM", "1", 1);
#endif
    n = xxusb_devices_find(devs);
    CHECK(n >= 1);
    for (i = 0; i < n; i++)
        if (strcmp(devs[i].SerialString, "SIM") == 0)
            udev = xxusb_device_open(devs[i].usbdev);
    CHECK(udev != NULL);
    if (!udev) return 1;

    CHECK(CAMAC_write(udev, 3, 0, 16, 0x123456, &q, &x) == 2);
    CHECK(q == 1 && x == 1);
    CHECK(CAMAC_read(udev, 3, 0, 0, &data, &q, &x) == 4);
    CHECK(data == 0x123456 && q == 1 && x == 1);
    /* the original library refuses mismatched function classes */
    CHECK(CAMAC_write(udev, 3, 0, 0, 1, &q, &x) < 0);
    CHECK(CAMAC_read(udev, 3, 0, 16, &data, &q, &x) < 0);

    CHECK(CAMAC_Z(udev) > 0);
    CHECK(CAMAC_C(udev) > 0);
    CHECK(CAMAC_I(udev, 1) > 0);
    CHECK(CAMAC_I(udev, 0) > 0);

    CHECK(CAMAC_register_read(udev, 0, &data) == 4 && data == 0x8E000601L);
    CHECK(CAMAC_write_LAM_mask(udev, 0x40) > 0);
    CHECK(CAMAC_read_LAM_mask(udev, &data) > 0 && data == 0x40);
    CHECK(CAMAC_LED_settings(udev, 2, 3, 1, 0) > 0);
    CHECK(CAMAC_register_read(udev, 4, &data) == 4 && data == 0x1300);
    CHECK(CAMAC_Output_settings(udev, 3, 2, 0, 1) > 0);
    CHECK(CAMAC_register_read(udev, 5, &data) == 4 && data == 0x220000);
    CHECK(CAMAC_DGG(udev, 0, 7, 1, 0x20010, 100, 0, 0) > 0);
    CHECK(CAMAC_register_read(udev, 7, &data) == 4 && data == (100L << 16 | 0x10));
    CHECK(CAMAC_register_read(udev, 13, &data) == 4 && data == 2);
    CHECK(CAMAC_register_read(udev, 6, &data) == 4 && ((data >> 16) & 0xFF) == 7);

    /* raw stack execute: write then read back N3 A1 */
    stack[0] = 4;
    stack[1] = 16 + 1 * 32 + 3 * 512 + 0x4000;
    stack[2] = 0x5555;
    stack[3] = 0x0011;
    stack[4] = 0 + 1 * 32 + 3 * 512 + 0x4000;
    CHECK(xxusb_stack_execute(udev, stack) == 6);
    CHECK(stack[0] == 3);                 /* Q|X of the write */
    CHECK(stack[1] == 0x5555);            /* read: low word   */
    CHECK((stack[2] & 0xFF) == 0x11);     /* read: high byte  */

    /* stack download / readback */
    stack[0] = 2;
    stack[1] = 0 + 0 * 32 + 7 * 512;
    stack[2] = 0 + 1 * 32 + 7 * 512;
    CHECK(xxusb_stack_write(udev, XXUSB_READOUT_STACK, stack) == 8);
    memset(stack, 0, sizeof(stack));
    CHECK(xxusb_stack_read(udev, XXUSB_READOUT_STACK, stack) == 6);
    CHECK(stack[0] == 2 && stack[2] == 32 + 7 * 512);

    /* action register via xxusb_register_write: start, read data, stop */
    {
        unsigned short buf[8192];
        int got, k, total = 0;
        CHECK(xxusb_register_write(udev, XXUSB_ACTION_REGISTER, 1) == 6);
        for (k = 0; k < 3; k++) {
            got = xxusb_bulk_read(udev, buf, sizeof(buf), 500);
            if (got > 0) total += got;
        }
        CHECK(xxusb_register_write(udev, XXUSB_ACTION_REGISTER, 0) == 6);
        while ((got = xxusb_bulk_read(udev, buf, sizeof(buf), 200)) > 0)
            total += got;
        CHECK(total > 0);
    }

    CHECK(xxusb_device_close(udev) == 1);
    udev = xxusb_serial_open("SIM");
    CHECK(udev != NULL);
    if (udev) xxusb_device_close(udev);

    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("legacy: all checks passed\n");
    return 0;
}
