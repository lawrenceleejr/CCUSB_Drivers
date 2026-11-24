/*
 * example.c - minimal use of libccusb.
 *
 *   cc example.c -o example $(pkg-config --cflags --libs ccusb)
 *   ./example            # first CC-USB found
 *   ./example SIM        # simulator
 */
#include <ccusb/ccusb.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ccusb_device *cc;
    uint32_t fw, data;
    int q, x, rc;

    rc = ccusb_open(&cc, argc > 1 ? argv[1] : NULL);
    if (rc != CCUSB_OK) {
        fprintf(stderr, "open: %s\n", ccusb_strerror(rc));
        return 1;
    }
    ccusb_firmware(cc, &fw);
    printf("connected to %s, firmware 0x%08x\n", ccusb_serial(cc), fw);

    ccusb_z(cc);                                 /* initialise the crate */
    ccusb_inhibit(cc, 0);

    /* write then read station 3, subaddress 0 */
    rc = ccusb_write(cc, 3, 0, 16, 0x1234, &q, &x);
    rc = rc ? rc : ccusb_read(cc, 3, 0, 0, &data, &q, &x);
    if (rc != CCUSB_OK)
        fprintf(stderr, "cycle: %s\n", ccusb_strerror(rc));
    else
        printf("N3 A0 F0 -> 0x%06x  Q=%d X=%d\n", data, q, x);

    ccusb_close(cc);
    return 0;
}
