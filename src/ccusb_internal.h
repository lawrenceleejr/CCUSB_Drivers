/* Internal declarations shared between the library's source files. */
#ifndef CCUSB_INTERNAL_H
#define CCUSB_INTERNAL_H

#include "ccusb/ccusb.h"

/* Target-address (first) word of every OUT packet. */
#define TA_STACK_SEL   0x0002  /* select stack memory                    */
#define TA_STACK_ID0   0x0001  /* 0 = data stack, 1 = scaler stack       */
#define TA_WRITE       0x0004
#define TA_IMMEDIATE   0x0008
#define TA_EXECUTE     (TA_WRITE | TA_IMMEDIATE)  /* 0x0C */
#define TA_REGISTER    0x0005  /* register block (action register)       */
#define ACTION_REG_ADDR 10

#define CCUSB_RX_BUFFER_BYTES (64 * 1024)  /* multiple of 512 */

struct ccusb_device {
    ccusb_transport ops;
    void           *ctx;
    char            serial[32];
    unsigned int    timeout_ms;
    int             trace;
    int             daq_running;
    int             is_sim;
    uint8_t        *rx;          /* CCUSB_RX_BUFFER_BYTES staging buffer */
};

/* libusb transport (usb.c) */
int ccusb__usb_list(ccusb_device_info *out, int max);
int ccusb__usb_open(const char *serial, ccusb_transport *ops, void **ctx,
                    char *serial_out, size_t serial_len);

/* simulator transport (sim.c) */
int ccusb__sim_open(ccusb_transport *ops, void **ctx);

/* portable helpers (platform.c) */
uint64_t ccusb__now_ms(void);
void     ccusb__sleep_ms(unsigned int ms);

#endif
