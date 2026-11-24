/*
 * libxxusb.h - source compatible replacement for W-IE-NE-R's libxxusb
 * (CC-USB functions), implemented on top of libccusb / libusb-1.0.
 *
 * Existing programs written against the old libusb-0.1 based libxxusb can
 * be rebuilt against this header and linked with -lccusb.  Do NOT also
 * include libusb-0.1's <usb.h>: usb_dev_handle and struct usb_device are
 * opaque types provided here.
 *
 * Return values follow the original library: number of bytes transferred
 * on success, a negative number on failure.
 *
 * Differences from the original:
 *  - VME_* (VM-USB) functions are not provided.
 *  - xxusb_register_read() reads CC-USB register A=RegAddr via N=25.
 *  - xxusb_devices_find() fills at most XXUSB_MAX_DEVICES entries.
 */
#ifndef libxxusb_h
#define libxxusb_h

#include "ccusb/ccusb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XXUSB_WIENER_VENDOR_ID  0x16DC
#define XXUSB_VMUSB_PRODUCT_ID  0x000B
#define XXUSB_CCUSB_PRODUCT_ID  0x0001
#define XXUSB_ENDPOINT_OUT      2
#define XXUSB_ENDPOINT_IN       0x86
#define XXUSB_FIRMWARE_REGISTER 0
#define XXUSB_GLOBAL_REGISTER   1
#define XXUSB_ACTION_REGISTER   10
#define XXUSB_DELAYS_REGISTER   2
#define XXUSB_WATCHDOG_REGISTER 3
#define XXUSB_SELLEDA_REGISTER  6
#define XXUSB_SELNIM_REGISTER   7
#define XXUSB_SELLEDB_REGISTER  4
#define XXUSB_SERIAL_REGISTER   15
#define XXUSB_LAMMASK_REGISTER  8
#define XXUSB_LAM_REGISTER      12
#define XXUSB_READOUT_STACK     2
#define XXUSB_SCALER_STACK      3
#define XXUSB_NAF_DIRECT        12
#define XXUSB_MAX_DEVICES       32

typedef struct ccusb_device usb_dev_handle;
struct usb_device;                 /* opaque */

struct xxusb_device_typ {
    struct usb_device *usbdev;
    char SerialString[7];
};
typedef struct xxusb_device_typ xxusb_device_type;
typedef unsigned char UCHAR;

CCUSB_API int xxusb_longstack_execute(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout);
CCUSB_API int xxusb_bulk_read(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout);
CCUSB_API int xxusb_bulk_write(usb_dev_handle *hDev, void *DataBuffer, int lDataLen, int timeout);
CCUSB_API int xxusb_usbfifo_read(usb_dev_handle *hDev, int *DataBuffer, int lDataLen, int timeout);

CCUSB_API short xxusb_register_read(usb_dev_handle *hDev, short RegAddr, long *RegData);
CCUSB_API short xxusb_register_write(usb_dev_handle *hDev, short RegAddr, long RegData);
CCUSB_API short xxusb_stack_read(usb_dev_handle *hDev, short StackAddr, long *StackData);
CCUSB_API short xxusb_stack_write(usb_dev_handle *hDev, short StackAddr, long *StackData);
CCUSB_API short xxusb_stack_execute(usb_dev_handle *hDev, long *StackData);
CCUSB_API short xxusb_reset_toggle(usb_dev_handle *hDev);

CCUSB_API short xxusb_devices_find(xxusb_device_type *xxusbDev);
CCUSB_API short xxusb_device_close(usb_dev_handle *hDev);
CCUSB_API usb_dev_handle *xxusb_device_open(struct usb_device *dev);
CCUSB_API usb_dev_handle *xxusb_serial_open(char *SerialString);
CCUSB_API short xxusb_flash_program(usb_dev_handle *hDev, char *config, short nsect);
CCUSB_API short xxusb_flashblock_program(usb_dev_handle *hDev, UCHAR *config);

CCUSB_API short CAMAC_DGG(usb_dev_handle *hdev, short channel, short trigger, short output, int delay, int gate, short invert, short latch);
CCUSB_API short CAMAC_register_read(usb_dev_handle *hdev, int A, long *Data);
CCUSB_API short CAMAC_register_write(usb_dev_handle *hdev, int A, long Data);
CCUSB_API short CAMAC_LED_settings(usb_dev_handle *hdev, int LED, int code, int invert, int latch);
CCUSB_API short CAMAC_Output_settings(usb_dev_handle *hdev, int Channel, int code, int invert, int latch);
CCUSB_API short CAMAC_read_LAM_mask(usb_dev_handle *hdev, long *Data);
CCUSB_API short CAMAC_write_LAM_mask(usb_dev_handle *hdev, long Data);

CCUSB_API short CAMAC_write(usb_dev_handle *hdev, int N, int A, int F, long Data, int *Q, int *X);
CCUSB_API short CAMAC_read(usb_dev_handle *hdev, int N, int A, int F, long *Data, int *Q, int *X);
CCUSB_API short CAMAC_Z(usb_dev_handle *hdev);
CCUSB_API short CAMAC_C(usb_dev_handle *hdev);
CCUSB_API short CAMAC_I(usb_dev_handle *hdev, int inhibit);

#ifdef __cplusplus
}
#endif
#endif /* libxxusb_h */
