/*
 * ccusb.h - Modern driver for the W-IE-NE-R CC-USB CAMAC crate controller.
 *
 * A clean, portable (Linux / Windows / macOS) user-space driver built on
 * libusb-1.0.  It replaces the old libusb-0.1 based "libxxusb" for the
 * CC-USB.  A source compatible libxxusb API is provided separately in
 * <libxxusb.h> for existing programs.
 *
 * Conventions
 *  - Every function that can fail returns an int: 0 (CCUSB_OK) on success
 *    or a negative CCUSB_ERR_* code.  ccusb_strerror() describes it.
 *  - CAMAC addressing uses the usual N (station 0..31), A (subaddress 0..15)
 *    and F (function 0..31).  N=25 is the CC-USB's own register block,
 *    N=28/29 are the crate-controller "pseudo stations" for Z/C/I.
 *  - A handle is not thread safe; serialise access if several threads use it.
 */
#ifndef CCUSB_CCUSB_H
#define CCUSB_CCUSB_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && !defined(CCUSB_STATIC)
#  ifdef CCUSB_BUILDING
#    define CCUSB_API __declspec(dllexport)
#  else
#    define CCUSB_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define CCUSB_API __attribute__((visibility("default")))
#else
#  define CCUSB_API
#endif

#define CCUSB_VERSION_MAJOR 1
#define CCUSB_VERSION_MINOR 0
#define CCUSB_VERSION_PATCH 0

/* ---- USB identifiers ------------------------------------------------- */
#define CCUSB_VENDOR_ID        0x16DC  /* W-IE-NE-R, Plein & Baus */
#define CCUSB_PRODUCT_ID       0x0001  /* CC-USB                  */
#define CCUSB_VMUSB_PRODUCT_ID 0x000B  /* VM-USB (not supported)  */
#define CCUSB_EP_OUT           0x02
#define CCUSB_EP_IN            0x86

/* ---- Error codes ------------------------------------------------------ */
enum ccusb_error {
    CCUSB_OK                = 0,
    CCUSB_ERR_IO            = -1,   /* USB transfer failed                 */
    CCUSB_ERR_INVALID_PARAM = -2,   /* bad N/A/F, NULL pointer, ...        */
    CCUSB_ERR_ACCESS        = -3,   /* permission denied (udev / driver)   */
    CCUSB_ERR_NO_DEVICE     = -4,   /* device unplugged                    */
    CCUSB_ERR_NOT_FOUND     = -5,   /* no CC-USB (with that serial) found  */
    CCUSB_ERR_BUSY          = -6,   /* claimed by another program, or DAQ
                                       mode is running                     */
    CCUSB_ERR_TIMEOUT       = -7,   /* no answer / no data within timeout  */
    CCUSB_ERR_OVERFLOW      = -8,   /* reply larger than the buffer        */
    CCUSB_ERR_PIPE          = -9,   /* endpoint stalled                    */
    CCUSB_ERR_NO_MEM        = -10,
    CCUSB_ERR_PROTOCOL      = -11,  /* unexpected reply from the module    */
    CCUSB_ERR_NOT_SUPPORTED = -12,
    CCUSB_ERR_OTHER         = -99
};

/* ---- CC-USB internal registers: N=25, A=<register>, F0 read / F16 write */
enum ccusb_register {
    CCUSB_REG_FIRMWARE    = 0,   /* read only                              */
    CCUSB_REG_GLOBAL_MODE = 1,   /* buffer length, header options          */
    CCUSB_REG_DELAYS      = 2,   /* trigger delay [7:0], LAM timeout [15:8]*/
    CCUSB_REG_SCALER_CTRL = 3,   /* scaler readout period / event count    */
    CCUSB_REG_LED_SRC     = 4,   /* LED source selector                    */
    CCUSB_REG_OUTPUT_SRC  = 5,   /* NIM O1..O3 source selector             */
    CCUSB_REG_DEVICE_SRC  = 6,   /* scaler A/B and DGG A/B input selectors */
    CCUSB_REG_DGG_A       = 7,   /* gate width [31:16], delay [15:0]       */
    CCUSB_REG_DGG_B       = 8,
    CCUSB_REG_LAM_MASK    = 9,   /* LAMs that trigger the readout stack    */
    CCUSB_REG_LAM         = 10,  /* current LAM lines (read only)          */
    CCUSB_REG_SCALER_A    = 11,  /* read only                              */
    CCUSB_REG_SCALER_B    = 12,  /* read only                              */
    CCUSB_REG_DGG_EXT     = 13,  /* coarse delays: A [15:0], B [31:16]     */
    CCUSB_REG_USB_SETUP   = 14   /* USB bulk transfer setup                */
};
#define CCUSB_REGISTER_STATION 25

/* Action register bits (written with ccusb_write_action) */
#define CCUSB_ACTION_START_DAQ   0x0001
#define CCUSB_ACTION_USB_TRIGGER 0x0002
#define CCUSB_ACTION_CLEAR       0x0004
#define CCUSB_ACTION_SCALER_DUMP 0x0010

/* Global mode register fields */
#define CCUSB_GMODE_BUFLEN_MASK     0x000F
#define CCUSB_GMODE_BUFLEN_4096     0
#define CCUSB_GMODE_BUFLEN_2048     1
#define CCUSB_GMODE_BUFLEN_1024     2
#define CCUSB_GMODE_BUFLEN_512      3
#define CCUSB_GMODE_BUFLEN_256      4
#define CCUSB_GMODE_BUFLEN_128      5
#define CCUSB_GMODE_BUFLEN_64       6
#define CCUSB_GMODE_BUFLEN_SINGLE   7
#define CCUSB_GMODE_MIXED_BUFFERS   0x0020
#define CCUSB_GMODE_DOUBLE_HEADER   0x0100
#define CCUSB_GMODE_SECONDARY_EVENT 0x1000

/* Stack (readout list) identifiers */
enum ccusb_stack_id {
    CCUSB_STACK_DATA   = 0,   /* event readout stack (max 768 words)  */
    CCUSB_STACK_SCALER = 1    /* scaler readout stack (max 256 words) */
};
#define CCUSB_DATA_STACK_MAX   768
#define CCUSB_SCALER_STACK_MAX 256

/* ---- Device discovery / lifetime ------------------------------------- */
typedef struct ccusb_device ccusb_device;

typedef struct ccusb_device_info {
    char     serial[32];      /* e.g. "CC0123" (empty if unreadable) */
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t  bus;
    uint8_t  address;
    int      accessible;      /* 0 if the device could not be opened  */
} ccusb_device_info;

/* Version of the library as a string, e.g. "1.0.0". */
CCUSB_API const char *ccusb_version(void);
CCUSB_API const char *ccusb_strerror(int err);

/* Enumerate attached CC-USB controllers.  Fills up to max entries and
 * returns the number of controllers found (which may exceed max), or a
 * negative error. */
CCUSB_API int ccusb_list(ccusb_device_info *out, int max);

/* Open a controller.  serial == NULL or "" opens the first one found;
 * serial == "SIM" opens the built-in software simulator (no hardware). */
CCUSB_API int ccusb_open(ccusb_device **dev, const char *serial);
CCUSB_API int ccusb_open_simulator(ccusb_device **dev);
CCUSB_API void ccusb_close(ccusb_device *dev);

CCUSB_API const char *ccusb_serial(const ccusb_device *dev);
CCUSB_API int  ccusb_is_simulator(const ccusb_device *dev);
/* Default timeout (ms) for command transactions. Default 2000 ms. */
CCUSB_API void ccusb_set_timeout(ccusb_device *dev, unsigned int ms);
/* Enable a hex dump of every USB packet on stderr (debugging). */
CCUSB_API void ccusb_set_trace(ccusb_device *dev, int on);

/* ---- Single CAMAC cycles --------------------------------------------- */
/* Read function (F0..F7, or any F not 16..23): 24-bit data + Q/X. */
CCUSB_API int ccusb_read(ccusb_device *dev, int n, int a, int f,
                         uint32_t *data, int *q, int *x);
/* Write function (F16..F23): 24-bit data. */
CCUSB_API int ccusb_write(ccusb_device *dev, int n, int a, int f,
                          uint32_t data, int *q, int *x);
/* Control function (F8..F15, F24..F31): no data. */
CCUSB_API int ccusb_control(ccusb_device *dev, int n, int a, int f,
                            int *q, int *x);
/* Any NAF: dispatches to read / write / control from F.  For writes *data
 * is the input; for reads it receives the result (may be NULL otherwise). */
CCUSB_API int ccusb_naf(ccusb_device *dev, int n, int a, int f,
                        uint32_t *data, int *q, int *x);

/* Dataway operations */
CCUSB_API int ccusb_z(ccusb_device *dev);              /* initialise  N28 A8 F29 */
CCUSB_API int ccusb_c(ccusb_device *dev);              /* clear       N28 A9 F29 */
CCUSB_API int ccusb_inhibit(ccusb_device *dev, int on);/* set/clear I N29 A9 F24/F26 */
/* Inhibit state (bit 24 of the LAM register; firmware >= 0x0601). */
CCUSB_API int ccusb_is_inhibited(ccusb_device *dev, int *on);

/* ---- Controller registers -------------------------------------------- */
CCUSB_API int ccusb_reg_read(ccusb_device *dev, int reg, uint32_t *value);
CCUSB_API int ccusb_reg_write(ccusb_device *dev, int reg, uint32_t value);
CCUSB_API const char *ccusb_reg_name(int reg);  /* NULL if unknown */
CCUSB_API int ccusb_firmware(ccusb_device *dev, uint32_t *id);
CCUSB_API int ccusb_read_lams(ccusb_device *dev, uint32_t *lams);
CCUSB_API int ccusb_read_scalers(ccusb_device *dev, uint32_t *a, uint32_t *b);
CCUSB_API int ccusb_write_action(ccusb_device *dev, uint16_t value);

/* ---- Stacks (CAMAC command lists) ------------------------------------ */
typedef struct ccusb_stack {
    uint16_t *words;
    size_t    count;
    size_t    capacity;
    int       error;   /* first error encountered while building, or 0 */
} ccusb_stack;

/* Stack mode word bits (follow an NAF word whose bit 15 is set) */
#define CCUSB_NAF_LONG         0x4000  /* 24-bit transfer              */
#define CCUSB_NAF_CONTINUATION 0x8000  /* a mode word follows          */
#define CCUSB_MODE_HIT         0x0001
#define CCUSB_MODE_NO_S2       0x0002
#define CCUSB_MODE_NUMBER      0x0004
#define CCUSB_MODE_HITMODE     0x0008
#define CCUSB_MODE_QSTOP       0x0010
#define CCUSB_MODE_QSCAN       0x0020
#define CCUSB_MODE_REPEAT      0x0040
#define CCUSB_MODE_LAMWAIT     0x0080
#define CCUSB_MODE_FASTCAMAC   0x0100
#define CCUSB_MODE_ADDRPATTERN 0x0200

CCUSB_API void ccusb_stack_init(ccusb_stack *s);
CCUSB_API void ccusb_stack_free(ccusb_stack *s);
CCUSB_API void ccusb_stack_clear(ccusb_stack *s);
CCUSB_API uint16_t ccusb_naf_word(int n, int a, int f, int long_mode);
CCUSB_API int ccusb_stack_add_word(ccusb_stack *s, uint16_t w);
CCUSB_API int ccusb_stack_add_read(ccusb_stack *s, int n, int a, int f,
                                   int long_mode, int lam_wait);
CCUSB_API int ccusb_stack_add_write(ccusb_stack *s, int n, int a, int f,
                                    uint32_t data, int long_mode);
CCUSB_API int ccusb_stack_add_control(ccusb_stack *s, int n, int a, int f);
CCUSB_API int ccusb_stack_add_qstop(ccusb_stack *s, int n, int a, int f,
                                    uint16_t max, int long_mode, int lam_wait);
CCUSB_API int ccusb_stack_add_qscan(ccusb_stack *s, int n, int a, int f,
                                    uint16_t max, int long_mode, int lam_wait);
CCUSB_API int ccusb_stack_add_repeat(ccusb_stack *s, int n, int a, int f,
                                     uint16_t count, int long_mode, int lam_wait);
/* Insert a literal 16-bit marker word into the event data. */
CCUSB_API int ccusb_stack_add_marker(ccusb_stack *s, uint16_t value);

/* Execute a list of stack words immediately and collect the reply words.
 * reply_words receives the number of 16-bit words returned. */
CCUSB_API int ccusb_execute(ccusb_device *dev, const uint16_t *words,
                            size_t nwords, uint16_t *reply,
                            size_t reply_max, size_t *reply_words);
CCUSB_API int ccusb_stack_execute(ccusb_device *dev, const ccusb_stack *s,
                                  uint16_t *reply, size_t reply_max,
                                  size_t *reply_words);
/* Load a stack into the controller's DATA or SCALER stack memory. */
CCUSB_API int ccusb_stack_load(ccusb_device *dev, int which,
                               const uint16_t *words, size_t nwords);
/* Read back a stack from the controller (raw reply words). */
CCUSB_API int ccusb_stack_read(ccusb_device *dev, int which, uint16_t *words,
                               size_t max, size_t *nwords);

/* ---- Data acquisition (autonomous) mode ------------------------------ */
CCUSB_API int ccusb_daq_start(ccusb_device *dev);
/* Stop data taking.  Remaining buffers must then be drained with
 * ccusb_read_buffer() until it returns CCUSB_ERR_TIMEOUT. */
CCUSB_API int ccusb_daq_stop(ccusb_device *dev);
CCUSB_API int ccusb_daq_running(const ccusb_device *dev);
/* Read one data buffer (raw little-endian 16-bit words). */
CCUSB_API int ccusb_read_buffer(ccusb_device *dev, uint16_t *buf,
                                size_t max_words, size_t *nwords,
                                unsigned int timeout_ms);
/* Discard everything pending on the IN endpoint. Returns bytes dropped. */
CCUSB_API int ccusb_flush(ccusb_device *dev);

/* Buffer decoding.  The callback is called for each event segment. */
#define CCUSB_BUF_LAST        0x8000
#define CCUSB_BUF_SCALER      0x4000
#define CCUSB_BUF_CONTINUOUS  0x2000
#define CCUSB_BUF_MULTIBUFFER 0x1000
#define CCUSB_BUF_NEVENTS     0x0FFF
#define CCUSB_EVT_LENGTH      0x0FFF
#define CCUSB_EVT_CONTINUED   0x1000
#define CCUSB_EVT_STACK_ID    0x2000
#define CCUSB_BUFFER_TERMINATOR 0xFFFF

typedef struct ccusb_event {
    const uint16_t *data;    /* event body (header excluded)          */
    size_t          nwords;
    int             stack_id;/* 0 = data stack, 1 = scaler stack      */
    int             continued;/* more segments of this event follow   */
} ccusb_event;

typedef int (*ccusb_event_cb)(const ccusb_event *ev, void *user);

/* Decode a buffer. double_header must match CCUSB_GMODE_DOUBLE_HEADER.
 * Returns the number of events decoded, or a negative error. The callback
 * may return non-zero to stop decoding. */
CCUSB_API int ccusb_parse_buffer(const uint16_t *buf, size_t nwords,
                                 int double_header, ccusb_event_cb cb,
                                 void *user);

/* ---- Raw access -------------------------------------------------------- */
CCUSB_API int ccusb_bulk_write(ccusb_device *dev, const void *buf, size_t len,
                               size_t *sent, unsigned int timeout_ms);
CCUSB_API int ccusb_bulk_read(ccusb_device *dev, void *buf, size_t len,
                              size_t *got, unsigned int timeout_ms);

/* ---- Custom transports (tests, simulators, network bridges) ---------- */
typedef struct ccusb_transport {
    int  (*write)(void *ctx, const uint8_t *buf, size_t len, size_t *sent,
                  unsigned int timeout_ms);
    int  (*read)(void *ctx, uint8_t *buf, size_t len, size_t *got,
                 unsigned int timeout_ms);
    void (*close)(void *ctx);
} ccusb_transport;

CCUSB_API int ccusb_open_transport(ccusb_device **dev,
                                   const ccusb_transport *ops, void *ctx,
                                   const char *name);

#ifdef __cplusplus
}
#endif
#endif /* CCUSB_CCUSB_H */
