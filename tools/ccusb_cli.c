/*
 * ccusb - command line tool for the W-IE-NE-R CC-USB CAMAC controller.
 *
 *   ccusb list
 *   ccusb [-s SERIAL | --sim] [-v] [-t MS] <command> [args...]
 *   ccusb --sim shell
 *
 * Run "ccusb help" for the list of commands.
 */
#include "ccusb/ccusb.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_ARGS 32

typedef struct app {
    const char   *serial;
    int           trace;
    unsigned int  timeout;
    ccusb_device *dev;
} app;

static const char *usage_text =
    "usage: ccusb [-s SERIAL | --sim] [-v] [-t MS] <command> [args]\n"
    "\n"
    "options:\n"
    "  -s SERIAL     use the CC-USB with this serial number (e.g. CC0123)\n"
    "  --sim         use the built-in simulator instead of hardware\n"
    "  -t MS         command timeout in milliseconds (default 2000)\n"
    "  -v            trace every USB packet on stderr\n"
    "\n"
    "commands (numbers may be decimal or 0x hex):\n"
    "  list                       list attached CC-USB controllers\n"
    "  info                       firmware and all controller registers\n"
    "  naf N A F [DATA]           one CAMAC cycle (read, write or control)\n"
    "  read N A F                 24-bit read\n"
    "  write N A F DATA           24-bit write\n"
    "  control N A F              control function\n"
    "  z | c                      dataway initialise / clear\n"
    "  inhibit [on|off]           set, clear or show dataway inhibit\n"
    "  reg [NAME|A [VALUE]]       read/write a controller register (N25)\n"
    "  scan                       show which stations answer with X\n"
    "  lam                        show LAM lines\n"
    "  scalers                    read the internal scalers A and B\n"
    "  action VALUE               write the action register\n"
    "  stack run FILE             execute a stack file immediately\n"
    "  stack load data|scaler FILE  download a stack file\n"
    "  stack show data|scaler     read a stack back from the controller\n"
    "  daq FILE [-n SEC] [-o OUT] [-p N] [--scaler FILE]\n"
    "                             load FILE as data stack and take data;\n"
    "                             OUT gets the raw buffers, -p prints N events\n"
    "  shell                      interactive prompt\n"
    "  help                       this text\n"
    "\n"
    "stack file lines ('#' starts a comment):\n"
    "  read N A F [long] [lam]    write N A F DATA [long]    control N A F\n"
    "  qstop|qscan|repeat N A F MAX [long] [lam]   marker VALUE   word VALUE\n";

/* ------------------------------------------------------------------ */

static int parse_num(const char *s, long *out)
{
    char *end;
    long v;
    if (!s || !*s) return 0;
    errno = 0;
    v = strtol(s, &end, 0);
    if (errno || *end) return 0;
    *out = v;
    return 1;
}

static int need_nums(int argc, char **argv, int first, int count, long *v)
{
    int i;
    if (argc < first + count) {
        fprintf(stderr, "error: missing arguments\n");
        return 0;
    }
    for (i = 0; i < count; i++)
        if (!parse_num(argv[first + i], &v[i])) {
            fprintf(stderr, "error: '%s' is not a number\n", argv[first + i]);
            return 0;
        }
    return 1;
}

static int report(int rc)
{
    if (rc != CCUSB_OK) fprintf(stderr, "error: %s\n", ccusb_strerror(rc));
    return rc == CCUSB_OK ? 0 : 1;
}

static int ensure_open(app *a)
{
    int rc;
    if (a->dev) return 0;
    rc = ccusb_open(&a->dev, a->serial);
    if (rc != CCUSB_OK) {
        if (rc == CCUSB_ERR_NOT_FOUND)
            fprintf(stderr, "error: no CC-USB found%s%s (try 'ccusb list', or "
                    "--sim for the simulator)\n",
                    a->serial ? " with serial " : "", a->serial ? a->serial : "");
        else
            fprintf(stderr, "error: cannot open CC-USB: %s\n", ccusb_strerror(rc));
        return 1;
    }
    ccusb_set_trace(a->dev, a->trace);
    if (a->timeout) ccusb_set_timeout(a->dev, a->timeout);
    return 0;
}

static int reg_from_name(const char *s)
{
    long v;
    int i;
    if (parse_num(s, &v)) return (v >= 0 && v <= 15) ? (int)v : -1;
    for (i = 0; i < 16; i++) {
        const char *n = ccusb_reg_name(i);
        if (n && strcmp(n, s) == 0) return i;
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* stack files                                                         */

static int has_flag(int argc, char **argv, int from, const char *flag)
{
    int i;
    for (i = from; i < argc; i++)
        if (strcmp(argv[i], flag) == 0) return 1;
    return 0;
}

static int tokenize(char *line, char **argv, int max)
{
    int n = 0;
    char *p = line;
    while (*p && n < max) {
        while (*p && isspace((unsigned char)*p)) p++;
        if (!*p || *p == '#') break;
        argv[n++] = p;
        while (*p && !isspace((unsigned char)*p)) p++;
        if (*p) *p++ = 0;
    }
    return n;
}

static int load_stack_file(const char *path, ccusb_stack *s)
{
    FILE *f = fopen(path, "r");
    char line[512];
    int lineno = 0;
    if (!f) {
        fprintf(stderr, "error: cannot open %s: %s\n", path, strerror(errno));
        return 1;
    }
    ccusb_stack_init(s);
    while (fgets(line, sizeof(line), f)) {
        char *argv[MAX_ARGS];
        int argc, rc = CCUSB_OK;
        long v[4];
        lineno++;
        argc = tokenize(line, argv, MAX_ARGS);
        if (argc == 0) continue;
#define NUMS(k) if (!need_nums(argc, argv, 1, k, v)) { rc = CCUSB_ERR_INVALID_PARAM; } else
        if (strcmp(argv[0], "read") == 0) {
            NUMS(3) rc = ccusb_stack_add_read(s, (int)v[0], (int)v[1], (int)v[2],
                                              has_flag(argc, argv, 4, "long"),
                                              has_flag(argc, argv, 4, "lam"));
        } else if (strcmp(argv[0], "write") == 0) {
            NUMS(4) rc = ccusb_stack_add_write(s, (int)v[0], (int)v[1], (int)v[2],
                                               (uint32_t)v[3],
                                               has_flag(argc, argv, 5, "long"));
        } else if (strcmp(argv[0], "control") == 0) {
            NUMS(3) rc = ccusb_stack_add_control(s, (int)v[0], (int)v[1], (int)v[2]);
        } else if (strcmp(argv[0], "qstop") == 0 || strcmp(argv[0], "qscan") == 0 ||
                   strcmp(argv[0], "repeat") == 0) {
            NUMS(4) {
                int lng = has_flag(argc, argv, 5, "long");
                int lam = has_flag(argc, argv, 5, "lam");
                if (argv[0][1] == 's')
                    rc = ccusb_stack_add_qstop(s, (int)v[0], (int)v[1], (int)v[2], (uint16_t)v[3], lng, lam);
                else if (argv[0][1] == 'c')
                    rc = ccusb_stack_add_qscan(s, (int)v[0], (int)v[1], (int)v[2], (uint16_t)v[3], lng, lam);
                else
                    rc = ccusb_stack_add_repeat(s, (int)v[0], (int)v[1], (int)v[2], (uint16_t)v[3], lng, lam);
            }
        } else if (strcmp(argv[0], "marker") == 0) {
            NUMS(1) rc = ccusb_stack_add_marker(s, (uint16_t)v[0]);
        } else if (strcmp(argv[0], "word") == 0) {
            NUMS(1) rc = ccusb_stack_add_word(s, (uint16_t)v[0]);
        } else {
            rc = CCUSB_ERR_INVALID_PARAM;
            fprintf(stderr, "unknown stack command '%s'\n", argv[0]);
        }
#undef NUMS
        if (rc != CCUSB_OK) {
            fprintf(stderr, "%s:%d: invalid stack line\n", path, lineno);
            fclose(f);
            ccusb_stack_free(s);
            return 1;
        }
    }
    fclose(f);
    return 0;
}

static void print_words(const uint16_t *w, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
        printf("%04x%s", w[i], (i % 8 == 7 || i + 1 == n) ? "\n" : " ");
}

/* ------------------------------------------------------------------ */
/* commands                                                            */

static int cmd_list(void)
{
    ccusb_device_info info[16];
    int n = ccusb_list(info, 16), i;
    if (n < 0) return report(n);
    if (n == 0) {
        printf("no CC-USB found (use --sim for the simulator)\n");
        return 0;
    }
    printf("%-8s %-4s %-4s %s\n", "serial", "bus", "addr", "status");
    for (i = 0; i < n && i < 16; i++)
        printf("%-8s %-4u %-4u %s\n", info[i].serial[0] ? info[i].serial : "?",
               info[i].bus, info[i].address,
               info[i].accessible ? "ok" : "no permission (see README)");
    return 0;
}

static int cmd_info(app *a)
{
    uint32_t v;
    int r, rc;
    rc = ccusb_firmware(a->dev, &v);
    if (rc) return report(rc);
    printf("serial      %s%s\n", ccusb_serial(a->dev),
           ccusb_is_simulator(a->dev) ? " (simulator)" : "");
    printf("firmware    0x%08x\n", v);
    printf("library     %s\n\n", ccusb_version());
    printf("A   register      value\n");
    for (r = 1; r <= 14; r++) {
        rc = ccusb_reg_read(a->dev, r, &v);
        if (rc) return report(rc);
        printf("%-3d %-12s  0x%08x\n", r, ccusb_reg_name(r), v);
    }
    return 0;
}

static int cmd_naf(app *a, int argc, char **argv, const char *kind)
{
    long v[4];
    uint32_t data = 0;
    int q = 0, x = 0, rc, f;
    int want = (kind && strcmp(kind, "write") == 0) ? 4 : 3;
    if (!need_nums(argc, argv, 1, want, v)) return 1;
    f = (int)v[2];
    if (argc > 4 && parse_num(argv[4], &v[3])) data = (uint32_t)v[3];
    if (kind && strcmp(kind, "read") == 0)
        rc = ccusb_read(a->dev, (int)v[0], (int)v[1], f, &data, &q, &x);
    else if (kind && strcmp(kind, "write") == 0)
        rc = ccusb_write(a->dev, (int)v[0], (int)v[1], f, data, &q, &x);
    else if (kind && strcmp(kind, "control") == 0)
        rc = ccusb_control(a->dev, (int)v[0], (int)v[1], f, &q, &x);
    else
        rc = ccusb_naf(a->dev, (int)v[0], (int)v[1], f, &data, &q, &x);
    if (rc) return report(rc);
    if (f < 8 || f > 23 || (kind && strcmp(kind, "read") == 0))
        printf("N%ld A%ld F%d  data=0x%06x (%u)  Q=%d X=%d\n", v[0], v[1], f,
               data, data, q, x);
    else
        printf("N%ld A%ld F%d  Q=%d X=%d\n", v[0], v[1], f, q, x);
    return 0;
}

static int cmd_reg(app *a, int argc, char **argv)
{
    uint32_t v;
    long nv;
    int r, rc;
    if (argc < 2) return cmd_info(a);
    r = reg_from_name(argv[1]);
    if (r < 0) {
        fprintf(stderr, "error: unknown register '%s'\n", argv[1]);
        return 1;
    }
    if (argc > 2) {
        if (!parse_num(argv[2], &nv)) {
            fprintf(stderr, "error: '%s' is not a number\n", argv[2]);
            return 1;
        }
        rc = ccusb_reg_write(a->dev, r, (uint32_t)nv);
        if (rc) return report(rc);
    }
    rc = ccusb_reg_read(a->dev, r, &v);
    if (rc) return report(rc);
    printf("%s (A%d) = 0x%08x\n", ccusb_reg_name(r), r, v);
    return 0;
}

static int cmd_scan(app *a)
{
    uint32_t d, lams = 0;
    int n, q, x, rc;
    char xs[24], qs[24], ls[24];
    rc = ccusb_read_lams(a->dev, &lams);
    if (rc) return report(rc);
    for (n = 1; n <= 23; n++) {
        rc = ccusb_read(a->dev, n, 0, 0, &d, &q, &x);
        if (rc) return report(rc);
        xs[n - 1] = x ? 'X' : '.';
        qs[n - 1] = q ? 'Q' : '.';
        ls[n - 1] = (lams >> (n - 1)) & 1 ? 'L' : '.';
    }
    printf("station   ");
    for (n = 1; n <= 23; n++) printf("%3d", n);
    printf("\nX (A0F0)  ");
    for (n = 0; n < 23; n++) printf("%3c", xs[n]);
    printf("\nQ (A0F0)  ");
    for (n = 0; n < 23; n++) printf("%3c", qs[n]);
    printf("\nLAM       ");
    for (n = 0; n < 23; n++) printf("%3c", ls[n]);
    printf("\n");
    return 0;
}

static int cmd_stack(app *a, int argc, char **argv)
{
    ccusb_stack s;
    int which, rc;
    if (argc < 3) {
        fprintf(stderr, "usage: stack run FILE | stack load data|scaler FILE | "
                        "stack show data|scaler\n");
        return 1;
    }
    if (strcmp(argv[1], "run") == 0) {
        uint16_t reply[8192];
        size_t n;
        if (load_stack_file(argv[2], &s)) return 1;
        rc = ccusb_stack_execute(a->dev, &s, reply, 8192, &n);
        ccusb_stack_free(&s);
        if (rc) return report(rc);
        printf("%u reply words\n", (unsigned)n);
        print_words(reply, n);
        return 0;
    }
    which = strcmp(argv[2], "scaler") == 0 ? CCUSB_STACK_SCALER : CCUSB_STACK_DATA;
    if (strcmp(argv[2], "scaler") != 0 && strcmp(argv[2], "data") != 0) {
        fprintf(stderr, "error: stack must be 'data' or 'scaler'\n");
        return 1;
    }
    if (strcmp(argv[1], "load") == 0) {
        if (argc < 4) {
            fprintf(stderr, "error: missing stack file\n");
            return 1;
        }
        if (load_stack_file(argv[3], &s)) return 1;
        rc = ccusb_stack_load(a->dev, which, s.words, s.count);
        if (rc == CCUSB_OK) printf("loaded %u words into %s stack\n", (unsigned)s.count, argv[2]);
        ccusb_stack_free(&s);
        return report(rc);
    }
    if (strcmp(argv[1], "show") == 0) {
        uint16_t w[1024];
        size_t n;
        rc = ccusb_stack_read(a->dev, which, w, 1024, &n);
        if (rc) return report(rc);
        print_words(w, n);
        return 0;
    }
    fprintf(stderr, "error: unknown stack subcommand '%s'\n", argv[1]);
    return 1;
}

typedef struct daq_print {
    int remaining;
} daq_print;

static int print_event(const ccusb_event *ev, void *user)
{
    daq_print *p = (daq_print *)user;
    if (p->remaining <= 0) return 0;
    p->remaining--;
    printf("event (%s, %u words):", ev->stack_id ? "scaler" : "data",
           (unsigned)ev->nwords);
    {
        size_t i;
        for (i = 0; i < ev->nwords && i < 16; i++) printf(" %04x", ev->data[i]);
        if (ev->nwords > 16) printf(" ...");
    }
    printf("\n");
    return 0;
}

static int cmd_daq(app *a, int argc, char **argv)
{
    ccusb_stack s;
    const char *out = NULL, *scaler = NULL;
    double seconds = 5;
    long nprint = 0;
    FILE *fo = NULL;
    uint16_t *buf;
    size_t n;
    int i, rc, double_header = 0;
    unsigned long nbuf = 0, nev = 0, nwords = 0;
    uint32_t gm = 0;
    clock_t t0;
    time_t start;
    daq_print pr;

    if (argc < 2) {
        fprintf(stderr, "usage: daq STACKFILE [-n SEC] [-o OUT] [-p N] [--scaler FILE]\n");
        return 1;
    }
    for (i = 2; i < argc; i++) {
        long v;
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc && parse_num(argv[i + 1], &v)) {
            seconds = (double)v;
            i++;
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out = argv[++i];
        } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc && parse_num(argv[i + 1], &v)) {
            nprint = v;
            i++;
        } else if (strcmp(argv[i], "--scaler") == 0 && i + 1 < argc) {
            scaler = argv[++i];
        } else {
            fprintf(stderr, "error: bad daq option '%s'\n", argv[i]);
            return 1;
        }
    }
    if (load_stack_file(argv[1], &s)) return 1;
    rc = ccusb_stack_load(a->dev, CCUSB_STACK_DATA, s.words, s.count);
    ccusb_stack_free(&s);
    if (rc) return report(rc);
    if (scaler) {
        if (load_stack_file(scaler, &s)) return 1;
        rc = ccusb_stack_load(a->dev, CCUSB_STACK_SCALER, s.words, s.count);
        ccusb_stack_free(&s);
        if (rc) return report(rc);
    }
    if (ccusb_reg_read(a->dev, CCUSB_REG_GLOBAL_MODE, &gm) == CCUSB_OK)
        double_header = (gm & CCUSB_GMODE_DOUBLE_HEADER) != 0;
    if (out) {
        fo = fopen(out, "wb");
        if (!fo) {
            fprintf(stderr, "error: cannot create %s: %s\n", out, strerror(errno));
            return 1;
        }
    }
    buf = (uint16_t *)malloc(32768 * sizeof(uint16_t));
    if (!buf) return report(CCUSB_ERR_NO_MEM);
    pr.remaining = (int)nprint;
    rc = ccusb_daq_start(a->dev);
    if (rc) {
        free(buf);
        if (fo) fclose(fo);
        return report(rc);
    }
    fprintf(stderr, "taking data for %.0f s ...\n", seconds);
    start = time(NULL);
    t0 = clock();
    (void)t0;
    for (;;) {
        int stop = difftime(time(NULL), start) >= seconds;
        if (stop && ccusb_daq_running(a->dev)) ccusb_daq_stop(a->dev);
        rc = ccusb_read_buffer(a->dev, buf, 32768, &n, stop ? 300 : 1000);
        if (rc == CCUSB_ERR_TIMEOUT) {
            if (stop) break;
            continue;
        }
        if (rc) {
            ccusb_daq_stop(a->dev);
            report(rc);
            break;
        }
        nbuf++;
        nwords += (unsigned long)n;
        {
            int e = ccusb_parse_buffer(buf, n, double_header, print_event, &pr);
            if (e > 0) nev += (unsigned long)e;
        }
        if (fo) {
            uint8_t hdr[4];
            size_t k;
            hdr[0] = (uint8_t)(n & 0xFF);
            hdr[1] = (uint8_t)((n >> 8) & 0xFF);
            hdr[2] = (uint8_t)((n >> 16) & 0xFF);
            hdr[3] = (uint8_t)((n >> 24) & 0xFF);
            fwrite(hdr, 1, 4, fo);
            for (k = 0; k < n; k++) {
                uint8_t b[2];
                b[0] = (uint8_t)(buf[k] & 0xFF);
                b[1] = (uint8_t)(buf[k] >> 8);
                fwrite(b, 1, 2, fo);
            }
        }
    }
    if (fo) fclose(fo);
    free(buf);
    printf("%lu buffers, %lu events, %lu words (%.1f events/s)\n", nbuf, nev,
           nwords, seconds > 0 ? nev / seconds : 0.0);
    if (out) printf("raw buffers written to %s (each: uint32 word count + words)\n", out);
    return 0;
}

static int run_command(app *a, int argc, char **argv);

static int cmd_shell(app *a)
{
    char line[512];
    printf("ccusb shell - '%s'. Type 'help' for commands, 'quit' to exit.\n",
           ccusb_serial(a->dev));
    for (;;) {
        char *argv[MAX_ARGS];
        int argc;
        printf("ccusb> ");
        fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;
        argc = tokenize(line, argv, MAX_ARGS);
        if (argc == 0) continue;
        if (strcmp(argv[0], "quit") == 0 || strcmp(argv[0], "exit") == 0) break;
        if (strcmp(argv[0], "shell") == 0) continue;
        run_command(a, argc, argv);
    }
    printf("\n");
    return 0;
}

static int run_command(app *a, int argc, char **argv)
{
    const char *c = argv[0];
    if (strcmp(c, "help") == 0 || strcmp(c, "-h") == 0 || strcmp(c, "--help") == 0) {
        fputs(usage_text, stdout);
        return 0;
    }
    if (strcmp(c, "list") == 0) return cmd_list();
    if (ensure_open(a)) return 1;
    if (strcmp(c, "info") == 0) return cmd_info(a);
    if (strcmp(c, "naf") == 0 || strcmp(c, "read") == 0 ||
        strcmp(c, "write") == 0 || strcmp(c, "control") == 0)
        return cmd_naf(a, argc, argv, strcmp(c, "naf") == 0 ? NULL : c);
    if (strcmp(c, "z") == 0) return report(ccusb_z(a->dev));
    if (strcmp(c, "c") == 0) return report(ccusb_c(a->dev));
    if (strcmp(c, "inhibit") == 0) {
        int on, rc;
        if (argc > 1) {
            rc = ccusb_inhibit(a->dev, strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0);
            if (rc) return report(rc);
        }
        rc = ccusb_is_inhibited(a->dev, &on);
        if (rc) return report(rc);
        printf("inhibit %s\n", on ? "on" : "off");
        return 0;
    }
    if (strcmp(c, "reg") == 0) return cmd_reg(a, argc, argv);
    if (strcmp(c, "scan") == 0) return cmd_scan(a);
    if (strcmp(c, "lam") == 0) {
        uint32_t l;
        int rc = ccusb_read_lams(a->dev, &l), n;
        if (rc) return report(rc);
        printf("LAM 0x%06x:", l);
        for (n = 1; n <= 24; n++)
            if ((l >> (n - 1)) & 1) printf(" N%d", n);
        printf("\n");
        return 0;
    }
    if (strcmp(c, "scalers") == 0) {
        uint32_t sa, sb;
        int rc = ccusb_read_scalers(a->dev, &sa, &sb);
        if (rc) return report(rc);
        printf("scaler A %u\nscaler B %u\n", sa, sb);
        return 0;
    }
    if (strcmp(c, "action") == 0) {
        long v;
        if (!need_nums(argc, argv, 1, 1, &v)) return 1;
        return report(ccusb_write_action(a->dev, (uint16_t)v));
    }
    if (strcmp(c, "stack") == 0) return cmd_stack(a, argc, argv);
    if (strcmp(c, "daq") == 0) return cmd_daq(a, argc, argv);
    if (strcmp(c, "shell") == 0) return cmd_shell(a);
    fprintf(stderr, "unknown command '%s' (try 'help')\n", c);
    return 1;
}

int main(int argc, char **argv)
{
    app a;
    int i, rc;
    memset(&a, 0, sizeof(a));
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc)
            a.serial = argv[++i];
        else if (strcmp(argv[i], "--sim") == 0)
            a.serial = "SIM";
        else if (strcmp(argv[i], "-v") == 0)
            a.trace = 1;
        else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            a.timeout = (unsigned)strtoul(argv[++i], NULL, 0);
        else
            break;
    }
    if (i >= argc) {
        fputs(usage_text, stderr);
        return 2;
    }
    rc = run_command(&a, argc - i, argv + i);
    ccusb_close(a.dev);
    return rc;
}
