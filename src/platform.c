/* platform.c - tiny portability helpers (time, sleep). */
#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200809L
#endif
#include "ccusb_internal.h"

#ifdef _WIN32
#include <windows.h>

uint64_t ccusb__now_ms(void)
{
    return (uint64_t)GetTickCount64();
}

void ccusb__sleep_ms(unsigned int ms)
{
    Sleep(ms);
}
#else
#include <time.h>

uint64_t ccusb__now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

void ccusb__sleep_ms(unsigned int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}
#endif
