// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <xs1.h>
#include <xclib.h>
#include <stdint.h>
#include <stdio.h>

#include "spdif.h"

#if(!LEGACY_SPDIF_RECEIVER)

static inline int cls(int idata)
{
    int x;
#if __XS3A__
    asm volatile("cls %0, %1" : "=r"(x)  : "r"(idata));
#else
    x = (clz(idata) + clz(~idata));
#endif
    return x;
}

static inline int xor4(int idata1, int idata2, int idata3, int idata4)
{
    int x;

#if !defined(__XS3A__) || !defined(__XS2A__)
    /* For doc build only */
    x = idata1 ^ idata2 ^ idata3 ^ idata4;
#else
    asm volatile("xor4 %0, %1, %2, %3, %4" : "=r"(x)  : "r"(idata1), "r"(idata2), "r"(idata3), "r"(idata4));
#endif
    return x;
}

// Lookup tables for port time adder based on where the reference transition was.
// Index can be max of 32 so need 33 element array.
// Index 0 is never used.
const unsigned error_lookup_441[33]     = {0,36,36,35,35,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42};
const unsigned error_lookup_48[33]      = {0,33,33,32,32,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39};

const unsigned error_lookup_441_2X[33]  = {0,36,36,36,36,35,35,35,35,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49};
const unsigned error_lookup_48_2X[33]   = {0,33,33,33,32,32,32,32,32,32,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46};

#pragma unsafe arrays
static inline void spdif_rx_8UI_48(buffered in port:32 p, unsigned &t, unsigned &sample, unsigned &outword, unsigned &unlock_cnt)
{
    unsigned crc;
    unsigned ref_tran;

    // 48k standard
    unsigned unscramble_0x04040404_0xF[16] = {
    0x80000000, 0x90000000, 0xC0000000, 0xD0000000,
    0x70000000, 0x60000000, 0x30000000, 0x20000000,
    0xA0000000, 0xB0000000, 0xE0000000, 0xF0000000,
    0x50000000, 0x40000000, 0x10000000, 0x00000000};

    // Now receive data
    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));
    ref_tran = cls(sample<<9); // Expected value is 2 Possible values are 1 to 32.
    t += error_lookup_48[ref_tran]; // Lookup next port time based off where current transition was.
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    if (ref_tran > 4)
      unlock_cnt++;
    crc = sample & 0x04040404;
    crc32(crc, 0xF, 0xF);
    outword >>= 4;
    outword |= unscramble_0x04040404_0xF[crc];
}

// 6, 22 is middle of data eye. 5, 21 works better for optical at limit as ISI makes shorts shorter so second half of eye shifted earlier.
#define SAMP_BIT_LO_48  5
#define SAMP_BIT_HI_48  21

#pragma unsafe arrays
static inline void spdif_rx_8UI_48_2X(buffered in port:32 p, unsigned &t, unsigned &sample1, unsigned &sample2, unsigned &outword, unsigned &unlock_cnt)
{
    unsigned ref_tran;
    
    // First 4UI - default time
    asm volatile("in %0, res[%1]" : "=r"(sample1)  : "r"(p));
    t += 33;
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    outword >>= 2;
    outword |= ((sample1 & (1<<SAMP_BIT_HI_48)) << (31-SAMP_BIT_HI_48)) | ((sample1 & (1<<SAMP_BIT_LO_48)) << (30-SAMP_BIT_LO_48));

    // Now receive data - second 4UI
    asm volatile("in %0, res[%1]" : "=r"(sample2)  : "r"(p));
    ref_tran = cls(sample2<<18); // Expected value is 4 Possible values are 1 to 32.
    t += error_lookup_48_2X[ref_tran]; // Lookup next port time based off where current transition was.
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    if (ref_tran > 8)
      unlock_cnt++;
    outword >>= 2;
    outword |= ((sample2 & (1<<SAMP_BIT_HI_48)) << (31-SAMP_BIT_HI_48)) | ((sample2 & (1<<SAMP_BIT_LO_48)) << (30-SAMP_BIT_LO_48));
}

#pragma unsafe arrays
static inline void spdif_rx_8UI_441(buffered in port:32 p, unsigned &t, unsigned &sample, unsigned &outword, unsigned &unlock_cnt)
{
    unsigned crc;
    unsigned ref_tran;

    // 44.1k standard
    unsigned unscramble_0x08040201_0xF[16] = {
    0xF0000000, 0x70000000, 0xB0000000, 0x30000000,
    0xD0000000, 0x50000000, 0x90000000, 0x10000000,
    0xE0000000, 0x60000000, 0xA0000000, 0x20000000,
    0xC0000000, 0x40000000, 0x80000000, 0x00000000};

    // Now receive data
    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));
    ref_tran = cls(sample<<9); // Expected value is 2 Possible values are 1 to 32.
    t += error_lookup_441[ref_tran]; // Lookup next port time based off where current transition was.
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    if (ref_tran > 4)
      unlock_cnt++;
    crc = sample & 0x08040201;
    crc32(crc, 0xF, 0xF);
    outword >>= 4;
    outword |= unscramble_0x08040201_0xF[crc];
}

// Orig settings 4,22. 4 is middle of "perfect" coax data eye from testing. 21 or 22 is middle of eye. not much in it. Pick 21.
#define SAMP_BIT_LO_441 4
#define SAMP_BIT_HI_441 21

#pragma unsafe arrays
static inline void spdif_rx_8UI_441_2X(buffered in port:32 p, unsigned &t, unsigned &sample1, unsigned &sample2, unsigned &outword, unsigned &unlock_cnt)
{
    unsigned ref_tran;
    
    // First 4UI - default time
    asm volatile("in %0, res[%1]" : "=r"(sample1)  : "r"(p));
    t += 35;
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    outword >>= 2;
    outword |= ((sample1 & (1<<SAMP_BIT_HI_441)) << (31-SAMP_BIT_HI_441)) | ((sample1 & (1<<SAMP_BIT_LO_441)) << (30-SAMP_BIT_LO_441));

    // Now receive data - second 4UI
    asm volatile("in %0, res[%1]" : "=r"(sample2)  : "r"(p));
    ref_tran = cls(sample2<<19); // Expected value is 4 Possible values are 1 to 32.
    t += error_lookup_441_2X[ref_tran]; // Lookup next port time based off where current transition was.
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));
    if (ref_tran > 8)
      unlock_cnt++;
    outword >>= 2;
    outword |= ((sample2 & (1<<SAMP_BIT_HI_441)) << (31-SAMP_BIT_HI_441)) | ((sample2 & (1<<SAMP_BIT_LO_441)) << (30-SAMP_BIT_LO_441));
}

void spdif_rx_48(streaming chanend c, buffered in port:32 p)
{
    unsigned sample;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;

    // Read the port counter and add a bit.
    p :> void @ t; // read port counter
    t+= 100;
    // Note, this is inline asm since xc can only express a timed input/output
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));

    // Now receive data
    while(unlock_cnt < 32)
    {
        spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
        if (cls(sample) > 9) // Last three bits of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            z_pre_sample = sample;
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            if (cls(z_pre_sample<<13) > 8)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_48(p, t, sample, outword, unlock_cnt);
        }
    }
}

void spdif_rx_48_2X(streaming chanend c, buffered in port:32 p)
{
    unsigned sample1, sample2;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;

    // Read the port counter and add a bit.
    p :> void @ t; // read port counter
    t+= 100;
    // Note, this is inline asm since xc can only express a timed input/output
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));

    // Now receive data
    while(unlock_cnt < 64)
    {
        spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
        if (cls(sample2) > 18) // Last bit of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            z_pre_sample = sample1;
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            if (cls(z_pre_sample) > 10)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_48_2X(p, t, sample1, sample2, outword, unlock_cnt);
        }
    }
}

void spdif_rx_441(streaming chanend c, buffered in port:32 p)
{
    unsigned sample;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;

    // Read the port counter and add a bit.
    p :> void @ t; // read port counter
    t+= 100;
    // Note, this is inline asm since xc can only express a timed input/output
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));

    // Now receive data
    while(unlock_cnt < 32)
    {
        spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
        if (cls(sample) > 9) // Last three bits of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            z_pre_sample = sample;
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            if (cls(z_pre_sample<<13) > 9)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
            spdif_rx_8UI_441(p, t, sample, outword, unlock_cnt);
        }
    }
}

void spdif_rx_441_2X(streaming chanend c, buffered in port:32 p)
{
    unsigned sample1, sample2;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;

    // Read the port counter and add a bit.
    p :> void @ t; // read port counter
    t+= 100;
    // Note, this is inline asm since xc can only express a timed input/output
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t));

    // Now receive data
    while(unlock_cnt < 64)
    {
        spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
        if (cls(sample2) > 19) // Last one bit of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            z_pre_sample = sample1;
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            if (cls(z_pre_sample) > 10)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
            spdif_rx_8UI_441_2X(p, t, sample1, sample2, outword, unlock_cnt);
        }
    }
}

// This function checks the input signal is approximately the correct sample rate for the given mode/clock setting.
int check_clock_div(buffered in port:32 p, unsigned decode_2x)
{
    unsigned sample;
    unsigned max_pulse = 0;

    // Flush the port
    p :> void;
    p :> void;

    // Capture a large number of samples directly from the port and record the maximum pulse length seen.
    // Need enough samples to ensure we get a realistic 3UI max pulse which only happen in the preambles.
    // Only looking at leading pulse on each word which will be shorter than actual but due to async sampling
    // will eventually move into timing to correctly capture correct length.
    for(int i=0; i<2000;i++) // 2000 32 bit samples @ 100MHz takes 640us
    {
        p :> sample;
        if (cls(sample) > max_pulse)
        {
            max_pulse = cls(sample);
        }
    }
    
    // printf("max_pulse = %d\n", max_pulse);
    // Check if the max_pulse is in expected range.
    // Shortest expected is 3UI @ 96k = 244ns nominal. Sampled @ 10ns (decode_2x) = 24 bits. Sampled at 20ns (1x) = 12 bits.
    // Longest expected is 3UI @ 88.2k = 266ns nominal but up to 300ns w/jitter.
    // Sampled @ 10ns (decode_2x) = 31 bits. Sampled at 20ns (1x) = 16 bits.
    // Note DC (all 0 or all 1s) will correctly fail (return 1) as max_pulse = 32.
    if (decode_2x)
    {
        if ((max_pulse > 22) && (max_pulse < 32))
            return 0;
    }
    else
    {
        if ((max_pulse > 11) && (max_pulse < 17))
            return 0;
    }
    return 1;
}

#endif

