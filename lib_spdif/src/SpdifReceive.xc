// Copyright 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include <xs1.h>
#include <xclib.h>
#include <stdint.h>

#include "spdif.h"

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

// Lookup table for error signal based on where the reference transition was.
// Index can be max of 32 so need 33 element array.
// Index 0 is never used.
// To maximise timing margins, I actually need 0 error at input of 2.375 (2+3/8).
// This implements ((input-2.375)*8) - so we now have a 29.3 fixed point number. The 0 error is now at an input of 2.375
// We also apply an additional << 3 (*8) to save instructions later on in the PLL control loop.
// Above an index of 6 we apply a max limit to the error to avoid port INs missing timing (this level of error only used during initial lock)
const int error_lookup[33] = {-152,-88,-24,40,104,168,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232,232};

#pragma unsafe arrays
static inline void spdif_rx_8UI_48(buffered in port:32 p, unsigned &t, unsigned &sample, unsigned &outword, unsigned &adder)
{
    // 48k standard
    const unsigned unscramble_0x04040404_0xF[16] = {
    0x80000000, 0x90000000, 0xC0000000, 0xD0000000,
    0x70000000, 0x60000000, 0x30000000, 0x20000000,
    0xA0000000, 0xB0000000, 0xE0000000, 0xF0000000,
    0x50000000, 0x40000000, 0x10000000, 0x00000000};

    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));  // Input data sample
    t += adder;                                               // Add current adder value to the time for next input
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t>>16));     // Write that time to the port
    unsigned crc = sample & 0x04040404;                       // Apply a mask to sample the data bits we want
    crc32(crc, 0xF, 0xF);                                     // Use crc as a hash function to produce a unique 4 bit value based off value of sampled bits
    outword >>= 4;                                            // Shift output word by 4 to make room for new data
    outword |= unscramble_0x04040404_0xF[crc];                // OR the sampled data bits into the output word using a de-hash lookup table from the crc value.
}

#pragma unsafe arrays
static inline void spdif_rx_8UI_441(buffered in port:32 p, unsigned &t, unsigned &sample, unsigned &outword, unsigned &adder)
{
    // 44.1k standard
    const unsigned unscramble_0x08040201_0xF[16] = {
    0xF0000000, 0x70000000, 0xB0000000, 0x30000000,
    0xD0000000, 0x50000000, 0x90000000, 0x10000000,
    0xE0000000, 0x60000000, 0xA0000000, 0x20000000,
    0xC0000000, 0x40000000, 0x80000000, 0x00000000};

    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));  // Input data sample
    t += adder;                                               // Add current adder value to the time for next input
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t>>16));     // Write the top 16 bits of that 16.16 fixed point time to the port
    unsigned crc = sample & 0x08040201;                       // Apply a mask to sample the data bits we want
    crc32(crc, 0xF, 0xF);                                     // Use crc as a hash function into a unique 4 bit value based off value of sampled bits
    outword >>= 4;                                            // Shift output word by 4 to make room for new data
    outword |= unscramble_0x08040201_0xF[crc];                // OR the sampled data bits into the output word using a de-hash lookup table from the crc value
}

#pragma unsafe arrays
static inline void spdif_rx_lock(buffered in port:32 p, unsigned &t, unsigned &adder)
{
    unsigned sample, raw_err;
    unsigned pre_count = 0;

    // Read the port counter and add a bit.
    p :> void @ t; // read port counter
    t+= 100;
    t <<= 16; // t is now in 16.16 fixed point format

    // Run the loop for 512 INs, just running the PLL every time, no preamble check. This is because in this initial course lock phase we may have port times that are below nominal giving us even less time.
    // We will exit this phase in lock but maybe locked to the wrong edge. The adder value will be approx correct.
    for(int i = 0; i < 512;i++)
    {
        asm volatile("setpt res[%0], %1"::"r"(p),"r"(t>>16));
        asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));
        raw_err = error_lookup[cls(sample<<9)];
        adder -= raw_err;
        t += adder - (raw_err<<9);
    }

    // We then run loop again for a fixed time looking for preambles.
    // Check preambles are there, if not we add 2UI to port time and exit. This will bump the port time to the correct reference edge.
    for(int i = 0; i < 512;i++)
    {
        asm volatile("setpt res[%0], %1"::"r"(p),"r"(t>>16));
        if (cls(sample) > 9)
          pre_count++;
        asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p));
        raw_err = error_lookup[cls(sample<<9)];
        adder -= raw_err;
        t += adder - (raw_err<<9);
    }

    if (pre_count < 16)
       t += (17<<15); // 8.5

    // Set the new port time ready for the first IN.
    asm volatile("setpt res[%0], %1"::"r"(p),"r"(t>>16));
}

#pragma unsafe arrays
int spdif_rx_48(streaming chanend c, buffered in port:32 p)
{
    unsigned sample;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;
    unsigned adder = 2133333; // ideal 48 (32.552 * 65536)
    unsigned char tmp;

    // Run function to lock to input stream, retain port time and adder to be used directly below.
    spdif_rx_lock(p, t, adder);

    // Now receive data
    while(unlock_cnt < 32)
    {
        spdif_rx_8UI_48(p, t, sample, outword, adder);
        if (cls(sample) > 9) // Last three bits of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_48(p, t, sample, outword, adder);
            z_pre_sample = sample;
            // Measure the position of reference edge and apply correction to PLL.
            unsigned ref_tran = cls(sample<<9);
            int raw_err = error_lookup[ref_tran];
            if (ref_tran > 5)
                unlock_cnt++;
            adder -= raw_err;
            t -= raw_err<<6;
            spdif_rx_8UI_48(p, t, sample, outword, adder);
            spdif_rx_8UI_48(p, t, sample, outword, adder);
            spdif_rx_8UI_48(p, t, sample, outword, adder);
            if (cls(z_pre_sample<<13) > 8)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_48(p, t, sample, outword, adder);

            /* Check for exit */
            select
            {
                case sinct_byref(c, tmp):
                    soutct(c, XS1_CT_END);
                    return 1;
                default:
                    break;
            }

            spdif_rx_8UI_48(p, t, sample, outword, adder);
            spdif_rx_8UI_48(p, t, sample, outword, adder);
        }
    }

    return 0;
}

#pragma unsafe arrays
int spdif_rx_441(streaming chanend c, buffered in port:32 p)
{
    unsigned sample;
    unsigned outword = 0;
    unsigned z_pre_sample = 0;
    unsigned unlock_cnt = 0;
    unsigned t;
    unsigned adder = 2321995; // ideal 44.1 (35.430 * 65536).
    unsigned char tmp;

    // Run function to lock to input stream, retain port time and adder to be used directly below.
    spdif_rx_lock(p, t, adder);

    // Now receive data
    while(unlock_cnt < 32)
    {
        spdif_rx_8UI_441(p, t, sample, outword, adder);
        if (cls(sample) > 9) // Last three bits of old subframe and first "bit" of preamble.
        {
            outword = xor4(outword, (outword << 1), 0xFFFFFFFF, z_pre_sample); // This achieves the xor decode plus inverting the output in one step.
            outword <<= 1;
            c <: outword;

            spdif_rx_8UI_441(p, t, sample, outword, adder);
            z_pre_sample = sample;
            // Measure the position of reference edge and apply correction to PLL.
            unsigned ref_tran = cls(sample<<9);
            int raw_err = error_lookup[ref_tran];
            if (ref_tran > 5)
                unlock_cnt++;
            adder -= raw_err;
            t -= raw_err<<6;
            spdif_rx_8UI_441(p, t, sample, outword, adder);
            spdif_rx_8UI_441(p, t, sample, outword, adder);
            spdif_rx_8UI_441(p, t, sample, outword, adder);
            if (cls(z_pre_sample<<13) > 9)
              z_pre_sample = 2;
            else
              z_pre_sample = 0;
            spdif_rx_8UI_441(p, t, sample, outword, adder);

            /* Check for exit */
            select
            {
                case sinct_byref(c, tmp):
                    soutct(c, XS1_CT_END);
                    return 1;
                default:
                    break;
            }

            spdif_rx_8UI_441(p, t, sample, outword, adder);
            spdif_rx_8UI_441(p, t, sample, outword, adder);
        }
    }

    return 0;
}

// This function checks the input signal is approximately the correct sample rate for the given mode/clock setting.
int check_clock_div(buffered in port:32 p)
{
    unsigned sample;
    unsigned max_pulse = 0;
    unsigned min_pulse = 1000;

    // Flush the port
    p :> void;
    p :> void;

    // Capture a large number of samples directly from the port and record the maximum pulse length seen.
    // Need enough samples to ensure we get a realistic 3UI max pulse which only happen in the preambles.
    // Only looking at leading pulse on each word which will be shorter than actual but due to async sampling
    // will eventually move into timing to correctly capture correct length.
    for(int i=0; i<5000;i++) // 5000 32 bit samples @ 100MHz takes 1.6ms
    {
        p :> sample;
        if (cls(sample) > max_pulse)
        {
            max_pulse = cls(sample);
        }

        // Now find the minimum pulse width
        sample <<= cls(sample); // Shift off the top pulse (likely to not be a complete pulse)
        if (cls(sample) < min_pulse)
        {
            min_pulse = cls(sample);
        }
    }

    // Check if the max_pulse is in expected range.
    // Shortest expected is 3UI @ 96k = 244ns nominal. Sampled at 20ns = 12 bits.
    // Longest expected is 3UI @ 88.2k = 266ns nominal but up to 300ns w/jitter.
    // Sampled at 20ns = 16 bits.
    // Note DC (all 0 or all 1s) will correctly fail (return 1) as max_pulse = 32.
    if ((max_pulse > 11) && (max_pulse < 17) && (min_pulse > 1) && (min_pulse < 7))
    {
        return 0;
    }
    return 1;
}

